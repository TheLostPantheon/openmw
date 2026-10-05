// Decompile the Thumb functions in one address range of a program imported
// without auto-analysis (e.g. a 30 MB homebrew VELF where full analysis
// would take hours). Marks the range Thumb, creates functions at push-lr
// prologues, analyzes just that range, and writes one pseudo-C file per
// function plus an index (address, size, callees) to <outDir>.
// Usage (headless): -postScript DecompileRange.java <startHex> <endHex> <outDir>
// @category Vita

import java.io.File;
import java.io.PrintWriter;
import java.math.BigInteger;

import ghidra.app.cmd.disassemble.DisassembleCommand;
import ghidra.app.cmd.function.CreateFunctionCmd;
import ghidra.app.decompiler.DecompInterface;
import ghidra.app.decompiler.DecompileResults;
import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.address.AddressSet;
import ghidra.program.model.lang.Register;
import ghidra.program.model.listing.Function;
import ghidra.program.model.mem.Memory;

public class DecompileRange extends GhidraScript {
    @Override
    protected void run() throws Exception {
        String[] args = getScriptArgs();
        Address start = toAddr(Long.parseLong(args[0].replace("0x", ""), 16));
        Address end = toAddr(Long.parseLong(args[1].replace("0x", ""), 16));
        File outDir = new File(args[2]);
        outDir.mkdirs();
        AddressSet range = new AddressSet(start, end);

        Register tmode = currentProgram.getRegister("TMode");
        currentProgram.getProgramContext().setValue(tmode, start, end, BigInteger.ONE);

        // Function starts: 16-bit PUSH {..., LR} (0xB5xx) or 32-bit
        // PUSH.W {..., LR} (0xE92D 0x4xxx).
        Memory mem = currentProgram.getMemory();
        int created = 0;
        for (Address a = start; a.compareTo(end) < 0; a = a.add(2)) {
            int hw = mem.getShort(a) & 0xFFFF;
            boolean prologue = (hw & 0xFF00) == 0xB500;
            if (!prologue && hw == 0xE92D) {
                int hw2 = mem.getShort(a.add(2)) & 0xFFFF;
                prologue = (hw2 & 0x4000) != 0;
            }
            if (!prologue || getFunctionContaining(a) != null)
                continue;
            new DisassembleCommand(a, null, true).applyTo(currentProgram, monitor);
            if (new CreateFunctionCmd(a).applyTo(currentProgram, monitor))
                created++;
        }
        analyzeChanges(currentProgram);
        println("created " + created + " functions");

        DecompInterface dec = new DecompInterface();
        dec.openProgram(currentProgram);
        try (PrintWriter index = new PrintWriter(new File(outDir, "index.tsv"))) {
            for (Function f : currentProgram.getFunctionManager().getFunctions(range, true)) {
                DecompileResults r = dec.decompileFunction(f, 60, monitor);
                String body = r.decompileCompleted() ? r.getDecompiledFunction().getC() : "// decompile failed\n";
                String name = String.format("%08x", f.getEntryPoint().getOffset());
                try (PrintWriter out = new PrintWriter(new File(outDir, name + ".c"))) {
                    out.print(body);
                }
                StringBuilder callees = new StringBuilder();
                for (Function c : f.getCalledFunctions(monitor))
                    callees.append(c.getName()).append(',');
                index.println(name + "\t" + f.getBody().getNumAddresses() + "\t" + callees);
            }
        }
        dec.dispose();
    }
}
