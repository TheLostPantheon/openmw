// Dump every function of the current program as "offset<TAB>size<TAB>name",
// offset relative to the first executable block (a Vita module's text
// segment, "Module@1 + 0x..." in vita-parse-core output).
// Usage (headless): -postScript DumpFunctionTable.java <out.tsv>
// @category Vita

import java.io.PrintWriter;

import ghidra.app.script.GhidraScript;
import ghidra.program.model.listing.Function;
import ghidra.program.model.mem.MemoryBlock;

public class DumpFunctionTable extends GhidraScript {
    @Override
    protected void run() throws Exception {
        String[] args = getScriptArgs();
        if (args.length < 1) {
            printerr("usage: DumpFunctionTable.java <out.tsv>");
            return;
        }
        long textBase = -1;
        for (MemoryBlock b : currentProgram.getMemory().getBlocks()) {
            if (b.isExecute()) {
                textBase = b.getStart().getOffset();
                break;
            }
        }
        if (textBase < 0) {
            printerr("no executable block");
            return;
        }
        int n = 0;
        try (PrintWriter out = new PrintWriter(args[0])) {
            out.println("# " + currentProgram.getName() + " text_base=0x" + Long.toHexString(textBase));
            for (Function f : currentProgram.getFunctionManager().getFunctions(true)) {
                long off = f.getEntryPoint().getOffset() - textBase;
                if (off < 0)
                    continue;
                out.println(String.format("0x%x\t%d\t%s", off, f.getBody().getNumAddresses(), f.getName()));
                n++;
            }
        }
        println("wrote " + n + " functions to " + args[0]);
    }
}
