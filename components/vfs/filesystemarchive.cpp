#include "filesystemarchive.hpp"

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>

#include "pathutil.hpp"

#include <components/debug/debuglog.hpp>
#include <components/files/constrainedfilestream.hpp>
#include <components/files/conversion.hpp>

#ifdef __vita__
#include <cstdio>
#include <cstring>
#include <vector>

#include <psp2/io/dirent.h>
#include <psp2/kernel/processmgr.h>

extern "C" void vitaBreadcrumb(const char* msg); // apps/openmw/vita/VitaInit.cpp
#endif

namespace VFS
{
    namespace
    {
        // macOS metadata: copying a mod to an exFAT memory card from a Mac
        // leaves a ._<name> file beside every file and folder, and Finder adds
        // .DS_Store. Never game data; don't index them.
        bool isMacMetadata(std::string_view name)
        {
            return name.starts_with("._") || name == ".DS_Store";
        }
    }

    FileSystemArchive::FileSystemArchive(const std::filesystem::path& path,
        const std::filesystem::path& cacheFile)
        : mPath(path)
    {
        if (!cacheFile.empty() && loadCache(cacheFile))
            return;

        walkDirectory();

        if (!cacheFile.empty())
            saveCache(cacheFile);
    }

#ifdef __vita__
    // sceIoDread hands back each entry's type with its name. The
    // std::filesystem walk stat()s every entry again (to tell files from
    // folders, and again to decide whether to descend), a memory-card lookup
    // each: the first scan of a data folder with tens of thousands of loose
    // files (Tamriel_Data: 54k) took minutes. False when the folder can't be
    // opened; the portable walk then reports it.
    bool FileSystemArchive::walkDirectoryVita()
    {
        std::string root = Files::pathToUnicodeString(mPath);
        while (!root.empty() && (root.back() == '/' || root.back() == '\\'))
            root.pop_back();
        const SceUInt64 start = sceKernelGetProcessTimeWide();
        std::vector<std::string> pending{ std::string() }; // folders to read, relative to root
        bool openedRoot = false;
        while (!pending.empty())
        {
            const std::string rel = std::move(pending.back());
            pending.pop_back();
            const std::string dir = rel.empty() ? root : root + '/' + rel;
            const SceUID dd = sceIoDopen(dir.c_str());
            if (dd < 0)
            {
                if (rel.empty())
                    return false;
                Log(Debug::Warning) << "Failed to read \"" << dir << "\": " << dd;
                continue;
            }
            openedRoot = true;
            SceIoDirent ent;
            memset(&ent, 0, sizeof(ent));
            while (sceIoDread(dd, &ent) > 0)
            {
                const std::string_view name(ent.d_name);
                if (!name.empty() && name != "." && name != ".." && !isMacMetadata(name))
                {
                    std::string childRel = rel.empty() ? std::string(name) : rel + '/' + std::string(name);
                    if (SCE_S_ISDIR(ent.d_stat.st_mode))
                        pending.push_back(std::move(childRel));
                    else
                    {
                        VFS::Path::Normalized searchable(childRel);
                        const auto inserted = mIndex.emplace(
                            std::move(searchable), FileSystemArchiveFile(root + '/' + childRel));
                        if (!inserted.second)
                            Log(Debug::Warning) << "Found duplicate file for '" << root << '/' << childRel
                                                << "', please check your file system for two files with the same "
                                                   "name in different cases.";
                    }
                }
                memset(&ent, 0, sizeof(ent));
            }
            sceIoDclose(dd);
        }
        char crumb[256];
        snprintf(crumb, sizeof(crumb), "[VFS] indexed %u files in %s (%u ms)", static_cast<unsigned>(mIndex.size()),
            root.c_str(), static_cast<unsigned>((sceKernelGetProcessTimeWide() - start) / 1000));
        vitaBreadcrumb(crumb);
        return openedRoot;
    }
#endif

    void FileSystemArchive::walkDirectory()
    {
#ifdef __vita__
        if (walkDirectoryVita())
            return;
#endif
        const auto str = mPath.u8string();
        std::size_t prefix = str.size();

        if (prefix > 0 && str[prefix - 1] != '\\' && str[prefix - 1] != '/')
            ++prefix;

        std::filesystem::recursive_directory_iterator iterator(mPath);

        for (auto it = std::filesystem::begin(iterator), end = std::filesystem::end(iterator); it != end;)
        {
            const std::filesystem::directory_entry& entry = *it;

            if (!isMacMetadata(Files::pathToUnicodeString(entry.path().filename())) && !entry.is_directory())
            {
                const std::filesystem::path& filePath = entry.path();
                const std::string proper = Files::pathToUnicodeString(filePath);
                VFS::Path::Normalized searchable(std::string_view{ proper }.substr(prefix));
                FileSystemArchiveFile file{ std::string(proper) };

                const auto inserted = mIndex.emplace(std::move(searchable), std::move(file));
                if (!inserted.second)
                    Log(Debug::Warning)
                        << "Found duplicate file for '" << proper
                        << "', please check your file system for two files with the same name in different cases.";
            }

            const std::filesystem::path prevPath = entry.path();
            std::error_code ec;
            it.increment(ec);
            if (ec != std::error_code())
                throw std::runtime_error("Failed to recursively iterate over \"" + Files::pathToUnicodeString(mPath)
                    + "\" when incrementing to the next item from \"" + Files::pathToUnicodeString(prevPath)
                    + "\": " + ec.message());
        }
    }

    // The cache is read and written in one piece: field by field through
    // the streams it was one memory-card call per field (Vita), and saving
    // Tamriel_Data's 54k-file list took 49 s.
    bool FileSystemArchive::loadCache(const std::filesystem::path& cacheFile)
    {
        std::string data;
        {
            std::ifstream is(cacheFile, std::ios::binary | std::ios::ate);
            if (!is)
                return false;
            const std::streamoff size = is.tellg();
            if (size <= 0)
                return false;
            data.resize(static_cast<std::size_t>(size));
            is.seekg(0);
            if (!is.read(data.data(), size))
                return false;
        }

        constexpr std::uint32_t kMagic = 0x43534656; // "VFSC"
        constexpr std::uint32_t kSchema = 1;
        std::size_t pos = 0;

        auto readU32 = [&](std::uint32_t& v) {
            if (data.size() - pos < 4)
                return false;
            std::memcpy(&v, data.data() + pos, 4);
            pos += 4;
            return true;
        };
        auto readStr = [&](std::string& s) {
            std::uint32_t len;
            if (!readU32(len) || len > 4096 || data.size() - pos < len)
                return false;
            s.assign(data.data() + pos, len);
            pos += len;
            return true;
        };

        std::uint32_t magic, schema, numEntries;
        if (!readU32(magic) || magic != kMagic) return false;
        if (!readU32(schema) || schema != kSchema) return false;

        std::string cachedDir;
        if (!readStr(cachedDir) || cachedDir != Files::pathToUnicodeString(mPath))
            return false;

        if (!readU32(numEntries)) return false;

        for (std::uint32_t i = 0; i < numEntries; ++i)
        {
            std::string key, filepath;
            if (!readStr(key) || !readStr(filepath))
            {
                mIndex.clear(); // truncated: walk the folder instead
                return false;
            }
            // Caches written before macOS metadata was skipped list it.
            const std::size_t slash = key.find_last_of("/\\");
            if (isMacMetadata(std::string_view(key).substr(slash == std::string::npos ? 0 : slash + 1)))
                continue;
            mIndex.emplace(VFS::Path::Normalized(std::move(key)), FileSystemArchiveFile(std::move(filepath)));
        }

        Log(Debug::Info) << "VFS cache loaded: " << mIndex.size() << " files for " << mPath;
        return true;
    }

    void FileSystemArchive::saveCache(const std::filesystem::path& cacheFile) const
    {
        std::error_code ec;
        std::filesystem::create_directories(cacheFile.parent_path(), ec);

        std::string out;
        auto writeU32 = [&](std::uint32_t v) { out.append(reinterpret_cast<const char*>(&v), 4); };
        auto writeStr = [&](std::string_view s) {
            writeU32(static_cast<std::uint32_t>(s.size()));
            out.append(s.data(), s.size());
        };

        writeU32(0x43534656); // magic
        writeU32(1);          // schema
        writeStr(Files::pathToUnicodeString(mPath));
        writeU32(static_cast<std::uint32_t>(mIndex.size()));

        for (const auto& [key, file] : mIndex)
        {
            writeStr(key.value());
            writeStr(file.getPath());
        }

        std::ofstream os(cacheFile, std::ios::binary | std::ios::trunc);
        if (!os || !os.write(out.data(), static_cast<std::streamsize>(out.size())))
            return;

        Log(Debug::Info) << "VFS cache saved: " << mIndex.size() << " files for " << mPath;
    }

    void FileSystemArchive::listResources(FileMap& out)
    {
        for (auto& [k, v] : mIndex)
            out[k] = &v;
    }

    bool FileSystemArchive::contains(Path::NormalizedView file) const
    {
        return mIndex.find(file) != mIndex.end();
    }

    std::string FileSystemArchive::getDescription() const
    {
        return "DIR: " + Files::pathToUnicodeString(mPath);
    }

    // ----------------------------------------------------------------------------------

    FileSystemArchiveFile::FileSystemArchiveFile(const std::filesystem::path& path)
        : mPath(Files::pathToUnicodeString(path))
    {
    }

    Files::IStreamPtr FileSystemArchiveFile::open()
    {
        return Files::openConstrainedFileStream(Files::pathFromUnicodeString(mPath));
    }

    std::filesystem::file_time_type FileSystemArchiveFile::getLastModified() const
    {
        return std::filesystem::last_write_time(Files::pathFromUnicodeString(mPath));
    }

    std::string FileSystemArchiveFile::getStem() const
    {
        return Files::pathToUnicodeString(Files::pathFromUnicodeString(mPath).stem());
    }

}
