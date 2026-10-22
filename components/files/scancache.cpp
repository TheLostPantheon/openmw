#include "scancache.hpp"

#include <cstdint>
#include <fstream>
#include <system_error>

#include <components/debug/debuglog.hpp>

#include "conversion.hpp"

namespace Files
{
    namespace
    {
        constexpr std::uint32_t kMagic = 0x53574D4F; // "OMWS"
        constexpr std::uint32_t kSchema = 1;

        bool writeU32(std::ostream& os, std::uint32_t v)
        {
            os.write(reinterpret_cast<const char*>(&v), sizeof(v));
            return os.good();
        }

        bool writeString(std::ostream& os, std::string_view s)
        {
            if (!writeU32(os, static_cast<std::uint32_t>(s.size())))
                return false;
            os.write(s.data(), static_cast<std::streamsize>(s.size()));
            return os.good();
        }

        bool readU32(std::istream& is, std::uint32_t& v)
        {
            is.read(reinterpret_cast<char*>(&v), sizeof(v));
            return is.good();
        }

        bool readString(std::istream& is, std::string& out, std::uint32_t maxLen = 4096)
        {
            std::uint32_t len;
            if (!readU32(is, len) || len > maxLen)
                return false;
            out.resize(len);
            is.read(out.data(), static_cast<std::streamsize>(len));
            return is.good();
        }

        // The directory list identifies the cache. Any change (add, remove, reorder)
        // invalidates: priority ordering matters for MultiDirCollection's conflict
        // resolution, so reordering would silently change results.
        bool directoryListMatches(std::istream& is, const PathContainer& directories)
        {
            std::uint32_t n;
            if (!readU32(is, n) || n != directories.size())
                return false;
            for (const auto& dir : directories)
            {
                std::string cached;
                if (!readString(is, cached))
                    return false;
                if (cached != pathToUnicodeString(dir))
                    return false;
            }
            return true;
        }
    }

    bool loadScanCache(const std::filesystem::path& cachePath,
        const PathContainer& directories,
        CollectionsMap& collections)
    {
        std::ifstream is(cachePath, std::ios::binary);
        if (!is)
            return false;

        std::uint32_t magic, schema;
        if (!readU32(is, magic) || magic != kMagic)
            return false;
        if (!readU32(is, schema) || schema != kSchema)
            return false;

        if (!directoryListMatches(is, directories))
            return false;

        std::uint32_t numExt;
        if (!readU32(is, numExt))
            return false;

        CollectionsMap loaded;
        for (std::uint32_t i = 0; i < numExt; ++i)
        {
            std::string ext;
            if (!readString(is, ext))
                return false;

            std::uint32_t numFiles;
            if (!readU32(is, numFiles))
                return false;

            MultiDirCollection::TContainer files;
            for (std::uint32_t j = 0; j < numFiles; ++j)
            {
                std::string filename, fullPath;
                if (!readString(is, filename) || !readString(is, fullPath))
                    return false;
                files.emplace(std::move(filename), std::filesystem::path(fullPath));
            }

            loaded.emplace(std::move(ext), MultiDirCollection(std::move(files)));
        }

        collections = std::move(loaded);
        Log(Debug::Info) << "Loaded scan cache: " << collections.size() << " extension(s) from " << cachePath;
        return true;
    }

    bool saveScanCache(const std::filesystem::path& cachePath,
        const PathContainer& directories,
        const CollectionsMap& collections)
    {
        std::error_code ec;
        std::filesystem::create_directories(cachePath.parent_path(), ec);

        const auto tmp = cachePath.parent_path() / (cachePath.filename().string() + ".tmp");
        {
            std::ofstream os(tmp, std::ios::binary | std::ios::trunc);
            if (!os)
                return false;

            if (!writeU32(os, kMagic) || !writeU32(os, kSchema))
                return false;

            if (!writeU32(os, static_cast<std::uint32_t>(directories.size())))
                return false;
            for (const auto& dir : directories)
                if (!writeString(os, pathToUnicodeString(dir)))
                    return false;

            if (!writeU32(os, static_cast<std::uint32_t>(collections.size())))
                return false;
            for (const auto& [ext, collection] : collections)
            {
                if (!writeString(os, ext))
                    return false;
                const auto& files = collection.getFiles();
                if (!writeU32(os, static_cast<std::uint32_t>(files.size())))
                    return false;
                for (const auto& [filename, path] : files)
                {
                    if (!writeString(os, filename))
                        return false;
                    if (!writeString(os, pathToUnicodeString(path)))
                        return false;
                }
            }
        }

#ifdef __vita__
        // No replacing rename on the Vita: a stale cache was never updated.
        std::filesystem::remove(cachePath, ec);
#endif
        std::filesystem::rename(tmp, cachePath, ec);
        if (ec)
        {
            Log(Debug::Warning) << "Failed to commit scan cache: " << ec.message();
            std::filesystem::remove(tmp, ec);
            return false;
        }

        Log(Debug::Info) << "Wrote scan cache: " << collections.size() << " extension(s) to " << cachePath;
        return true;
    }

    void clearScanCache(const std::filesystem::path& cachePath)
    {
        std::error_code ec;
        std::filesystem::remove(cachePath, ec);
        if (!ec)
            Log(Debug::Info) << "Cleared scan cache at " << cachePath;
    }
}
