#ifndef OPENMW_COMPONENTS_RESOURCE_FILESYSTEMARCHIVE_H
#define OPENMW_COMPONENTS_RESOURCE_FILESYSTEMARCHIVE_H

#include "archive.hpp"
#include "file.hpp"

#include <filesystem>
#include <string>

namespace VFS
{

    class FileSystemArchiveFile : public File
    {
    public:
        FileSystemArchiveFile(const std::filesystem::path& path);
        explicit FileSystemArchiveFile(std::string&& path)
            : mPath(std::move(path))
        {
        }

        Files::IStreamPtr open() override;

        std::filesystem::file_time_type getLastModified() const override;

        std::string getStem() const override;

        const std::string& getPath() const { return mPath; }

    private:
        // UTF-8, not std::filesystem::path: libstdc++ keeps a path's components
        // as separate heap entries (~300 bytes for a mod's
        // data/openmw/mods/<mod>/Data Files/meshes/x/y.nif), once per loose
        // file. Tamriel_Data alone has 54k.
        std::string mPath;
    };

    class FileSystemArchive : public Archive
    {
    public:
        /// Walk `path` recursively. If `cacheFile` is set, load that instead and
        /// write it on cache miss.
        FileSystemArchive(const std::filesystem::path& path,
            const std::filesystem::path& cacheFile = {});

        void listResources(FileMap& out) override;

        bool contains(Path::NormalizedView file) const override;

        std::string getDescription() const override;

    private:
        bool loadCache(const std::filesystem::path& cacheFile);
        void saveCache(const std::filesystem::path& cacheFile) const;
        void walkDirectory();
#ifdef __vita__
        bool walkDirectoryVita();
#endif

        std::map<VFS::Path::Normalized, FileSystemArchiveFile, std::less<>> mIndex;
        std::filesystem::path mPath;
    };

}

#endif
