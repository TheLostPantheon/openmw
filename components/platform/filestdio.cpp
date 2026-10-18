#include "file.hpp"

#include <cassert>
#include <errno.h>
#include <stdexcept>
#include <string.h>
#include <string>
#include <vector>
#include <cstdint>
#include <filesystem>

#include <components/files/conversion.hpp>

namespace Platform::File
{

    static auto getNativeHandle(Handle handle)
    {
        assert(handle != Handle::Invalid);

        return reinterpret_cast<FILE*>(static_cast<intptr_t>(handle));
    }

    static int getNativeSeekType(SeekType seek)
    {
        if (seek == SeekType::Begin)
            return SEEK_SET;
        if (seek == SeekType::Current)
            return SEEK_CUR;
        if (seek == SeekType::End)
            return SEEK_END;
        return -1;
    }

#ifdef __vita__
    // Per-thread pooled handles for LARGE archives (BSAs). Every asset read
    // used to fopen the 300MB BSA, seek deep into it (FAT chain walk), read,
    // fclose — 100s of ms per NIF/texture, ~half of all cold-load time.
    // Pool: one persistent FILE* per (thread, path); close() is a no-op for
    // pooled handles. Streams keep their own position (see
    // ConstrainedFileStreamBuf), so sharing a handle across streams within
    // a thread is safe. Only files >= kPoolMinBytes are pooled, so loose
    // files keep normal open/close and no per-asset heap churn is added.
    namespace
    {
        constexpr std::uintmax_t kPoolMinBytes = 8u << 20;
        struct PoolEntry
        {
            std::string path;
            FILE* file;
        };
        thread_local std::vector<PoolEntry> tPool;

        FILE* pooledOpen(const std::filesystem::path& filename, bool& pooled)
        {
            pooled = false;
            std::error_code ec;
            const std::uintmax_t sz = std::filesystem::file_size(filename, ec);
            if (ec || sz < kPoolMinBytes)
                return nullptr;
            const std::string key = filename.string();
            for (PoolEntry& e : tPool)
                if (e.path == key)
                {
                    pooled = true;
                    return e.file;
                }
            FILE* f = fopen(filename.c_str(), "rb");
            if (f == nullptr)
                return nullptr;
            setvbuf(f, nullptr, _IONBF, 0); // stream layer buffers; avoid double buffering
            tPool.push_back({ key, f });
            pooled = true;
            return f;
        }

        bool isPooled(FILE* f)
        {
            for (const PoolEntry& e : tPool)
                if (e.file == f)
                    return true;
            return false;
        }
    }
#endif

    Handle open(const std::filesystem::path& filename)
    {
#ifdef __vita__
        bool pooled = false;
        if (FILE* pf = pooledOpen(filename, pooled))
            return static_cast<Handle>(reinterpret_cast<intptr_t>(pf));
#endif
        FILE* handle = fopen(filename.c_str(), "rb");
        if (handle == nullptr)
        {
            throw std::system_error(errno, std::generic_category(),
                std::string("Failed to open '") + Files::pathToUnicodeString(filename) + "' for reading");
        }
        return static_cast<Handle>(reinterpret_cast<intptr_t>(handle));
    }

    void close(Handle handle)
    {
        auto nativeHandle = getNativeHandle(handle);
#ifdef __vita__
        if (isPooled(nativeHandle))
            return; // lives for the thread's lifetime
#endif
        fclose(nativeHandle);
    }

    void seek(Handle handle, size_t position, SeekType type /*= SeekType::Begin*/)
    {
        const auto nativeHandle = getNativeHandle(handle);
        const auto nativeSeekType = getNativeSeekType(type);
        if (fseek(nativeHandle, position, nativeSeekType) != 0)
        {
            throw std::system_error(errno, std::generic_category(), std::string("An fseek() call failed"));
        }
    }

    size_t size(Handle handle)
    {
        auto nativeHandle = getNativeHandle(handle);

        const auto oldPos = tell(handle);
        seek(handle, 0, SeekType::End);
        const auto fileSize = tell(handle);
        seek(handle, oldPos, SeekType::Begin);

        return static_cast<size_t>(fileSize);
    }

    size_t tell(Handle handle)
    {
        auto nativeHandle = getNativeHandle(handle);

        long position = ftell(nativeHandle);
        if (position == -1)
        {
            throw std::system_error(errno, std::generic_category(), std::string("An ftell() call failed"));
        }
        return static_cast<size_t>(position);
    }

    size_t read(Handle handle, void* data, size_t size)
    {
        auto nativeHandle = getNativeHandle(handle);

        int amount = fread(data, 1, size, nativeHandle);
        if (amount == 0 && ferror(nativeHandle))
        {
            throw std::system_error(errno, std::generic_category(),
                std::string("An attempt to read ") + std::to_string(size) + " bytes failed");
        }
        return static_cast<size_t>(amount);
    }

}
