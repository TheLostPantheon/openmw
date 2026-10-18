#include "openfile.hpp"
#include "conversion.hpp"

#include <cstring>
#include <fstream>
#ifdef __vita__
#include <memory>
#endif

namespace Files
{
#ifdef __vita__
    namespace
    {
        // newlib's default filebuf buffer is 1 KB, so loading a cell's refs
        // (30-60 KB, contiguous) took dozens of memory-card reads at 17-64 us
        // each: 16-144 ms per cell store on the main thread. One 64 KB buffer
        // per open stream (the ESM readers, a handful).
        class BufferedIfstream : public std::ifstream
        {
        public:
            explicit BufferedIfstream(const std::filesystem::path& path)
                : mBuf(new char[kSize])
            {
                rdbuf()->pubsetbuf(mBuf.get(), kSize); // only honoured before open
                open(path, std::ios::binary);
            }
            ~BufferedIfstream() override { close(); } // detach before the buffer goes

        private:
            static constexpr std::streamsize kSize = 64 * 1024;
            std::unique_ptr<char[]> mBuf;
        };
    }
#endif

    std::unique_ptr<std::ifstream> openBinaryInputFileStream(const std::filesystem::path& path)
    {
#ifdef __vita__
        std::unique_ptr<std::ifstream> stream = std::make_unique<BufferedIfstream>(path);
#else
        auto stream = std::make_unique<std::ifstream>(path, std::ios::binary);
#endif
        if (!stream->is_open())
            throw std::system_error(errno, std::generic_category(),
                "Failed to open '" + Files::pathToUnicodeString(path) + "' for reading");
        stream->exceptions(std::ios::badbit);
        return stream;
    }
}
