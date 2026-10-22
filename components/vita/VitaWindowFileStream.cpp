#include "VitaWindowFileStream.h"

#ifdef __vita__

#include <psp2/io/fcntl.h>

#include <algorithm>
#include <cstdint>
#include <streambuf>

#include <components/files/conversion.hpp>

namespace Vita
{
    namespace
    {
        // Parsing a large plugin that is not held in RAM through std::ifstream
        // took ~54 s for 71 MB: the reader asks for its position (tellg) per
        // dialogue response, cell and land record. Through the window, ~7.5 s.
        class VitaWindowFileBuf : public std::streambuf
        {
        public:
            static constexpr int64_t sWindow = 1024 * 1024;

            explicit VitaWindowFileBuf(const std::filesystem::path& path)
                : mBuf(new char[sWindow])
            {
                mFd = sceIoOpen(Files::pathToUnicodeString(path).c_str(), SCE_O_RDONLY, 0);
                if (mFd >= 0)
                    mSize = sceIoLseek(mFd, 0, SCE_SEEK_END);
                setg(mBuf.get(), mBuf.get(), mBuf.get());
            }
            ~VitaWindowFileBuf() override
            {
                if (mFd >= 0)
                    sceIoClose(mFd);
            }
            bool isOpen() const { return mFd >= 0 && mSize >= 0; }

        protected:
            int_type underflow() override
            {
                if (gptr() < egptr())
                    return traits_type::to_int_type(*gptr());
                const int64_t next = mStart + (egptr() - eback());
                if (next >= mSize || sceIoLseek(mFd, next, SCE_SEEK_SET) != next)
                    return traits_type::eof();
                const int n = sceIoRead(mFd, mBuf.get(), static_cast<SceSize>(std::min(sWindow, mSize - next)));
                if (n <= 0)
                    return traits_type::eof();
                mStart = next;
                setg(mBuf.get(), mBuf.get(), mBuf.get() + n);
                return traits_type::to_int_type(*gptr());
            }

            pos_type seekoff(off_type off, std::ios_base::seekdir dir, std::ios_base::openmode) override
            {
                const int64_t here = mStart + (gptr() - eback());
                const int64_t base = dir == std::ios_base::beg ? 0 : dir == std::ios_base::cur ? here : mSize;
                return seekTo(base + off);
            }

            pos_type seekpos(pos_type pos, std::ios_base::openmode) override { return seekTo(off_type(pos)); }

        private:
            pos_type seekTo(int64_t target)
            {
                if (target < 0 || target > mSize)
                    return pos_type(off_type(-1));
                const int64_t end = mStart + (egptr() - eback());
                if (target >= mStart && target <= end)
                    setg(eback(), eback() + (target - mStart), egptr());
                else
                {
                    // Outside the window: refill from the target on the next read.
                    mStart = target;
                    setg(mBuf.get(), mBuf.get(), mBuf.get());
                }
                return pos_type(off_type(target));
            }

            std::unique_ptr<char[]> mBuf;
            SceUID mFd = -1;
            int64_t mSize = -1;
            int64_t mStart = 0; // file offset of eback()
        };

        class VitaWindowFileStream : public std::istream
        {
        public:
            explicit VitaWindowFileStream(const std::filesystem::path& path)
                : std::istream(nullptr)
                , mBuf(path)
            {
                rdbuf(&mBuf);
                if (!mBuf.isOpen())
                    setstate(std::ios_base::failbit);
            }

        private:
            VitaWindowFileBuf mBuf;
        };
    }

    std::unique_ptr<std::istream> openWindowFileStream(const std::filesystem::path& path)
    {
        auto stream = std::make_unique<VitaWindowFileStream>(path);
        if (!stream->good())
            return nullptr;
        return stream;
    }
}

#endif // __vita__
