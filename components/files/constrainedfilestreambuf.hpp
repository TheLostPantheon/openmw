#ifndef OPENMW_CONSTRAINEDFILESTREAMBUF_H
#define OPENMW_CONSTRAINEDFILESTREAMBUF_H

#include <filesystem>
#include <streambuf>

#include <components/platform/file.hpp>

namespace Files
{
    /// A file streambuf constrained to a specific region in the file, specified by the 'start' and 'length' parameters.
    class ConstrainedFileStreamBuf final : public std::streambuf
    {
    public:
        ConstrainedFileStreamBuf(const std::filesystem::path& fname, std::size_t start, std::size_t length);

        int_type underflow() final;

        pos_type seekoff(off_type offset, std::ios_base::seekdir whence, std::ios_base::openmode mode) final;

        pos_type seekpos(pos_type pos, std::ios_base::openmode mode) final;

    private:
        std::size_t mOrigin;
        std::size_t mSize;
        Platform::File::ScopedHandle mFile;
#ifdef __vita__
        // Own absolute position: handles are pooled per thread on Vita, so
        // the OS file position cannot be trusted across streams.
        std::size_t mPos = 0;
#endif
#ifdef __vita__
        // 64K: fewer sceIo calls; bigger regresses seek-heavy ESM reads.
        char mBuffer[65536]{ 0 };
#else
        char mBuffer[8192]{ 0 };
#endif
    };
}

#endif
