#include "constrainedfilestreambuf.hpp"

#include <algorithm>
#include <filesystem>
#include <limits>

namespace Files
{
    namespace File = Platform::File;

    ConstrainedFileStreamBuf::ConstrainedFileStreamBuf(
        const std::filesystem::path& fname, std::size_t start, std::size_t length)
        : mOrigin(start)
    {
        mFile = File::open(fname);
        mSize = length != std::numeric_limits<std::size_t>::max() ? length : File::size(mFile) - start;

#ifdef __vita__
        mPos = start;
#else
        if (start != 0)
            File::seek(mFile, start);
#endif

        setg(nullptr, nullptr, nullptr);
    }

    std::streambuf::int_type ConstrainedFileStreamBuf::underflow()
    {
        if (gptr() == egptr())
        {
#ifdef __vita__
            // Seek to OUR position first: the handle is shared per thread.
            File::seek(mFile, mPos);
            const std::size_t toRead = std::min((mOrigin + mSize) - mPos, sizeof(mBuffer));
            const std::size_t got = File::read(mFile, mBuffer, toRead);
            mPos += got;
#else
            const std::size_t toRead = std::min((mOrigin + mSize) - (File::tell(mFile)), sizeof(mBuffer));
            // Read in the next chunk of data, and set the read pointers on success
            // Failure will throw exception.
            const std::size_t got = File::read(mFile, mBuffer, toRead);
#endif
            setg(mBuffer, mBuffer, mBuffer + got);
        }
        if (gptr() == egptr())
            return traits_type::eof();

        return traits_type::to_int_type(*gptr());
    }

    std::streambuf::pos_type ConstrainedFileStreamBuf::seekoff(
        off_type offset, std::ios_base::seekdir whence, std::ios_base::openmode mode)
    {
        if ((mode & std::ios_base::out) || !(mode & std::ios_base::in))
            return traits_type::eof();

        // new file position, relative to mOrigin
        size_t newPos;
        switch (whence)
        {
            case std::ios_base::beg:
                newPos = offset;
                break;
            case std::ios_base::cur:
#ifdef __vita__
                newPos = (mPos - mOrigin - (egptr() - gptr())) + offset;
#else
                newPos = (File::tell(mFile) - mOrigin - (egptr() - gptr())) + offset;
#endif
                break;
            case std::ios_base::end:
                newPos = mSize + offset;
                break;
            default:
                return traits_type::eof();
        }

        if (newPos > mSize)
            return traits_type::eof();

#ifdef __vita__
        mPos = mOrigin + newPos;
#else
        File::seek(mFile, mOrigin + newPos);
#endif

        // Clear read pointers so underflow() gets called on the next read attempt.
        setg(nullptr, nullptr, nullptr);

        return newPos;
    }

    std::streambuf::pos_type ConstrainedFileStreamBuf::seekpos(pos_type pos, std::ios_base::openmode mode)
    {
        if ((mode & std::ios_base::out) || !(mode & std::ios_base::in))
            return traits_type::eof();

        if (static_cast<std::size_t>(pos) > mSize)
            return traits_type::eof();

#ifdef __vita__
        mPos = mOrigin + static_cast<std::size_t>(pos);
#else
        File::seek(mFile, mOrigin + pos);
#endif

        // Clear read pointers so underflow() gets called on the next read attempt.
        setg(nullptr, nullptr, nullptr);
        return pos;
    }
}
