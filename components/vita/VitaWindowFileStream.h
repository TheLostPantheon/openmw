#ifndef COMPONENTS_VITA_WINDOWFILESTREAM_H
#define COMPONENTS_VITA_WINDOWFILESTREAM_H

#ifdef __vita__

#include <filesystem>
#include <istream>
#include <memory>

namespace Vita
{
    /// A content file read through a 1 MB window. Position queries and seeks
    /// inside the window are arithmetic; leaving it refills on the next read.
    /// libstdc++'s ifstream answers every tellg with an lseek and drops its
    /// buffer on every seek. Null if the file can't be opened.
    std::unique_ptr<std::istream> openWindowFileStream(const std::filesystem::path& path);
}

#endif // __vita__
#endif
