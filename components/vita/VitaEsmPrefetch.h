#ifndef COMPONENTS_VITA_ESMPREFETCH_H
#define COMPONENTS_VITA_ESMPREFETCH_H

#ifdef __vita__

#include <filesystem>
#include <istream>
#include <memory>
#include <vector>

/// ESM read/parse pipelining: a background thread reads content files
/// into RAM while the main thread parses behind a watermark.
namespace Vita::EsmPrefetch
{
    /// Start reading \a files (load order) on a background thread.
    void start(std::vector<std::filesystem::path> files);

    /// Stream over the prefetched file; reads wait for the watermark.
    /// Null if the file is not prefetched (caller falls back to disk).
    std::unique_ptr<std::istream> takeStream(const std::filesystem::path& path);

    /// Free the buffer of \a path once it is parsed and its stream closed:
    /// holding every buffer until loading ended left no room to parse a
    /// large plugin after them (Tamriel Rebuilt: ~88 MB of records).
    /// No-op while keepUntilFinish().
    void release(const std::filesystem::path& path);

    /// Ask to keep every buffer (and the readers over them) until finish():
    /// a first boot re-reads every cell after parsing (reference count,
    /// intervention markers), 10 s from RAM against 22 s from the card for
    /// vanilla.
    void setKeepUntilFinish(bool keep);

    /// The request above, honoured only if every content file was prefetched
    /// (none over the budget): otherwise the buffers go as files are parsed.
    bool keepUntilFinish();

    /// Join the reader thread. Cheap once parsing has consumed everything.
    void finish();
}

#endif // __vita__
#endif
