#include "VitaEsmPrefetch.h"

#ifdef __vita__

#include <atomic>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <streambuf>
#include <string>
#include <thread>

#include <psp2/kernel/threadmgr.h>

extern "C" void vitaBreadcrumb(const char* msg); // apps/openmw/vita/VitaInit.cpp

namespace Vita::EsmPrefetch
{
    namespace
    {
        constexpr size_t kChunkBytes = 4 * 1024 * 1024;
        // All prefetch buffers live until loading ends, on top of the parsed
        // store (vanilla: Morrowind + Tribunal + Bloodmoon = 94 MB, heap at
        // 153 of 268 MB mid-load). A Turkish translation with two ~70 MB
        // plugins asked for ~237 MB of buffers and ran out of memory at boot.
        // Files are taken in load order while they fit; the rest parse from
        // disk. The allocation must not fail: the OOM handler exits instead
        // of letting new(std::nothrow) return null.
        constexpr size_t kBudgetBytes = 112 * 1024 * 1024;

        struct Entry
        {
            std::filesystem::path mPath;
            std::unique_ptr<char[]> mData;
            size_t mSize = 0;
            std::atomic<size_t> mAvail{ 0 };
            std::atomic<bool> mFailed{ false };
            std::atomic<bool> mTaken{ false };
        };

        std::vector<std::shared_ptr<Entry>> sEntries;
        std::thread sThread;

        void crumb(const char* fmt, const char* name, unsigned mb)
        {
            char buf[160];
            snprintf(buf, sizeof(buf), fmt, name, mb);
            vitaBreadcrumb(buf);
        }

        void readerLoop()
        {
            for (const auto& entry : sEntries)
            {
                std::ifstream in(entry->mPath, std::ios::binary);
                size_t done = 0;
                while (in && done < entry->mSize)
                {
                    const size_t want = std::min(kChunkBytes, entry->mSize - done);
                    in.read(entry->mData.get() + done, static_cast<std::streamsize>(want));
                    done += static_cast<size_t>(in.gcount());
                    entry->mAvail.store(done, std::memory_order_release);
                    if (static_cast<size_t>(in.gcount()) < want)
                        break;
                }
                if (done < entry->mSize)
                {
                    entry->mFailed.store(true, std::memory_order_release);
                    crumb("[EsmLoad] prefetch FAILED %s (%u MB)", entry->mPath.filename().string().c_str(),
                        static_cast<unsigned>(done >> 20));
                }
                else
                    crumb("[EsmLoad] prefetched %s (%u MB)", entry->mPath.filename().string().c_str(),
                        static_cast<unsigned>(entry->mSize >> 20));
            }
        }

        /// streambuf over an Entry; reads wait for the watermark, seeks don't.
        class WaitingBuf : public std::streambuf
        {
        public:
            explicit WaitingBuf(std::shared_ptr<Entry> entry)
                : mEntry(std::move(entry))
            {
                char* base = mEntry->mData.get();
                setg(base, base, base);
            }

        private:
            // Waits until \a target bytes are readable; false on read failure.
            bool waitFor(size_t target)
            {
                if (mComplete)
                    return true;
                size_t avail = mEntry->mAvail.load(std::memory_order_acquire);
                while (avail < target)
                {
                    if (mEntry->mFailed.load(std::memory_order_acquire))
                        return mEntry->mAvail.load(std::memory_order_acquire) >= target;
                    sceKernelDelayThread(200);
                    avail = mEntry->mAvail.load(std::memory_order_acquire);
                }
                // Millions of tiny reads follow; skip atomics once all bytes landed.
                if (avail >= mEntry->mSize)
                    mComplete = true;
                return true;
            }

            size_t pos() const { return static_cast<size_t>(gptr() - eback()); }

            void window(size_t newPos)
            {
                char* base = mEntry->mData.get();
                const size_t avail
                    = mComplete ? mEntry->mSize : mEntry->mAvail.load(std::memory_order_acquire);
                setg(base, base + newPos, base + std::max(newPos, avail));
            }

            int_type underflow() override
            {
                const size_t p = pos();
                if (p >= mEntry->mSize || !waitFor(p + 1))
                    return traits_type::eof();
                window(p);
                return traits_type::to_int_type(*gptr());
            }

            std::streamsize xsgetn(char* s, std::streamsize n) override
            {
                const size_t p = pos();
                const size_t want = std::min(static_cast<size_t>(n), mEntry->mSize - p);
                if (want == 0 || !waitFor(p + want))
                    return 0;
                std::memcpy(s, mEntry->mData.get() + p, want);
                window(p + want);
                return static_cast<std::streamsize>(want);
            }

            std::streamsize showmanyc() override
            {
                const size_t avail
                    = mComplete ? mEntry->mSize : mEntry->mAvail.load(std::memory_order_acquire);
                return static_cast<std::streamsize>(avail - pos());
            }

            pos_type seekoff(off_type off, std::ios_base::seekdir dir, std::ios_base::openmode) override
            {
                ptrdiff_t target = 0;
                if (dir == std::ios_base::beg)
                    target = off;
                else if (dir == std::ios_base::cur)
                    target = static_cast<ptrdiff_t>(pos()) + off;
                else
                    target = static_cast<ptrdiff_t>(mEntry->mSize) + off;
                if (target < 0 || static_cast<size_t>(target) > mEntry->mSize)
                    return pos_type(off_type(-1));
                window(static_cast<size_t>(target));
                return pos_type(target);
            }

            pos_type seekpos(pos_type sp, std::ios_base::openmode which) override
            {
                return seekoff(off_type(sp), std::ios_base::beg, which);
            }

            std::shared_ptr<Entry> mEntry;
            bool mComplete = false;
        };

        class WaitingIMemStream : public std::istream
        {
        public:
            explicit WaitingIMemStream(std::shared_ptr<Entry> entry)
                : std::istream(nullptr)
                , mBuf(std::move(entry))
            {
                rdbuf(&mBuf);
            }

        private:
            WaitingBuf mBuf;
        };
    }

    void start(std::vector<std::filesystem::path> files)
    {
        if (!sEntries.empty())
            return; // engine already kicked the reader at boot
        size_t budgetLeft = kBudgetBytes;
        for (auto& path : files)
        {
            std::error_code ec;
            const auto size = std::filesystem::file_size(path, ec);
            if (ec || size == 0)
                continue;
            if (size > budgetLeft)
            {
                crumb("[EsmLoad] %s (%u MB) over the prefetch budget: parsed from disk",
                    path.filename().string().c_str(), static_cast<unsigned>(size >> 20));
                continue;
            }
            budgetLeft -= static_cast<size_t>(size);
            auto entry = std::make_shared<Entry>();
            entry->mPath = std::move(path);
            entry->mSize = static_cast<size_t>(size);
            entry->mData.reset(new (std::nothrow) char[entry->mSize]);
            if (!entry->mData)
                continue; // fall back to disk for this file
            sEntries.push_back(std::move(entry));
        }
        if (sEntries.empty())
            return;
        char buf[96];
        snprintf(buf, sizeof(buf), "[EsmLoad] prefetch start, %u files", (unsigned)sEntries.size());
        vitaBreadcrumb(buf);
        sThread = std::thread(readerLoop);
    }

    std::unique_ptr<std::istream> takeStream(const std::filesystem::path& path)
    {
        for (const auto& entry : sEntries)
        {
            if (entry->mPath != path || entry->mTaken.load(std::memory_order_relaxed))
                continue;
            if (entry->mFailed.load(std::memory_order_acquire))
                return nullptr; // short read; parse from disk instead
            entry->mTaken.store(true, std::memory_order_relaxed);
            return std::make_unique<WaitingIMemStream>(entry);
        }
        return nullptr;
    }

    void finish()
    {
        if (sThread.joinable())
            sThread.join();
        sEntries.clear(); // streams keep their own entry alive
    }
}

#endif // __vita__
