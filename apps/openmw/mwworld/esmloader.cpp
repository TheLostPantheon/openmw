#include "esmloader.hpp"
#include "esmstore.hpp"

#include <cstdio>
#include <fstream>

#include <components/esm/format.hpp>
#include <components/esm3/esmreader.hpp>
#include <components/esm3/readerscache.hpp>
#include <components/esm4/reader.hpp>
#include <components/files/conversion.hpp>
#include <components/files/memorystream.hpp>
#include <components/files/openfile.hpp>
#include <components/misc/strings/lower.hpp>
#include <components/resource/resourcesystem.hpp>

#include "../mwbase/environment.hpp"

#ifdef __vita__
#include "../vita/VitaInit.h"
#include <components/vita/VitaDialogueText.h>
#include <components/vita/VitaEsmPrefetch.h>
#include <components/vita/VitaWindowFileStream.h>
#endif

namespace MWWorld
{

    EsmLoader::EsmLoader(MWWorld::ESMStore& store, ESM::ReadersCache& readers, ToUTF8::Utf8Encoder* encoder,
        std::vector<int>& esmVersions)
        : mReaders(readers)
        , mStore(store)
        , mEncoder(encoder)
        , mDialogue(nullptr) // A content file containing INFO records without a DIAL record appends them to the
                             // previous file's dialogue
        , mESMVersions(esmVersions)
    {
    }

    void EsmLoader::load(const std::filesystem::path& filepath, int& index, Loading::Listener* listener)
    {

        auto stream = Files::openBinaryInputFileStream(filepath);
        const ESM::Format format = ESM::readFormat(*stream);
        stream->seekg(0);

        switch (format)
        {
            case ESM::Format::Tes3:
            {
                const ESM::ReadersCache::BusyItem reader = mReaders.get(static_cast<std::size_t>(index));
                reader->setEncoder(mEncoder);
                reader->setIndex(index);
#ifdef __vita__
                Vita::DialogueText::registerContentFile(index, filepath);
                if (auto prefetched = Vita::EsmPrefetch::takeStream(filepath))
                {
                    // Parse behind the background reader's watermark.
                    reader->open(std::move(prefetched), filepath);
                }
                else
                // Slurp the whole file: one bulk SD read, then parse from RAM
                // with zero syscalls. Buffers released after setUp (worldimp
                // clears the readers cache; contexts reopen from disk later).
                // Only for small files: the files over the prefetch budget are
                // the large ones (two ~70 MB translation plugins), and holding
                // them whole ran out of memory at boot. Those parse from disk.
                {
                    stream->seekg(0, std::ios::end);
                    const std::streamoff size = stream->tellg();
                    stream->seekg(0);
                    constexpr std::streamoff kSlurpMaxBytes = 16 * 1024 * 1024;
                    if (size > kSlurpMaxBytes)
                    {
                        char crumb[160];
                        snprintf(crumb, sizeof(crumb), "[EsmLoad] parsing %s (%d MB) from disk",
                            Files::pathToUnicodeString(filepath.filename()).c_str(),
                            static_cast<int>(size / (1024 * 1024)));
                        Vita::breadcrumb(crumb);
                        if (auto windowed = Vita::openWindowFileStream(filepath))
                            reader->open(std::move(windowed), filepath);
                        else
                            reader->open(filepath);
                    }
                    else
                    {
                        std::string buf;
                        buf.resize(static_cast<size_t>(size));
                        stream->read(buf.data(), size);
                        {
                            char crumb[160];
                            snprintf(crumb, sizeof(crumb), "[EsmLoad] slurped %s (%d MB)",
                                Files::pathToUnicodeString(filepath.filename()).c_str(),
                                static_cast<int>(size / (1024 * 1024)));
                            Vita::breadcrumb(crumb);
                        }
                        if (stream->gcount() == size)
                            reader->open(std::make_unique<Files::OwningIMemStream>(std::move(buf)), filepath);
                        else
                            reader->open(filepath); // slurp failed; parse from disk
                    }
                }
#else
                reader->open(filepath);
#endif
                reader->resolveParentFileIndices(mReaders);

                const std::vector<int>& parentIndices = reader->getParentFileIndices();
                assert(reader->getGameFiles().size() == parentIndices.size());
                for (std::size_t i = 0, n = parentIndices.size(); i < n; ++i)
                    if (parentIndices[i] == reader->getIndex())
                        throw std::runtime_error("File " + Files::pathToUnicodeString(reader->getName()) + " asks for parent file "
                + reader->getGameFiles()[i].name
                + ", but it is not available or has been loaded in the wrong order. "
                  "Please run the launcher to fix this issue.");

                mESMVersions[index] = reader->getVer();
                mStore.load(*reader, listener, mDialogue);

                if (!mMasterFileFormat.has_value()
                    && (Misc::StringUtils::ciEndsWith(reader->getName().u8string(), u8".esm")
                        || Misc::StringUtils::ciEndsWith(reader->getName().u8string(), u8".omwgame")))
                    mMasterFileFormat = reader->getFormatVersion();
                break;
            }
            case ESM::Format::Tes4:
            {
                ESM4::Reader reader(std::move(stream), filepath,
                    MWBase::Environment::get().getResourceSystem()->getVFS(),
                    mEncoder != nullptr ? &mEncoder->getStatelessEncoder() : nullptr);
                reader.setModIndex(index);
                reader.updateModIndices(mNameToIndex);
                mStore.loadESM4(reader, listener);
                break;
            }
        }
        mNameToIndex[Misc::StringUtils::lowerCase(Files::pathToUnicodeString(filepath.filename()))] = index;
#ifdef __vita__
        // Parsed: drop the file's in-RAM copy now rather than when loading
        // ends. Readers needed later (first-boot reference count) reopen
        // the file from disk.
        if (!Vita::EsmPrefetch::keepUntilFinish())
        {
            mReaders.vitaClose(static_cast<std::size_t>(index));
            Vita::EsmPrefetch::release(filepath);
        }
#endif
    }

} /* namespace MWWorld */
