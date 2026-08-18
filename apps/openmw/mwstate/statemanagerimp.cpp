#include "statemanagerimp.hpp"

#include <filesystem>

#include <SDL_clipboard.h>

#include <components/debug/debuglog.hpp>

#ifdef __vita__
#include "../vita/VitaInit.h"
#include "../mwrender/renderingmanager.hpp"
#include "../mwrender/animation.hpp"
#include "../mwmechanics/aipackage.hpp"
#include <components/resource/resourcesystem.hpp>
#include <malloc.h>
#define VITA_CRUMB(msg) Vita::breadcrumb(msg)
#else
#define VITA_CRUMB(msg)
#endif

#include <components/esm3/actoridconverter.hpp>
#include <components/esm3/esmreader.hpp>
#include <components/esm3/esmwriter.hpp>
#include <components/esm3/loadcell.hpp>
#include <components/esm3/loadclas.hpp>

#include <components/l10n/manager.hpp>

#include <components/loadinglistener/loadinglistener.hpp>

#include <components/files/conversion.hpp>
#include <components/misc/algorithm.hpp>
#include <components/settings/values.hpp>

#include <osg/Image>

#include <osgDB/Registry>

#include "../mwbase/dialoguemanager.hpp"
#include "../mwbase/environment.hpp"
#include "../mwbase/inputmanager.hpp"
#include "../mwbase/journal.hpp"
#include "../mwbase/luamanager.hpp"
#include "../mwbase/mechanicsmanager.hpp"
#include "../mwbase/scriptmanager.hpp"
#include "../mwbase/soundmanager.hpp"
#include "../mwbase/windowmanager.hpp"
#include "../mwbase/world.hpp"

#include "../mwworld/cellstore.hpp"
#include "../mwworld/class.hpp"
#include "../mwworld/datetimemanager.hpp"
#include "../mwworld/esmstore.hpp"
#include "../mwworld/globals.hpp"
#include "../mwworld/scene.hpp"
#include "../mwworld/worldmodel.hpp"

#include "../mwmechanics/actorutil.hpp"
#include "../mwmechanics/npcstats.hpp"

#include "../mwscript/globalscripts.hpp"

#include "quicksavemanager.hpp"

#ifdef __vita__
namespace
{
    // Seekable RAM stream in fixed chunks: no contiguous growth.
    class VitaChunkedBuf final : public std::streambuf
    {
        static constexpr std::size_t kChunk = 256 * 1024;
        std::vector<std::unique_ptr<char[]>> mChunks;
        std::size_t mPos = 0;
        std::size_t mSize = 0;

        void ensure(std::size_t pos)
        {
            while (mChunks.size() * kChunk < pos + 1)
                mChunks.emplace_back(new char[kChunk]);
        }

    protected:
        std::streamsize xsputn(const char* s, std::streamsize n) override
        {
            std::size_t left = static_cast<std::size_t>(n);
            while (left > 0)
            {
                ensure(mPos);
                const std::size_t inChunk = mPos % kChunk;
                const std::size_t take = std::min(kChunk - inChunk, left);
                std::memcpy(mChunks[mPos / kChunk].get() + inChunk, s, take);
                mPos += take;
                s += take;
                left -= take;
            }
            mSize = std::max(mSize, mPos);
            return n;
        }

        int_type overflow(int_type c) override
        {
            if (traits_type::eq_int_type(c, traits_type::eof()))
                return traits_type::not_eof(c);
            const char ch = traits_type::to_char_type(c);
            xsputn(&ch, 1);
            return c;
        }

        pos_type seekoff(off_type off, std::ios_base::seekdir dir, std::ios_base::openmode which) override
        {
            if (!(which & std::ios_base::out))
                return pos_type(off_type(-1));
            const off_type base = dir == std::ios_base::beg ? 0
                : dir == std::ios_base::cur               ? static_cast<off_type>(mPos)
                                                          : static_cast<off_type>(mSize);
            const off_type target = base + off;
            if (target < 0 || static_cast<std::size_t>(target) > mSize)
                return pos_type(off_type(-1));
            mPos = static_cast<std::size_t>(target);
            return pos_type(static_cast<off_type>(mPos));
        }

        pos_type seekpos(pos_type pos, std::ios_base::openmode which) override
        {
            return seekoff(static_cast<off_type>(pos), std::ios_base::beg, which);
        }

    public:
        bool writeTo(std::ostream& out) const
        {
            std::size_t left = mSize;
            for (const auto& chunk : mChunks)
            {
                const std::size_t take = std::min(left, kChunk);
                out.write(chunk.get(), static_cast<std::streamsize>(take));
                left -= take;
                if (left == 0)
                    break;
            }
            return !out.fail();
        }
    };
}
#endif

void MWState::StateManager::cleanup(bool force)
{
    if (mState != State_NoGame || force)
    {
        MWBase::Environment::get().getSoundManager()->clear();
        MWBase::Environment::get().getDialogueManager()->clear();
        MWBase::Environment::get().getJournal()->clear();
        MWBase::Environment::get().getScriptManager()->clear();
        MWBase::Environment::get().getWindowManager()->clear();
        MWBase::Environment::get().getWorld()->clear();
        MWBase::Environment::get().getInputManager()->clear();
        MWBase::Environment::get().getMechanicsManager()->clear();

#ifdef __vita__
        // Vita: free cached resources immediately (save loader needs the RAM).
        MWBase::Environment::get().getWorld()->getRenderingManager()->flushUnrefQueueImmediate();
        MWBase::Environment::get().getResourceSystem()->clearCache();
        Vita::logMemoryStatus("Post-cleanup");
#endif

        mCharacterManager.setCurrentCharacter(nullptr);
        mTimePlayed = 0;
        mLastSavegame.clear();

        mState = State_NoGame;
        MWBase::Environment::get().getLuaManager()->noGame();
    }
    else
    {
        // TODO: do we need this cleanup?
        MWBase::Environment::get().getLuaManager()->clear();
    }
}

std::map<int, int> MWState::StateManager::buildContentFileIndexMap(const ESM::ESMReader& reader) const
{
    const std::vector<std::string>& current = MWBase::Environment::get().getWorld()->getContentFiles();

    const std::vector<ESM::Header::MasterData>& prev = reader.getGameFiles();

    std::map<int, int> map;

    for (int iPrev = 0; iPrev < static_cast<int>(prev.size()); ++iPrev)
    {
        for (int iCurrent = 0; iCurrent < static_cast<int>(current.size()); ++iCurrent)
            if (Misc::StringUtils::ciEqual(prev[iPrev].name, current[iCurrent]))
            {
                map.insert(std::make_pair(iPrev, iCurrent));
                break;
            }
    }

    return map;
}

MWState::StateManager::StateManager(const std::filesystem::path& saves, const std::vector<std::string>& contentFiles)
    : mQuitRequest(false)
    , mAskLoadRecent(false)
    , mState(State_NoGame)
    , mCharacterManager(saves, contentFiles)
    , mTimePlayed(0)
{
}

void MWState::StateManager::requestQuit()
{
    mQuitRequest = true;
}

bool MWState::StateManager::hasQuitRequest() const
{
    return mQuitRequest;
}

void MWState::StateManager::askLoadRecent()
{
    if (MWBase::Environment::get().getWindowManager()->getMode() == MWGui::GM_MainMenu)
        return;

    if (!mAskLoadRecent)
    {
        if (mLastSavegame.empty()) // no saves
        {
            MWBase::Environment::get().getWindowManager()->pushGuiMode(MWGui::GM_MainMenu);
        }
        else
        {
            std::string saveName = Files::pathToUnicodeString(mLastSavegame.filename());
            // Assume the last saved game belongs to the current character's slot list.
            const Character* character = getCurrentCharacter();
            if (character)
            {
                for (const auto& slot : *character)
                {
                    if (slot.mPath == mLastSavegame)
                    {
                        saveName = slot.mProfile.mDescription;
                        break;
                    }
                }
            }

            std::vector<std::string> buttons;
            buttons.emplace_back("#{Interface:Yes}");
            buttons.emplace_back("#{Interface:No}");
            auto l10n = MWBase::Environment::get().getL10nManager()->getContext("OMWEngine");
            std::string message = l10n->formatMessage("AskLoadLastSave", { "save" }, { L10n::toUnicode(saveName) });
            MWBase::Environment::get().getWindowManager()->interactiveMessageBox(message, buttons);
            mAskLoadRecent = true;
        }
    }
}

MWState::StateManager::State MWState::StateManager::getState() const
{
    return mState;
}

void MWState::StateManager::newGame(bool bypass)
{
    VITA_CRUMB("newGame() enter");
    cleanup();

    if (!bypass)
        MWBase::Environment::get().getWindowManager()->setNewGame(true);

    try
    {
        Log(Debug::Info) << "Starting a new game";
        VITA_CRUMB("newGame() addStartup");
        MWBase::Environment::get().getScriptManager()->getGlobalScripts().addStartup();
        VITA_CRUMB("newGame() startNewGame");
        MWBase::Environment::get().getWorld()->startNewGame(bypass);
        VITA_CRUMB("newGame() startNewGame done");

        mState = State_Running;
        MWBase::Environment::get().getLuaManager()->gameLoaded();

        MWBase::Environment::get().getWindowManager()->fadeScreenOut(0);
        MWBase::Environment::get().getWindowManager()->fadeScreenIn(1);
        VITA_CRUMB("newGame() done");
    }
    catch (std::exception& e)
    {
        std::stringstream error;
        error << "Failed to start new game: " << e.what();

        Log(Debug::Error) << error.str();
        cleanup(true);

        MWBase::Environment::get().getWindowManager()->pushGuiMode(MWGui::GM_MainMenu);

        std::vector<std::string> buttons;
        buttons.emplace_back("#{Interface:OK}");
        MWBase::Environment::get().getWindowManager()->interactiveMessageBox(error.str(), buttons);
    }
}

void MWState::StateManager::endGame()
{
    mState = State_Ended;
    MWBase::Environment::get().getLuaManager()->gameEnded();
}

void MWState::StateManager::resumeGame()
{
    mState = State_Running;
    MWBase::Environment::get().getLuaManager()->gameLoaded();
}

#ifdef __vita__
// Save marker for [Slow] attribution.
extern "C" uint32_t vitastat_save_flag;
uint32_t vitastat_save_flag = 0;
#endif

void MWState::StateManager::saveGame(std::string_view description, const Slot* slot)
{
    MWBase::Environment::get().getLuaManager()->applyDelayedActions();
#ifdef __vita__
    vitastat_save_flag = 1;
#endif

#ifdef __vita__
    // libpng's internal mallocs fail under heap pressure during save thumbnail
    // and fog-of-war PNG writes; OSG's writer doesn't NULL-check png_ptr.
    // Force-flush caches AND the unref queue so libpng has room to allocate.
    if (auto* rm = MWBase::Environment::get().getWorld()->getRenderingManager())
        rm->flushUnrefQueueImmediate();
    MWBase::Environment::get().getResourceSystem()->clearCache();
    MWMechanics::AiPackage::clearPathgridCache();
    MWRender::clearAnimationModelCache();
    if (auto* rm = MWBase::Environment::get().getWorld()->getRenderingManager())
        rm->flushUnrefQueueImmediate();
#endif

    MWState::Character* character = getCurrentCharacter();

    try
    {
        const auto start = std::chrono::steady_clock::now();

        MWBase::Environment::get().getWindowManager()->asyncPrepareSaveMap();

        if (!character)
        {
            MWWorld::ConstPtr player = MWMechanics::getPlayer();
            const std::string& name = player.get<ESM::NPC>()->mBase->mName;

            character = mCharacterManager.createCharacter(name);
            mCharacterManager.setCurrentCharacter(character);
        }

        ESM::SavedGame profile;

        MWBase::World& world = *MWBase::Environment::get().getWorld();

        MWWorld::Ptr player = world.getPlayerPtr();

        profile.mContentFiles = world.getContentFiles();

        profile.mPlayerName = player.get<ESM::NPC>()->mBase->mName;
        profile.mPlayerLevel = player.getClass().getNpcStats(player).getLevel();

        const ESM::RefId& classId = player.get<ESM::NPC>()->mBase->mClass;
        if (world.getStore().get<ESM::Class>().isDynamic(classId))
            profile.mPlayerClassName = world.getStore().get<ESM::Class>().find(classId)->mName;
        else
            profile.mPlayerClassId = classId;

        const MWMechanics::CreatureStats& stats = player.getClass().getCreatureStats(player);

        profile.mPlayerCellName = world.getCellName();
        profile.mInGameTime = world.getTimeManager()->getEpochTimeStamp();
        profile.mTimePlayed = mTimePlayed;
        profile.mDescription = description;
        profile.mCurrentDay = world.getTimeManager()->getTimeStamp().getDay();
        profile.mCurrentHealth = stats.getHealth().getCurrent();
        profile.mMaximumHealth = stats.getHealth().getModified();

        Log(Debug::Info) << "Making a screenshot for saved game '" << description << "'";
        writeScreenshot(profile.mScreenshot);

        if (!slot)
            slot = character->createSlot(profile);
        else
            slot = character->updateSlot(slot, profile);

        // Make sure the animation state held by references is up to date before saving the game.
        MWBase::Environment::get().getMechanicsManager()->persistAnimationStates();

        Log(Debug::Info) << "Writing saved game '" << description << "' for character '" << profile.mPlayerName << "'";

        // Write to a memory stream first. If there is an exception during the save process, we don't want to trash the
        // existing save file we are overwriting.
#ifdef __vita__
        // Write tmp, rename on success; old save never trashed.
        std::filesystem::path vitaTmpPath = slot->mPath;
        vitaTmpPath += ".tmp";
        struct VitaTmpGuard
        {
            const std::filesystem::path* mPath{};
            bool mKeep = false;
            ~VitaTmpGuard()
            {
                if (mPath && !mKeep)
                {
                    std::error_code ec;
                    std::filesystem::remove(*mPath, ec);
                }
            }
        } vitaTmpGuard{ &vitaTmpPath };
        // RAM-backed for ESMWriter's seeks; chunked because a growing
        // stringstream doubles contiguously (sv_splice fragmentation OOM).
        VitaChunkedBuf vitaSaveBuf;
        std::ostream stream(&vitaSaveBuf);
        vitaMainPhase("savebody");
#else
        std::stringstream stream;
#endif

        ESM::ESMWriter writer;

        for (const std::string& contentFile : MWBase::Environment::get().getWorld()->getContentFiles())
            writer.addMaster(contentFile, 0); // not using the size information anyway -> use value of 0

        writer.setFormatVersion(ESM::CurrentSaveGameFormatVersion);

        // all unused
        writer.setVersion(0);
        writer.setType(0);
        writer.setAuthor("");
        writer.setDescription("");

        size_t recordCount = 1 // saved game header
            + MWBase::Environment::get().getJournal()->countSavedGameRecords()
            + MWBase::Environment::get().getLuaManager()->countSavedGameRecords()
            + MWBase::Environment::get().getWorld()->countSavedGameRecords()
            + MWBase::Environment::get().getScriptManager()->getGlobalScripts().countSavedGameRecords()
            + MWBase::Environment::get().getDialogueManager()->countSavedGameRecords()
            + MWBase::Environment::get().getMechanicsManager()->countSavedGameRecords()
            + MWBase::Environment::get().getInputManager()->countSavedGameRecords()
            + MWBase::Environment::get().getWindowManager()->countSavedGameRecords();
        writer.setRecordCount(static_cast<int>(recordCount));

        writer.save(stream);

        Loading::Listener& listener = *MWBase::Environment::get().getWindowManager()->getLoadingScreen();
        // Using only Cells for progress information, since they typically have the largest records by far
        listener.setProgressRange(MWBase::Environment::get().getWorld()->countSavedGameCells());
        listener.setLabel("#{OMWEngine:SavingInProgress}", true);

        Loading::ScopedLoad load(&listener);

#ifdef __vita__
        // Trim under screen; screenshot must capture the game, not it.
        {
            vitaMainPhase("savetrim");
            const int stBefore = Vita::getHeapUsedMBFresh();
            MWBase::Environment::get().getResourceSystem()->clearCache();
            malloc_trim(0);
            char stb[80];
            snprintf(stb, sizeof(stb), "[SaveTrim] %dMB->%dMB", stBefore, Vita::getHeapUsedMBFresh());
            Vita::breadcrumb(stb);
            vitaMainPhase("savebody");
        }
#endif

        writer.startRecord(ESM::REC_SAVE);
        slot->mProfile.save(writer);
        writer.endRecord(ESM::REC_SAVE);

        vitaMainPhase("sv_journal");
        MWBase::Environment::get().getJournal()->write(writer, listener);
        vitaMainPhase("sv_dialog");
        MWBase::Environment::get().getDialogueManager()->write(writer, listener);
        // LuaManager::write should be called before World::write because world also saves
        // local scripts that depend on LuaManager.
        vitaMainPhase("sv_lua");
        MWBase::Environment::get().getLuaManager()->write(writer, listener);
        vitaMainPhase("sv_world");
        MWBase::Environment::get().getWorld()->write(writer, listener);
        vitaMainPhase("sv_scripts");
        MWBase::Environment::get().getScriptManager()->getGlobalScripts().write(writer, listener);
        vitaMainPhase("sv_mech");
        MWBase::Environment::get().getMechanicsManager()->write(writer, listener);
        vitaMainPhase("sv_input");
        MWBase::Environment::get().getInputManager()->write(writer, listener);
        vitaMainPhase("sv_wm");
        MWBase::Environment::get().getWindowManager()->write(writer, listener);

        // Ensure we have written the number of records that was estimated
        if (static_cast<size_t>(writer.getRecordCount()) != recordCount + 1) // 1 extra for TES3 record
            Log(Debug::Warning) << "Warning: number of written savegame records does not match. Estimated: "
                                << recordCount + 1 << ", written: " << writer.getRecordCount();

        vitaMainPhase("sv_close");
        writer.close();

        if (stream.fail())
            throw std::runtime_error(
                "Write operation failed (memory stream): " + std::generic_category().message(errno));

#ifdef __vita__
        vitaMainPhase("savefile");
        {
            std::ofstream tmpOut(vitaTmpPath, std::ios::binary);
            if (!vitaSaveBuf.writeTo(tmpOut))
                throw std::runtime_error(
                    "Write operation failed (temp save): " + std::generic_category().message(errno));
        }
        // Tmp is authoritative once written; keep even if rename fails.
        vitaTmpGuard.mKeep = true;
        std::error_code vitaRmEc;
        std::filesystem::remove(slot->mPath, vitaRmEc); // FAT: no atomic replace
        std::filesystem::rename(vitaTmpPath, slot->mPath);
#else
        // All good, write to file
        std::ofstream filestream(slot->mPath, std::ios::binary);
        filestream << stream.rdbuf();

        if (filestream.fail())
            throw std::runtime_error("Write operation failed (file stream): " + std::generic_category().message(errno));
#endif

        Settings::saves().mCharacter.set(Files::pathToUnicodeString(slot->mPath.parent_path().filename()));
        mLastSavegame = slot->mPath;

        const auto finish = std::chrono::steady_clock::now();

        Log(Debug::Info) << '\'' << description << "' is saved in "
                         << std::chrono::duration_cast<std::chrono::duration<float, std::milli>>(finish - start).count()
                         << "ms";
    }
    catch (const std::exception& e)
    {
        std::stringstream error;
        error << "Failed to save game: " << e.what();

        Log(Debug::Error) << error.str();

        std::vector<std::string> buttons;
        buttons.emplace_back("#{Interface:OK}");
        MWBase::Environment::get().getWindowManager()->interactiveMessageBox(error.str(), buttons);

        // If no file was written, clean up the slot
        if (character && slot && !std::filesystem::exists(slot->mPath))
        {
            character->deleteSlot(slot);
            character->cleanup();
        }
    }
}

void MWState::StateManager::quickSave(std::string name)
{
    if (!(mState == State_Running
            && MWBase::Environment::get().getWorld()->getGlobalInt(MWWorld::Globals::sCharGenState) == -1 // char gen
            && MWBase::Environment::get().getWindowManager()->isSavingAllowed()))
    {
        // You can not save your game right now
        MWBase::Environment::get().getWindowManager()->messageBox("#{OMWEngine:SaveGameDenied}");
        return;
    }

    Character* currentCharacter = getCurrentCharacter(); // Get current character
    QuickSaveManager saveFinder(name, Settings::saves().mMaxQuicksaves);

    if (currentCharacter)
    {
        for (auto& save : *currentCharacter)
        {
            // Visiting slots allows the quicksave finder to find the oldest quicksave
            saveFinder.visitSave(&save);
        }
    }

    // Once all the saves have been visited, the save finder can tell us which
    // one to replace (or create)
    saveGame(name, saveFinder.getNextQuickSaveSlot());
}

#ifdef __vita__
// Same record set saveGame writes, straight into RAM: no profile screenshot,
// no slot, no file. Mirrors the block in saveGame — keep them in step.
void MWState::StateManager::saveGameToMemory(std::string& out)
{
    ESM::SavedGame profile;
    MWBase::World& world = *MWBase::Environment::get().getWorld();
    for (const std::string& contentFile : world.getContentFiles())
        profile.mContentFiles.push_back(contentFile);
    profile.mPlayerName = world.getPlayerPtr().get<ESM::NPC>()->mBase->mName;
    profile.mInGameTime = world.getTimeManager()->getEpochTimeStamp();
    profile.mTimePlayed = mTimePlayed;
    profile.mDescription = "streaming mode switch";

    MWBase::Environment::get().getMechanicsManager()->persistAnimationStates();

    std::stringstream stream;
    ESM::ESMWriter writer;
    for (const std::string& contentFile : world.getContentFiles())
        writer.addMaster(contentFile, 0);
    writer.setFormatVersion(ESM::CurrentSaveGameFormatVersion);
    writer.setVersion(0);
    writer.setType(0);
    writer.setAuthor("");
    writer.setDescription("");

    size_t recordCount = 1
        + MWBase::Environment::get().getJournal()->countSavedGameRecords()
        + MWBase::Environment::get().getLuaManager()->countSavedGameRecords()
        + MWBase::Environment::get().getWorld()->countSavedGameRecords()
        + MWBase::Environment::get().getScriptManager()->getGlobalScripts().countSavedGameRecords()
        + MWBase::Environment::get().getDialogueManager()->countSavedGameRecords()
        + MWBase::Environment::get().getMechanicsManager()->countSavedGameRecords()
        + MWBase::Environment::get().getInputManager()->countSavedGameRecords()
        + MWBase::Environment::get().getWindowManager()->countSavedGameRecords();
    writer.setRecordCount(static_cast<int>(recordCount));
    writer.save(stream);

    Loading::Listener& listener = *MWBase::Environment::get().getWindowManager()->getLoadingScreen();
    listener.setProgressRange(MWBase::Environment::get().getWorld()->countSavedGameCells());
    listener.setLabel("#{OMWEngine:SavingInProgress}", true);
    Loading::ScopedLoad load(&listener);

    writer.startRecord(ESM::REC_SAVE);
    profile.save(writer);
    writer.endRecord(ESM::REC_SAVE);

    MWBase::Environment::get().getJournal()->write(writer, listener);
    MWBase::Environment::get().getDialogueManager()->write(writer, listener);
    MWBase::Environment::get().getLuaManager()->write(writer, listener);
    MWBase::Environment::get().getWorld()->write(writer, listener);
    MWBase::Environment::get().getScriptManager()->getGlobalScripts().write(writer, listener);
    MWBase::Environment::get().getMechanicsManager()->write(writer, listener);
    MWBase::Environment::get().getInputManager()->write(writer, listener);
    MWBase::Environment::get().getWindowManager()->write(writer, listener);

    if (static_cast<size_t>(writer.getRecordCount()) != recordCount + 1)
        Log(Debug::Warning) << "mode switch: record count mismatch, estimated " << recordCount + 1 << " wrote "
                            << writer.getRecordCount();
    writer.close();
    if (stream.fail())
        throw std::runtime_error("mode switch: memory save failed");
    out = stream.str();
}
#endif

void MWState::StateManager::loadGame(const std::filesystem::path& filepath)
{
    for (const auto& character : mCharacterManager)
    {
        for (const auto& slot : character)
        {
            if (std::filesystem::equivalent(slot.mPath, filepath))
            {
                loadGame(&character, slot.mPath);
                return;
            }
        }
    }

    MWState::Character* character = getCurrentCharacter();
    loadGame(character, filepath);
}

struct SaveFormatVersionError : public std::exception
{
    using std::exception::exception;

    SaveFormatVersionError(ESM::FormatVersion savegameFormat, const std::string& message)
        : mSavegameFormat(savegameFormat)
        , mErrorMessage(message)
    {
    }

    const char* what() const noexcept override { return mErrorMessage.c_str(); }
    ESM::FormatVersion getFormatVersion() const { return mSavegameFormat; }

protected:
    ESM::FormatVersion mSavegameFormat = ESM::DefaultFormatVersion;
    std::string mErrorMessage;
};

struct SaveVersionTooOldError : SaveFormatVersionError
{
    SaveVersionTooOldError(ESM::FormatVersion savegameFormat)
        : SaveFormatVersionError(savegameFormat, "format version " + std::to_string(savegameFormat) + " is too old")
    {
    }
};

struct SaveVersionTooNewError : SaveFormatVersionError
{
    SaveVersionTooNewError(ESM::FormatVersion savegameFormat)
        : SaveFormatVersionError(savegameFormat, "format version " + std::to_string(savegameFormat) + " is too new")
    {
    }
};

void MWState::StateManager::loadGame(const Character* character, const std::filesystem::path& filepath)
{
    ESM::ESMReader reader;
    reader.open(filepath);
    loadGameFromReader(character, filepath, reader);
}

#ifdef __vita__
// Switching the streaming architecture rebuilds the world through the save
// path, but must not touch the player's saves: serialise to RAM, reload from
// RAM, release. No slot, no file, no quicksave rotation.
void MWState::StateManager::loadGameFromMemory(std::string&& data)
{
    ESM::ESMReader reader;
    reader.open(std::make_unique<std::istringstream>(std::move(data)), "memory:modeswitch");
    loadGameFromReader(getCurrentCharacter(), mLastSavegame, reader, /*fileBacked*/ false);
}
#endif

void MWState::StateManager::loadGameFromReader(
    const Character* character, const std::filesystem::path& filepath, ESM::ESMReader& reader, bool fileBacked)
{
    try
    {
        cleanup();

        Log(Debug::Info) << "Reading save file " << filepath.filename();


        ESM::FormatVersion version = reader.getFormatVersion();
        if (version > ESM::CurrentSaveGameFormatVersion)
            throw SaveVersionTooNewError(version);
        else if (version < ESM::MinSupportedSaveGameFormatVersion)
            throw SaveVersionTooOldError(version);

        std::map<int, int> contentFileMap = buildContentFileIndexMap(reader);
        reader.setContentFileMapping(&contentFileMap);
        MWBase::Environment::get().getLuaManager()->setContentFileMapping(contentFileMap);

        ESM::ActorIdConverter actorIdConverter;
        if (version <= ESM::MaxActorIdSaveGameFormatVersion)
            reader.mActorIdConverter = &actorIdConverter;

        Loading::Listener& listener = *MWBase::Environment::get().getWindowManager()->getLoadingScreen();

        listener.setProgressRange(100);
        listener.setLabel("#{OMWEngine:LoadingInProgress}");

        Loading::ScopedLoad load(&listener);

#ifdef __vita__
        // Under the screen: the purge can take seconds.
        MWBase::Environment::get().getWorldScene()->vitaLoadPurge();
        Vita::logMemoryStatus("Pre-save-load");
#endif

        bool firstPersonCam = false;

        size_t total = reader.getFileSize();
        int currentPercent = 0;
#ifdef __vita__
        // Peak one store, not one per visited cell.
        MWBase::Environment::get().getWorldModel()->vitaSetLoadDemote(true);
        // Old formats must parse (migrate); spans splice verbatim.
        if (fileBacked && reader.getFormatVersion() == ESM::CurrentSaveGameFormatVersion)
            MWBase::Environment::get().getWorldModel()->vitaBeginFileLoad(filepath.string());
#endif
#ifdef __vita__
        // Per-record-type RAM census; drop ux0:data/openmw/loadcensus.txt
        // to enable (mallinfo per record slows the load).
        const bool vitaCensus = [] {
            if (FILE* f = fopen("ux0:data/openmw/loadcensus.txt", "rb"))
            {
                fclose(f);
                return true;
            }
            return false;
        }();
        std::map<uint32_t, std::pair<long long, int>> vitaTypeBytes;
#endif
        while (reader.hasMoreRecs())
        {
#ifdef __vita__
            vitaMainPhase("loadrec"); // heartbeat: big saves parse for minutes
            MWBase::Environment::get().getWorldModel()->vitaNoteRecordStart((std::size_t)reader.getFileOffset());
            long long vitaRecBefore = 0;
            if (vitaCensus)
                vitaRecBefore = (long long)mallinfo().uordblks;
#endif
            ESM::NAME n = reader.getRecName();
            reader.getRecHeader();

            switch (n.toInt())
            {
                case ESM::REC_SAVE:
                {
                    ESM::SavedGame profile;
                    profile.load(reader);
                    const auto& selectedContentFiles = MWBase::Environment::get().getWorld()->getContentFiles();
                    auto missingFiles = profile.getMissingContentFiles(selectedContentFiles);
                    if (!missingFiles.empty() && !confirmLoading(missingFiles))
                    {
                        cleanup(true);
                        MWBase::Environment::get().getWindowManager()->pushGuiMode(MWGui::GM_MainMenu);
                        return;
                    }
                    mTimePlayed = profile.mTimePlayed;
                    Log(Debug::Info) << "Loading saved game '" << profile.mDescription << "' for character '"
                                     << profile.mPlayerName << "'";
                }
                break;

                case ESM::REC_JOUR:
                case ESM::REC_QUES:

                    MWBase::Environment::get().getJournal()->readRecord(reader, n.toInt());
                    break;

                case ESM::REC_DIAS:

                    MWBase::Environment::get().getDialogueManager()->readRecord(reader, n.toInt());
                    break;

                case ESM::REC_ALCH:
                case ESM::REC_MISC:
                case ESM::REC_ACTI:
                case ESM::REC_ARMO:
                case ESM::REC_BOOK:
                case ESM::REC_CLAS:
                case ESM::REC_CLOT:
                case ESM::REC_ENCH:
                case ESM::REC_NPC_:
                case ESM::REC_SPEL:
                case ESM::REC_WEAP:
                case ESM::REC_GLOB:
                case ESM::REC_PLAY:
                case ESM::REC_CSTA:
                case ESM::REC_WTHR:
                case ESM::REC_DYNA:
                case ESM::REC_ACTC:
                case ESM::REC_PROJ:
                case ESM::REC_MPRJ:
                case ESM::REC_ENAB:
                case ESM::REC_LEVC:
                case ESM::REC_LEVI:
                case ESM::REC_LIGH:
                case ESM::REC_CREA:
                case ESM::REC_CONT:
                case ESM::REC_RAND:
                    MWBase::Environment::get().getWorld()->readRecord(reader, n.toInt());
                    break;

                case ESM::REC_CAM_:
                    reader.getHNT(firstPersonCam, "FIRS");
                    break;

                case ESM::REC_GSCR:

                    MWBase::Environment::get().getScriptManager()->getGlobalScripts().readRecord(reader, n.toInt());
                    break;

                case ESM::REC_GMAP:
                case ESM::REC_KEYS:
                case ESM::REC_ASPL:
                case ESM::REC_MARK:

                    MWBase::Environment::get().getWindowManager()->readRecord(reader, n.toInt());
                    break;

                case ESM::REC_DCOU:
                case ESM::REC_STLN:

                    MWBase::Environment::get().getMechanicsManager()->readRecord(reader, n.toInt());
                    break;

                case ESM::REC_INPU:
                    MWBase::Environment::get().getInputManager()->readRecord(reader, n.toInt());
                    break;

                case ESM::REC_LUAM:
                    MWBase::Environment::get().getLuaManager()->readRecord(reader, n.toInt());
                    break;

                default:

                    // ignore invalid records
                    Log(Debug::Warning) << "Warning: Ignoring unknown record: " << n.toStringView();
                    reader.skipRecord();
            }
#ifdef __vita__
            if (vitaCensus)
            {
                auto& [bytes, count] = vitaTypeBytes[n.toInt()];
                bytes += (long long)mallinfo().uordblks - vitaRecBefore;
                ++count;
            }
#endif
            int progressPercent = static_cast<int>(float(reader.getFileOffset()) / total * 100);
            if (progressPercent > currentPercent)
            {
                listener.increaseProgress(progressPercent - currentPercent);
                currentPercent = progressPercent;
#ifdef __vita__
                // Mid-load spike valve; mirrors midload housekeep.
                if (Vita::getHeapUsedMBFresh() > 190)
                {
                    vitaMainPhase("loadtrim");
                    MWBase::Environment::get().getResourceSystem()->clearCache();
                    malloc_trim(0);
                    char lb[64];
                    snprintf(lb, sizeof(lb), "[LoadTrim] %d%% heap=%dMB", currentPercent,
                        Vita::getHeapUsedMBFresh());
                    Vita::breadcrumb(lb);
                }
#endif
            }
        }
        mCharacterManager.setCurrentCharacter(character);

#ifdef __vita__
        if (vitaCensus)
        {
            std::vector<std::pair<long long, uint32_t>> byBytes;
            for (const auto& [type, bc] : vitaTypeBytes)
                byBytes.push_back({ bc.first, type });
            std::sort(byBytes.rbegin(), byBytes.rend());
            for (std::size_t i = 0; i < byBytes.size() && i < 10; ++i)
            {
                const auto& [bytes, type] = byBytes[i];
                const auto& bc = vitaTypeBytes[type];
                const char* tc = reinterpret_cast<const char*>(&type);
                char cb[96];
                snprintf(cb, sizeof(cb), "[LoadCensus] %.4s n=%d %lldKB", tc, bc.second, bytes / 1024);
                Vita::breadcrumb(cb);
            }
        }
#endif

#ifdef __vita__
        MWBase::Environment::get().getWorldModel()->vitaSetLoadDemote(false);
        MWBase::Environment::get().getWorldModel()->vitaEndFileLoad();
        // Re-queue the warm pool vitaLoadPurge dropped for load headroom.
        MWBase::Environment::get().getWorldScene()->vitaLoadRefill();
#endif
        mState = State_Running;
#ifdef __vita__
        MWBase::Environment::get().getInputManager()->vitaShowTouchIntro();
#endif

        if (character)
            Settings::saves().mCharacter.set(Files::pathToUnicodeString(character->getPath().filename()));
        mLastSavegame = filepath;

        MWBase::Environment::get().getWindowManager()->setNewGame(false);
        MWBase::Environment::get().getWorld()->saveLoaded(reader);
        actorIdConverter.apply();
        MWBase::Environment::get().getWorld()->setupPlayer();
        MWBase::Environment::get().getWorld()->renderPlayer();
        MWBase::Environment::get().getWindowManager()->updatePlayer();
        MWBase::Environment::get().getMechanicsManager()->playerLoaded();
        MWBase::Environment::get().getWorld()->toggleVanityMode(false);

        if (firstPersonCam != MWBase::Environment::get().getWorld()->isFirstPerson())
            MWBase::Environment::get().getWorld()->togglePOV();

        MWWorld::ConstPtr ptr = MWMechanics::getPlayer();

        if (ptr.isInCell())
        {
            const ESM::RefId cellId = ptr.getCell()->getCell()->getId();

            // Use detectWorldSpaceChange=false, otherwise some of the data we just loaded would be cleared again
            MWBase::Environment::get().getWorld()->changeToCell(cellId, ptr.getRefData().getPosition(), false, false);
        }
        else
        {
            // Cell no longer exists (i.e. changed game files), choose a default cell
            Log(Debug::Warning) << "Player character's cell no longer exists, changing to the default cell";
            ESM::ExteriorCellLocation cellIndex(0, 0, ESM::Cell::sDefaultWorldspaceId);
            MWWorld::CellStore& cell = MWBase::Environment::get().getWorldModel()->getExterior(cellIndex);
            const osg::Vec2f posFromIndex = ESM::indexToPosition(cellIndex, false);
            ESM::Position pos;
            pos.pos[0] = posFromIndex.x();
            pos.pos[1] = posFromIndex.y();
            pos.pos[2] = 0; // should be adjusted automatically (adjustPlayerPos=true)
            pos.rot[0] = 0;
            pos.rot[1] = 0;
            pos.rot[2] = 0;
            MWBase::Environment::get().getWorld()->changeToCell(cell.getCell()->getId(), pos, true, false);
        }

        MWBase::Environment::get().getWorld()->updateProjectilesCasters();

        // Vanilla MW will restart startup scripts when a save game is loaded. This is unintuitive,
        // but some mods may be using it as a reload detector.
        MWBase::Environment::get().getScriptManager()->getGlobalScripts().addStartup();

        // Since we passed "changeEvent=false" to changeCell, we shouldn't have triggered the cell change flag.
        // But make sure the flag is cleared anyway in case it was set from an earlier game.
        MWBase::Environment::get().getWorldScene()->markCellAsUnchanged();

        MWBase::Environment::get().getLuaManager()->gameLoaded();
        for (int actorId : actorIdConverter.mGraveyard)
        {
            auto mapped = actorIdConverter.mMappings.find(actorId);
            if (mapped != actorIdConverter.mMappings.end())
                MWBase::Environment::get().getMechanicsManager()->cleanupSummonedCreature(mapped->second);
        }

#ifdef __vita__
        // Safety sweep for anything skip-parse pinned.
        MWBase::Environment::get().getWorldScene()->vitaPostLoadDemote();
#endif
    }
    catch (const SaveVersionTooNewError& e)
    {
        std::string error = "#{OMWEngine:LoadingRequiresNewVersionError}";
        printSavegameFormatError(e.what(), error);
    }
    catch (const SaveVersionTooOldError& e)
    {
        const char* release;
        // Report the last version still capable of reading this save
        if (e.getFormatVersion() < ESM::OpenMW0_49MinSaveGameFormatVersion)
            release = "OpenMW 0.48.0";
        else
        {
            // Insert additional else if statements above to cover future releases
            static_assert(ESM::MinSupportedSaveGameFormatVersion <= ESM::OpenMW0_49MinSaveGameFormatVersion);
            release = "OpenMW 0.51.0";
        }
        auto l10n = MWBase::Environment::get().getL10nManager()->getContext("OMWEngine");
        std::string error = l10n->formatMessage("LoadingRequiresOldVersionError", { "version" }, { release });
        printSavegameFormatError(e.what(), error);
    }
    catch (const std::exception& e)
    {
        std::string error = "#{OMWEngine:LoadingFailed}: " + std::string(e.what());
        printSavegameFormatError(e.what(), error);
    }
}

void MWState::StateManager::printSavegameFormatError(
    const std::string& exceptionText, const std::string& messageBoxText)
{
    Log(Debug::Error) << "Failed to load saved game: " << exceptionText;

    cleanup(true);

    MWBase::Environment::get().getWindowManager()->pushGuiMode(MWGui::GM_MainMenu);

    std::vector<std::string> buttons;
    buttons.emplace_back("#{Interface:OK}");

    MWBase::Environment::get().getWindowManager()->interactiveMessageBox(messageBoxText, buttons);
}

void MWState::StateManager::quickLoad()
{
    if (Character* currentCharacter = getCurrentCharacter())
    {
        if (currentCharacter->begin() == currentCharacter->end())
            return;
        // use requestLoad, otherwise we can crash by loading during the wrong part of the frame
        requestLoad(currentCharacter, currentCharacter->begin()->mPath);
    }
}

void MWState::StateManager::deleteGame(const MWState::Character* character, const MWState::Slot* slot)
{
    const std::filesystem::path savePath = slot->mPath;
    mCharacterManager.deleteSlot(slot, character);
    if (mLastSavegame == savePath)
    {
        if (character != nullptr)
            mLastSavegame = character->begin()->mPath;
        else
            mLastSavegame.clear();
    }
}

MWState::Character* MWState::StateManager::getCurrentCharacter()
{
    return mCharacterManager.getCurrentCharacter();
}

MWState::StateManager::CharacterIterator MWState::StateManager::characterBegin()
{
    return mCharacterManager.begin();
}

MWState::StateManager::CharacterIterator MWState::StateManager::characterEnd()
{
    return mCharacterManager.end();
}

void MWState::StateManager::update(float duration)
{
    mTimePlayed += duration;

    // Note: It would be nicer to trigger this from InputManager, i.e. the very beginning of the frame update.
    if (mAskLoadRecent)
    {
        int iButton = MWBase::Environment::get().getWindowManager()->readPressedButton();
        MWState::Character* curCharacter = getCurrentCharacter();
        if (iButton == 0 && curCharacter)
        {
            mAskLoadRecent = false;
            // Load last saved game for current character
            // loadGame resets the game state along with mLastSavegame so we want to preserve it
            const std::filesystem::path filePath = std::move(mLastSavegame);
            loadGame(curCharacter, filePath);
        }
        else if (iButton == 1)
        {
            mAskLoadRecent = false;
            MWBase::Environment::get().getWindowManager()->pushGuiMode(MWGui::GM_MainMenu);
        }
    }

    if (mNewGameRequest)
    {
        MWBase::Environment::get().getWindowManager()->removeGuiMode(MWGui::GM_MainMenu);
        newGame();
        mNewGameRequest = false;
    }

    if (mLoadRequest)
    {
        MWBase::Environment::get().getWindowManager()->removeGuiMode(MWGui::GM_MainMenu);
        const Character* character = mLoadRequest->first;
        // The character may have been deleted after the request was made
        const bool validCharacter = std::ranges::find_if(mCharacterManager, [=](const Character& c) {
            return &c == character;
        }) != mCharacterManager.end();
        if (!validCharacter)
            character = getCurrentCharacter();
        loadGame(character, mLoadRequest->second);
        mLoadRequest = std::nullopt;
    }
}

bool MWState::StateManager::confirmLoading(const std::vector<std::string_view>& missingFiles) const
{
    std::ostringstream stream;
    for (auto& contentFile : missingFiles)
    {
        Log(Debug::Warning) << "Warning: Saved game dependency " << contentFile << " is missing.";
        stream << contentFile << "\n";
    }

    auto fullList = stream.str();
    if (!fullList.empty())
        fullList.pop_back();

    constexpr size_t missingPluginsDisplayLimit = 12;

    std::vector<std::string> buttons;
    buttons.emplace_back("#{Interface:Yes}");
    buttons.emplace_back("#{Interface:Copy}");
    buttons.emplace_back("#{Interface:No}");
    std::string message = "#{OMWEngine:MissingContentFilesConfirmation}";

    auto l10n = MWBase::Environment::get().getL10nManager()->getContext("OMWEngine");
    message += l10n->formatMessage("MissingContentFilesList", { "files" }, { static_cast<int>(missingFiles.size()) });
    auto cappedSize = std::min(missingFiles.size(), missingPluginsDisplayLimit);
    if (cappedSize == missingFiles.size())
    {
        message += fullList;
    }
    else
    {
        for (size_t i = 0; i < cappedSize - 1; ++i)
        {
            message += missingFiles[i];
            message += "\n";
        }

        message += "...";
    }

    message
        += l10n->formatMessage("MissingContentFilesListCopy", { "files" }, { static_cast<int>(missingFiles.size()) });

    int selectedButton = -1;
    while (true)
    {
        auto windowManager = MWBase::Environment::get().getWindowManager();
        windowManager->interactiveMessageBox(message, buttons, true, selectedButton);
        selectedButton = windowManager->readPressedButton();
        if (selectedButton == 0)
            break;

        if (selectedButton == 1)
        {
            SDL_SetClipboardText(fullList.c_str());
            continue;
        }

        return false;
    }

    return true;
}

void MWState::StateManager::writeScreenshot(std::vector<char>& imageData) const
{
    int screenshotW = 259 * 2, screenshotH = 133 * 2; // *2 to get some nice antialiasing

    osg::ref_ptr<osg::Image> screenshot(new osg::Image);

    MWBase::Environment::get().getWorld()->screenshot(screenshot.get(), screenshotW, screenshotH);

    osgDB::ReaderWriter* readerwriter = osgDB::Registry::instance()->getReaderWriterForExtension("jpg");
    if (!readerwriter)
    {
        Log(Debug::Error) << "Error: Unable to write screenshot, can't find a jpg ReaderWriter";
        return;
    }

    std::ostringstream ostream;
    osgDB::ReaderWriter::WriteResult result = readerwriter->writeImage(*screenshot, ostream);
    if (!result.success())
    {
        Log(Debug::Error) << "Error: Unable to write screenshot: " << result.message() << " code " << result.status();
        return;
    }

    std::string data = ostream.str();
    imageData = std::vector<char>(data.begin(), data.end());
}
