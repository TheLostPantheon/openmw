#include "esmstore.hpp"

#include <algorithm>
#include <fstream>
#include <tuple>

#include <components/debug/debuglog.hpp>

#include <components/esm/records.hpp>
#include <components/esm3/esmreader.hpp>
#include <components/esm3/esmwriter.hpp>
#include <components/esm3/readerscache.hpp>
#include <components/esm4/common.hpp>
#include <components/esm4/reader.hpp>
#include <components/esm4/readerutils.hpp>
#include <components/esmloader/load.hpp>
#include <components/loadinglistener/loadinglistener.hpp>
#include <components/lua/configuration.hpp>
#include <components/misc/algorithm.hpp>

#include "../mwmechanics/spelllist.hpp"

#ifdef __vita__
#include <cstdio>

#include <components/vita/VitaEsmPrefetch.h>
#include <components/vita/VitaWindowFileStream.h>

#include "../vita/VitaInit.h"
#endif

namespace
{
    struct Ref
    {
        ESM::RefNum mRefNum;
        std::size_t mRefID;
#ifdef __vita__
        uint32_t mCellIdx = 0; // the cell it was read from (sole-cell index)
#endif

        Ref(ESM::RefNum refNum, std::size_t refID)
            : mRefNum(refNum)
            , mRefID(refID)
        {
        }
    };

    constexpr std::size_t deletedRefID = std::numeric_limits<std::size_t>::max();

    // Index of a ref's id in refIDs. With refIdIndex each distinct id is
    // stored once (Vita: Tamriel Rebuilt places 1.35M objects of far fewer
    // kinds; one RefId per placed object was ~25 MB at boot).
    struct RefIdList
    {
        std::vector<ESM::RefId>& mIds;
        std::unordered_map<ESM::RefId, std::size_t>* mIndex;

        std::size_t add(ESM::RefId&& id)
        {
            if (mIndex == nullptr)
            {
                mIds.push_back(std::move(id));
                return mIds.size() - 1;
            }
            const auto [it, inserted] = mIndex->try_emplace(id, mIds.size());
            if (inserted)
                mIds.push_back(std::move(id));
            return it->second;
        }
    };

    // The refs of one of the cell's contexts; the reader is restored to it.
    void readContextRefs(
        const ESM::Cell& cell, ESM::ESMReader& reader, std::vector<Ref>& refs, RefIdList ids, std::set<ESM::RefId>& keyIDs)
    {
        ESM::CellRef ref;
        bool deleted = false;
        while (cell.getNextRef(reader, ref, deleted))
        {
            if (deleted)
                refs.emplace_back(ref.mRefNum, deletedRefID);
            else if (std::find(cell.mMovedRefs.begin(), cell.mMovedRefs.end(), ref.mRefNum) == cell.mMovedRefs.end())
            {
                if (!ref.mKey.empty())
                    keyIDs.insert(std::move(ref.mKey));
                refs.emplace_back(ref.mRefNum, ids.add(std::move(ref.mRefID)));
            }
        }
    }

    void readLeasedRefs(const ESM::Cell& cell, std::vector<Ref>& refs, RefIdList ids, std::set<ESM::RefId>& keyIDs)
    {
        for (const auto& [value, deleted] : cell.mLeasedRefs)
        {
            if (deleted)
                refs.emplace_back(value.mRefNum, deletedRefID);
            else
            {
                if (!value.mKey.empty())
                    keyIDs.insert(std::move(value.mKey));
                refs.emplace_back(value.mRefNum, ids.add(ESM::RefId(value.mRefID)));
            }
        }
    }

    void readRefs(const ESM::Cell& cell, std::vector<Ref>& refs, std::vector<ESM::RefId>& refIDs,
        std::set<ESM::RefId>& keyIDs, ESM::ReadersCache& readers)
    {
        // TODO: we have many similar copies of this code.
        for (size_t i = 0; i < cell.mContextList.size(); i++)
        {
            const std::size_t index = static_cast<std::size_t>(cell.mContextList[i].index);
            const ESM::ReadersCache::BusyItem reader = readers.get(index);
            cell.restore(*reader, i);
            readContextRefs(cell, *reader, refs, RefIdList{ refIDs, nullptr }, keyIDs);
        }
        readLeasedRefs(cell, refs, RefIdList{ refIDs, nullptr }, keyIDs);
    }

    const ESM::RefId& getDefaultClass(const MWWorld::Store<ESM::Class>& classes)
    {
        auto it = classes.begin();
        if (it != classes.end())
            return it->mId;
        throw std::runtime_error("List of NPC classes is empty!");
    }

    const ESM::RefId& getDefaultRace(const MWWorld::Store<ESM::Race>& races)
    {
        auto it = races.begin();
        if (it != races.end())
            return it->mId;
        throw std::runtime_error("List of NPC races is empty!");
    }

    std::vector<ESM::NPC> getNPCsToReplace(const MWWorld::Store<ESM::Faction>& factions,
        const MWWorld::Store<ESM::Class>& classes, const MWWorld::Store<ESM::Race>& races,
        const MWWorld::Store<ESM::Script>& scripts, const std::unordered_map<ESM::RefId, ESM::NPC>& npcs)
    {
        // Cache first class from store - we will use it if current class is not found
        const ESM::RefId& defaultCls = getDefaultClass(classes);
        // Same for races
        const ESM::RefId& defaultRace = getDefaultRace(races);

        // Validate NPCs for non-existing class and faction.
        // We will replace invalid entries by fixed ones
        std::vector<ESM::NPC> npcsToReplace;

        for (const auto& npcIter : npcs)
        {
            ESM::NPC npc = npcIter.second;
            bool changed = false;

            const ESM::RefId& npcFaction = npc.mFaction;
            if (!npcFaction.empty())
            {
                const ESM::Faction* fact = factions.search(npcFaction);
                if (!fact)
                {
                    Log(Debug::Verbose) << "NPC " << npc.mId << " (" << npc.mName << ") has nonexistent faction "
                                        << npc.mFaction << ", ignoring it.";
                    npc.mFaction = ESM::RefId();
                    npc.mNpdt.mRank = 0;
                    changed = true;
                }
            }

            const ESM::Class* cls = classes.search(npc.mClass);
            if (!cls)
            {
                Log(Debug::Verbose) << "NPC " << npc.mId << " (" << npc.mName << ") has nonexistent class "
                                    << npc.mClass << ", using " << defaultCls << " class as replacement.";
                npc.mClass = defaultCls;
                changed = true;
            }

            const ESM::Race* race = races.search(npc.mRace);
            if (!race)
            {
                Log(Debug::Verbose) << "NPC " << npc.mId << " (" << npc.mName << ") has nonexistent race " << npc.mRace
                                    << ", using " << defaultRace << " race as replacement.";
                npc.mRace = defaultRace;
                changed = true;
            }

            if (!npc.mScript.empty() && !scripts.search(npc.mScript))
            {
                Log(Debug::Verbose) << "NPC " << npc.mId << " (" << npc.mName << ") has nonexistent script "
                                    << npc.mScript << ", ignoring it.";
                npc.mScript = ESM::RefId();
                changed = true;
            }

            if (changed)
                npcsToReplace.push_back(std::move(npc));
        }

        return npcsToReplace;
    }

    template <class RecordType>
    std::vector<RecordType> getSpellsToReplace(
        const MWWorld::Store<RecordType>& spells, const MWWorld::Store<ESM::MagicEffect>& magicEffects)
    {
        std::vector<RecordType> spellsToReplace;

        for (RecordType spell : spells)
        {
            if (spell.mEffects.mList.empty())
                continue;

            bool changed = false;
            auto iter = spell.mEffects.mList.begin();
            while (iter != spell.mEffects.mList.end())
            {
                const ESM::MagicEffect* mgef = magicEffects.search(iter->mData.mEffectID);
                if (!mgef)
                {
                    Log(Debug::Verbose) << RecordType::getRecordType() << " " << spell.mId
                                        << ": dropping invalid effect (index " << iter->mData.mEffectID << ")";
                    iter = spell.mEffects.mList.erase(iter);
                    changed = true;
                    continue;
                }

                if (!(mgef->mData.mFlags & ESM::MagicEffect::TargetAttribute) && !iter->mData.mAttribute.empty())
                {
                    iter->mData.mAttribute = ESM::RefId();
                    Log(Debug::Verbose) << RecordType::getRecordType() << " " << spell.mId
                                        << ": dropping unexpected attribute argument of " << iter->mData.mEffectID
                                        << " effect";
                    changed = true;
                }

                if (!(mgef->mData.mFlags & ESM::MagicEffect::TargetSkill) && !iter->mData.mSkill.empty())
                {
                    iter->mData.mSkill = ESM::RefId();
                    Log(Debug::Verbose) << RecordType::getRecordType() << " " << spell.mId
                                        << ": dropping unexpected skill argument of " << iter->mData.mEffectID
                                        << " effect";
                    changed = true;
                }

                ++iter;
            }

            if (changed)
                spellsToReplace.emplace_back(spell);
        }

        return spellsToReplace;
    }

    // Custom enchanted items can reference scripts that no longer exist, this doesn't necessarily mean the base item no
    // longer exists however. So instead of removing the item altogether, we're only removing the script.
    template <class MapT>
    void removeMissingScripts(const MWWorld::Store<ESM::Script>& scripts, MapT& items)
    {
        for (auto& [id, item] : items)
        {
            if (!item.mScript.empty() && !scripts.search(item.mScript))
            {
                Log(Debug::Verbose) << MapT::mapped_type::getRecordType() << ' ' << id << " (" << item.mName
                                    << ") has nonexistent script " << item.mScript << ", ignoring it.";
                item.mScript = ESM::RefId();
            }
        }
    }
}

namespace MWWorld
{
    using IDMap = std::unordered_map<ESM::RefId, int>;

    struct ESMStoreImp
    {
        ESMStore::StoreTuple mStores;

        std::map<ESM::RecNameInts, DynamicStore*> mRecNameToStore;

        // Lookup of all IDs. Makes looking up references faster. Just
        // maps the id name to the record type.
        IDMap mIds;
        IDMap mStaticIds;

        template <typename T>
        static void assignStoreToIndex(ESMStore& stores, Store<T>& store)
        {
            const std::size_t storeIndex = ESMStore::getTypeIndex<T>();
            if (stores.mStores.size() <= storeIndex)
                stores.mStores.resize(storeIndex + 1);

            assert(&store == &std::get<Store<T>>(stores.mStoreImp->mStores));

            stores.mStores[storeIndex] = &store;
            if constexpr (std::is_convertible_v<Store<T>*, DynamicStore*>)
            {
                stores.mDynamicStores.push_back(&store);
                constexpr ESM::RecNameInts recName = T::sRecordId;
                if constexpr (recName != ESM::REC_INTERNAL_PLAYER)
                {
                    stores.mStoreImp->mRecNameToStore[recName] = &store;
                }
            }
        }

        template <typename T>
        static bool typedReadRecordESM4(ESM4::Reader& reader, Store<T>& store)
        {
            auto recordType = static_cast<ESM4::RecordTypes>(reader.hdr().record.typeId);

            ESM::RecNameInts esm4RecName = static_cast<ESM::RecNameInts>(ESM::esm4Recname(recordType));
            if constexpr (HasRecordId<T>::value)
            {
                if constexpr (ESM::isESM4Rec(T::sRecordId))
                {
                    if (T::sRecordId == esm4RecName)
                    {
                        reader.getRecordData();
                        T value;
                        value.load(reader);
                        store.insertStatic(value);
                        return true;
                    }
                }
            }
            return false;
        }

        static bool readRecord(ESM4::Reader& reader, ESMStore& store)
        {
            return std::apply(
                [&reader](auto&... x) { return (typedReadRecordESM4(reader, x) || ...); }, store.mStoreImp->mStores);
        }
    };

    int ESMStore::find(const ESM::RefId& id) const
    {
        IDMap::const_iterator it = mStoreImp->mIds.find(id);
        if (it == mStoreImp->mIds.end())
        {
            return 0;
        }
        return it->second;
    }

    int ESMStore::findStatic(const ESM::RefId& id) const
    {
        IDMap::const_iterator it = mStoreImp->mStaticIds.find(id);
        if (it == mStoreImp->mStaticIds.end())
        {
            return 0;
        }
        return it->second;
    }

    ESMStore::ESMStore()
    {
        mStoreImp = std::make_unique<ESMStoreImp>();
        std::apply([this](auto&... x) { (ESMStoreImp::assignStoreToIndex(*this, x), ...); }, mStoreImp->mStores);
        mDynamicCount = 0;
        getWritable<ESM::Pathgrid>().setCells(getWritable<ESM::Cell>());
    }

    ESMStore::~ESMStore() = default;

    void ESMStore::clearDynamic()
    {
        for (const auto& store : mDynamicStores)
            store->clearDynamic();
        mStoreImp->mIds = mStoreImp->mStaticIds;

        movePlayerRecord();
    }

    static bool isCacheableRecord(int id)
    {
        switch (id)
        {
            case ESM::REC_ACTI:
            case ESM::REC_ALCH:
            case ESM::REC_APPA:
            case ESM::REC_ARMO:
            case ESM::REC_BOOK:
            case ESM::REC_CLOT:
            case ESM::REC_CONT:
            case ESM::REC_CREA:
            case ESM::REC_DOOR:
            case ESM::REC_INGR:
            case ESM::REC_LEVC:
            case ESM::REC_LEVI:
            case ESM::REC_LIGH:
            case ESM::REC_LOCK:
            case ESM::REC_MISC:
            case ESM::REC_NPC_:
            case ESM::REC_PROB:
            case ESM::REC_REPA:
            case ESM::REC_STAT:
            case ESM::REC_WEAP:
            case ESM::REC_BODY:
            case ESM::REC_ACTI4:
            case ESM::REC_ALCH4:
            case ESM::REC_AMMO4:
            case ESM::REC_ARMO4:
            case ESM::REC_BOOK4:
            case ESM::REC_CONT4:
            case ESM::REC_CREA4:
            case ESM::REC_DOOR4:
            case ESM::REC_FLOR4:
            case ESM::REC_FURN4:
            case ESM::REC_IMOD4:
            case ESM::REC_INGR4:
            case ESM::REC_LIGH4:
            case ESM::REC_LVLI4:
            case ESM::REC_LVLC4:
            case ESM::REC_LVLN4:
            case ESM::REC_MISC4:
            case ESM::REC_MSTT4:
            case ESM::REC_NPC_4:
            case ESM::REC_SCOL4:
            case ESM::REC_STAT4:
            case ESM::REC_TERM4:
            case ESM::REC_TREE4:
            case ESM::REC_WEAP4:
                return true;
                break;
        }
        return false;
    }

    void ESMStore::load(ESM::ESMReader& esm, Loading::Listener* listener, ESM::Dialogue*& dialogue)
    {
        if (listener != nullptr)
            listener->setProgressRange(::EsmLoader::fileProgress);

        // Loop through all records
        while (esm.hasMoreRecs())
        {
            ESM::NAME n = esm.getRecName();
            esm.getRecHeader();
            if (esm.getRecordFlags() & ESM::FLAG_Ignored)
            {
                esm.skipRecord();
                continue;
            }

            // Look up the record type.
            ESM::RecNameInts recName = static_cast<ESM::RecNameInts>(n.toInt());
            const auto& it = mStoreImp->mRecNameToStore.find(recName);

            if (it == mStoreImp->mRecNameToStore.end())
            {
                if (recName == ESM::REC_INFO)
                {
                    if (dialogue)
                    {
                        dialogue->readInfo(esm);
                    }
                    else
                    {
                        Log(Debug::Error) << "Error: info record without dialog";
                        esm.skipRecord();
                    }
                }
                else if (n.toInt() == ESM::REC_MGEF)
                {
                    getWritable<ESM::MagicEffect>().load(esm);
                }
                else if (n.toInt() == ESM::REC_SKIL)
                {
                    getWritable<ESM::Skill>().load(esm);
                }
                else if (n.toInt() == ESM::REC_FILT || n.toInt() == ESM::REC_DBGP)
                {
                    // ignore project file only records
                    esm.skipRecord();
                }
                else if (n.toInt() == ESM::REC_LUAL)
                {
                    ESM::LuaScriptsCfg cfg;
                    cfg.load(esm);
                    cfg.adjustRefNums(esm);
                    mLuaContent.push_back(std::move(cfg));
                }
                else
                {
                    throw std::runtime_error("Unknown record: " + n.toString());
                }
            }
            else
            {
                RecordId id = it->second->load(esm);
                if (id.mIsDeleted)
                {
                    it->second->eraseStatic(id.mId);
                    continue;
                }

                if (n.toInt() == ESM::REC_DIAL)
                {
                    dialogue = const_cast<ESM::Dialogue*>(getWritable<ESM::Dialogue>().find(id.mId));
                }
                else
                {
                    dialogue = nullptr;
                }
            }
            if (listener != nullptr)
                listener->setProgress(::EsmLoader::fileProgress * esm.getFileOffset() / esm.getFileSize());
        }
    }

    void ESMStore::loadESM4(ESM4::Reader& reader, Loading::Listener* listener)
    {
        if (listener != nullptr)
            listener->setProgressRange(::EsmLoader::fileProgress);
        auto visitorRec = [this, listener](ESM4::Reader& r) {
            bool result = ESMStoreImp::readRecord(r, *this);
            if (listener != nullptr)
                listener->setProgress(::EsmLoader::fileProgress * r.getFileOffset() / r.getFileSize());
            return result;
        };
        ESM4::ReaderUtils::readAll(reader, visitorRec, [](ESM4::Reader&) {});
    }

    void ESMStore::setIdType(const ESM::RefId& id, ESM::RecNameInts type)
    {
        mStoreImp->mIds[id] = type;
    }

    ESM::LuaScriptsCfg ESMStore::getLuaScriptsCfg() const
    {
        ESM::LuaScriptsCfg cfg;
        for (const LuaContent& c : mLuaContent)
        {
            if (std::holds_alternative<std::filesystem::path>(c))
            {
                // *.omwscripts are intentionally reloaded every time when `getLuaScriptsCfg` is called.
                // It is important for the `reloadlua` console command.
                try
                {
                    auto file = std::ifstream(std::get<std::filesystem::path>(c));
                    std::string fileContent(std::istreambuf_iterator<char>(file), {});
                    LuaUtil::parseOMWScripts(cfg, fileContent);
                }
                catch (std::exception& e)
                {
                    Log(Debug::Error) << e.what();
                }
            }
            else
            {
                const ESM::LuaScriptsCfg& addition = std::get<ESM::LuaScriptsCfg>(c);
                cfg.mScripts.insert(cfg.mScripts.end(), addition.mScripts.begin(), addition.mScripts.end());
            }
        }
        return cfg;
    }

    void ESMStore::setUp()
    {
        if (mIsSetUpDone)
            throw std::logic_error("ESMStore::setUp() is called twice");
        mIsSetUpDone = true;

        for (const auto& [_, store] : mStoreImp->mRecNameToStore)
            store->setUp();

        getWritable<ESM::Skill>().setUp(get<ESM::GameSetting>());
        getWritable<ESM::MagicEffect>().setUp(get<ESM::GameSetting>());
        getWritable<ESM::Attribute>().setUp(get<ESM::GameSetting>());
        getWritable<ESM4::Land>().updateLandPositions(get<ESM4::Cell>());
        getWritable<ESM4::Reference>().preprocessReferences(get<ESM4::Cell>());
        getWritable<ESM4::ActorCharacter>().preprocessReferences(get<ESM4::Cell>());
        getWritable<ESM4::ActorCreature>().preprocessReferences(get<ESM4::Cell>());

        rebuildIdsIndex();
        mStoreImp->mStaticIds = mStoreImp->mIds;
    }

    void ESMStore::rebuildIdsIndex()
    {
        mStoreImp->mIds.clear();
        for (const auto& [recordType, store] : mStoreImp->mRecNameToStore)
        {
            if (isCacheableRecord(recordType))
            {
                std::vector<ESM::RefId> identifiers;
                store->listIdentifier(identifiers);
                for (auto& record : identifiers)
                    mStoreImp->mIds[record] = recordType;
            }
        }
    }

    void ESMStore::validateRecords(ESM::ReadersCache& readers)
    {
        validate();
        countAllCellRefsAndMarkKeys(readers);
    }

#ifdef __vita__
    void vitaForEachCellContext(const std::vector<const ESM::Cell*>& cells, ESM::ReadersCache& readers,
        const std::function<void(std::size_t, std::size_t, ESM::ESMReader&)>& visit)
    {
        struct Task
        {
            std::size_t mCell;
            std::size_t mContext;
            int mFile;
            std::size_t mPos;
        };
        std::vector<Task> tasks;
        for (std::size_t c = 0; c < cells.size(); ++c)
            for (std::size_t i = 0; i < cells[c]->mContextList.size(); ++i)
                tasks.push_back(Task{ c, i, cells[c]->mContextList[i].index, cells[c]->mContextList[i].filePos });
        std::stable_sort(tasks.begin(), tasks.end(),
            [](const Task& l, const Task& r) { return l.mFile < r.mFile || (l.mFile == r.mFile && l.mPos < r.mPos); });
        // Readers still over their in-RAM copies (EsmPrefetch kept them for
        // this first boot) stay as they are.
        const bool fromRam = Vita::EsmPrefetch::keepUntilFinish();
        for (std::size_t t = 0; t < tasks.size();)
        {
            const int file = tasks[t].mFile;
            {
                const ESM::ReadersCache::BusyItem reader = readers.get(static_cast<std::size_t>(file));
                const std::filesystem::path& name = cells[tasks[t].mCell]->mContextList[tasks[t].mContext].filename;
                if (!fromRam)
                    if (auto windowed = Vita::openWindowFileStream(name))
                        reader->open(std::move(windowed), name);
                for (; t < tasks.size() && tasks[t].mFile == file; ++t)
                    visit(tasks[t].mCell, tasks[t].mContext, *reader);
            }
            if (!fromRam)
                readers.vitaClose(static_cast<std::size_t>(file)); // drop the 1 MB window
        }
    }
#endif

    void ESMStore::countAllCellRefsAndMarkKeys(ESM::ReadersCache& readers)
    {
        // TODO: We currently need to read entire files here again.
        // We should consider consolidating or deferring this reading.
        if (!mRefCount.empty())
            return;
        std::vector<Ref> refs;
        std::set<ESM::RefId> keyIDs;
        std::vector<ESM::RefId> refIDs;
        const Store<ESM::Cell>& cells = get<ESM::Cell>();
#ifdef __vita__
        // One 16-byte Ref per placed object and each distinct id once: the
        // per-object id copies, cell index vector and RefNum hash map this
        // replaces came to ~70 bytes per object, ~115 MB for Tamriel Rebuilt.
        std::unordered_map<ESM::RefId, std::size_t> refIdIndex;
        const RefIdList ids{ refIDs, &refIdIndex };
        std::vector<const ESM::Cell*> walkOrder;
        for (auto it = cells.intBegin(); it != cells.intEnd(); ++it)
            walkOrder.push_back(&*it);
        for (auto it = cells.extBegin(); it != cells.extEnd(); ++it)
            walkOrder.push_back(&*it);
        std::vector<ESM::RefId> walkedCells;
        walkedCells.reserve(walkOrder.size());
        for (const ESM::Cell* cell : walkOrder)
            walkedCells.push_back(cell->mId);
        // Read in file order; the sort below puts refs back in cell order.
        vitaForEachCellContext(walkOrder, readers, [&](std::size_t cellIdx, std::size_t context, ESM::ESMReader& reader) {
            const ESM::Cell& cell = *walkOrder[cellIdx];
            cell.restore(reader, context);
            const std::size_t first = refs.size();
            readContextRefs(cell, reader, refs, ids, keyIDs);
            for (std::size_t i = first; i < refs.size(); ++i)
                refs[i].mCellIdx = static_cast<uint32_t>(cellIdx);
        });
        for (std::size_t cellIdx = 0; cellIdx < walkOrder.size(); ++cellIdx)
        {
            const std::size_t first = refs.size();
            readLeasedRefs(*walkOrder[cellIdx], refs, ids, keyIDs);
            for (std::size_t i = first; i < refs.size(); ++i)
                refs[i].mCellIdx = static_cast<uint32_t>(cellIdx);
        }
        {
            char crumb[128];
            snprintf(crumb, sizeof(crumb), "[RefScan] %u refs, %u ids, %u cells", static_cast<unsigned>(refs.size()),
                static_cast<unsigned>(refIDs.size()), static_cast<unsigned>(walkedCells.size()));
            vitaMemBreadcrumb(crumb);
        }
        refIdIndex = {};
#else
        for (auto it = cells.intBegin(); it != cells.intEnd(); ++it)
            readRefs(*it, refs, refIDs, keyIDs, readers);
        for (auto it = cells.extBegin(); it != cells.extEnd(); ++it)
            readRefs(*it, refs, refIDs, keyIDs, readers);
#endif
#ifdef __vita__
        // Per RefNum in cell walk order, as upstream reads them; within a
        // cell, contexts come in load order either way.
        const auto lessByRefNum = [](const Ref& l, const Ref& r) {
            return l.mRefNum < r.mRefNum || (l.mRefNum == r.mRefNum && l.mCellIdx < r.mCellIdx);
        };
#else
        const auto lessByRefNum = [](const Ref& l, const Ref& r) { return l.mRefNum < r.mRefNum; };
#endif
        std::stable_sort(refs.begin(), refs.end(), lessByRefNum);
#ifdef __vita__
        // Ref -> sole cell. Sorted stably, each RefNum's refs are adjacent in
        // load order and the last one is the one that stands.
        mVitaRefCellIndex.clear();
        for (std::size_t end = refs.size(); end > 0;)
        {
            const Ref& last = refs[end - 1];
            std::size_t first = end - 1;
            while (first > 0 && refs[first - 1].mRefNum == last.mRefNum)
                --first;
            if (last.mRefID != deletedRefID)
            {
                const ESM::RefId& cellId = walkedCells[last.mCellIdx];
                auto [slot, inserted] = mVitaRefCellIndex.try_emplace(refIDs[last.mRefID], cellId);
                if (!inserted && slot->second != cellId)
                    slot->second = ESM::RefId(); // ambiguous
            }
            end = first;
        }
        std::erase_if(mVitaRefCellIndex, [](const auto& p) { return p.second.empty(); });
        // Ids are shared between refs here: copy, don't move.
        const auto incrementRefCount = [&](const Ref& value) {
            if (value.mRefID != deletedRefID)
                ++mRefCount[refIDs[value.mRefID]];
        };
#else
        const auto incrementRefCount = [&](const Ref& value) {
            if (value.mRefID != deletedRefID)
            {
                ESM::RefId& refId = refIDs[value.mRefID];
                ++mRefCount[std::move(refId)];
            }
        };
#endif
        const auto equalByRefNum = [](const Ref& l, const Ref& r) { return l.mRefNum == r.mRefNum; };
        Misc::forEachUnique(refs.rbegin(), refs.rend(), equalByRefNum, incrementRefCount);
        auto& store = getWritable<ESM::Miscellaneous>().mStatic;
        for (const auto& id : keyIDs)
        {
            auto it = store.find(id);
            if (it != store.end())
                it->second.mData.mFlags |= ESM::Miscellaneous::Key;
        }
    }

#ifdef __vita__
    void ESMStore::vitaSeedRefCounts(std::unordered_map<ESM::RefId, int>&& counts, const std::vector<ESM::RefId>& keyIds)
    {
        mRefCount = std::move(counts);
        auto& store = getWritable<ESM::Miscellaneous>().mStatic;
        for (const auto& id : keyIds)
        {
            auto it = store.find(id);
            if (it != store.end())
                it->second.mData.mFlags |= ESM::Miscellaneous::Key;
        }
    }

    std::vector<ESM::RefId> ESMStore::vitaGetKeyIds() const
    {
        std::vector<ESM::RefId> ids;
        const auto& store = get<ESM::Miscellaneous>();
        for (auto it = store.begin(); it != store.end(); ++it)
            if (it->mData.mFlags & ESM::Miscellaneous::Key)
                ids.push_back(it->mId);
        return ids;
    }
#endif

    int ESMStore::getRefCount(const ESM::RefId& id) const
    {
        auto it = mRefCount.find(id);
        if (it == mRefCount.end())
            return 0;
        return it->second;
    }

    void ESMStore::validate()
    {
        auto& npcs = getWritable<ESM::NPC>();
        std::vector<ESM::NPC> npcsToReplace = getNPCsToReplace(getWritable<ESM::Faction>(), getWritable<ESM::Class>(),
            getWritable<ESM::Race>(), getWritable<ESM::Script>(), npcs.mStatic);

        for (const ESM::NPC& npc : npcsToReplace)
        {
            npcs.eraseStatic(npc.mId);
            npcs.insertStatic(npc);
        }

        removeMissingScripts(getWritable<ESM::Script>(), getWritable<ESM::Creature>().mStatic);

        // Validate spell effects and enchantments for invalid arguments
        auto& spells = getWritable<ESM::Spell>();
        auto& enchantments = getWritable<ESM::Enchantment>();
        auto& magicEffects = getWritable<ESM::MagicEffect>();

        std::vector<ESM::Spell> spellsToReplace = getSpellsToReplace(spells, magicEffects);
        for (const ESM::Spell& spell : spellsToReplace)
        {
            spells.eraseStatic(spell.mId);
            spells.insertStatic(spell);
        }

        std::vector<ESM::Enchantment> enchantmentsToReplace = getSpellsToReplace(enchantments, magicEffects);
        for (const ESM::Enchantment& enchantment : enchantmentsToReplace)
        {
            enchantments.eraseStatic(enchantment.mId);
            enchantments.insertStatic(enchantment);
        }
    }

    void ESMStore::movePlayerRecord()
    {
        auto& npcs = getWritable<ESM::NPC>();
        auto player = npcs.find(ESM::RefId::stringRefId("Player"));
        npcs.insert(*player);
    }

    void ESMStore::validateDynamic()
    {
        auto& npcs = getWritable<ESM::NPC>();
        auto& scripts = getWritable<ESM::Script>();

        std::vector<ESM::NPC> npcsToReplace = getNPCsToReplace(getWritable<ESM::Faction>(), getWritable<ESM::Class>(),
            getWritable<ESM::Race>(), getWritable<ESM::Script>(), npcs.mDynamic);

        for (const ESM::NPC& npc : npcsToReplace)
            npcs.insert(npc);

        removeMissingScripts(scripts, getWritable<ESM::Armor>().mDynamic);
        removeMissingScripts(scripts, getWritable<ESM::Book>().mDynamic);
        removeMissingScripts(scripts, getWritable<ESM::Clothing>().mDynamic);
        removeMissingScripts(scripts, getWritable<ESM::Creature>().mDynamic);
        removeMissingScripts(scripts, getWritable<ESM::Weapon>().mDynamic);

        removeMissingObjects(getWritable<ESM::CreatureLevList>());
        removeMissingObjects(getWritable<ESM::ItemLevList>());
    }

    // Leveled lists can be modified by scripts. This removes items that no longer exist (presumably because the
    // plugin was removed) from modified lists
    template <class T>
    void ESMStore::removeMissingObjects(Store<T>& store)
    {
        for (auto& entry : store.mDynamic)
        {
            auto first = std::remove_if(entry.second.mList.begin(), entry.second.mList.end(), [&](const auto& item) {
                if (!find(item.mId))
                {
                    Log(Debug::Verbose) << "Leveled list " << entry.first << " has nonexistent object " << item.mId
                                        << ", ignoring it.";
                    return true;
                }
                return false;
            });
            entry.second.mList.erase(first, entry.second.mList.end());
        }
    }

    size_t ESMStore::countSavedGameRecords() const
    {
        return 1 // DYNA (dynamic name counter)
            + get<ESM::Potion>().getDynamicSize() + get<ESM::Armor>().getDynamicSize()
            + get<ESM::Book>().getDynamicSize() + get<ESM::Class>().getDynamicSize()
            + get<ESM::Clothing>().getDynamicSize() + get<ESM::Enchantment>().getDynamicSize()
            + get<ESM::NPC>().getDynamicSize() + get<ESM::Spell>().getDynamicSize()
            + get<ESM::Activator>().getDynamicSize() + get<ESM::Miscellaneous>().getDynamicSize()
            + get<ESM::Weapon>().getDynamicSize() + get<ESM::CreatureLevList>().getDynamicSize()
            + get<ESM::ItemLevList>().getDynamicSize() + get<ESM::Creature>().getDynamicSize()
            + get<ESM::Container>().getDynamicSize() + get<ESM::Light>().getDynamicSize();
    }

    void ESMStore::write(ESM::ESMWriter& writer, Loading::Listener& progress) const
    {
        writer.startRecord(ESM::REC_DYNA);
        writer.startSubRecord("COUN");
        writer.writeT(mDynamicCount);
        writer.endRecord("COUN");
        writer.endRecord(ESM::REC_DYNA);

        get<ESM::Potion>().write(writer, progress);
        get<ESM::Armor>().write(writer, progress);
        get<ESM::Book>().write(writer, progress);
        get<ESM::Class>().write(writer, progress);
        get<ESM::Clothing>().write(writer, progress);
        get<ESM::Enchantment>().write(writer, progress);
        get<ESM::NPC>().write(writer, progress);
        get<ESM::Miscellaneous>().write(writer, progress);
        get<ESM::Activator>().write(writer, progress);
        get<ESM::Spell>().write(writer, progress);
        get<ESM::Weapon>().write(writer, progress);
        get<ESM::CreatureLevList>().write(writer, progress);
        get<ESM::ItemLevList>().write(writer, progress);
        get<ESM::Creature>().write(writer, progress);
        get<ESM::Container>().write(writer, progress);
        get<ESM::Light>().write(writer, progress);
    }

    bool ESMStore::readRecord(ESM::ESMReader& reader, uint32_t typeId)
    {
        ESM::RecNameInts type = static_cast<ESM::RecNameInts>(typeId);
        switch (type)
        {
            case ESM::REC_ALCH:
            case ESM::REC_ARMO:
            case ESM::REC_BOOK:
            case ESM::REC_CLAS:
            case ESM::REC_CLOT:
            case ESM::REC_ENCH:
            case ESM::REC_SPEL:
            case ESM::REC_WEAP:
                mStoreImp->mRecNameToStore[type]->read(reader);
                return true;
            case ESM::REC_NPC_:
            case ESM::REC_CREA:
            case ESM::REC_CONT:
            case ESM::REC_MISC:
            case ESM::REC_ACTI:
            case ESM::REC_LEVI:
            case ESM::REC_LEVC:
            case ESM::REC_LIGH:
                mStoreImp->mRecNameToStore[type]->read(reader, true);
                return true;

            case ESM::REC_DYNA:
                reader.getSubNameIs("COUN");
                if (reader.getFormatVersion() <= ESM::MaxActiveSpellTypeVersion)
                {
                    uint32_t dynamicCount32 = 0;
                    reader.getHT(dynamicCount32);
                    mDynamicCount = dynamicCount32;
                }
                else
                {
                    reader.getHT(mDynamicCount);
                }
                return true;

            default:

                return false;
        }
    }

    void ESMStore::checkPlayer()
    {
        const ESM::NPC* player = get<ESM::NPC>().find(ESM::RefId::stringRefId("Player"));

        if (!get<ESM::Race>().find(player->mRace) || !get<ESM::Class>().find(player->mClass))
            throw std::runtime_error("Invalid player record (race or class unavailable");
    }

    std::pair<std::shared_ptr<MWMechanics::SpellList>, bool> ESMStore::getSpellList(const ESM::RefId& id) const
    {
        auto result = mSpellListCache.find(id);
        std::shared_ptr<MWMechanics::SpellList> ptr;
        if (result != mSpellListCache.end())
            ptr = result->second.lock();
        if (!ptr)
        {
            int type = find(id);
            ptr = std::make_shared<MWMechanics::SpellList>(id, type);
            if (result != mSpellListCache.end())
                result->second = ptr;
            else
                mSpellListCache.insert({ id, ptr });
            return { ptr, false };
        }
        return { ptr, true };
    }

    template <>
    const ESM::Cell* ESMStore::insert<ESM::Cell>(const ESM::Cell& cell)
    {
        return getWritable<ESM::Cell>().insert(cell);
    }

    template <>
    const ESM::NPC* ESMStore::insert<ESM::NPC>(const ESM::NPC& npc)
    {

        auto& npcs = getWritable<ESM::NPC>();
        if (npc.mId == "Player")
        {
            return npcs.insert(npc);
        }
        const ESM::RefId id = ESM::RefId::generated(mDynamicCount++);
        if (npcs.search(id) != nullptr)
            throw std::runtime_error("Try to override existing record: " + id.toDebugString());
        ESM::NPC record = npc;

        record.mId = id;

        ESM::NPC* ptr = npcs.insert(record);
        mStoreImp->mIds[ptr->mId] = ESM::REC_NPC_;
        return ptr;
    }

} // end namespace
