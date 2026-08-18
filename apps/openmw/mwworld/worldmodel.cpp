#include "worldmodel.hpp"

#include <algorithm>
#include <cassert>
#include <optional>
#include <stdexcept>
#ifdef __vita__
#include <unordered_set>
#endif

#include <components/debug/debuglog.hpp>
#include <components/esm/defs.hpp>
#include <components/esm/util.hpp>
#include <components/esm3/actoridconverter.hpp>
#include <components/esm3/cellid.hpp>
#include <components/esm3/cellref.hpp>
#include <components/esm3/cellstate.hpp>
#include <components/esm3/esmreader.hpp>
#include <components/esm3/esmwriter.hpp>
#include <components/esm3/formatversion.hpp>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>

namespace
{
    const std::string& vitaObjectIndexBlob();
    bool vitaSpillRead(const std::string& path, std::string& out)
    {
        FILE* f = fopen(path.c_str(), "rb");
        if (!f)
            return false;
        fseek(f, 0, SEEK_END);
        const long n = ftell(f);
        fseek(f, 0, SEEK_SET);
        out.resize((std::size_t)n);
        const bool ok = fread(out.data(), 1, out.size(), f) == out.size();
        fclose(f);
        return ok;
    }

    const std::string& vitaObjectIndexBlob()
    {
        static std::string blob = [] {
            std::string b;
            if (!vitaSpillRead("ux0:data/openmw/cache/object_index.txt", b))
                vitaSpillRead("app0:warm/object_index.txt", b);
            if (!b.empty())
                b.insert(0, 1, '\n');
            return b;
        }();
        return blob;
    }

    bool vitaObjectIndexLookup(const ESM::RefId& name, std::string& out)
    {
        const std::string& blob = vitaObjectIndexBlob();
        if (blob.empty())
            return false;
        const std::string key = "\n" + name.serializeText() + "\t";
        const auto pos = blob.find(key);
        if (pos == std::string::npos)
            return false;
        const auto valueStart = pos + key.size();
        const auto valueEnd = blob.find('\n', valueStart);
        out = blob.substr(valueStart, valueEnd - valueStart);
        return true;
    }
}
#include <components/esm3/loadregn.hpp>
#include <components/esm4/loadwrld.hpp>
#include <components/loadinglistener/loadinglistener.hpp>
#include <components/settings/values.hpp>

#include "cellstore.hpp"
#include "esmstore.hpp"

#include "../mwbase/environment.hpp"
#include "../mwbase/luamanager.hpp"

#ifdef __vita__
#include "../vita/VitaInit.h"
#endif

namespace MWWorld
{
    namespace
    {
        const ESM::RefId draftCellId = ESM::RefId::index(ESM::REC_CSTA, 0);

        template <class T>
        CellStore& emplaceCellStore(ESM::RefId id, const T& cell, ESMStore& store, ESM::ReadersCache& readers,
            std::unordered_map<ESM::RefId, CellStore>& cells)
        {
            const auto [it, inserted] = cells.emplace(
                std::piecewise_construct, std::forward_as_tuple(id), std::forward_as_tuple(Cell(cell), store, readers));
            assert(inserted);
            return it->second;
        }

        CellStore* emplaceInteriorCellStore(std::string_view name, ESMStore& store, ESM::ReadersCache& readers,
            std::unordered_map<ESM::RefId, CellStore>& cells)
        {
            if (const ESM::Cell* cell = store.get<ESM::Cell>().search(name))
                return &emplaceCellStore(cell->mId, *cell, store, readers, cells);
            if (const ESM4::Cell* cell = store.get<ESM4::Cell>().searchCellName(name);
                cell != nullptr && !cell->isExterior())
            {
                return &emplaceCellStore(cell->mId, *cell, store, readers, cells);
            }
            return nullptr;
        }

        const ESM::Cell* createEsmCell(ESM::ExteriorCellLocation location, ESMStore& store)
        {
            ESM::Cell record = {};
            record.mData.mFlags = ESM::Cell::HasWater;
            record.mData.mX = location.mX;
            record.mData.mY = location.mY;
            record.updateId();
            return store.insert(record);
        }

        const ESM4::Cell* createEsm4Cell(ESM::ExteriorCellLocation location, ESMStore& store)
        {
            ESM4::Cell record = {};
            record.mParent = location.mWorldspace;
            record.mX = location.mX;
            record.mY = location.mY;
            return store.insert(record);
        }

        std::tuple<Cell, bool> createExteriorCell(ESM::ExteriorCellLocation location, ESMStore& store)
        {
            if (ESM::isEsm4Ext(location.mWorldspace))
            {
                if (store.get<ESM4::World>().search(location.mWorldspace) == nullptr)
                    throw std::runtime_error(
                        "Exterior ESM4 world is not found: " + location.mWorldspace.toDebugString());
                const ESM4::Cell* cell = store.get<ESM4::Cell>().searchExterior(location);
                bool created = cell == nullptr;
                if (created)
                    cell = createEsm4Cell(location, store);
                assert(cell != nullptr);
                return { MWWorld::Cell(*cell), created };
            }

            const ESM::Cell* cell = store.get<ESM::Cell>().search(location.mX, location.mY);
            bool created = cell == nullptr;
            if (created)
                cell = createEsmCell(location, store);
            assert(cell != nullptr);
            return { Cell(*cell), created };
        }

        std::optional<Cell> createCell(ESM::RefId id, const ESMStore& store)
        {
            if (const ESM4::Cell* cell = store.get<ESM4::Cell>().search(id))
                return Cell(*cell);
            if (const ESM::Cell* cell = store.get<ESM::Cell>().search(id))
                return Cell(*cell);
            return std::nullopt;
        }

        CellStore* getOrCreateExterior(const ESM::ExteriorCellLocation& location,
            std::map<ESM::ExteriorCellLocation, MWWorld::CellStore*>& exteriors, ESMStore& store,
            ESM::ReadersCache& readers, std::unordered_map<ESM::RefId, CellStore>& cells, bool triggerEvent)
        {
            if (const auto it = exteriors.find(location); it != exteriors.end())
            {
                assert(it->second != nullptr);
                return it->second;
            }
            auto [cell, created] = createExteriorCell(location, store);
            const ESM::RefId id = cell.getId();
            CellStore* const cellStore = &emplaceCellStore(id, std::move(cell), store, readers, cells);
            exteriors.emplace(location, cellStore);
            if (created && triggerEvent)
                MWBase::Environment::get().getLuaManager()->exteriorCreated(*cellStore);
            return cellStore;
        }
    }
}

MWWorld::CellStore& MWWorld::WorldModel::getOrInsertCellStore(const ESM::Cell& cell)
{
    const auto it = mCells.find(cell.mId);
    if (it != mCells.end())
        return it->second;
    return insertCellStore(cell);
}

MWWorld::CellStore& MWWorld::WorldModel::insertCellStore(const ESM::Cell& cell)
{
    CellStore& cellStore = emplaceCellStore(cell.mId, cell, mStore, mReaders, mCells);
    if (cell.mData.mFlags & ESM::Cell::Interior)
        mInteriors.emplace(cell.mName, &cellStore);
    else
        mExteriors.emplace(
            ESM::ExteriorCellLocation(cell.getGridX(), cell.getGridY(), ESM::Cell::sDefaultWorldspaceId), &cellStore);
    return cellStore;
}

void MWWorld::WorldModel::clear()
{
    mPtrRegistry.clear();
    mInteriors.clear();
    mExteriors.clear();
    mCells.clear();
    std::fill(mIdCache.begin(), mIdCache.end(), std::make_pair(ESM::RefId(), (MWWorld::CellStore*)nullptr));
    mIdCacheIndex = 0;
#ifdef __vita__
    mVitaEvictedState.clear();
    if (mVitaLedgerFile)
    {
        fclose(mVitaLedgerFile);
        mVitaLedgerFile = nullptr;
    }
    std::remove("ux0:data/openmw/cache/session_ledger.bin");
    mVitaLedgerEnd = 0;
    mVitaLoadDemote = false;
#endif
}

MWWorld::Ptr MWWorld::WorldModel::getPtrAndCache(const ESM::RefId& name, CellStore& cellStore)
{
    Ptr ptr = cellStore.getPtr(name);

    if (!ptr.isEmpty() && ptr.isInCell())
    {
        mIdCache[mIdCacheIndex].first = name;
        mIdCache[mIdCacheIndex].second = &cellStore;
        if (++mIdCacheIndex >= mIdCache.size())
            mIdCacheIndex = 0;
    }

    return ptr;
}

void MWWorld::WorldModel::writeCell(ESM::ESMWriter& writer, CellStore& cell) const
{
    if (cell.getState() != CellStore::State_Loaded)
        vitaLoadWithState(cell);

    ESM::CellState cellState;

    cell.saveState(cellState);

    writer.startRecord(ESM::REC_CSTA);

    writer.writeCellId(cellState.mId);
    cellState.save(writer);
    cell.writeFog(writer);
    cell.writeReferences(writer);
    writer.endRecord(ESM::REC_CSTA);
}

MWWorld::WorldModel::WorldModel(MWWorld::ESMStore& store, ESM::ReadersCache& readers)
    : mStore(store)
    , mReaders(readers)
    , mIdCache(Settings::cells().mPointersCacheSize, { ESM::RefId(), nullptr })
{
    mDraftCell.mId = draftCellId;
}

namespace MWWorld
{
    void WorldModel::vitaLoadWithState(CellStore& store) const
    {
        store.load();
#ifdef __vita__
        // State follows identity: every materialization path yields the
        // same cell regardless of eviction history.
        const_cast<WorldModel*>(this)->vitaApplyEvictedState(store.getCell()->getId());
#endif
    }

    CellStore& WorldModel::getExterior(ESM::ExteriorCellLocation location, bool forceLoad) const
    {
        CellStore* cellStore = getOrCreateExterior(location, mExteriors, mStore, mReaders, mCells, true);

        if (forceLoad && cellStore->getState() != CellStore::State_Loaded)
            vitaLoadWithState(*cellStore);

        return *cellStore;
    }

    CellStore* WorldModel::findInterior(std::string_view name, bool forceLoad) const
    {
        const auto it = mInteriors.find(name);
        CellStore* cellStore = nullptr;

        if (it == mInteriors.end())
        {
            cellStore = emplaceInteriorCellStore(name, mStore, mReaders, mCells);
            if (cellStore == nullptr)
                return cellStore;
            mInteriors.emplace(name, cellStore);
        }
        else
        {
            assert(it->second != nullptr);
            cellStore = it->second;
        }

        if (forceLoad && cellStore->getState() != CellStore::State_Loaded)
            vitaLoadWithState(*cellStore);

        return cellStore;
    }

    CellStore& WorldModel::getInterior(std::string_view name, bool forceLoad) const
    {
        CellStore* const cellStore = findInterior(name, forceLoad);
        if (cellStore == nullptr)
            throw std::runtime_error("Interior cell is not found: '" + std::string(name) + "'");
        return *cellStore;
    }

    CellStore* WorldModel::findCell(ESM::RefId id, bool forceLoad) const
    {
        auto it = mCells.find(id);
        if (it != mCells.end())
        {
            CellStore& cellStore = it->second;
            if (forceLoad && cellStore.getState() != CellStore::State_Loaded)
                vitaLoadWithState(cellStore);
            return &cellStore;
        }

        if (id == draftCellId)
        {
            CellStore& cellStore = emplaceCellStore(id, Cell(mDraftCell), mStore, mReaders, mCells);
            vitaLoadWithState(cellStore);
            return &cellStore;
        }

        if (const auto* exteriorId = id.getIf<ESM::ESM3ExteriorCellRefId>())
            return &getExterior(
                ESM::ExteriorCellLocation(exteriorId->getX(), exteriorId->getY(), ESM::Cell::sDefaultWorldspaceId),
                forceLoad);

        std::optional<Cell> cell = createCell(id, mStore);
        if (!cell.has_value())
            return nullptr;

        CellStore& cellStore = emplaceCellStore(id, std::move(*cell), mStore, mReaders, mCells);

        if (cellStore.isExterior())
            mExteriors.emplace(ESM::ExteriorCellLocation(cellStore.getCell()->getGridX(),
                                   cellStore.getCell()->getGridY(), cellStore.getCell()->getWorldSpace()),
                &cellStore);
        else
            mInteriors.emplace(cellStore.getCell()->getNameId(), &cellStore);

        if (forceLoad && cellStore.getState() != CellStore::State_Loaded)
            vitaLoadWithState(cellStore);

        return &cellStore;
    }

    CellStore& WorldModel::getCell(ESM::RefId id, bool forceLoad) const
    {
        CellStore* const result = findCell(id, forceLoad);
        if (result == nullptr)
            throw std::runtime_error("Cell does not exist: " + id.toDebugString());
        return *result;
    }

    CellStore& WorldModel::getDraftCell() const
    {
        return getCell(draftCellId);
    }

    CellStore* WorldModel::findCell(std::string_view name, bool forceLoad) const
    {
        if (CellStore* const cellStore = findInterior(name, forceLoad))
            return cellStore;

        // try named exteriors
        const ESM::Cell* cell = nullptr;
        const Store<ESM::Cell>& cells = mStore.get<ESM::Cell>();
        const Store<ESM::GameSetting>& gmsts = mStore.get<ESM::GameSetting>();
        const Store<ESM::Region>& regions = mStore.get<ESM::Region>();
        static const std::string& defaultName = gmsts.find("sDefaultCellname")->mValue.getString();

        for (auto it = cells.extBegin(); it != cells.extEnd(); ++it)
        {
            std::string_view resolvedName = defaultName;
            if (!it->mName.empty())
                resolvedName = it->mName;
            else if (!it->mRegion.empty())
            {
                const ESM::Region* region = regions.search(it->mRegion);
                if (region != nullptr)
                    resolvedName = !region->mName.empty() ? region->mName : region->mId.getRefIdString();
            }

            if (Misc::StringUtils::ciEqual(resolvedName, name))
            {
                cell = &(*it);
                break;
            }
        }

        if (cell != nullptr)
            return &getExterior(
                ESM::ExteriorCellLocation(cell->getGridX(), cell->getGridY(), ESM::Cell::sDefaultWorldspaceId),
                forceLoad);

        if (const ESM4::Cell* cell4 = mStore.get<ESM4::Cell>().searchCellName(name);
            cell4 != nullptr && cell4->isExterior())
        {
            return &getExterior(cell4->getExteriorCellLocation(), forceLoad);
        }

        return nullptr;
    }

    CellStore& WorldModel::getCell(std::string_view name, bool forceLoad) const
    {
        CellStore* const result = findCell(name, forceLoad);
        if (result == nullptr)
            throw std::runtime_error(std::string("Can't find cell with name ") + std::string(name));
        return *result;
    }

    void WorldModel::registerPtr(const Ptr& ptr)
    {
        if (ptr.mRef == nullptr)
            throw std::logic_error("Ptr with nullptr mRef is not allowed to be registered");
        mPtrRegistry.insert(ptr);
        ptr.mRef->mWorldModel = this;
    }

    void WorldModel::deregisterLiveCellRef(LiveCellRefBase& ref) noexcept
    {
        mPtrRegistry.remove(ref);
        ref.mWorldModel = nullptr;
    }
}

MWWorld::Ptr MWWorld::WorldModel::getPtrByRefId(const ESM::RefId& name)
{
    for (const auto& [cachedId, cellStore] : mIdCache)
    {
        if (cachedId != name || cellStore == nullptr)
            continue;
        Ptr ptr = cellStore->getPtr(name);
        if (!ptr.isEmpty())
            return ptr;
    }

    // Then check cells that are already listed
    // Search in reverse, this is a workaround for an ambiguous chargen_plank reference in the vanilla game.
    // there is one at -22,16 and one at -2,-9, the latter should be used.
    for (auto iter = mExteriors.rbegin(); iter != mExteriors.rend(); ++iter)
    {
        Ptr ptr = getPtrAndCache(name, *iter->second);
        if (!ptr.isEmpty())
            return ptr;
    }

    for (auto iter = mInteriors.begin(); iter != mInteriors.end(); ++iter)
    {
        Ptr ptr = getPtrAndCache(name, *iter->second);
        if (!ptr.isEmpty())
            return ptr;
    }

#ifdef __vita__
    // Baked object->cell index: jump straight to the owning cell instead of
    // materializing a store for every cell in the game (measured: 16s burst).
    {
        std::string spec;
        if (vitaObjectIndexLookup(name, spec) && spec.size() > 2)
        {
            CellStore* hinted = nullptr;
            if (spec[0] == 'E')
            {
                int gx = 0, gy = 0;
                if (sscanf(spec.c_str() + 2, "%d %d", &gx, &gy) == 2)
                    hinted = &getExterior(
                        ESM::ExteriorCellLocation(gx, gy, ESM::Cell::sDefaultWorldspaceId), false);
            }
            else if (spec[0] == 'I')
                hinted = findCell(std::string_view(spec).substr(2), false);
            if (hinted != nullptr)
            {
                vitaApplyEvictedState(hinted->getCell()->getId());
                Ptr ptr = getPtrAndCache(name, *hinted);
                if (!ptr.isEmpty())
                    return ptr;
            }
        }
    }
#endif

    // Now try the other cells
    const MWWorld::Store<ESM::Cell>& cells = mStore.get<ESM::Cell>();

    const Ptr found = [&] {
        for (auto iter = cells.extBegin(); iter != cells.extEnd(); ++iter)
        {
            if (mCells.contains(iter->mId))
                continue;

            Ptr ptr = getPtrAndCache(name, insertCellStore(*iter));

            if (!ptr.isEmpty())
                return ptr;
        }

        for (auto iter = cells.intBegin(); iter != cells.intEnd(); ++iter)
        {
            if (mCells.contains(iter->mId))
                continue;

            Ptr ptr = getPtrAndCache(name, insertCellStore(*iter));

            if (!ptr.isEmpty())
                return ptr;
        }

        // giving up
        return Ptr();
    }();

#ifdef __vita__
    // Sweep pins stores; the memory watchdog evicts them under pressure.
    {
        char buf[160];
        snprintf(buf, sizeof(buf), "[VitaAudit] getPtrByRefId('%s') found=%d, cellstores now %u",
            name.toDebugString().c_str(), found.isEmpty() ? 0 : 1, (unsigned)mCells.size());
        vitaMemBreadcrumb(buf);
    }
#endif

    return found;
}

#ifdef __vita__
std::size_t MWWorld::WorldModel::evictSweptCellStores(const std::set<CellStore*, std::less<>>& protectedCells)
{
    std::unordered_set<const CellStore*> toEvict;
    for (auto& [id, store] : mCells)
    {
        if (store.getState() == CellStore::State_Loaded || store.hasState())
            continue;
        toEvict.insert(&store);
    }
    for (CellStore* pc : protectedCells)
        toEvict.erase(pc);
    for (auto& entry : mIdCache)
        if (entry.second != nullptr && toEvict.count(entry.second) > 0)
            entry = { ESM::RefId(), nullptr };

    if (toEvict.empty())
        return 0;

    // Erase by pointer identity to keep the indices consistent.
    std::erase_if(mInteriors, [&](const auto& entry) { return toEvict.count(entry.second) > 0; });
    std::erase_if(mExteriors, [&](const auto& entry) { return toEvict.count(entry.second) > 0; });
    return std::erase_if(mCells, [&](auto& entry) { return toEvict.count(&entry.second) > 0; });
}

bool MWWorld::WorldModel::vitaEvictOneDistant(const std::set<CellStore*, std::less<>>& protectedCells,
    float playerX, float playerY, float keepNearUnits, const std::function<void(CellStore&)>& onEvict)
{
    const CellStore* victim = nullptr;
    bool victimNeedsState = false;
    int movedPinned = 0;
    for (int pass = 0; pass < 2 && victim == nullptr; ++pass)
    {
        for (const auto& [loc, store] : mExteriors)
        {
            if (store->getState() != CellStore::State_Loaded)
                continue;
            if (protectedCells.count(store) > 0)
                continue;
            std::string why;
            const bool safe = store->isSafeToEvict(&why);
            // movedRefs: partner stores hold raw pointers into our lists;
            // destruction would dangle them (crash in updateMergedRefs).
            if (pass == 1 && why == "movedRefs")
                ++movedPinned;
            if (pass == 0 ? !safe : (safe || why == "cellState" || why == "movedRefs"))
                continue;
            // Player-distance guard, same metric the radial streamer uses;
            // grid-center cell math lagged the player and over-protected.
            const float cs = static_cast<float>(ESM::getCellSize(loc.mWorldspace));
            const float ex = std::clamp(playerX, loc.mX * cs, (loc.mX + 1) * cs) - playerX;
            const float ey = std::clamp(playerY, loc.mY * cs, (loc.mY + 1) * cs) - playerY;
            if (ex * ex + ey * ey <= keepNearUnits * keepNearUnits)
                continue;
            victim = store;
            victimNeedsState = !safe;
            break;
        }
    }
    if (!victim)
    {
        if (movedPinned > 0)
        {
            static std::chrono::steady_clock::time_point sLast{};
            const auto now = std::chrono::steady_clock::now();
            if (now - sLast > std::chrono::seconds(30))
            {
                sLast = now;
                char buf[80];
                snprintf(buf, sizeof(buf), "[Evict] movedRefs pins %d stores", movedPinned);
                Vita::breadcrumb(buf);
            }
        }
        return false;
    }
    CellStore* mut = const_cast<CellStore*>(victim);
    if (victimNeedsState)
    {
        if (!vitaSerializeToLedger(*mut))
            return false; // ledger unavailable: keep the store resident
    }
    if (onEvict)
        onEvict(*mut);
    vitaInvalidateIdCache(victim);
    std::erase_if(mExteriors, [&](const auto& entry) { return entry.second == victim; });
    std::erase_if(mCells, [&](auto& entry) { return &entry.second == victim; });
    return true;
}

void MWWorld::WorldModel::vitaApplyBuffer(const std::string& data)
{
    // Nested rehydrate must never demote: outer frames hold pointers.
    struct DepthGuard
    {
        int& mDepth;
        DepthGuard(int& d)
            : mDepth(d)
        {
            ++mDepth;
        }
        ~DepthGuard() { --mDepth; }
    } depthGuard{ mVitaApplyDepth };
    ESM::ESMReader reader;
    reader.open(std::make_unique<std::istringstream>(data), "vita-evicted-state");
    while (reader.hasMoreRecs())
    {
        const ESM::NAME n = reader.getRecName();
        reader.getRecHeader();
        readRecord(reader, n.toInt());
    }
}

bool MWWorld::WorldModel::vitaLedgerEnsure() const
{
    if (mVitaLedgerFile)
        return true;
    mVitaLedgerFile = fopen("ux0:data/openmw/cache/session_ledger.bin", "w+b");
    if (mVitaLedgerFile)
        setvbuf(mVitaLedgerFile, nullptr, _IONBF, 0); // buffering wastes seeky reads
    return mVitaLedgerFile != nullptr;
}

bool MWWorld::WorldModel::vitaLedgerAppend(const std::string& data, std::size_t skip, VitaLedgerSpan& out)
{
    if (data.size() <= skip || !vitaLedgerEnsure())
        return false;
    const std::size_t len = data.size() - skip;
    if (fseek(mVitaLedgerFile, (long)mVitaLedgerEnd, SEEK_SET) != 0)
        return false;
    if (fwrite(data.data() + skip, 1, len, mVitaLedgerFile) != len)
        return false;
    fflush(mVitaLedgerFile);
    out.mOff = mVitaLedgerEnd;
    out.mLen = (uint32_t)len;
    mVitaLedgerEnd += len;
    return true;
}

bool MWWorld::WorldModel::vitaLedgerReadSpan(const VitaLedgerSpan& span, std::string& out) const
{
    if (!vitaLedgerEnsure())
        return false;
    out.resize(span.mLen);
    if (fseek(mVitaLedgerFile, (long)span.mOff, SEEK_SET) != 0)
        return false;
    return fread(out.data(), 1, span.mLen, mVitaLedgerFile) == span.mLen;
}

void MWWorld::WorldModel::vitaLedgerHeaderEnsure()
{
    if (!mVitaLedgerHeader.empty())
        return;
    // Fixed length per config; readers ignore the count field.
    std::stringstream hs;
    ESM::ESMWriter hw;
    hw.setFormatVersion(ESM::CurrentSaveGameFormatVersion);
    hw.save(hs);
    hw.close();
    mVitaLedgerHeader = std::move(hs).str();
}

bool MWWorld::WorldModel::vitaSerializeToLedger(CellStore& store)
{
    std::stringstream ss;
    ESM::ESMWriter writer;
    writer.setFormatVersion(ESM::CurrentSaveGameFormatVersion);
    writer.save(ss);
    writeCell(writer, store);
    writer.close();
    std::string data = std::move(ss).str();
    vitaLedgerHeaderEnsure();
    VitaLedgerSpan span;
    if (!vitaLedgerAppend(data, mVitaLedgerHeader.size(), span))
        return false;
    mVitaEvictedState[store.getCell()->getId()] = span;
    return true;
}

bool MWWorld::WorldModel::vitaSpanHasActors(const std::string& span)
{
    // Flat name4+size4 walk; malformed reads as "has actors".
    if (span.size() < 16)
        return true;
    std::size_t off = 16;
    while (off + 8 <= span.size())
    {
        const char* name = span.data() + off;
        uint32_t size;
        std::memcpy(&size, span.data() + off + 4, 4);
        if (off + 8 + size > span.size())
            return true; // malformed: be conservative
        if (std::memcmp(name, "ACID", 4) == 0)
            return true;
        off += 8 + size;
    }
    return off != span.size();
}

bool MWWorld::WorldModel::vitaApplyEvictedState(const ESM::RefId& id)
{
    const auto it = mVitaEvictedState.find(id);
    if (it == mVitaEvictedState.end())
        return false;
    const VitaLedgerSpan span = it->second;
    mVitaEvictedState.erase(it);
    std::string payload;
    if (!vitaLedgerReadSpan(span, payload))
    {
        char lb[96];
        snprintf(lb, sizeof(lb), "[Ledger] READ FAIL %s", id.toDebugString().c_str());
        Vita::breadcrumb(lb);
        return false;
    }
    std::string data;
    data.reserve(mVitaLedgerHeader.size() + payload.size());
    data += mVitaLedgerHeader;
    data += payload;
    const auto sa0 = std::chrono::steady_clock::now();
    vitaApplyBuffer(data);
    const int saMs = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - sa0)
                         .count();
    if (saMs > 20)
    {
        char sb[112];
        snprintf(sb, sizeof(sb), "[StateApply] %s %dms", id.toDebugString().c_str(), saMs);
        Vita::breadcrumb(sb);
    }
    return true;
}

std::size_t MWWorld::WorldModel::vitaEvictInteriors(const std::set<CellStore*, std::less<>>& protectedCells,
    std::size_t maxCount, const std::function<void(CellStore&)>& onEvict)
{
    std::size_t evicted = 0;
    for (auto it = mInteriors.begin(); it != mInteriors.end() && evicted < maxCount;)
    {
        CellStore* store = it->second;
        if (store->getState() != CellStore::State_Loaded || protectedCells.count(store) > 0)
        {
            ++it;
            continue;
        }
        std::string why;
        const bool safe = store->isSafeToEvict(&why);
        if (!safe && (why == "cellState" || why == "movedRefs"))
        {
            ++it;
            continue;
        }
        if (!safe)
        {
            if (!vitaSerializeToLedger(*store))
            {
                ++it;
                continue; // ledger unavailable: keep the store resident
            }
        }
        if (onEvict)
            onEvict(*store);
        vitaInvalidateIdCache(store);
        it = mInteriors.erase(it);
        std::erase_if(mCells, [&](auto& e) { return &e.second == store; });
        ++evicted;
    }
    return evicted;
}

std::size_t MWWorld::WorldModel::evictInactiveLoadedCellStores(
    const std::set<CellStore*, std::less<>>& activeCells, const std::function<void(CellStore&)>& onEvict)
{
    std::unordered_set<const CellStore*> toEvict;
    for (auto& [id, store] : mCells)
    {
        if (activeCells.count(&store) > 0)
            continue;
        // PtrRegistry self-cleans via ~LiveCellRefBase.
        if (!store.isSafeToEvict())
            continue;
        toEvict.insert(&store);
    }
    for (auto& entry : mIdCache)
        if (entry.second != nullptr && toEvict.count(entry.second) > 0)
            entry = { ESM::RefId(), nullptr };

    if (toEvict.empty())
        return 0;

    // Engine registries drop their Ptrs before the refs die.
    if (onEvict)
        for (auto& [id, store] : mCells)
            if (toEvict.count(&store) > 0)
                onEvict(store);

    std::erase_if(mInteriors, [&](const auto& entry) { return toEvict.count(entry.second) > 0; });
    std::erase_if(mExteriors, [&](const auto& entry) { return toEvict.count(entry.second) > 0; });
    return std::erase_if(mCells, [&](auto& entry) { return toEvict.count(&entry.second) > 0; });
}

#endif

void MWWorld::WorldModel::getExteriorPtrs(const ESM::RefId& name, std::vector<MWWorld::Ptr>& out)
{
    const MWWorld::Store<ESM::Cell>& cells = mStore.get<ESM::Cell>();
    for (MWWorld::Store<ESM::Cell>::iterator iter = cells.extBegin(); iter != cells.extEnd(); ++iter)
    {
        CellStore& cellStore = getOrInsertCellStore(*iter);

        Ptr ptr = getPtrAndCache(name, cellStore);

        if (!ptr.isEmpty())
            out.push_back(ptr);
    }

#ifdef __vita__
    // Full-world sweep; eviction deferred to the watchdog.
    {
        char buf[160];
        snprintf(buf, sizeof(buf), "[VitaAudit] getExteriorPtrs('%s') matches=%u, cellstores now %u",
            name.toDebugString().c_str(), (unsigned)out.size(), (unsigned)mCells.size());
        vitaMemBreadcrumb(buf);
    }
#endif
}

std::vector<MWWorld::Ptr> MWWorld::WorldModel::getAll(const ESM::RefId& id)
{
    std::vector<Ptr> result;
    for (auto& [cellId, cellStore] : mCells)
    {
        if (cellStore.getState() == CellStore::State_Unloaded)
            cellStore.preload();
        if (cellStore.getState() == CellStore::State_Preloaded)
        {
            if (!cellStore.hasId(id))
                continue;
            vitaLoadWithState(cellStore);
        }
        cellStore.forEach([&](const Ptr& ptr) {
            if (ptr.getCellRef().getRefId() == id)
                result.push_back(ptr);
            return true;
        });
    }
    return result;
}

size_t MWWorld::WorldModel::countSavedGameRecords() const
{
    std::size_t n = std::count_if(mCells.begin(), mCells.end(), [](const auto& v) { return v.second.hasState(); });
#ifdef __vita__
    n += mVitaEvictedState.size();
#endif
    return n;
}

void MWWorld::WorldModel::write(ESM::ESMWriter& writer, Loading::Listener& progress) const
{
    for (auto& [id, cellStore] : mCells)
        if (cellStore.hasState())
        {
            writeCell(writer, cellStore);
            progress.increaseProgress();
        }
#ifdef __vita__
    // Splice evicted cells verbatim, offset-ordered; never rehydrate.
    vitaMainPhase("sv_splice");
    std::vector<std::pair<VitaLedgerSpan, ESM::RefId>> spliceOrder;
    spliceOrder.reserve(mVitaEvictedState.size());
    for (const auto& [id, span] : mVitaEvictedState)
        spliceOrder.push_back({ span, id });
    std::sort(spliceOrder.begin(), spliceOrder.end(),
        [](const auto& a, const auto& b) { return a.first.mOff < b.first.mOff; });
    std::string payload;
    unsigned spliced = 0;
    for (const auto& [span, id] : spliceOrder)
    {
        if (!vitaLedgerReadSpan(span, payload))
        {
            char wb[96];
            snprintf(wb, sizeof(wb), "[SaveSplice] ledger read fail %s", id.toDebugString().c_str());
            Vita::breadcrumb(wb);
            continue;
        }
        writer.write(payload.data(), payload.size());
        if ((++spliced & 31) == 0)
            progress.increaseProgress(32); // each tick may render a frame
    }
#endif
}

struct MWWorld::WorldModel::GetCellStoreCallback : public CellStore::GetCellStoreCallback
{
public:
    GetCellStoreCallback(WorldModel& worldModel)
        : mWorldModel(worldModel)
    {
    }

    WorldModel& mWorldModel;

    CellStore* getCellStore(const ESM::RefId& cellId) override
    {
        if (const auto* exteriorId = cellId.getIf<ESM::ESM3ExteriorCellRefId>())
        {
            ESM::ExteriorCellLocation location(exteriorId->getX(), exteriorId->getY(), ESM::Cell::sDefaultWorldspaceId);
            return getOrCreateExterior(
                location, mWorldModel.mExteriors, mWorldModel.mStore, mWorldModel.mReaders, mWorldModel.mCells, false);
        }
        return mWorldModel.findCell(cellId);
    }
};

bool MWWorld::WorldModel::readRecord(ESM::ESMReader& reader, uint32_t type)
{
    if (type == ESM::REC_CSTA)
    {
#ifdef __vita__
        // Actor-free: raw span to ledger. Actor-bearing: parse from span.
        if (mVitaLoadDemote && mVitaApplyDepth == 0 && !mVitaLoadFilePath.empty())
        {
            const std::size_t spanStart = mVitaRecStart;
            reader.skipRecord();
            const std::size_t spanEnd = (std::size_t)reader.getFileOffset();
            std::string span;
            bool ok = spanEnd > spanStart;
            if (ok)
            {
                span.resize(spanEnd - spanStart);
                if (!mVitaLoadFile)
                {
                    mVitaLoadFile = fopen(mVitaLoadFilePath.c_str(), "rb");
                    if (mVitaLoadFile)
                        setvbuf(mVitaLoadFile, nullptr, _IONBF, 0);
                }
                ok = mVitaLoadFile && fseek(mVitaLoadFile, (long)spanStart, SEEK_SET) == 0
                    && fread(span.data(), 1, span.size(), mVitaLoadFile) == span.size();
            }
            if (!ok)
            {
                char eb[112];
                snprintf(eb, sizeof(eb), "[SpanLoad] SIDE-READ FAIL off=%u len=%u", (unsigned)spanStart,
                    (unsigned)(spanEnd - spanStart));
                Vita::breadcrumb(eb);
                return true; // loud loss beats silent corruption
            }
            vitaLedgerHeaderEnsure();
            if (!vitaSpanHasActors(span))
            {
                ESM::RefId spanId;
                try
                {
                    std::string buf;
                    buf.reserve(mVitaLedgerHeader.size() + span.size());
                    buf += mVitaLedgerHeader;
                    buf += span;
                    ESM::ESMReader sr;
                    sr.open(std::make_unique<std::istringstream>(std::move(buf)), "vita-span-id");
                    const ESM::NAME n = sr.getRecName();
                    sr.getRecHeader();
                    if (n.toInt() == ESM::REC_CSTA)
                        spanId = sr.getCellId();
                }
                catch (const std::exception&)
                {
                    spanId = ESM::RefId();
                }
                VitaLedgerSpan ls;
                if (!spanId.empty() && vitaLedgerAppend(span, 0, ls))
                {
                    mVitaEvictedState[spanId] = ls;
                    return true;
                }
                // fall through to span-parse on any anomaly
            }
            try
            {
                std::string buf;
                buf.reserve(mVitaLedgerHeader.size() + span.size());
                buf += mVitaLedgerHeader;
                buf += span;
                ESM::ESMReader sr;
                sr.open(std::make_unique<std::istringstream>(std::move(buf)), "vita-span-parse");
                sr.mActorIdConverter = reader.mActorIdConverter;
                const ESM::NAME n = sr.getRecName();
                sr.getRecHeader();
                if (n.toInt() == ESM::REC_CSTA)
                    readCellRecordBody(sr);
            }
            catch (const std::exception& e)
            {
                char eb[128];
                snprintf(eb, sizeof(eb), "[SpanLoad] span-parse fail: %.90s", e.what());
                Vita::breadcrumb(eb);
            }
            return true;
        }
#endif
        return readCellRecordBody(reader);
    }

    return false;
}

bool MWWorld::WorldModel::readCellRecordBody(ESM::ESMReader& reader)
{
    ESM::CellState state;
    state.mId = reader.getCellId();

        GetCellStoreCallback callback(*this);

        CellStore* const cellStore = callback.getCellStore(state.mId);

        if (cellStore == nullptr)
        {
            Log(Debug::Warning) << "Dropping state for cell " << state.mId << " (cell no longer exists)";
            reader.skipRecord();
            return true;
        }

#ifdef __vita__
        // Converter entries reference this store's parsed data.
        ESM::ActorIdConverter* const vitaConv = reader.mActorIdConverter;
        const std::size_t vitaPend0 = vitaConv ? vitaConv->pendingCount() : 0;
#endif
        state.load(reader);
        cellStore->loadState(state);

        if (state.mHasFogOfWar)
            cellStore->readFog(reader);

        if (cellStore->getState() != CellStore::State_Loaded)
            vitaLoadWithState(*cellStore);

        cellStore->readReferences(reader, &callback);

#ifdef __vita__
        // Demote after parse; same guards as the interior evictor.
        if (mVitaLoadDemote && mVitaApplyDepth == 0)
        {
            std::string why;
            const bool safe = cellStore->isSafeToEvict(&why);
            bool pinned = false;
            if (!safe && (why == "cellState" || why == "movedRefs"))
                pinned = true;
            // Forward actor refs pin the store until apply().
            if (!pinned && vitaConv && !vitaConv->resolveFrom(vitaPend0))
                pinned = true;
            if (!pinned)
                vitaInvalidateIdCache(cellStore);
            if (!pinned && vitaSerializeToLedger(*cellStore))
            {
                std::erase_if(mInteriors, [&](const auto& e) { return e.second == cellStore; });
                std::erase_if(mExteriors, [&](const auto& e) { return e.second == cellStore; });
                std::erase_if(mCells, [&](auto& e) { return &e.second == cellStore; });
            }
        }
#endif

        return true;
}
