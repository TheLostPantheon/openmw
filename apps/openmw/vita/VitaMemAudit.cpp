#ifdef __vita__

#include "VitaMemAudit.h"
#include "VitaInit.h"
#include "VitaSimWorker.h"

#include <cstdio>
#include <cstring>
#include <map>
#include <string>

#include <psp2/kernel/clib.h>
#include <psp2/kernel/processmgr.h>

#include <vitaGL.h>

#include <osg/BufferObject>
#include <osg/Texture>

extern "C"
{
    extern uint32_t osgprof_mat_us, osgprof_state_us, osgprof_unif_us, osgprof_draw_us, osgprof_leaves;
    extern uint32_t osgprof_replayable, osgprof_replayed, osgprof_streplayed;
    extern unsigned int cullprof_node, cullprof_group, cullprof_transform, cullprof_geode, cullprof_drawable,
        cullprof_dcull, cullprof_leaves, cullprof_sg, cullprof_xf_bone;
    extern unsigned int cullprof_drw_us, cullprof_cb_us, cullprof_xf_us, cullprof_grp_us;
    extern uint32_t vita_sim_script_us, vita_sim_mech_us, vita_sim_phys_us, vita_sim_gscript_us;
    int cullprof_cb_report(char* buf, unsigned int buflen);
    int vita_script_hist_report(char* buf, unsigned int buflen);
    extern uint32_t vgl_memo_hits, vgl_memo_miss;
    extern uint32_t vgl_vprog_hits, vgl_vprog_miss;
    extern uint32_t vgl_swap_block_us, vgl_swap_block_max, vgl_qdepth_sum, vgl_qdepth_max, vgl_gpu_frames;
    unsigned int terr_chunks = 0, terr_pass_leaves = 0;
    uint32_t phase_evt_us = 0, phase_upd_us = 0, phase_focus_us = 0, phase_lua_us = 0;
    uint32_t phase_pre_us = 0, phase_pace_us = 0;
    uint32_t phase_fin_us = 0, phase_inp_us = 0, phase_unref_us = 0, phase_stats_us = 0;
    uint32_t phase_snd_us = 0, phase_lsync_us = 0, phase_state_us = 0;
    uint32_t phase_world_us = 0, phase_wm_us = 0;
    unsigned int vita_bin2_graphs = 0, vita_bin2_leaves = 0;
    uint32_t gl_draw_us = 0, gl_swap_us = 0, gl_draw_max = 0, gl_swap_max = 0;
    extern uint32_t osgprof_dyn_leaves, osgprof_dyn_verts, osgprof_dyn_us;
    unsigned int rig_cull_count = 0, rig_cull_verts = 0;
    unsigned int cullprof_creplay = 0, cullprof_crep_drop = 0;
    uint32_t cullprof_terr_us = 0;
    extern uint32_t osgapply_calls, osgapply_tex_us, osgapply_mode_us, osgapply_attr_us, osgapply_unif_us;
    extern uint32_t osgapply_unif_n, osgapply_unif_up;
    extern uint32_t osgapply_unif_rup;
    extern uint32_t osgapply_push, osgapply_pop;
    int osgapply_unif_hist_report(char* buf, unsigned int buflen);
}

namespace MyGUIPlatform
{
    extern uint32_t g_vitaGuiWalks;
    extern uint32_t g_vitaGuiSkips;
}

#include <cstdint>

#include <osg/Camera>
#include <osg/Geometry>
#include <osg/Image>
#include <osg/NodeVisitor>
#include <osg/Stats>
#include <osgViewer/Viewer>

#include <components/esm3/loaddial.hpp>
#include <components/esm3/loadinfo.hpp>
#include <components/esm3/loadland.hpp>
#include <components/esm3/loadscpt.hpp>
#include <components/resource/bulletshapemanager.hpp>
#include <components/resource/imagemanager.hpp>
#include <components/resource/keyframemanager.hpp>
#include <components/resource/niffilemanager.hpp>
#include <components/resource/objectcache.hpp>
#include <components/resource/resourcesystem.hpp>
#include <components/resource/scenemanager.hpp>
#include <components/sceneutil/lightmanager.hpp>

#include "../mwworld/cellstore.hpp"
#include "../mwworld/esmstore.hpp"
#include "../mwworld/worldmodel.hpp"

namespace
{
    unsigned long long gLastRenderUs = 0;

    void auditLog(const char* buf)
    {
        sceClibPrintf("%s\n", buf);
        vitaMemBreadcrumb(buf);
    }

    // String heap bytes beyond SSO (newlib SSO = 15).
    size_t strHeapBytes(const std::string& s)
    {
        return s.capacity() > 15 ? s.capacity() + 1 : 0;
    }

    // malloc + std::list node overhead.
    constexpr size_t kListNodeOverhead = 2 * sizeof(void*) + 8;

    size_t dialInfoBytes(const ESM::DialInfo& info)
    {
        size_t bytes = sizeof(ESM::DialInfo) + kListNodeOverhead;
        bytes += strHeapBytes(info.mSound);
        bytes += strHeapBytes(info.mResponse);
        bytes += strHeapBytes(info.mResultScript);
        bytes += info.mSelects.capacity() * sizeof(ESM::DialogueCondition);
        for (const auto& select : info.mSelects)
            bytes += strHeapBytes(select.mVariable);
        return bytes;
    }

    // Geometry bytes under a node; textures counted by the image audit.
    class GeometryBytesVisitor : public osg::NodeVisitor
    {
    public:
        GeometryBytesVisitor()
            : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
        {
        }

        void apply(osg::Node& node) override
        {
            ++mNodes;
            traverse(node);
        }

        void apply(osg::Drawable& drawable) override
        {
            ++mDrawables;
            if (const osg::Geometry* geom = drawable.asGeometry())
            {
                addArray(geom->getVertexArray());
                addArray(geom->getNormalArray());
                addArray(geom->getColorArray());
                addArray(geom->getSecondaryColorArray());
                addArray(geom->getFogCoordArray());
                for (const auto& tc : geom->getTexCoordArrayList())
                    addArray(tc.get());
                for (const auto& va : geom->getVertexAttribArrayList())
                    addArray(va.get());
                for (const auto& ps : geom->getPrimitiveSetList())
                    if (ps)
                        mBytes += ps->getTotalDataSize();
            }
            traverse(drawable);
        }

        size_t mBytes = 0;
        size_t mNodes = 0;
        size_t mDrawables = 0;

    private:
        void addArray(const osg::Array* array)
        {
            if (array)
                mBytes += array->getTotalDataSize();
        }
    };
}

namespace Vita
{
    void auditDialogueStore(const MWWorld::ESMStore& store)
    {
        const auto& dialogues = store.get<ESM::Dialogue>();

        size_t topics = 0;
        size_t infos = 0;
        size_t liveBytes = 0; // Dialogue::mInfo — the list actually used at runtime
        size_t dupBytes = 0; // InfoOrder::mOrderedInfo — load-order copy never freed

        for (auto it = dialogues.begin(); it != dialogues.end(); ++it)
        {
            const ESM::Dialogue& dial = *it;
            ++topics;
            liveBytes += sizeof(ESM::Dialogue) + strHeapBytes(dial.mStringId);
            for (const ESM::DialInfo& info : dial.mInfo)
            {
                ++infos;
                liveBytes += dialInfoBytes(info);
            }
            for (const ESM::DialInfo& info : dial.mInfoOrder.getOrderedInfo())
                dupBytes += dialInfoBytes(info);
        }

        char buf[256];
        snprintf(buf, sizeof(buf),
            "[VitaAudit] dialogue: %u topics, %u infos, live=%uKB, infoOrderDup=%uKB, total=%uMB",
            (unsigned)topics, (unsigned)infos, (unsigned)(liveBytes / 1024), (unsigned)(dupBytes / 1024),
            (unsigned)((liveBytes + dupBytes) / (1024 * 1024)));
        auditLog(buf);

        // OpenMW recompiles from source; vanilla bytecode is dead weight.
        const auto& scripts = store.get<ESM::Script>();
        size_t scriptCount = 0;
        size_t textBytes = 0;
        size_t vanillaBytes = 0;
        size_t varBytes = 0;
        for (auto it = scripts.begin(); it != scripts.end(); ++it)
        {
            ++scriptCount;
            textBytes += strHeapBytes(it->mScriptText);
            vanillaBytes += it->mScriptData.capacity();
            for (const auto& var : it->mVarNames)
                varBytes += sizeof(std::string) + strHeapBytes(var);
        }
        snprintf(buf, sizeof(buf),
            "[VitaAudit] scripts: %u records, sourceText=%uKB, vanillaBytecode=%uKB (unused), varNames=%uKB",
            (unsigned)scriptCount, (unsigned)(textBytes / 1024), (unsigned)(vanillaBytes / 1024),
            (unsigned)(varBytes / 1024));
        auditLog(buf);

        snprintf(buf, sizeof(buf), "[VitaAudit] store counts: cells=%u, lands=%u",
            (unsigned)store.get<ESM::Cell>().getSize(), (unsigned)store.get<ESM::Land>().getSize());
        auditLog(buf);
    }

    void auditWorldModel(MWWorld::WorldModel& worldModel)
    {
        size_t total = 0;
        size_t loaded = 0;
        size_t preloaded = 0;
        size_t refs = 0;

        worldModel.forEachLoadedCellStore([&](MWWorld::CellStore& cell) {
            ++total;
            switch (cell.getState())
            {
                case MWWorld::CellStore::State_Loaded:
                    ++loaded;
                    refs += cell.count();
                    break;
                case MWWorld::CellStore::State_Preloaded:
                    ++preloaded;
                    break;
                default:
                    break;
            }
        });

        char buf[224];
        snprintf(buf, sizeof(buf),
            "[VitaAudit] worldmodel: %u cellstores (%u loaded, %u preloaded), %u live refs resident",
            (unsigned)total, (unsigned)loaded, (unsigned)preloaded, (unsigned)refs);
        auditLog(buf);

        // Evictability breakdown: tally why stores are retained.
        size_t evictable = 0;
        std::map<std::string, size_t> blocked;
        std::string firstDetail;
        worldModel.forEachLoadedCellStore([&](MWWorld::CellStore& cell) {
            std::string why;
            if (cell.isSafeToEvict(&why))
            {
                ++evictable;
                return;
            }
            const std::string key = why.substr(0, why.find(':'));
            if (++blocked[key] == 1 && why.size() > key.size() && firstDetail.empty())
                firstDetail = why;
        });
        std::string summary = "[VitaAudit] evictable=" + std::to_string(evictable);
        for (const auto& [reason, count] : blocked)
            summary += " " + reason + "=" + std::to_string(count);
        if (!firstDetail.empty())
            summary += " e.g. " + firstDetail;
        auditLog(summary.c_str());
    }

    void auditResourceCaches(Resource::ResourceSystem* resourceSystem)
    {
        if (!resourceSystem)
            return;

        size_t imageCount = 0;
        size_t imageBytes = 0;
        resourceSystem->getImageManager()->getObjectCache()->call([&](const auto&, osg::Object* obj) {
            if (const osg::Image* image = dynamic_cast<osg::Image*>(obj))
            {
                ++imageCount;
                imageBytes += image->getTotalSizeInBytesIncludingMipmaps();
            }
        });

        size_t nodeCount = 0;
        GeometryBytesVisitor geomBytes;
        resourceSystem->getSceneManager()->getObjectCache()->call([&](const auto&, osg::Object* obj) {
            if (osg::Node* node = dynamic_cast<osg::Node*>(obj))
            {
                ++nodeCount;
                node->accept(geomBytes);
            }
        });

        size_t keyframeCount = 0;
        resourceSystem->getKeyframeManager()->getObjectCache()->call(
            [&](const auto&, osg::Object*) { ++keyframeCount; });

        size_t nifCount = 0;
        resourceSystem->getNifFileManager()->getObjectCache()->call([&](const auto&, osg::Object*) { ++nifCount; });

        char buf[256];
        snprintf(buf, sizeof(buf),
            "[VitaAudit] caches: images=%u (%uKB), sceneTemplates=%u (geom %uKB nodes=%u drw=%u), keyframes=%u, "
            "nifs=%u",
            (unsigned)imageCount, (unsigned)(imageBytes / 1024), (unsigned)nodeCount,
            (unsigned)(geomBytes.mBytes / 1024), (unsigned)geomBytes.mNodes, (unsigned)geomBytes.mDrawables,
            (unsigned)keyframeCount, (unsigned)nifCount);
        auditLog(buf);

        // The unaudited majors: vitaGL pool occupancy + newlib-side use.
        const auto used = [](vglMemType t) {
            const size_t total = vglMemTotal(t);
            return total > 0 ? total - vglMemFree(t) : (size_t)0;
        };
        snprintf(buf, sizeof(buf), "[VitaAudit] vgl: ram=%u/%uMB vram=%u/%uMB slow=%u/%uMB ext=%uMB",
            (unsigned)(used(VGL_MEM_RAM) >> 20), (unsigned)(vglMemTotal(VGL_MEM_RAM) >> 20),
            (unsigned)(used(VGL_MEM_VRAM) >> 20), (unsigned)(vglMemTotal(VGL_MEM_VRAM) >> 20),
            (unsigned)(used(VGL_MEM_SLOW) >> 20), (unsigned)(vglMemTotal(VGL_MEM_SLOW) >> 20),
            (unsigned)(used(VGL_MEM_EXTERNAL) >> 20));
        auditLog(buf);

    }

    void auditBulletShapes(Resource::BulletShapeManager* shapes)
    {
        if (shapes == nullptr)
            return;
        size_t n = 0;
        shapes->getObjectCache()->call([&](const auto&, osg::Object*) { ++n; });
        char buf[96];
        snprintf(buf, sizeof(buf), "[VitaAudit] bulletShapes=%u", (unsigned)n);
        auditLog(buf);
    }

    namespace
    {
        uint64_t s_renderUsAccum = 0;
        unsigned s_renderSamples = 0;

        // Engine frame numbers lag the viewer's; scan back.
        double msOf(const osg::Stats* stats, const char* name)
        {
            double v = 0.0;
            if (stats)
            {
                const unsigned int latest = stats->getLatestFrameNumber();
                const unsigned int earliest = stats->getEarliestFrameNumber();
                for (unsigned int f = latest;; --f)
                {
                    if (stats->getAttribute(f, name, v))
                        break;
                    if (f == earliest || f == 0)
                        break;
                }
            }
            return v * 1000.0;
        }

        double countOf(const osg::Stats* stats, const char* name)
        {
            double v = 0.0;
            if (stats)
                stats->getAttribute(stats->getLatestFrameNumber() > 0 ? stats->getLatestFrameNumber() - 1 : 0, name, v);
            return v;
        }
    }

    void auditFrameStats(osgViewer::Viewer& viewer)
    {
        constexpr int kReportEveryFrames = 150; // ~5s at 30fps
        static int s_frames = 0;
        static uint64_t s_lastReportUs = 0;
        static bool s_enabled = false;

        osg::Stats* viewerStats = viewer.getViewerStats();
        osg::Stats* camStats = viewer.getCamera() ? viewer.getCamera()->getStats() : nullptr;
        if (!s_enabled && viewerStats && camStats)
        {
            viewerStats->collectStats("engine", true);
            viewerStats->collectStats("update", true);
            camStats->collectStats("rendering", true);
            camStats->collectStats("scene", true);
            s_enabled = true;
        }
        if (!s_enabled)
            return;

        static uint64_t s_prevFrameUs = 0;
        static unsigned long long s_prevWaitUs = 0;
        static double s_worstMs = 0, s_worstRender = 0, s_worstDraw = 0, s_worstCull = 0, s_worstWait = 0;
        static double s_worstEvt = 0, s_worstUpd = 0, s_worstFocus = 0, s_worstLua = 0;
        static double s_worstPre = 0, s_worstPace = 0;
        static double s_worstFin = 0, s_worstInp = 0, s_worstUnref = 0, s_worstStats = 0;
        static double s_worstSnd = 0, s_worstLsync = 0, s_worstState = 0;
        static double s_worstWorld = 0, s_worstWm = 0;
        static uint64_t s_sumEvt = 0, s_sumUpd = 0, s_sumFoc = 0, s_sumLua = 0, s_sumPre = 0, s_sumPace = 0,
            s_sumRnd = 0;
        static unsigned s_over40 = 0;
        {
            const uint64_t nowF = sceKernelGetProcessTimeWide();
            if (s_prevFrameUs)
            {
                const double dtMs = (nowF - s_prevFrameUs) / 1000.0;
                const double waitMs = (Vita::gWorkerWaitUs - s_prevWaitUs) / 1000.0;
                if (dtMs > 40.0)
                    ++s_over40;
                if (dtMs > s_worstMs)
                {
                    s_worstMs = dtMs;
                    s_worstRender = gLastRenderUs / 1000.0;
                    s_worstDraw = msOf(camStats, "Draw traversal time taken");
                    s_worstCull = msOf(camStats, "Cull traversal time taken");
                    s_worstWait = waitMs;
                    s_worstEvt = phase_evt_us / 1000.0;
                    s_worstUpd = phase_upd_us / 1000.0;
                    s_worstFocus = phase_focus_us / 1000.0;
                    s_worstLua = phase_lua_us / 1000.0;
                    s_worstPre = phase_pre_us / 1000.0;
                    s_worstPace = phase_pace_us / 1000.0;
                    s_worstFin = phase_fin_us / 1000.0;
                    s_worstInp = phase_inp_us / 1000.0;
                    s_worstUnref = phase_unref_us / 1000.0;
                    s_worstStats = phase_stats_us / 1000.0;
                    s_worstSnd = phase_snd_us / 1000.0;
                    s_worstLsync = phase_lsync_us / 1000.0;
                    s_worstState = phase_state_us / 1000.0;
                    s_worstWorld = phase_world_us / 1000.0;
                    s_worstWm = phase_wm_us / 1000.0;
                }
            }
            s_prevFrameUs = nowF;
            s_prevWaitUs = Vita::gWorkerWaitUs;
            s_sumEvt += phase_evt_us;
            s_sumUpd += phase_upd_us;
            s_sumFoc += phase_focus_us;
            s_sumLua += phase_lua_us;
            s_sumPre += phase_pre_us;
            s_sumPace += phase_pace_us;
            s_sumRnd += (uint32_t)gLastRenderUs;
        }

        if (++s_frames < kReportEveryFrames)
            return;
        const uint64_t nowUs = sceKernelGetProcessTimeWide();
        const double frameMs
            = s_lastReportUs ? (nowUs - s_lastReportUs) / 1000.0 / static_cast<double>(s_frames) : 0.0;
        s_lastReportUs = nowUs;
        s_frames = 0;

        const double renderMs
            = s_renderSamples ? s_renderUsAccum / 1000.0 / static_cast<double>(s_renderSamples) : 0.0;
        s_renderUsAccum = 0;
        s_renderSamples = 0;

        const unsigned vramFreeMB = static_cast<unsigned>(vglMemFree(VGL_MEM_VRAM) >> 20);
        const unsigned vramTotalMB = static_cast<unsigned>(vglMemTotal(VGL_MEM_VRAM) >> 20);

        char buf[256];
        snprintf(buf, sizeof(buf),
            "[Frame] avg=%.1fms render=%.1f (cull=%.1f draw=%.1f) update=%.1f wait=%.1f | mech=%.1f phys=%.1f "
            "world=%.1f gui=%.1f lua=%.1f script=%.1f input=%.1f sound=%.1f | vram=%u/%uMB free",
            frameMs, renderMs, msOf(camStats, "Cull traversal time taken"),
            msOf(camStats, "Draw traversal time taken"), msOf(viewerStats, "Update traversal time taken"),
            Vita::gWorkerWaitUs / 1000.0 / kReportEveryFrames,
            msOf(viewerStats, "mechanics_time_taken"), msOf(viewerStats, "physics_time_taken"),
            msOf(viewerStats, "world_time_taken"), msOf(viewerStats, "gui_time_taken"),
            msOf(viewerStats, "lua_time_taken"), msOf(viewerStats, "script_time_taken"),
            msOf(viewerStats, "input_time_taken"), msOf(viewerStats, "sound_time_taken"), vramFreeMB, vramTotalMB);
        auditLog(buf);
        Vita::gWorkerWaitUs = 0;
        snprintf(buf, sizeof(buf),
            "[Worst] dt=%.0fms rnd=%.1f wt=%.1f evt=%.1f upd=%.1f foc=%.1f lua=%.1f pre=%.1f "
            "(inp=%.1f snd=%.1f lsync=%.1f state=%.1f wld=%.1f wm=%.1f) pace=%.1f fin=%.1f unref=%.1f stats=%.1f "
            "n40=%u",
            s_worstMs, s_worstRender, s_worstWait, s_worstEvt, s_worstUpd, s_worstFocus,
            s_worstLua, s_worstPre, s_worstInp, s_worstSnd, s_worstLsync, s_worstState, s_worstWorld, s_worstWm,
            s_worstPace, s_worstFin, s_worstUnref, s_worstStats, s_over40);
        auditLog(buf);
        s_worstMs = s_worstRender = s_worstDraw = s_worstCull = s_worstWait = 0;
        s_worstEvt = s_worstUpd = s_worstFocus = s_worstLua = 0;
        s_worstPre = s_worstPace = 0;
        s_worstFin = s_worstInp = s_worstUnref = s_worstStats = 0;
        s_worstSnd = s_worstLsync = s_worstState = 0;
        s_worstWorld = s_worstWm = 0;
        {
            const double n = kReportEveryFrames * 1000.0;
            snprintf(buf, sizeof(buf), "[PhaseAvg] rnd=%.1f evt=%.1f upd=%.1f foc=%.1f lua=%.1f pre=%.1f pace=%.1f",
                s_sumRnd / n, s_sumEvt / n, s_sumUpd / n, s_sumFoc / n, s_sumLua / n, s_sumPre / n, s_sumPace / n);
            auditLog(buf);
            s_sumEvt = s_sumUpd = s_sumFoc = s_sumLua = s_sumPre = s_sumPace = s_sumRnd = 0;
        }
        s_over40 = 0;
        s_prevWaitUs = 0;
        {
            static uint32_t s_prevGpuFrames = 0;
            const uint32_t gfNow = vgl_gpu_frames;
            const unsigned gf = (gfNow - s_prevGpuFrames) ? (gfNow - s_prevGpuFrames) : 1;
            s_prevGpuFrames = gfNow;
            snprintf(buf, sizeof(buf),
                "[GlJob] draw=%.1f/%.1fms swap=%.1f/%.1fms rig=%u/%uk/%.1fms /frame",
                gl_draw_us / 1000.0 / kReportEveryFrames, gl_draw_max / 1000.0,
                gl_swap_us / 1000.0 / kReportEveryFrames, gl_swap_max / 1000.0,
                rig_cull_count / kReportEveryFrames, rig_cull_verts / kReportEveryFrames / 1000,
                osgprof_dyn_us / 1000.0 / kReportEveryFrames);
            auditLog(buf);
            gl_draw_us = gl_swap_us = gl_draw_max = gl_swap_max = 0;
            osgprof_dyn_leaves = osgprof_dyn_verts = osgprof_dyn_us = 0;
            rig_cull_count = rig_cull_verts = 0;
            snprintf(buf, sizeof(buf), "[Gpu] qd=%.2f/%u blk=%.2f/%.1fms tchunk=%u tleaf=%u /frame",
                (double)vgl_qdepth_sum / gf, vgl_qdepth_max, (double)vgl_swap_block_us / 1000.0 / gf,
                vgl_swap_block_max / 1000.0, terr_chunks / kReportEveryFrames, terr_pass_leaves / kReportEveryFrames);
            auditLog(buf);
            vgl_swap_block_us = vgl_swap_block_max = vgl_qdepth_sum = vgl_qdepth_max = 0;
            terr_chunks = terr_pass_leaves = 0;
        }
        snprintf(buf, sizeof(buf),
            "[Scene] drawables=%.0f fast=%.0f lights=%.0f bins=%.0f tris=%.0f strips=%.0f lightCb=%u/%.2fms "
            "glmemo=%u/%u vprog=%u/%u bin2=%u/%u gui=%u/%u",
            countOf(camStats, "Visible number of drawables"), countOf(camStats, "Visible number of fast drawables"),
            countOf(camStats, "Visible number of lights"), countOf(camStats, "Visible number of render bins"),
            countOf(camStats, "Visible number of GL_TRIANGLES"),
            countOf(camStats, "Visible number of GL_TRIANGLE_STRIP"),
            SceneUtil::gLightCbCalls / kReportEveryFrames,
            SceneUtil::gLightCbUs / 1000.0 / kReportEveryFrames,
            vgl_memo_hits / kReportEveryFrames, vgl_memo_miss / kReportEveryFrames,
            vgl_vprog_hits / kReportEveryFrames, vgl_vprog_miss / kReportEveryFrames,
            vita_bin2_graphs / kReportEveryFrames, vita_bin2_leaves / kReportEveryFrames,
            MyGUIPlatform::g_vitaGuiSkips, MyGUIPlatform::g_vitaGuiWalks);
        SceneUtil::gLightCbCalls = 0;
        SceneUtil::gLightCbUs = 0;
        vgl_memo_hits = vgl_memo_miss = 0;
        vgl_vprog_hits = vgl_vprog_miss = 0;
        vita_bin2_graphs = vita_bin2_leaves = 0;
        MyGUIPlatform::g_vitaGuiSkips = MyGUIPlatform::g_vitaGuiWalks = 0;
        auditLog(buf);

        // Per-leaf OSG draw-dispatch cost (probes in RenderLeaf.cpp).
        if (osgprof_leaves > 0)
        {
            snprintf(buf, sizeof(buf), "[OsgProf] repl=%u rep=%u srep=%u leaves=%u mat=%.1f state=%.1f unif=%.1f draw=%.1f us/leaf",
                osgprof_replayable, osgprof_replayed, osgprof_streplayed,
                osgprof_leaves, (double)osgprof_mat_us / osgprof_leaves, (double)osgprof_state_us / osgprof_leaves,
                (double)osgprof_unif_us / osgprof_leaves, (double)osgprof_draw_us / osgprof_leaves);
            auditLog(buf);
        }
        osgprof_mat_us = osgprof_state_us = osgprof_unif_us = osgprof_draw_us = osgprof_leaves = 0;
        osgprof_replayable = osgprof_replayed = osgprof_streplayed = 0;

        // Cull traversal shape: visits per frame by node kind.
        if (cullprof_node + cullprof_group + cullprof_drawable + cullprof_sg > 0)
        {
            snprintf(buf, sizeof(buf),
                "[CullProf] node=%u grp=%u xf=%u bone=%u geode=%u drw=%u dcull=%u leaves=%u crep=%u sg=%u terr=%.2fms "
                "dus=%.2f cbus=%.2f xfus=%.2f gus=%.2f cdrop=%u /frame",
                cullprof_node / kReportEveryFrames, cullprof_group / kReportEveryFrames,
                cullprof_transform / kReportEveryFrames, cullprof_xf_bone / kReportEveryFrames,
                cullprof_geode / kReportEveryFrames, cullprof_drawable / kReportEveryFrames,
                cullprof_dcull / kReportEveryFrames, cullprof_leaves / kReportEveryFrames,
                cullprof_creplay / kReportEveryFrames, cullprof_sg / kReportEveryFrames,
                (double)cullprof_terr_us / 1000.0 / kReportEveryFrames,
                (double)cullprof_drw_us / 1000.0 / kReportEveryFrames,
                (double)cullprof_cb_us / 1000.0 / kReportEveryFrames,
                (double)cullprof_xf_us / 1000.0 / kReportEveryFrames,
                (double)cullprof_grp_us / 1000.0 / kReportEveryFrames, cullprof_crep_drop / kReportEveryFrames);
            auditLog(buf);
        }
        cullprof_node = cullprof_group = cullprof_transform = cullprof_geode = 0;
        cullprof_drawable = cullprof_dcull = cullprof_leaves = cullprof_sg = 0;
        cullprof_xf_bone = cullprof_creplay = cullprof_crep_drop = 0;
        cullprof_terr_us = 0;
        cullprof_drw_us = cullprof_cb_us = cullprof_xf_us = cullprof_grp_us = 0;

        // Sim-worker phase split: what the main thread's join actually waits on.
        if (vita_sim_script_us + vita_sim_mech_us + vita_sim_phys_us > 0)
        {
            snprintf(buf, sizeof(buf), "[SimSplit] scr=%.1f (glob=%.1f) mech=%.1f phys=%.1f ms/frame",
                (double)vita_sim_script_us / 1000.0 / kReportEveryFrames,
                (double)vita_sim_gscript_us / 1000.0 / kReportEveryFrames,
                (double)vita_sim_mech_us / 1000.0 / kReportEveryFrames,
                (double)vita_sim_phys_us / 1000.0 / kReportEveryFrames);
            auditLog(buf);
        }
        vita_sim_script_us = vita_sim_mech_us = vita_sim_phys_us = vita_sim_gscript_us = 0;

        // Per-script and per-cull-callback cost breakdowns.
        {
            char hb[224];
            if (vita_script_hist_report(hb, sizeof(hb)) > 0)
            {
                snprintf(buf, sizeof(buf), "[ScriptHist] %s", hb);
                auditLog(buf);
            }
            if (cullprof_cb_report(hb, sizeof(hb)) > 0)
            {
                snprintf(buf, sizeof(buf), "[CullCb] %s", hb);
                auditLog(buf);
            }
        }

        // Inside State::apply: which section eats the state bucket.
        if (osgapply_calls > 0)
        {
            snprintf(buf, sizeof(buf),
                "[OsgApply] calls=%u tex=%.1f mode=%.1f attr=%.1f unif=%.1f us/call push=%u pop=%u un=%u up=%u rup=%u /%dfr",
                osgapply_calls, (double)osgapply_tex_us / osgapply_calls, (double)osgapply_mode_us / osgapply_calls,
                (double)osgapply_attr_us / osgapply_calls, (double)osgapply_unif_us / osgapply_calls, osgapply_push,
                osgapply_pop, osgapply_unif_n, osgapply_unif_up, osgapply_unif_rup, kReportEveryFrames);
            auditLog(buf);
        }
        osgapply_calls = osgapply_tex_us = osgapply_mode_us = osgapply_attr_us = osgapply_unif_us = 0;
        osgapply_unif_n = osgapply_unif_up = osgapply_unif_rup = 0;
        osgapply_push = osgapply_pop = 0;

        // Upload histogram: name=up/redundant, top 8 per window.
        {
            char hb[224];
            if (osgapply_unif_hist_report(hb, sizeof(hb)) > 0)
            {
                snprintf(buf, sizeof(buf), "[UnifHist] %s", hb);
                auditLog(buf);
            }
        }
    }

    void noteRenderTime(unsigned long long us)
    {
        s_renderUsAccum += us;
        ++s_renderSamples;
        gLastRenderUs = us;
    }
}

#endif // __vita__
