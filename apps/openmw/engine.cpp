#include "engine.hpp"

#include <cerrno>
#include <chrono>
#include <future>
#include <map>
#include <mutex>
#include <system_error>
#include <thread>
#ifdef __vita__
#include <malloc.h>
#endif

#include <osgDB/ReaderWriter>
#include <osgDB/Registry>
#include <osgUtil/RenderBin>
#include <osgUtil/UpdateVisitor>
#include <osgViewer/Renderer>
#include <osgViewer/ViewerEventHandlers>

#include <SDL.h>

#include <components/debug/debuglog.hpp>
#include <components/debug/gldebug.hpp>

#ifdef __vita__
#include "vita/VitaInit.h"
#include "vita/VitaMemAudit.h"
#include "vita/VitaSimWorker.h"
#include "vita/VitaGLWorker.h"
#include <components/vita/VitaDialogueText.h>
#include <components/vita/VitaEsmPrefetch.h>
#include <psp2/io/fcntl.h>
#include <psp2/display.h>
// Pin API; vitasdk's stock vitaGL.h lacks it.
extern "C" void vglSetStaticVboRam(unsigned char enable);
extern "C" uint32_t phase_evt_us, phase_upd_us, phase_focus_us, phase_lua_us, phase_pre_us, phase_pace_us;
extern "C" uint32_t phase_fin_us, phase_inp_us, phase_unref_us, phase_stats_us;
extern "C" uint32_t phase_snd_us, phase_lsync_us, phase_state_us;
extern "C" uint32_t phase_world_us, phase_wm_us;
extern "C" unsigned int vita_bin2_graphs, vita_bin2_leaves;
extern "C" uint32_t gl_draw_us, gl_swap_us, gl_draw_max, gl_swap_max;

namespace
{
    // Texture-sorted iteration over the merged-statics bin.
    struct VitaStaticBinCallback : osgUtil::RenderBin::DrawCallback
    {
        static const osg::StateAttribute* texOf(const osgUtil::StateGraph* sg)
        {
            const osg::StateSet* ss = sg->getStateSet();
            return ss ? ss->getTextureAttribute(0, osg::StateAttribute::TEXTURE) : nullptr;
        }

        void drawImplementation(
            osgUtil::RenderBin* bin, osg::RenderInfo& renderInfo, osgUtil::RenderLeaf*& previous) override
        {
            osgUtil::RenderBin::StateGraphList& graphs = bin->getStateGraphList();
            unsigned int leaves = 0;
            for (const osgUtil::StateGraph* sg : graphs)
                leaves += sg->_leaves.size();
            vita_bin2_graphs += graphs.size();
            vita_bin2_leaves += leaves;
            // Texture-first order: adjacent deltas skip the rebind.
            std::sort(graphs.begin(), graphs.end(),
                [](const osgUtil::StateGraph* a, const osgUtil::StateGraph* b) {
                    const osg::StateAttribute* ta = texOf(a);
                    const osg::StateAttribute* tb = texOf(b);
                    if (ta != tb)
                        return ta < tb;
                    return a->getStateSet() < b->getStateSet();
                });
            bin->drawImplementation(renderInfo, previous);
        }
    };
}
#include <psp2/kernel/processmgr.h>
#define VITA_CRUMB(msg) Vita::breadcrumb(msg)
// Fork replay switches (fetched-OSG RenderLeaf.cpp).
extern int vita_draw_replay;
extern int vita_state_replay;
#else
#define VITA_CRUMB(msg)
#endif

#include <components/misc/rng.hpp>
#include <components/misc/pathhelpers.hpp>
#include <components/misc/strings/format.hpp>

#include <components/vfs/manager.hpp>
#include <components/vfs/registerarchives.hpp>

#include <components/sdlutil/imagetosurface.hpp>
#include <components/sdlutil/sdlgraphicswindow.hpp>

#include <components/resource/resourcesystem.hpp>
#include <components/resource/scenemanager.hpp>
#include <components/resource/stats.hpp>

#include <components/compiler/extensions0.hpp>

#include <components/stereo/stereomanager.hpp>

#include <components/sceneutil/glextensions.hpp>
#include <components/sceneutil/workqueue.hpp>

#include <components/files/configurationmanager.hpp>
#include <components/files/scancache.hpp>

#include <components/version/version.hpp>

#include <components/l10n/manager.hpp>

#include <components/loadinglistener/asynclistener.hpp>
#include <components/loadinglistener/loadinglistener.hpp>

#include <components/misc/frameratelimiter.hpp>

#include <components/sceneutil/color.hpp>
#include <components/sceneutil/depth.hpp>
#include <components/sceneutil/screencapture.hpp>
#include <components/sceneutil/unrefqueue.hpp>
#include <components/sceneutil/util.hpp>

#include <components/settings/settings.hpp>
#include <components/settings/shadermanager.hpp>
#include <components/settings/values.hpp>

#include "mwinput/inputmanagerimp.hpp"

#include "mwgui/windowmanagerimp.hpp"

#include "mwlua/luamanagerimp.hpp"
#include "mwlua/worker.hpp"

#include "mwscript/interpretercontext.hpp"
#include "mwscript/scriptmanagerimp.hpp"

#include "mwsound/constants.hpp"
#include "mwsound/soundmanagerimp.hpp"

#include "mwworld/class.hpp"
#include "mwworld/datetimemanager.hpp"
#include "mwworld/worldimp.hpp"

#include "mwrender/vismask.hpp"

#include "mwclass/classes.hpp"

#include "mwdialogue/dialoguemanagerimp.hpp"
#include "mwdialogue/journalimp.hpp"
#include "mwdialogue/scripttest.hpp"

#include "mwmechanics/mechanicsmanagerimp.hpp"

#include "mwstate/statemanagerimp.hpp"

#include "profile.hpp"

namespace
{
    void checkSDLError(int ret)
    {
        if (ret != 0)
            Log(Debug::Error) << "SDL error: " << SDL_GetError();
    }

    void initStatsHandler(Resource::Profiler& profiler)
    {
        const osg::Vec4f textColor(1.f, 1.f, 1.f, 1.f);
        const osg::Vec4f barColor(1.f, 1.f, 1.f, 1.f);
        const float multiplier = 1000;
        const bool average = true;
        const bool averageInInverseSpace = false;
        const float maxValue = 10000;

        OMW::forEachUserStatsValue([&](const OMW::UserStats& v) {
            profiler.addUserStatsLine(v.mLabel, textColor, barColor, v.mTaken, multiplier, average,
                averageInInverseSpace, v.mBegin, v.mEnd, maxValue);
        });
        // the forEachUserStatsValue loop is "run" at compile time, hence the settings manager is not available.
        // Unconditionnally add the async physics stats, and then remove it at runtime if necessary
        if (Settings::physics().mAsyncNumThreads == 0)
            profiler.removeUserStatsLine(" -Async");
    }

    struct ScreenCaptureMessageBox
    {
        void operator()(std::string filePath) const
        {
            if (filePath.empty())
            {
                MWBase::Environment::get().getWindowManager()->scheduleMessageBox(
                    "#{OMWEngine:ScreenshotFailed}", MWGui::ShowInDialogueMode_Never);

                return;
            }

            auto l10n = MWBase::Environment::get().getL10nManager()->getContext("OMWEngine");
            std::string message = l10n->formatMessage("ScreenshotMade", { "file" }, { L10n::toUnicode(filePath) });

            MWBase::Environment::get().getWindowManager()->scheduleMessageBox(
                std::move(message), MWGui::ShowInDialogueMode_Never);
        }
    };

    struct IgnoreString
    {
        void operator()(std::string) const {}
    };

    class IdentifyOpenGLOperation : public osg::GraphicsOperation
    {
    public:
        IdentifyOpenGLOperation()
            : GraphicsOperation("IdentifyOpenGLOperation", false)
        {
        }

        void operator()(osg::GraphicsContext* graphicsContext) override
        {
            Log(Debug::Info) << "OpenGL Vendor: " << glGetString(GL_VENDOR);
            Log(Debug::Info) << "OpenGL Renderer: " << glGetString(GL_RENDERER);
            Log(Debug::Info) << "OpenGL Version: " << glGetString(GL_VERSION);
            glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &mMaxTextureImageUnits);
        }

        int getMaxTextureImageUnits() const
        {
            if (mMaxTextureImageUnits == 0)
                throw std::logic_error("mMaxTextureImageUnits is not initialized");
            return mMaxTextureImageUnits;
        }

    private:
        int mMaxTextureImageUnits = 0;
    };

    void reportStats(unsigned frameNumber, osgViewer::Viewer& viewer, std::ostream& stream)
    {
        viewer.getViewerStats()->report(stream, frameNumber);
        osgViewer::Viewer::Cameras cameras;
        viewer.getCameras(cameras);
        for (osg::Camera* camera : cameras)
            camera->getStats()->report(stream, frameNumber);
    }
}

#ifdef __vita__
namespace
{
    std::mutex sVitaScriptHistMutex;
    std::map<ESM::RefId, uint32_t> sVitaScriptHist;
}

extern "C" int vita_script_hist_report(char* buf, unsigned int buflen)
{
    // Top 6 by time this window, then reset.
    const std::lock_guard<std::mutex> lock(sVitaScriptHistMutex);
    int written = 0;
    for (int rank = 0; rank < 6; ++rank)
    {
        auto best = sVitaScriptHist.end();
        for (auto it = sVitaScriptHist.begin(); it != sVitaScriptHist.end(); ++it)
            if (it->second > 0 && (best == sVitaScriptHist.end() || it->second > best->second))
                best = it;
        if (best == sVitaScriptHist.end())
            break;
        int n = snprintf(buf + written, buflen - written, "%s%s=%ums", written ? " " : "",
            best->first.toDebugString().c_str(), best->second / 1000);
        best->second = 0;
        if (n < 0 || (unsigned)n >= buflen - written)
            break;
        written += n;
    }
    sVitaScriptHist.clear();
    return written;
}
#endif

void OMW::Engine::executeLocalScripts()
{
    MWWorld::LocalScripts& localScripts = mWorld->getLocalScripts();

    localScripts.startIteration();
    std::pair<ESM::RefId, MWWorld::Ptr> script;
#ifdef __vita__
    // Far-tier scheduling: scripted objects beyond kNearR tick every 4th
    // frame with dt scaled 4x (thread-local, opcodes see correct elapsed).
    static unsigned sScriptFrame = 0;
    ++sScriptFrame;
    constexpr float kNearR = 2048.f;
    const osg::Vec3f playerPos = mWorld->getPlayerPtr().getRefData().getPosition().asVec3();
#endif
    while (localScripts.getNext(script))
    {
#ifdef __vita__
        bool farTier = false;
        if (script.second.isInCell())
        {
            const osg::Vec3f op = script.second.getRefData().getPosition().asVec3();
            farTier = (op - playerPos).length2() > kNearR * kNearR;
        }
        if (farTier)
        {
            const unsigned phase = (unsigned)((uintptr_t)script.second.mRef >> 4);
            if ((sScriptFrame & 3u) != (phase & 3u))
                continue;
        }
        MWScript::InterpreterContext interpreterContext(&script.second.getRefData().getLocals(), script.second);
        const osg::Timer* const stimer = osg::Timer::instance();
        const osg::Timer_t st0 = stimer->tick();
        if (farTier)
            MWBase::Environment::sVitaDtScale = 4.f;
        mScriptManager->run(script.first, interpreterContext);
        MWBase::Environment::sVitaDtScale = 1.f;
        const uint32_t sus = (uint32_t)stimer->delta_u(st0, stimer->tick());
        {
            const std::lock_guard<std::mutex> lock(sVitaScriptHistMutex);
            sVitaScriptHist[script.first] += sus;
        }
#else
        MWScript::InterpreterContext interpreterContext(&script.second.getRefData().getLocals(), script.second);
        mScriptManager->run(script.first, interpreterContext);
#endif
    }
}

#ifdef __vita__
extern "C" {
uint32_t vita_sim_script_us = 0, vita_sim_mech_us = 0, vita_sim_phys_us = 0;
uint32_t vita_sim_gscript_us = 0;
}
#define VITA_SIM_T0() const osg::Timer_t simT0 = timer->tick();
#define VITA_SIM_ADD(var) var += (uint32_t)timer->delta_u(simT0, timer->tick());
#else
#define VITA_SIM_T0()
#define VITA_SIM_ADD(var)
#endif

void OMW::Engine::runSimPhases(osg::Timer_t frameStart, unsigned frameNumber, float frametime, bool paused)
{
    const osg::Timer* const timer = osg::Timer::instance();
    osg::Stats* const stats = mViewer->getViewerStats();

        {
            ScopedProfile<UserStatsType::Script> profile(frameStart, frameNumber, *timer, *stats);
            VITA_SIM_T0()

            if (mStateManager->getState() != MWBase::StateManager::State_NoGame)
            {
                if (!mWindowManager->containsMode(MWGui::GM_MainMenu) || !paused)
                {
                    if (mWorld->getScriptsEnabled())
                    {
                        // local scripts
                        executeLocalScripts();

                        // global scripts
#ifdef __vita__
                        {
                            const osg::Timer_t gt0 = timer->tick();
                            mScriptManager->getGlobalScripts().run();
                            vita_sim_gscript_us += (uint32_t)timer->delta_u(gt0, timer->tick());
                        }
#else
                        mScriptManager->getGlobalScripts().run();
#endif
                    }

                    mWorld->getWorldScene().markCellAsUnchanged();
                }

                if (!paused)
                {
                    double hours = (frametime * mWorld->getTimeManager()->getGameTimeScale()) / 3600.0;
                    mWorld->advanceTime(hours, true);
#ifdef __vita__
                    // rechargeItems iterates every NPC/Creature/Container in
                    // every active cell each frame. Recharge math is
                    // FPS-invariant; throttle to ~4 Hz with accumulated dt.
                    static float s_rechargeAccum = 0.f;
                    s_rechargeAccum += frametime;
                    if (s_rechargeAccum >= 0.25f)
                    {
                        mWorld->rechargeItems(s_rechargeAccum, true);
                        s_rechargeAccum = 0.f;
                    }
#else
                    mWorld->rechargeItems(frametime, true);
#endif
                }
            }
            VITA_SIM_ADD(vita_sim_script_us)
        }

        // update mechanics
        {
            ScopedProfile<UserStatsType::Mechanics> profile(frameStart, frameNumber, *timer, *stats);
            VITA_SIM_T0()

            if (mStateManager->getState() != MWBase::StateManager::State_NoGame)
            {
                mMechanicsManager->update(frametime, paused);
            }

            if (mStateManager->getState() == MWBase::StateManager::State_Running)
            {
                MWWorld::Ptr player = mWorld->getPlayerPtr();
                if (!paused && player.getClass().getCreatureStats(player).isDead())
                    mStateManager->endGame();
            }
            VITA_SIM_ADD(vita_sim_mech_us)
        }

        // update physics
        {
            ScopedProfile<UserStatsType::Physics> profile(frameStart, frameNumber, *timer, *stats);
            VITA_SIM_T0()

            if (mStateManager->getState() != MWBase::StateManager::State_NoGame)
            {
#ifdef __vita__
                try {
                    mWorld->updatePhysics(frametime, paused, frameStart, frameNumber, *stats);
                } catch (const std::exception& e) {
                    Vita::breadcrumb(("[PhysCrash] std::exception: " + std::string(e.what())).c_str());
                    vitaLogFlushNow();
                } catch (...) {
                    Vita::breadcrumb("[PhysCrash] non-std exception caught");
                    vitaLogFlushNow();
                }
#else
                mWorld->updatePhysics(frametime, paused, frameStart, frameNumber, *stats);
#endif
            }
            VITA_SIM_ADD(vita_sim_phys_us)
        }

}

bool OMW::Engine::frame(unsigned frameNumber, float frametime)
{
    const osg::Timer_t frameStart = mViewer->getStartTick();
    const osg::Timer* const timer = osg::Timer::instance();
    osg::Stats* const stats = mViewer->getViewerStats();

    mEnvironment.setFrameDuration(frametime);

#ifdef __vita__
    uint64_t vitaFrameT0 = 0;
#endif
    try
    {
#ifdef __vita__
        phase_evt_us = phase_upd_us = phase_focus_us = phase_lua_us = phase_pre_us = 0;
        phase_fin_us = phase_inp_us = phase_unref_us = phase_stats_us = 0;
        phase_snd_us = phase_lsync_us = phase_state_us = 0;
        phase_world_us = phase_wm_us = 0;
        vitaFrameT0 = sceKernelGetProcessTimeWide();
        vitaMainPhase("simjoin");
        // Finish overlapped sim before touching game state.
        if (mSimWorker && mSimOverlap && mSimPrimed)
            mSimWorker->finish();
        phase_fin_us = (uint32_t)(sceKernelGetProcessTimeWide() - vitaFrameT0);
        vitaMainPhase("input");
#endif
        // update input
        {
#ifdef __vita__
            const uint64_t inpT0 = sceKernelGetProcessTimeWide();
#endif
            ScopedProfile<UserStatsType::Input> profile(frameStart, frameNumber, *timer, *stats);
            mInputManager->update(frametime, false);
#ifdef __vita__
            phase_inp_us = (uint32_t)(sceKernelGetProcessTimeWide() - inpT0);
#endif
        }

        // When the window is minimized, pause the game. Currently this *has* to be here to work around a MyGUI bug.
        // If we are not currently rendering, then RenderItems will not be reused resulting in a memory leak upon
        // changing widget textures (fixed in MyGUI 3.3.2), and destroyed widgets will not be deleted (not fixed yet,
        // https://github.com/MyGUI/mygui/issues/21)
        {
            ScopedProfile<UserStatsType::Sound> profile(frameStart, frameNumber, *timer, *stats);

            if (!mWindowManager->isWindowVisible())
            {
                mSoundManager->pausePlayback();
                return false;
            }
            else
                mSoundManager->resumePlayback();

#ifdef __vita__
            const uint64_t sndT0 = sceKernelGetProcessTimeWide();
            vitaMainPhase("sound");
#endif
            // sound
            if (mUseSound)
                mSoundManager->update(frametime);
#ifdef __vita__
            phase_snd_us = (uint32_t)(sceKernelGetProcessTimeWide() - sndT0);
#endif
        }

        {
            ScopedProfile<UserStatsType::LuaSyncUpdate> profile(frameStart, frameNumber, *timer, *stats);
#ifdef __vita__
            const uint64_t lsT0 = sceKernelGetProcessTimeWide();
            vitaMainPhase("lsync");
#endif
            // Should be called after input manager update and before any change to the game world.
            // It applies to the game world queued changes from the previous frame.
            mLuaManager->synchronizedUpdate();
#ifdef __vita__
            phase_lsync_us = (uint32_t)(sceKernelGetProcessTimeWide() - lsT0);
#endif
        }

        // update game state
        {
            ScopedProfile<UserStatsType::State> profile(frameStart, frameNumber, *timer, *stats);
#ifdef __vita__
            const uint64_t stT0 = sceKernelGetProcessTimeWide();
            vitaMainPhase("state");
#endif
            mStateManager->update(frametime);
#ifdef __vita__
            phase_state_us = (uint32_t)(sceKernelGetProcessTimeWide() - stT0);
#endif
        }

        bool paused = mWorld->getTimeManager()->isPaused();

#ifdef __vita__
        if (mSimWorker && mSimOverlap)
        {
            // Sim already ran during draw; first frame bootstraps here.
            if (!mSimPrimed)
            {
                mSimWorker->run([&] { runSimPhases(frameStart, frameNumber, frametime, paused); });
                mSimWorker->finish();
            }
        }
        else if (mSimWorker)
        {
            // Synchronous: sim on worker, same ordering.
            mSimWorker->run([&] { runSimPhases(frameStart, frameNumber, frametime, paused); });
            mSimWorker->finish();
        }
        else
            runSimPhases(frameStart, frameNumber, frametime, paused);
#else
        runSimPhases(frameStart, frameNumber, frametime, paused);
#endif

        // update world
        {
            ScopedProfile<UserStatsType::World> profile(frameStart, frameNumber, *timer, *stats);

#ifdef __vita__
            const uint64_t wldT0 = sceKernelGetProcessTimeWide();
            vitaMainPhase("wld");
#endif
            if (mStateManager->getState() != MWBase::StateManager::State_NoGame)
            {
                mWorld->update(frametime, paused);
            }
#ifdef __vita__
            phase_world_us = (uint32_t)(sceKernelGetProcessTimeWide() - wldT0);
#endif
        }

        // update GUI
        {
            ScopedProfile<UserStatsType::Gui> profile(frameStart, frameNumber, *timer, *stats);
#ifdef __vita__
            const uint64_t wmT0 = sceKernelGetProcessTimeWide();
            vitaMainPhase("gui");
#endif
            mWindowManager->update(frametime);
#ifdef __vita__
            phase_wm_us = (uint32_t)(sceKernelGetProcessTimeWide() - wmT0);
#endif
        }
    }
    catch (const std::exception& e)
    {
        Log(Debug::Error) << "Error in frame: " << e.what();
    }

    const bool reportResource = stats->collectStats("resource");

    if (reportResource)
        stats->setAttribute(frameNumber, "UnrefQueue", static_cast<double>(mUnrefQueue->getSize()));

#ifdef __vita__
    {
        const uint64_t unrefT0 = sceKernelGetProcessTimeWide();
        vitaMainPhase("unref");
        mUnrefQueue->flush(*mWorkQueue);
        phase_unref_us = (uint32_t)(sceKernelGetProcessTimeWide() - unrefT0);
    }
#else
    mUnrefQueue->flush(*mWorkQueue);
#endif

#ifdef __vita__
    const uint64_t statsT0 = sceKernelGetProcessTimeWide();
#endif
    if (reportResource)
    {
        stats->setAttribute(frameNumber, "FrameNumber", frameNumber);

        mResourceSystem->reportStats(frameNumber, stats);

        stats->setAttribute(frameNumber, "WorkQueue", static_cast<double>(mWorkQueue->getNumItems()));
        stats->setAttribute(frameNumber, "WorkThread", static_cast<double>(mWorkQueue->getNumActiveThreads()));

        mMechanicsManager->reportStats(frameNumber, *stats);
        mWorld->reportStats(frameNumber, *stats);
        mLuaManager->reportStats(frameNumber, *stats);

        stats->setAttribute(frameNumber, "StringRefId Count", static_cast<double>(ESM::StringRefId::totalCount()));
    }
#ifdef __vita__
    phase_stats_us = (uint32_t)(sceKernelGetProcessTimeWide() - statsT0);
#endif

    mStereoManager->updateSettings(Settings::camera().mNearClip, Settings::camera().mViewingDistance);

#ifdef __vita__
    phase_pre_us = (uint32_t)(sceKernelGetProcessTimeWide() - vitaFrameT0);
    uint64_t vitaPhaseT0 = sceKernelGetProcessTimeWide();
#endif
    mViewer->eventTraversal();
#ifdef __vita__
    phase_evt_us = (uint32_t)(sceKernelGetProcessTimeWide() - vitaPhaseT0);
    vitaPhaseT0 = sceKernelGetProcessTimeWide();
    if (mUpdateOverlap && mSimWorker && mSimOverlap && mCullOverlap && mViewer->getSceneData())
    {
        // Hazard classes update on main; the rest on the worker pre-cull.
        constexpr unsigned int kHazardMask = MWRender::Mask_Effect | MWRender::Mask_WeatherParticles
            | MWRender::Mask_ParticleSystem | MWRender::Mask_Water | MWRender::Mask_SimpleWater | MWRender::Mask_Sky;
        osgUtil::UpdateVisitor* uv = mViewer->getUpdateVisitor();
        const unsigned int fullMask = uv->getTraversalMask();
        uv->setTraversalMask(kHazardMask);
        mViewer->updateTraversal();
        uv->setTraversalMask(fullMask);
        // Hazard pass prunes at Mask_Scene, worker prunes at Mask_Sky:
        // no traversal reaches the sky (black-sky bug). Run it here.
        mWorld->vitaUpdateSky(*uv);
        mVitaWorkerUpdateMask = fullMask & ~kHazardMask;
        mVitaWorkerUpdatePending = true;
    }
    else
        mViewer->updateTraversal();
    phase_upd_us = (uint32_t)(sceKernelGetProcessTimeWide() - vitaPhaseT0);
#else
    mViewer->updateTraversal();
#endif

    // update focus object for GUI
    {
        ScopedProfile<UserStatsType::Focus> profile(frameStart, frameNumber, *timer, *stats);
#ifdef __vita__
        // Full scene raycast; 10 Hz is enough for tooltips.
        static float s_focusAccum = 0.f;
        s_focusAccum += frametime;
        if (s_focusAccum >= 0.1f)
        {
            s_focusAccum = 0.f;
            vitaPhaseT0 = sceKernelGetProcessTimeWide();
            mWorld->updateFocusObject();
            phase_focus_us = (uint32_t)(sceKernelGetProcessTimeWide() - vitaPhaseT0);
        }
#else
        mWorld->updateFocusObject();
#endif
    }

    // if there is a separate Lua thread, it starts the update now
#ifdef __vita__
    vitaPhaseT0 = sceKernelGetProcessTimeWide();
    mLuaWorker->allowUpdate(frameStart, frameNumber, *stats);
    phase_lua_us += (uint32_t)(sceKernelGetProcessTimeWide() - vitaPhaseT0);
#else
    mLuaWorker->allowUpdate(frameStart, frameNumber, *stats);
#endif

#ifdef __vita__
    const uint64_t renderStartUs = sceKernelGetProcessTimeWide();
    if (mSimWorker && mSimOverlap && mCullOverlap)
    {
        // Stage C: worker culls N then sims N+1; main draws N-1 meanwhile.
        // Draw lags cull by one frame (adds one frame of latency).
        auto* renderer = static_cast<osgViewer::Renderer*>(mViewer->getCamera()->getRenderer());
        if (renderer->getGraphicsThreadDoesCull())
            renderer->setGraphicsThreadDoesCull(false);
        const bool nextPaused = mWorld->getTimeManager()->isPaused();
        const float nextDt = frametime; // estimate; corrected next frame
        const unsigned nextFrame = frameNumber + 1;
        const bool havePrev = mCullPrimed;
        Vita::setDrawInFlight(true);
        mSimWorker->run([this, renderer, frameStart, nextFrame, nextDt, nextPaused] {
            if (mVitaWorkerUpdatePending)
            {
                // Non-hazard update callbacks; must precede cull.
                if (!mVitaWorkerUpdateVisitor)
                    mVitaWorkerUpdateVisitor = new osgUtil::UpdateVisitor;
                mVitaWorkerUpdateVisitor->reset();
                mVitaWorkerUpdateVisitor->setFrameStamp(mViewer->getFrameStamp());
                mVitaWorkerUpdateVisitor->setTraversalMask(mVitaWorkerUpdateMask);
                mViewer->getSceneData()->accept(*mVitaWorkerUpdateVisitor);
                mVitaWorkerUpdatePending = false;
            }
            renderer->cull();
            runSimPhases(frameStart, nextFrame, nextDt, nextPaused);
        });
        mSimPrimed = true;
        mCullPrimed = true;
        if (havePrev)
        {
            if (Vita::GLWorker* glw = Vita::getGLWorker())
            {
                osg::GraphicsContext* gc = mViewer->getCamera()->getGraphicsContext();
                auto* icoProbe = mViewer->getIncrementalCompileOperation();
                glw->run([renderer, gc, icoProbe] {
                    const uint64_t t0 = sceKernelGetProcessTimeWide();
                    // Spike anatomy: is a long draw compiling new GL objects
                    // (ICO), or is it dispatch/vitaGL itself?
                    const unsigned int icoSets
                        = icoProbe ? (unsigned int)icoProbe->getToCompile().size() : 0u;
                    const bool icoBefore = icoSets > 0;
                    renderer->draw();
                    const uint64_t t1 = sceKernelGetProcessTimeWide();
                    gc->swapBuffers();
                    const uint64_t t2 = sceKernelGetProcessTimeWide();
                    if ((uint32_t)(t1 - t0) > 25000)
                    {
                        // Name what compiled: a budget can only be checked
                        // BETWEEN objects, so one huge texture blows through it.
                        unsigned tex = 0, drw = 0, prog = 0;
                        unsigned biggestKB = 0;
                        char biggest[48] = "";
                        if (icoProbe)
                            for (const auto& cs : icoProbe->getToCompile())
                                for (const auto& cm : cs->_compileMap)
                                    for (const auto& opIt : cm.second._compileOps)
                                    {
                                        if (auto* t = dynamic_cast<osgUtil::IncrementalCompileOperation::
                                                    CompileTextureOp*>(opIt.get()))
                                        {
                                            ++tex;
                                            if (osg::Image* im = t->_texture->getImage(0))
                                            {
                                                const unsigned kb = (unsigned)(im->getTotalSizeInBytes() / 1024);
                                                if (kb > biggestKB)
                                                {
                                                    biggestKB = kb;
                                                    snprintf(biggest, sizeof(biggest), "%s", im->getFileName().c_str());
                                                }
                                            }
                                        }
                                        else if (dynamic_cast<osgUtil::IncrementalCompileOperation::
                                                     CompileProgramOp*>(opIt.get()))
                                            ++prog;
                                        else
                                            ++drw;
                                    }
                        char db[224];
                        snprintf(db, sizeof(db), "[DrawSpike] draw=%ums sets=%u tex=%u drw=%u prog=%u big=%uKB %s",
                            (unsigned)((t1 - t0) / 1000), icoSets, tex, drw, prog, biggestKB, biggest);
                        Vita::breadcrumb(db);
                    }
                    gl_draw_us += (uint32_t)(t1 - t0);
                    gl_swap_us += (uint32_t)(t2 - t1);
                    if ((uint32_t)(t1 - t0) > gl_draw_max)
                        gl_draw_max = (uint32_t)(t1 - t0);
                    if ((uint32_t)(t2 - t1) > gl_swap_max)
                        gl_swap_max = (uint32_t)(t2 - t1);
                    Vita::noteRenderTime(t2 - t0);
                    Vita::setDrawInFlight(false);
                });
            }
            else
            {
                renderer->draw();
                mViewer->getCamera()->getGraphicsContext()->swapBuffers();
                Vita::setDrawInFlight(false);
            }
        }
        else
            Vita::setDrawInFlight(false);
    }
    else if (mSimWorker && mSimOverlap)
    {
        // Kick next frame's sim after cull; it overlaps draw+swap.
        // Draw reads cull-cached matrices, so sim writes are safe.
        auto* renderer = static_cast<osgViewer::Renderer*>(mViewer->getCamera()->getRenderer());
        // Else cull() no-ops and draw() blocks forever.
        if (renderer->getGraphicsThreadDoesCull())
            renderer->setGraphicsThreadDoesCull(false);
        renderer->cull();
        const bool nextPaused = mWorld->getTimeManager()->isPaused();
        const float nextDt = frametime; // estimate; corrected next frame
        const unsigned nextFrame = frameNumber + 1;
        Vita::setDrawInFlight(true);
        mSimWorker->run([this, frameStart, nextFrame, nextDt, nextPaused] {
            runSimPhases(frameStart, nextFrame, nextDt, nextPaused);
        });
        mSimPrimed = true;
        renderer->draw();
        mViewer->getCamera()->getGraphicsContext()->swapBuffers();
        Vita::setDrawInFlight(false);
    }
    else
        mViewer->renderingTraversals();
    if (!Vita::getGLWorker())
        Vita::noteRenderTime(sceKernelGetProcessTimeWide() - renderStartUs);
#else
    mViewer->renderingTraversals();
#endif

#ifdef __vita__
    vitaPhaseT0 = sceKernelGetProcessTimeWide();
    mLuaWorker->finishUpdate(frameStart, frameNumber, *stats);
    phase_lua_us += (uint32_t)(sceKernelGetProcessTimeWide() - vitaPhaseT0);
#else
    mLuaWorker->finishUpdate(frameStart, frameNumber, *stats);
#endif

#ifdef __vita__
    Vita::auditFrameStats(*mViewer);
#endif

    return true;
}

OMW::Engine::Engine(Files::ConfigurationManager& configurationManager)
    : mWindow(nullptr)
    , mEncoding(ToUTF8::WINDOWS_1252)
    , mScreenCaptureOperation(nullptr)
    , mSelectDepthFormatOperation(new SceneUtil::SelectDepthFormatOperation())
    , mSelectColorFormatOperation(new SceneUtil::Color::SelectColorFormatOperation())
    , mStereoManager(nullptr)
    , mSkipMenu(false)
    , mUseSound(true)
    , mCompileAll(false)
    , mCompileAllDialogue(false)
    , mWarningsMode(1)
    , mScriptConsoleMode(false)
    , mActivationDistanceOverride(-1)
    , mGrab(true)
    , mExportFonts(false)
    , mRandomSeed(0)
    , mNewGame(false)
    , mCfgMgr(configurationManager)
    , mGlMaxTextureImageUnits(0)
{
#if SDL_VERSION_ATLEAST(2, 24, 0)
    SDL_SetHint(SDL_HINT_MAC_OPENGL_ASYNC_DISPATCH, "1");
#endif
    SDL_SetHint(SDL_HINT_ACCELEROMETER_AS_JOYSTICK, "0"); // We use only gamepads

    Uint32 flags
        = SDL_INIT_VIDEO | SDL_INIT_NOPARACHUTE | SDL_INIT_GAMECONTROLLER | SDL_INIT_JOYSTICK | SDL_INIT_SENSOR;
    if (SDL_WasInit(flags) == 0)
    {
        SDL_SetMainReady();
        if (SDL_Init(flags) != 0)
        {
            throw std::runtime_error("Could not initialize SDL! " + std::string(SDL_GetError()));
        }
    }
}

OMW::Engine::~Engine()
{
    if (mScreenCaptureOperation != nullptr)
    {
        mScreenCaptureOperation->stop();
        mScreenCaptureOperation = nullptr;
    }
    mScreenCaptureHandler = nullptr;

    mMechanicsManager = nullptr;
    mDialogueManager = nullptr;
    mJournal = nullptr;
    mWindowManager = nullptr;
    mScriptManager = nullptr;
    mWorld = nullptr;
    mStereoManager = nullptr;
    mSoundManager = nullptr;
    mInputManager = nullptr;
    mStateManager = nullptr;
    mLuaWorker = nullptr;
#ifdef __vita__
    mSimWorker = nullptr;
#endif
    mLuaManager = nullptr;
    mL10nManager = nullptr;

    mScriptContext = nullptr;

    mUnrefQueue = nullptr;
    mWorkQueue = nullptr;

    mViewer = nullptr;

    mResourceSystem.reset();

    mEncoder = nullptr;

    if (mWindow)
    {
        SDL_DestroyWindow(mWindow);
        mWindow = nullptr;
    }

    SDL_Quit();

    Log(Debug::Info) << "Quitting peacefully.";
#ifdef __vita__
    // Clean-exit marker; absent at boot = crashed.
    {
        SceUID fd = sceIoOpen("ux0:data/openmw/clean_exit", SCE_O_WRONLY | SCE_O_CREAT | SCE_O_TRUNC, 0777);
        if (fd >= 0)
        {
            sceIoWrite(fd, "ok", 2);
            sceIoClose(fd);
        }
        vitaLogFlushNow();
    }
#endif
}

// Set data dir

void OMW::Engine::setDataDirs(const Files::PathContainer& dataDirs)
{
    mDataDirs = dataDirs;
    mDataDirs.insert(mDataDirs.begin(), mResDir / "vfs");
    mFileCollections = Files::Collections(mDataDirs);

#ifdef __vita__
    const auto cachePath = mCfgMgr.getUserConfigPath() / "scan_cache.bin";
    Files::Collections::CollectionsMap cached;
    if (Files::loadScanCache(cachePath, mDataDirs, cached))
        mFileCollections.setCollections(std::move(cached));
#endif
}

// Add BSA archive
void OMW::Engine::addArchive(const std::string& archive)
{
    mArchives.push_back(archive);
}

// Set resource dir
void OMW::Engine::setResourceDir(const std::filesystem::path& parResDir)
{
    mResDir = parResDir;
    if (!Version::checkResourcesVersion(mResDir))
        Log(Debug::Error) << "Resources dir " << mResDir
                          << " doesn't match OpenMW binary, the game may work incorrectly.";
}

// Set start cell name
void OMW::Engine::setCell(const std::string& cellName)
{
    mCellName = cellName;
}

void OMW::Engine::addContentFile(const std::string& file)
{
    mContentFiles.push_back(file);
}

void OMW::Engine::addGroundcoverFile(const std::string& file)
{
    mGroundcoverFiles.emplace_back(file);
}

void OMW::Engine::setSkipMenu(bool skipMenu, bool newGame)
{
    mSkipMenu = skipMenu;
    mNewGame = newGame;
}

void OMW::Engine::createWindow()
{
    const int screen = Settings::video().mScreen;
    const int width = Settings::video().mResolutionX;
    const int height = Settings::video().mResolutionY;
    const Settings::WindowMode windowMode = Settings::video().mWindowMode;
    const bool windowBorder = Settings::video().mWindowBorder;
    const SDLUtil::VSyncMode vsync = Settings::video().mVsyncMode;
    unsigned antialiasing = static_cast<unsigned>(Settings::video().mAntialiasing);

    int posX = SDL_WINDOWPOS_CENTERED_DISPLAY(screen);
    int posY = SDL_WINDOWPOS_CENTERED_DISPLAY(screen);

    if (windowMode == Settings::WindowMode::Fullscreen || windowMode == Settings::WindowMode::WindowedFullscreen)
    {
        posX = SDL_WINDOWPOS_UNDEFINED_DISPLAY(screen);
        posY = SDL_WINDOWPOS_UNDEFINED_DISPLAY(screen);
    }

#ifdef __vita__
    Uint32 flags = SDL_WINDOW_SHOWN;
#else
    Uint32 flags = SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI;
#endif
    if (windowMode == Settings::WindowMode::Fullscreen)
        flags |= SDL_WINDOW_FULLSCREEN;
    else if (windowMode == Settings::WindowMode::WindowedFullscreen)
        flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;

    // Allows for Windows snapping features to properly work in borderless window
    SDL_SetHint("SDL_BORDERLESS_WINDOWED_STYLE", "1");
    SDL_SetHint("SDL_BORDERLESS_RESIZABLE_STYLE", "1");

    if (!windowBorder)
        flags |= SDL_WINDOW_BORDERLESS;

    SDL_SetHint(SDL_HINT_VIDEO_MINIMIZE_ON_FOCUS_LOSS, Settings::video().mMinimizeOnFocusLoss ? "1" : "0");

    checkSDLError(SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8));
    checkSDLError(SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8));
    checkSDLError(SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8));
    checkSDLError(SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 0));
    checkSDLError(SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24));
    if (Debug::shouldDebugOpenGL())
        checkSDLError(SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG));

    if (antialiasing > 0)
    {
        checkSDLError(SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1));
        checkSDLError(SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, antialiasing));
    }

    osg::ref_ptr<SDLUtil::GraphicsWindowSDL2> graphicsWindow;
    while (!graphicsWindow || !graphicsWindow->valid())
    {
        while (!mWindow)
        {
            mWindow = SDL_CreateWindow("OpenMW", posX, posY, width, height, flags);
            if (!mWindow)
            {
                // Try with a lower AA
                if (antialiasing > 0)
                {
                    Log(Debug::Warning) << "Warning: " << antialiasing << "x antialiasing not supported, trying "
                                        << antialiasing / 2;
                    antialiasing /= 2;
                    Settings::video().mAntialiasing.set(antialiasing);
                    checkSDLError(SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, antialiasing));
                    continue;
                }
                else
                {
                    std::stringstream error;
                    error << "Failed to create SDL window: " << SDL_GetError();
                    throw std::runtime_error(error.str());
                }
            }
        }

        // Since we use physical resolution internally, we have to create the window with scaled resolution,
        // but we can't get the scale before the window exists, so instead we have to resize aftewards.
        int w, h;
        SDL_GetWindowSize(mWindow, &w, &h);
        int dw, dh;
        SDL_GL_GetDrawableSize(mWindow, &dw, &dh);
        if (dw != w || dh != h)
        {
            SDL_SetWindowSize(mWindow, width / (dw / w), height / (dh / h));
        }

        setWindowIcon();

        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
        SDL_GetWindowPosition(mWindow, &traits->x, &traits->y);
        SDL_GL_GetDrawableSize(mWindow, &traits->width, &traits->height);
        traits->windowName = SDL_GetWindowTitle(mWindow);
        traits->windowDecoration = !(SDL_GetWindowFlags(mWindow) & SDL_WINDOW_BORDERLESS);
        traits->screenNum = SDL_GetWindowDisplayIndex(mWindow);
        traits->vsync = 0;
        traits->inheritedWindowData = new SDLUtil::GraphicsWindowSDL2::WindowData(mWindow);

        graphicsWindow = new SDLUtil::GraphicsWindowSDL2(traits, vsync);
        if (!graphicsWindow->valid())
            throw std::runtime_error("Failed to create GraphicsContext");

        if (traits->samples < antialiasing)
        {
            Log(Debug::Warning) << "Warning: Framebuffer MSAA level is only " << traits->samples << "x instead of "
                                << antialiasing << "x. Trying " << antialiasing / 2 << "x instead.";
            graphicsWindow->closeImplementation();
            SDL_DestroyWindow(mWindow);
            mWindow = nullptr;
            antialiasing /= 2;
            Settings::video().mAntialiasing.set(antialiasing);
            checkSDLError(SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, antialiasing));
            continue;
        }

        if (traits->red < 8)
            Log(Debug::Warning) << "Warning: Framebuffer only has a " << traits->red << " bit red channel.";
        if (traits->green < 8)
            Log(Debug::Warning) << "Warning: Framebuffer only has a " << traits->green << " bit green channel.";
        if (traits->blue < 8)
            Log(Debug::Warning) << "Warning: Framebuffer only has a " << traits->blue << " bit blue channel.";
        if (traits->depth < 24)
            Log(Debug::Warning) << "Warning: Framebuffer only has " << traits->depth << " bits of depth precision.";

        traits->alpha = 0; // set to 0 to stop ScreenCaptureHandler reading the alpha channel
    }

    osg::ref_ptr<osg::Camera> camera = mViewer->getCamera();
    camera->setGraphicsContext(graphicsWindow);
    camera->setViewport(0, 0, graphicsWindow->getTraits()->width, graphicsWindow->getTraits()->height);

    osg::ref_ptr<SceneUtil::OperationSequence> realizeOperations = new SceneUtil::OperationSequence(false);
    mViewer->setRealizeOperation(realizeOperations);
    osg::ref_ptr<IdentifyOpenGLOperation> identifyOp = new IdentifyOpenGLOperation();
    realizeOperations->add(identifyOp);
    realizeOperations->add(new SceneUtil::GetGLExtensionsOperation());

    if (Debug::shouldDebugOpenGL())
        realizeOperations->add(new Debug::EnableGLDebugOperation());

    realizeOperations->add(mSelectDepthFormatOperation);
    realizeOperations->add(mSelectColorFormatOperation);

    if (Stereo::getStereo())
    {
        Stereo::Settings settings;

        settings.mMultiview = Settings::stereo().mMultiview;
        settings.mAllowDisplayListsForMultiview = Settings::stereo().mAllowDisplayListsForMultiview;
        settings.mSharedShadowMaps = Settings::stereo().mSharedShadowMaps;

        if (Settings::stereo().mUseCustomView)
        {
            const osg::Vec3 leftEyeOffset(Settings::stereoView().mLeftEyeOffsetX,
                Settings::stereoView().mLeftEyeOffsetY, Settings::stereoView().mLeftEyeOffsetZ);

            const osg::Quat leftEyeOrientation(Settings::stereoView().mLeftEyeOrientationX,
                Settings::stereoView().mLeftEyeOrientationY, Settings::stereoView().mLeftEyeOrientationZ,
                Settings::stereoView().mLeftEyeOrientationW);

            const osg::Vec3 rightEyeOffset(Settings::stereoView().mRightEyeOffsetX,
                Settings::stereoView().mRightEyeOffsetY, Settings::stereoView().mRightEyeOffsetZ);

            const osg::Quat rightEyeOrientation(Settings::stereoView().mRightEyeOrientationX,
                Settings::stereoView().mRightEyeOrientationY, Settings::stereoView().mRightEyeOrientationZ,
                Settings::stereoView().mRightEyeOrientationW);

            settings.mCustomView = Stereo::CustomView{
                .mLeft = Stereo::View{
                    .pose = Stereo::Pose{
                        .position = leftEyeOffset,
                        .orientation = leftEyeOrientation,
                    },
                    .fov = Stereo::FieldOfView{
                        .angleLeft = Settings::stereoView().mLeftEyeFovLeft,
                        .angleRight = Settings::stereoView().mLeftEyeFovRight,
                        .angleUp = Settings::stereoView().mLeftEyeFovUp,
                        .angleDown = Settings::stereoView().mLeftEyeFovDown,
                    },
                },
                .mRight = Stereo::View{
                    .pose = Stereo::Pose{
                        .position = rightEyeOffset,
                        .orientation = rightEyeOrientation,
                    },
                    .fov = Stereo::FieldOfView{
                        .angleLeft = Settings::stereoView().mRightEyeFovLeft,
                        .angleRight = Settings::stereoView().mRightEyeFovRight,
                        .angleUp = Settings::stereoView().mRightEyeFovUp,
                        .angleDown = Settings::stereoView().mRightEyeFovDown,
                    },
                },
            };
        }

        if (Settings::stereo().mUseCustomEyeResolution)
            settings.mEyeResolution
                = osg::Vec2i(Settings::stereoView().mEyeResolutionX, Settings::stereoView().mEyeResolutionY);

        realizeOperations->add(new Stereo::InitializeStereoOperation(settings));
    }

    mViewer->realize();
    mGlMaxTextureImageUnits = identifyOp->getMaxTextureImageUnits();

#ifdef __vita__
    // Enable OSG matrix uniforms and vertex attribute aliasing for custom GLSL shaders.
    // Matrix uniforms: OSG provides osg_ModelViewProjectionMatrix, osg_ModelViewMatrix, osg_NormalMatrix.
    // Attribute aliasing: OSG maps glVertexPointer->glVertexAttribPointer(0), etc., so custom shaders
    // receive vertex data through generic attribute slots instead of FFP-only arrays.
    if (auto* gc = mViewer->getCamera()->getGraphicsContext())
    {
        gc->getState()->setUseModelViewAndProjectionUniforms(true);
        gc->getState()->setUseVertexAttributeAliasing(true);
    }
#endif

    mViewer->getEventQueue()->getCurrentEventState()->setWindowRectangle(
        0, 0, graphicsWindow->getTraits()->width, graphicsWindow->getTraits()->height);
}

void OMW::Engine::setWindowIcon()
{
    std::ifstream windowIconStream;
    const auto windowIcon = mResDir / "openmw.png";
    windowIconStream.open(windowIcon, std::ios_base::in | std::ios_base::binary);
    if (windowIconStream.fail())
        Log(Debug::Error) << "Error: Failed to open " << windowIcon;
    osgDB::ReaderWriter* reader = osgDB::Registry::instance()->getReaderWriterForExtension("png");
    if (!reader)
    {
        Log(Debug::Error) << "Error: Failed to read window icon, no png readerwriter found";
        return;
    }
    osgDB::ReaderWriter::ReadResult result = reader->readImage(windowIconStream);
    if (!result.success())
        Log(Debug::Error) << "Error: Failed to read " << windowIcon << ": " << result.message() << " code "
                          << result.status();
    else
    {
        osg::ref_ptr<osg::Image> image = result.getImage();
        auto surface = SDLUtil::imageToSurface(image, true);
        SDL_SetWindowIcon(mWindow, surface.get());
    }
}

void OMW::Engine::prepareEngine()
{
    VITA_CRUMB("prepareEngine() enter");
    mStateManager = std::make_unique<MWState::StateManager>(mCfgMgr.getUserDataPath() / "saves", mContentFiles);
    mEnvironment.setStateManager(*mStateManager);

    const bool stereoEnabled = Settings::stereo().mStereoEnabled || osg::DisplaySettings::instance().get()->getStereo();
    mStereoManager = std::make_unique<Stereo::Manager>(
        mViewer, stereoEnabled, Settings::camera().mNearClip, Settings::camera().mViewingDistance);

    osg::ref_ptr<osg::Group> rootNode(new osg::Group);
    mViewer->setSceneData(rootNode);

    VITA_CRUMB("prepareEngine() createWindow");
#ifdef __vita__
    Vita::logMemoryStatus("Pre-createWindow");
#endif
    createWindow();

    mVFS = std::make_unique<VFS::Manager>();

#ifdef __vita__
    {
        auto cacheDir = mCfgMgr.getUserConfigPath();
        if (mForceRescan)
        {
            Log(Debug::Info) << "Force rescan — clearing VFS caches";
            Files::clearScanCache(cacheDir / "scan_cache.bin");
            std::error_code ec;
            for (const auto& entry : std::filesystem::directory_iterator(cacheDir, ec))
                if (entry.path().filename().string().starts_with("vfs_dir_"))
                    std::filesystem::remove(entry.path(), ec);
        }
        VFS::registerArchives(mVFS.get(), mFileCollections, mArchives, true,
            &mEncoder.get()->getStatelessEncoder(), cacheDir);
    }

    {
        // Kick ESM reads now; the parser consumes them after GUI bring-up.
        std::vector<std::filesystem::path> toPrefetch;
        for (const std::string& file : mContentFiles)
        {
            if (Misc::getFileExtension(file) == "omwscripts")
                continue;
            const Files::MultiDirCollection& col = mFileCollections.getCollection(Misc::getFileExtension(file));
            if (col.doesExist(file))
                toPrefetch.push_back(col.getPath(file));
        }
        Vita::EsmPrefetch::start(std::move(toPrefetch));
    }
#else
    VFS::registerArchives(mVFS.get(), mFileCollections, mArchives, true, &mEncoder.get()->getStatelessEncoder());
#endif

    mResourceSystem = std::make_unique<Resource::ResourceSystem>(
        mVFS.get(), Settings::cells().mCacheExpiryDelay, &mEncoder.get()->getStatelessEncoder());
    mResourceSystem->getSceneManager()->getShaderManager().setMaxTextureUnits(mGlMaxTextureImageUnits);
#ifdef __vita__
    mResourceSystem->getSceneManager()->setUnRefImageDataAfterApply(true); // release CPU-side image after GPU upload
#else
    mResourceSystem->getSceneManager()->setUnRefImageDataAfterApply(
        false); // keep to Off for now to allow better state sharing
#endif
    mResourceSystem->getSceneManager()->setFilterSettings(Settings::general().mTextureMagFilter,
        Settings::general().mTextureMinFilter, Settings::general().mTextureMipmap,
        static_cast<float>(Settings::general().mAnisotropy));
    mEnvironment.setResourceSystem(*mResourceSystem);

    mWorkQueue = new SceneUtil::WorkQueue(Settings::cells().mPreloadNumThreads);
    mUnrefQueue = std::make_unique<SceneUtil::UnrefQueue>();

    mScreenCaptureOperation = new SceneUtil::AsyncScreenCaptureOperation(mWorkQueue,
        new SceneUtil::WriteScreenshotToFileOperation(mCfgMgr.getScreenshotPath(),
            Settings::general().mScreenshotFormat,
            Settings::general().mNotifyOnSavedScreenshot ? std::function<void(std::string)>(ScreenCaptureMessageBox{})
                                                         : std::function<void(std::string)>(IgnoreString{})));

    mScreenCaptureHandler = new osgViewer::ScreenCaptureHandler(mScreenCaptureOperation);

    mViewer->addEventHandler(mScreenCaptureHandler);

    mL10nManager = std::make_unique<L10n::Manager>(mVFS.get());
    mL10nManager->setPreferredLocales(Settings::general().mPreferredLocales, Settings::general().mGmstOverridesL10n);
    mEnvironment.setL10nManager(*mL10nManager);

    mLuaManager = std::make_unique<MWLua::LuaManager>(mVFS.get(), mResDir / "lua_libs");
    mEnvironment.setLuaManager(*mLuaManager);

    // Create input and UI first to set up a bootstrapping environment for
    // showing a loading screen and keeping the window responsive while doing so

    const auto keybinderUser = mCfgMgr.getUserConfigPath() / "input_v3.xml";
    bool keybinderUserExists = std::filesystem::exists(keybinderUser);
    if (!keybinderUserExists)
    {
        const auto input2 = (mCfgMgr.getUserConfigPath() / "input_v2.xml");
        if (std::filesystem::exists(input2))
        {
            keybinderUserExists = std::filesystem::copy_file(input2, keybinderUser);
            Log(Debug::Info) << "Loading keybindings file: " << keybinderUser;
        }
    }
    else
        Log(Debug::Info) << "Loading keybindings file: " << keybinderUser;

    const auto userdefault = mCfgMgr.getUserConfigPath() / "gamecontrollerdb.txt";
    const auto localdefault = mCfgMgr.getLocalPath() / "gamecontrollerdb.txt";

    std::filesystem::path userGameControllerdb;
    if (std::filesystem::exists(userdefault))
        userGameControllerdb = userdefault;

    std::filesystem::path gameControllerdb;
    if (std::filesystem::exists(localdefault))
        gameControllerdb = localdefault;
    else if (!mCfgMgr.getGlobalPath().empty())
    {
        const auto globaldefault = mCfgMgr.getGlobalPath() / "gamecontrollerdb.txt";
        if (std::filesystem::exists(globaldefault))
            gameControllerdb = globaldefault;
    }
    // else if it doesn't exist, pass in an empty path

    // gui needs our shaders path before everything else
    mResourceSystem->getSceneManager()->setShaderPath(mResDir / "shaders");

    osg::GLExtensions& exts = SceneUtil::getGLExtensions();
    bool shadersSupported = exts.glslLanguageVersion >= 1.2f;
#ifdef __vita__
    // vitaGL cannot compile OpenMW's GLSL shaders — force fixed-function path
    shadersSupported = false;
#endif

#if OSG_VERSION_LESS_THAN(3, 6, 6)
    // hack fix for https://github.com/openscenegraph/OpenSceneGraph/issues/1028
    if (!osg::isGLExtensionSupported(exts.contextID, "NV_framebuffer_multisample_coverage"))
        exts.glRenderbufferStorageMultisampleCoverageNV = nullptr;
#endif

    osg::ref_ptr<osg::Group> guiRoot = new osg::Group;
    guiRoot->setName("GUI Root");
    guiRoot->setNodeMask(MWRender::Mask_GUI);
    mStereoManager->disableStereoForNode(guiRoot);
    rootNode->addChild(guiRoot);

#ifdef __vita__
    Vita::logMemoryStatus("Post-VFS");
#endif
    VITA_CRUMB("prepareEngine() creating WindowManager");
    mWindowManager = std::make_unique<MWGui::WindowManager>(mWindow, mViewer, guiRoot, mResourceSystem.get(),
        mWorkQueue.get(), mCfgMgr.getLogPath(), mScriptConsoleMode, mTranslationDataStorage, mEncoding, mExportFonts,
        Version::getOpenmwVersionDescription(), shadersSupported, mCfgMgr);
    mEnvironment.setWindowManager(*mWindowManager);

    VITA_CRUMB("prepareEngine() creating InputManager");
    mInputManager = std::make_unique<MWInput::InputManager>(mWindow, mViewer, mScreenCaptureHandler, keybinderUser,
        keybinderUserExists, userGameControllerdb, gameControllerdb, mGrab);
    mEnvironment.setInputManager(*mInputManager);

    // Create sound system
#ifdef __vita__
    Vita::logMemoryStatus("Post-WindowManager");
#endif
    VITA_CRUMB("prepareEngine() creating SoundManager");
    mSoundManager = std::make_unique<MWSound::SoundManager>(mVFS.get(), mUseSound);
    mEnvironment.setSoundManager(*mSoundManager);

    // Create the world
    VITA_CRUMB("prepareEngine() creating World");
    mWorld = std::make_unique<MWWorld::World>(
        mResourceSystem.get(), mActivationDistanceOverride, mCellName, mCfgMgr.getUserDataPath());
    mEnvironment.setWorld(*mWorld);
    mEnvironment.setWorldModel(mWorld->getWorldModel());
    mEnvironment.setESMStore(mWorld->getStore());

    Loading::Listener* listener = MWBase::Environment::get().getWindowManager()->getLoadingScreen();
    Loading::AsyncListener asyncListener(*listener);
#ifdef __vita__
    if (Settings::general().mVitaLazyDialogue)
    {
        Vita::DialogueText::setEnabled(true);
        Vita::DialogueText::setEncoding(mEncoding);
    }
    Vita::logMemoryStatus("Pre-data-load");
    VITA_CRUMB("prepareEngine() loading data async");
    // Parse chases the prefetch reads started before createWindow.
    listener->loadingOn();
    {
        auto dataLoading = std::async(std::launch::async,
            [&] { mWorld->loadData(mFileCollections, mContentFiles, mGroundcoverFiles, mEncoder.get(), &asyncListener); });
        using namespace std::chrono_literals;
        while (dataLoading.wait_for(50ms) != std::future_status::ready)
            asyncListener.update();
        dataLoading.get();
    }
    listener->loadingOff();
    // Coalesce freed slurp buffers before initUI's allocation storm.
    malloc_trim(0);
    Files::saveScanCache(
        mCfgMgr.getUserConfigPath() / "scan_cache.bin", mDataDirs, mFileCollections.getCollections());
    Vita::logMemoryStatus("Post-data-load");
#else
    VITA_CRUMB("prepareEngine() loading data async");
    auto dataLoading = std::async(std::launch::async,
        [&] { mWorld->loadData(mFileCollections, mContentFiles, mGroundcoverFiles, mEncoder.get(), &asyncListener); });

    if (!mSkipMenu)
    {
        std::string_view logo = Fallback::Map::getString("Movies_Company_Logo");
        if (!logo.empty())
            mWindowManager->playVideo(logo, true);
    }

    listener->loadingOn();
    {
        using namespace std::chrono_literals;
        while (dataLoading.wait_for(50ms) != std::future_status::ready)
            asyncListener.update();
        dataLoading.get();
    }
    listener->loadingOff();
#endif
    VITA_CRUMB("prepareEngine() data loaded");

    VITA_CRUMB("prepareEngine() world init");
#ifdef __vita__
    Vita::logMemoryStatus("Pre-World::init");
#endif
    mWorld->init(mMaxRecastLogLevel, mViewer, std::move(rootNode), mWorkQueue.get(), *mUnrefQueue);
#ifdef __vita__
    Vita::logMemoryStatus("Post-World::init");
#endif
    VITA_CRUMB("prepareEngine() world init done");
    mEnvironment.setWorldScene(mWorld->getWorldScene());
    mWorld->setupPlayer();
    mWorld->setRandomSeed(mRandomSeed);

    const MWWorld::Store<ESM::GameSetting>* gmst = &mWorld->getStore().get<ESM::GameSetting>();
    mL10nManager->setGmstLoader([gmst, misses = std::set<std::string, Misc::StringUtils::CiComp>()](
                                    std::string_view gmstName) mutable -> const std::string* {
        const ESM::GameSetting* res = gmst->search(gmstName);
        if (res && res->mValue.getType() == ESM::VT_String)
            return &res->mValue.getString();
        if (misses.emplace(gmstName).second)
            Log(Debug::Error) << "GMST " << gmstName << " not found";
        return nullptr;
    });

    mWindowManager->setStore(mWorld->getStore());
#ifdef __vita__
    Vita::logMemoryStatus("Pre-initUI");
#endif
    mWindowManager->initUI();
#ifdef __vita__
    Vita::logMemoryStatus("Post-initUI");
    // UI textures land in the image cache; break down the initUI pool.
    Vita::auditResourceCaches(mResourceSystem.get());
#endif

    // Load translation data
    mTranslationDataStorage.setEncoder(mEncoder.get());
    for (auto& mContentFile : mContentFiles)
        mTranslationDataStorage.loadTranslationData(mFileCollections, mContentFile);

    Compiler::registerExtensions(mExtensions);

    // Create script system
    mScriptContext = std::make_unique<MWScript::CompilerContext>(MWScript::CompilerContext::Type_Full);
    mScriptContext->setExtensions(&mExtensions);

    mScriptManager = std::make_unique<MWScript::ScriptManager>(mWorld->getStore(), *mScriptContext, mWarningsMode);
    mEnvironment.setScriptManager(*mScriptManager);

    // Create game mechanics system
    mMechanicsManager = std::make_unique<MWMechanics::MechanicsManager>();
    mEnvironment.setMechanicsManager(*mMechanicsManager);

    // Create dialog system
    mJournal = std::make_unique<MWDialogue::Journal>();
    mEnvironment.setJournal(*mJournal);

    mDialogueManager = std::make_unique<MWDialogue::DialogueManager>(mExtensions, mTranslationDataStorage);
    mEnvironment.setDialogueManager(*mDialogueManager);

    // scripts
    if (mCompileAll)
    {
        std::pair<int, int> result = mScriptManager->compileAll();
        if (result.first)
            Log(Debug::Info) << "compiled " << result.second << " of " << result.first << " scripts ("
                             << 100 * static_cast<double>(result.second) / result.first << "%)";
    }
    if (mCompileAllDialogue)
    {
        std::pair<int, int> result = MWDialogue::ScriptTest::compileAll(&mExtensions, mWarningsMode);
        if (result.first)
            Log(Debug::Info) << "compiled " << result.second << " of " << result.first << " dialogue scripts ("
                             << 100 * static_cast<double>(result.second) / result.first << "%)";
    }

#ifdef __vita__
    Vita::logMemoryStatus("Pre-LuaInit");
#endif
    mLuaManager->loadPermanentStorage(mCfgMgr.getUserConfigPath());
    mLuaManager->init();

    // starts a separate lua thread if "lua num threads" > 0
    mLuaWorker = std::make_unique<MWLua::Worker>(*mLuaManager);
#ifdef __vita__
    Vita::logMemoryStatus("Post-LuaInit");
#endif
    VITA_CRUMB("prepareEngine() done");
}

// Initialise and enter main loop.
void OMW::Engine::go()
{
    assert(!mContentFiles.empty());

    Log(Debug::Info) << "OSG version: " << osgGetVersion();
    SDL_version sdlVersion;
    SDL_GetVersion(&sdlVersion);
    Log(Debug::Info) << "SDL version: " << (int)sdlVersion.major << "." << (int)sdlVersion.minor << "."
                     << (int)sdlVersion.patch;

    Misc::Rng::init(mRandomSeed);

    Settings::ShaderManager::get().load(mCfgMgr.getUserConfigPath() / "shaders.yaml");

    MWClass::registerClasses();

    // Create encoder
    mEncoder = std::make_unique<ToUTF8::Utf8Encoder>(mEncoding);

    // Setup viewer
    VITA_CRUMB("go() creating viewer");
    mViewer = new osgViewer::Viewer;
    mViewer->setReleaseContextAtEndOfFrameHint(false);

#ifdef __vita__
    // DrawThreadPerContext crashes on launch — vitaGL/SceGxm isn't safe
    // for draw submission from a non-main thread.
    mViewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    // Bin sort mode A/B via setting.
    if (Settings::general().mVitaStateSortedBins)
        osgUtil::RenderBin::setDefaultRenderBinSortMode(osgUtil::RenderBin::SORT_BY_STATE);
    else
        osgUtil::RenderBin::setDefaultRenderBinSortMode(osgUtil::RenderBin::TRAVERSAL_ORDER);
    // vitaGL is single-threaded; sim moves to a worker instead.
    mUpdateOverlap = Settings::general().mVitaUpdateOverlap;
    vita_draw_replay = Settings::general().mVitaDrawReplay ? 1 : 0;
    vita_state_replay = Settings::general().mVitaStateReplay ? 1 : 0;
    vglSetStaticVboRam(Settings::general().mVitaStaticVboRam ? 1 : 0);
    if (Settings::general().mVitaStaticBin)
    {
        osg::ref_ptr<osgUtil::RenderBin> proto = new osgUtil::RenderBin(osgUtil::RenderBin::SORT_BY_STATE);
        proto->setDrawCallback(new VitaStaticBinCallback);
        osgUtil::RenderBin::addRenderBinPrototype("VitaStaticBin", proto);
    }
    if (Settings::general().mVitaSimThread)
    {
        mSimWorker = std::make_unique<Vita::SimWorker>();
        mSimOverlap = Settings::general().mVitaSimOverlap;
        mCullOverlap = Settings::general().mVitaCullOverlap;
        // Nested render loops (loading, video) must consume a pending
        // culled frame first or the queue serves them a stale scene.
        Vita::setDrainDrawHook([this] {
            if (!mCullPrimed)
                return;
            // From sim thread: cull already ran (it precedes sim in the batch);
            // finish() here would deadlock on our own job.
            if (!Vita::isSimThread())
                mSimWorker->finish();
            auto* renderer = static_cast<osgViewer::Renderer*>(mViewer->getCamera()->getRenderer());
            if (Vita::GLWorker* glw = Vita::getGLWorker())
                glw->call([renderer] { renderer->draw(); });
            else
                renderer->draw();
            mCullPrimed = false;
        });
    }
#endif

    // Do not try to outsmart the OS thread scheduler (see bug #4785).
    mViewer->setUseConfigureAffinity(false);

    mEnvironment.setFrameRateLimit(Settings::video().mFramerateLimit);

    prepareEngine();

#ifdef _WIN32
    const auto* statsFile = _wgetenv(L"OPENMW_OSG_STATS_FILE");
#else
    const auto* statsFile = std::getenv("OPENMW_OSG_STATS_FILE");
#endif

    std::filesystem::path path;
    if (statsFile != nullptr)
        path = statsFile;

    std::ofstream stats;
    if (!path.empty())
    {
        stats.open(path, std::ios_base::out);
        if (stats.is_open())
            Log(Debug::Info) << "OSG stats will be written to: " << path;
        else
            Log(Debug::Warning) << "Failed to open file to write OSG stats \"" << path
                                << "\": " << std::generic_category().message(errno);
    }

    // Setup profiler
    osg::ref_ptr<Resource::Profiler> statsHandler = new Resource::Profiler(stats.is_open(), *mVFS);

    initStatsHandler(*statsHandler);

    mViewer->addEventHandler(statsHandler);

    osg::ref_ptr<Resource::StatsHandler> resourcesHandler = new Resource::StatsHandler(stats.is_open(), *mVFS);
    mViewer->addEventHandler(resourcesHandler);

    if (stats.is_open())
        Resource::collectStatistics(*mViewer);

    // Start the game
    VITA_CRUMB("go() starting game");
    if (!mSaveGameFile.empty())
    {
        mStateManager->loadGame(mSaveGameFile);
    }
    else if (!mSkipMenu)
    {
        // start in main menu
        VITA_CRUMB("go() pushing main menu");
        mWindowManager->pushGuiMode(MWGui::GM_MainMenu);

        if (mVFS->exists(MWSound::titleMusic))
            mSoundManager->streamMusic(MWSound::titleMusic, MWSound::MusicType::Normal);
        else
            Log(Debug::Warning) << "Title music not found";

        std::string_view logo = Fallback::Map::getString("Movies_Morrowind_Logo");
        if (!logo.empty())
            mWindowManager->playVideo(logo, /*allowSkipping*/ true, /*overrideSounds*/ false);
    }
    else
    {
        mStateManager->newGame(!mNewGame);
    }

    if (!mStartupScript.empty() && mStateManager->getState() == MWState::StateManager::State_Running)
    {
        mWindowManager->executeInConsole(mStartupScript);
    }

    // Start the main rendering loop
    VITA_CRUMB("go() entering main loop");
    MWWorld::DateTimeManager& timeManager = *mWorld->getTimeManager();
    Misc::FrameRateLimiter frameRateLimiter = Misc::makeFrameRateLimiter(mEnvironment.getFrameRateLimit());
    const std::chrono::steady_clock::duration maxSimulationInterval(std::chrono::milliseconds(200));
    while (!mViewer->done() && !mStateManager->hasQuitRequest())
    {
        const double dt = std::chrono::duration_cast<std::chrono::duration<double>>(
                              std::min(frameRateLimiter.getLastFrameDuration(), maxSimulationInterval))
                              .count()
            * timeManager.getSimulationTimeScale();

#ifdef __vita__
        // Cull overlap: worker reads the frame stamp; idle it before advance.
        if (mSimWorker && mCullOverlap && mSimPrimed)
            mSimWorker->finish();
#endif
        mViewer->advance(timeManager.getRenderingSimulationTime());

        const unsigned frameNumber = mViewer->getFrameStamp()->getFrameNumber();

        if (!frame(frameNumber, static_cast<float>(dt)))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        timeManager.updateIsPaused();
        if (!timeManager.isPaused())
        {
            timeManager.setSimulationTime(timeManager.getSimulationTime() + dt);
            timeManager.setRenderingSimulationTime(timeManager.getRenderingSimulationTime() + dt);
        }

        if (stats)
        {
            // The delay is required because rendering happens in parallel to the main thread and stats from there is
            // available with delay.
            constexpr unsigned statsReportDelay = 3;
            if (frameNumber >= statsReportDelay)
            {
                // Viewer frame number can be different from frameNumber because of loading screens which render new
                // frames inside a simulation frame.
                const unsigned currentFrameNumber = mViewer->getFrameStamp()->getFrameNumber();
                for (unsigned i = frameNumber; i <= currentFrameNumber; ++i)
                    reportStats(i - statsReportDelay, *mViewer, stats);
            }
        }

#ifdef __vita__
        // Vblank-locked pacing: kernel sleeps overshoot ~8ms.
        {
            static unsigned int s_lastVcount = 0;
            const float fpsLimit = mEnvironment.getFrameRateLimit();
            if (fpsLimit > 0.f)
            {
                const unsigned int vblanksPerFrame
                    = std::max(1u, (unsigned int)(60.f / fpsLimit + 0.5f));
                const uint64_t paceT0 = sceKernelGetProcessTimeWide();
                const unsigned int vc = sceDisplayGetVcount();
                if (s_lastVcount == 0 || vc >= s_lastVcount + vblanksPerFrame)
                    s_lastVcount = vc;
                else
                {
                    while (sceDisplayGetVcount() < s_lastVcount + vblanksPerFrame)
                        sceDisplayWaitVblankStart();
                    s_lastVcount += vblanksPerFrame;
                }
                phase_pace_us = (uint32_t)(sceKernelGetProcessTimeWide() - paceT0);
            }
        }
        frameRateLimiter.limit(); // dt bookkeeping only
#else
        frameRateLimiter.limit();
#endif
    }

    mLuaWorker->join();
#ifdef __vita__
    if (mSimWorker)
        mSimWorker->join();
    Vita::destroyGLWorker();
#endif

    // Save user settings
    Settings::Manager::saveUser(mCfgMgr.getUserConfigPath() / "settings.cfg");
    Settings::ShaderManager::get().save();
    mLuaManager->savePermanentStorage(mCfgMgr.getUserConfigPath());

#ifdef __vita__
    // Skip C++ static destructors — vitaGL/OSG/Bullet teardown paths can hang
    // on shutdown, leaving the app stuck instead of returning to LiveArea.
    // Saves above are already complete; nothing else needs to run for a clean
    // exit from the user's perspective.
    sceKernelExitProcess(0);
#endif
}

void OMW::Engine::setCompileAll(bool all)
{
    mCompileAll = all;
}

void OMW::Engine::setCompileAllDialogue(bool all)
{
    mCompileAllDialogue = all;
}

void OMW::Engine::setSoundUsage(bool soundUsage)
{
    mUseSound = soundUsage;
}

void OMW::Engine::setEncoding(const ToUTF8::FromType& encoding)
{
    mEncoding = encoding;
}

void OMW::Engine::setScriptConsoleMode(bool enabled)
{
    mScriptConsoleMode = enabled;
}

void OMW::Engine::setStartupScript(const std::filesystem::path& path)
{
    mStartupScript = path;
}

void OMW::Engine::setActivationDistanceOverride(int distance)
{
    mActivationDistanceOverride = distance;
}

void OMW::Engine::setWarningsMode(int mode)
{
    mWarningsMode = mode;
}

void OMW::Engine::enableFontExport(bool exportFonts)
{
    mExportFonts = exportFonts;
}

void OMW::Engine::setSaveGameFile(const std::filesystem::path& savegame)
{
    mSaveGameFile = savegame;
}

void OMW::Engine::setRandomSeed(unsigned int seed)
{
    mRandomSeed = seed;
}
