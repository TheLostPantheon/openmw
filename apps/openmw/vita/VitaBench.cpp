#include "VitaBench.h"

#ifdef __vita__

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <psp2/io/fcntl.h>
#include <psp2/io/stat.h>
#include <psp2/kernel/processmgr.h>

#include <components/esm/refid.hpp>
#include <components/esm3/loadcell.hpp>

#include "../mwbase/environment.hpp"
#include "../mwbase/inputmanager.hpp"
#include "../mwbase/rotationflags.hpp"
#include "../mwbase/statemanager.hpp"
#include "../mwbase/world.hpp"
#include "../mwworld/cellstore.hpp"
#include "../mwworld/ptr.hpp"
#include "../mwinput/actions.hpp"

#include "VitaBuildInfo.h"
#include "VitaInit.h"

// Frame pacing wait of the previous frame (engine.cpp): work = frame - pace.
extern "C" uint32_t phase_pace_us;
extern "C" void vitaLogFlushNow(void);

namespace Vita
{
    namespace
    {
        constexpr const char* kDir = "ux0:data/openmw/bench";
        constexpr const char* kRoute = "ux0:data/openmw/bench/route.txt";
        constexpr const char* kRoutePrev = "ux0:data/openmw/bench/route.prev.txt";
        constexpr const char* kResults = "ux0:data/openmw/bench/results.txt";
        constexpr float kSampleSeconds = 0.1f;

        enum class Mode
        {
            Off,
            Record,
            Replay
        };

        struct Sample
        {
            float mT;
            float mPos[3];
            float mPitch, mYaw;
            std::string mCell; // serialized cell id
        };

        struct Config
        {
            Mode mMode = Mode::Off;
            std::string mSave;
            std::string mRoute = kRoute; // replay: "route=<file in bench/>"
            float mSettleSeconds = 2.f; // replay: "settle=<seconds>" before timing
            float mFps = -1.f; // replay: "fps=<n>" frame cap and streaming target (0: no cap, target 60)
            // replay: "shot=<s>[,<s>...]" screenshots that many seconds after
            // the start is placed, with the touch-zone overlay shown first
            std::vector<float> mShots;
            // replay: "race" moves the player on main right after the sim
            // join without waiting for the physics step (the overlap that
            // corrupted Bullet; a lock test)
            bool mRace = false;
            bool mExit = false;
        };

        bool exists(const char* path)
        {
            SceIoStat st;
            return sceIoGetstat(path, &st) >= 0;
        }

        const Config& config()
        {
            static const Config sConfig = [] {
                Config c;
                if (FILE* f = fopen("ux0:data/openmw/bench.txt", "r"))
                {
                    c.mMode = Mode::Replay;
                    char line[256];
                    while (fgets(line, sizeof(line), f))
                    {
                        line[strcspn(line, "\r\n")] = 0;
                        if (strncmp(line, "save=", 5) == 0)
                            c.mSave = line + 5;
                        else if (strncmp(line, "route=", 6) == 0)
                            c.mRoute = std::string(kDir) + "/" + (line + 6);
                        else if (strncmp(line, "settle=", 7) == 0)
                            c.mSettleSeconds = (float)atof(line + 7);
                        else if (strncmp(line, "fps=", 4) == 0)
                            c.mFps = (float)atof(line + 4);
                        else if (strncmp(line, "shot=", 5) == 0)
                        {
                            for (const char* p = line + 5; *p;)
                            {
                                c.mShots.push_back((float)atof(p));
                                p += strcspn(p, ",");
                                if (*p == ',')
                                    ++p;
                            }
                        }
                        else if (strcmp(line, "exit") == 0)
                            c.mExit = true;
                        else if (strcmp(line, "race") == 0)
                            c.mRace = true;
                    }
                    fclose(f);
                }
                else if (exists("ux0:data/openmw/benchrec.txt"))
                    c.mMode = Mode::Record;
                return c;
            }();
            return sConfig;
        }

        double nowSeconds()
        {
            return sceKernelGetProcessTimeWide() / 1e6;
        }

        bool inGame()
        {
            return MWBase::Environment::get().getStateManager()->getState()
                == MWBase::StateManager::State_Running;
        }

        std::string cellIdOf(const MWWorld::Ptr& ptr)
        {
            return ptr.isInCell() ? ptr.getCell()->getCell()->getId().serializeText() : std::string();
        }

        // ---- record
        struct Recorder
        {
            FILE* mFile = nullptr;
            double mStart = 0;
            double mNext = 0;
            std::string mLastCell;
            unsigned mSamples = 0;

            void tick()
            {
                if (!inGame())
                    return;
                const double now = nowSeconds();
                if (!mFile)
                {
                    sceIoMkdir(kDir, 0777);
                    sceIoRemove(kRoutePrev);
                    sceIoRename(kRoute, kRoutePrev);
                    mFile = fopen(kRoute, "w");
                    if (!mFile)
                        return;
                    fprintf(mFile, "# OpenMW Vita bench route v1: t x y z pitch yaw; 'cell <id>' on change\n");
                    mStart = now;
                    mNext = now;
                    breadcrumb("[Bench] recording route to bench/route.txt (benchrec.txt)");
                }
                if (now < mNext)
                    return;
                mNext = now + kSampleSeconds;
                const MWWorld::Ptr player = MWBase::Environment::get().getWorld()->getPlayerPtr();
                const std::string cell = cellIdOf(player);
                if (cell != mLastCell)
                {
                    fprintf(mFile, "cell %s\n", cell.c_str());
                    mLastCell = cell;
                }
                const ESM::Position& p = player.getRefData().getPosition();
                fprintf(mFile, "%.2f %.1f %.1f %.1f %.4f %.4f\n", now - mStart, p.pos[0], p.pos[1], p.pos[2], p.rot[0],
                    p.rot[2]);
                // Flush every second: the route survives a crash or power-off.
                if (++mSamples % 10 == 0)
                    fflush(mFile);
            }
        };

        // The frame's target, computed on main, applied by the sim job.
        struct PendingMove
        {
            bool mValid = false;
            osg::Vec3f mPos;
            osg::Vec3f mRot;
        } sPending;

        // ---- replay
        struct Player
        {
            std::vector<Sample> mRoute;
            bool mLoaded = false;
            bool mStarted = false;
            bool mDone = false;
            double mStart = 0;
            size_t mCursor = 0;
            bool mPlaced = false; // at the route's first sample, loaded
            double mSettleUntil = 0; // timing starts once streaming has settled here
            double mPlacedAt = 0;
            size_t mNextShot = 0;
            std::string mCurrentCell;
            std::vector<float> mFrameMs;
            std::vector<float> mWorkMs; // frame time minus the frame cap's pacing wait

            bool load()
            {
                const std::string& path = config().mRoute;
                FILE* f = fopen(path.c_str(), "r");
                if (!f)
                {
                    const std::string msg = "[Bench] route file missing, nothing to replay: " + path;
                    breadcrumb(msg.c_str());
                    return false;
                }
                char line[256];
                std::string cell;
                while (fgets(line, sizeof(line), f))
                {
                    if (line[0] == '#')
                        continue;
                    line[strcspn(line, "\r\n")] = 0;
                    if (strncmp(line, "cell ", 5) == 0)
                    {
                        cell = line + 5;
                        continue;
                    }
                    Sample s;
                    if (sscanf(line, "%f %f %f %f %f %f", &s.mT, &s.mPos[0], &s.mPos[1], &s.mPos[2], &s.mPitch, &s.mYaw)
                        == 6)
                    {
                        s.mCell = cell;
                        mRoute.push_back(std::move(s));
                    }
                }
                fclose(f);
                char b[160];
                snprintf(b, sizeof(b), "[Bench] replaying %s: %u samples, %.0f s, settle %.0f s", path.c_str(),
                    (unsigned)mRoute.size(), mRoute.empty() ? 0.0 : mRoute.back().mT, config().mSettleSeconds);
                breadcrumb(b);
                return mRoute.size() >= 2;
            }

            static float lerpAngle(float a, float b, float f)
            {
                float d = b - a;
                while (d > 3.14159265f)
                    d -= 6.2831853f;
                while (d < -3.14159265f)
                    d += 6.2831853f;
                return a + d * f;
            }

            void finish()
            {
                mDone = true;
                std::vector<float> v = mFrameMs;
                if (v.empty())
                    return;
                std::sort(v.begin(), v.end());
                const auto pct = [&](double p) { return v[std::min(v.size() - 1, (size_t)(v.size() * p))]; };
                double sum = 0;
                unsigned over50 = 0, over100 = 0;
                for (float ms : v)
                {
                    sum += ms;
                    over50 += ms > 50.f;
                    over100 += ms > 100.f;
                }
                // Work time: what the frame cost before the 30 fps cap's
                // pacing wait, so savings show even when frames are capped.
                std::vector<float> w = mWorkMs;
                std::sort(w.begin(), w.end());
                double wsum = 0;
                for (float ms : w)
                    wsum += ms;
                const auto wpct = [&](double p) { return w[std::min(w.size() - 1, (size_t)(w.size() * p))]; };
                char b[320];
                snprintf(b, sizeof(b),
                    "[Bench] done build=%s frames=%u avg=%.1fms median=%.1f p90=%.1f p99=%.1f max=%.0f "
                    "fps_median=%.1f hitches>50ms=%u >100ms=%u work avg=%.1f median=%.1f p90=%.1f",
                    buildId(), (unsigned)v.size(), sum / v.size(), pct(0.5), pct(0.9), pct(0.99), v.back(),
                    1000.0 / pct(0.5), over50, over100, wsum / w.size(), wpct(0.5), wpct(0.9));
                breadcrumb(b);
                if (FILE* f = fopen(kResults, "a"))
                {
                    fprintf(f, "%s\n", b + strlen("[Bench] done "));
                    fclose(f);
                }
                // The result must reach boot.log (and the log server) before
                // the quit tears the process down.
                vitaLogFlushNow();
                if (config().mExit)
                    MWBase::Environment::get().getStateManager()->requestQuit();
            }

            void tick(float frameSeconds)
            {
                if (mDone)
                    return;
                if (!mLoaded)
                {
                    mLoaded = true;
                    if (!load())
                    {
                        mDone = true;
                        return;
                    }
                }
                if (!inGame())
                    return;
                MWBase::World* world = MWBase::Environment::get().getWorld();
                if (!mPlaced)
                {
                    // Start where the recording started, through a real
                    // teleport (loading screen), whatever the save's place:
                    // the first per-frame move would otherwise jump across
                    // the map with streaming behind it (that crashed).
                    mPlaced = true;
                    const Sample& first = mRoute.front();
                    const MWWorld::Ptr player = world->getPlayerPtr();
                    const osg::Vec3f at = player.getRefData().getPosition().asVec3();
                    const osg::Vec3f want(first.mPos[0], first.mPos[1], first.mPos[2]);
                    if (!first.mCell.empty() && (cellIdOf(player) != first.mCell || (at - want).length() > 256.f))
                    {
                        ESM::Position p;
                        p.pos[0] = want.x();
                        p.pos[1] = want.y();
                        p.pos[2] = want.z();
                        p.rot[0] = first.mPitch;
                        p.rot[1] = 0.f;
                        p.rot[2] = first.mYaw;
                        breadcrumb("[Bench] teleporting to the route start");
                        world->changeToCell(ESM::RefId::deserializeText(first.mCell), p, false);
                        mCurrentCell = first.mCell;
                    }
                    // Let streaming settle at the start point before timing: the
                    // post-load fill is not what a route measures.
                    mSettleUntil = nowSeconds() + config().mSettleSeconds;
                    mPlacedAt = nowSeconds();
                    if (!config().mShots.empty())
                        MWBase::Environment::get().getInputManager()->vitaShowTouchIntro();
                    return;
                }
                while (mNextShot < config().mShots.size()
                    && nowSeconds() - mPlacedAt >= config().mShots[mNextShot])
                {
                    ++mNextShot;
                    breadcrumb("[Bench] screenshot");
                    MWBase::Environment::get().getInputManager()->executeAction(MWInput::A_Screenshot);
                }
                if (nowSeconds() < mSettleUntil)
                    return;
                const double now = nowSeconds();
                if (!mStarted)
                {
                    mStarted = true;
                    mStart = now;
                    breadcrumb("[Bench] route started");
                }
                else
                {
                    const float frameMs = frameSeconds * 1000.f;
                    mFrameMs.push_back(frameMs);
                    mWorkMs.push_back(std::max(0.f, frameMs - phase_pace_us / 1000.f));
                }

                const float t = (float)(now - mStart);
                if (t >= mRoute.back().mT)
                {
                    finish();
                    return;
                }
                while (mCursor + 1 < mRoute.size() && mRoute[mCursor + 1].mT <= t)
                    ++mCursor;
                const Sample& a = mRoute[mCursor];
                const Sample& b = mRoute[std::min(mCursor + 1, mRoute.size() - 1)];
                const float span = b.mT - a.mT;
                const float f = span > 0.f ? std::clamp((t - a.mT) / span, 0.f, 1.f) : 0.f;

                MWWorld::Ptr player = world->getPlayerPtr();
                const osg::Vec3f pos(a.mPos[0] + (b.mPos[0] - a.mPos[0]) * f, a.mPos[1] + (b.mPos[1] - a.mPos[1]) * f,
                    a.mPos[2] + (b.mPos[2] - a.mPos[2]) * f);
                // Doors on the route: entering or leaving an interior is a
                // teleport, like the door was. Exterior to exterior stays a
                // move, so streaming crosses cells exactly as walking does.
                const ESM::RefId cellId
                    = a.mCell.empty() ? ESM::RefId() : ESM::RefId::deserializeText(a.mCell);
                const bool toExterior = cellId.is<ESM::ESM3ExteriorCellRefId>();
                const bool inExterior = player.isInCell() && player.getCell()->isExterior();
                if (!a.mCell.empty() && a.mCell != mCurrentCell && a.mCell != cellIdOf(player)
                    && !(toExterior && inExterior))
                {
                    ESM::Position p;
                    p.pos[0] = pos.x();
                    p.pos[1] = pos.y();
                    p.pos[2] = pos.z();
                    p.rot[0] = a.mPitch;
                    p.rot[1] = 0.f;
                    p.rot[2] = a.mYaw;
                    world->changeToCell(cellId, p, false);
                    mCurrentCell = a.mCell;
                    return;
                }
                mCurrentCell = a.mCell;
                // Applied at the start of the sim job (benchApplyMove), where
                // scripts move objects too and the async physics step is
                // normally already done.
                sPending.mPos = pos;
                sPending.mRot
                    = osg::Vec3f(lerpAngle(a.mPitch, b.mPitch, f), 0.f, lerpAngle(a.mYaw, b.mYaw, f));
                sPending.mValid = true;
            }
        };

        Recorder sRecorder;
        Player sPlayer;
    }

    std::filesystem::path benchSaveToLoad(const std::filesystem::path& savesDir)
    {
        const Config& c = config();
        if (c.mMode != Mode::Replay)
            return {};
        std::filesystem::path best;
        std::filesystem::file_time_type bestTime{};
        std::error_code ec;
        for (const auto& character : std::filesystem::directory_iterator(savesDir, ec))
        {
            if (!character.is_directory())
                continue;
            for (const auto& save : std::filesystem::directory_iterator(character.path(), ec))
            {
                if (save.path().extension() != ".omwsave")
                    continue;
                if (!c.mSave.empty())
                {
                    if (save.path().filename() == c.mSave)
                        return save.path();
                    continue;
                }
                const auto t = save.last_write_time(ec);
                if (best.empty() || t > bestTime)
                {
                    best = save.path();
                    bestTime = t;
                }
            }
        }
        const std::string msg = "[Bench] loading save " + (best.empty() ? std::string("(none found)") : best.string());
        breadcrumb(msg.c_str());
        return best;
    }

    bool benchReplayActive()
    {
        return config().mMode == Mode::Replay;
    }

    float benchFrameCap()
    {
        const Config& c = config();
        return c.mMode == Mode::Replay ? c.mFps : -1.f;
    }

    float benchStreamingTarget()
    {
        const Config& c = config();
        if (c.mMode != Mode::Replay || c.mFps < 0.f)
            return -1.f;
        return c.mFps > 0.f ? c.mFps : 60.f;
    }

    void benchApplyMove()
    {
        if (!sPending.mValid)
            return;
        sPending.mValid = false;
        MWBase::World* world = MWBase::Environment::get().getWorld();
        // Moving the player's collision object while the async physics step
        // ran corrupted Bullet's broadphase (two replay crashes): wait for
        // the step first. By the sim job's start it has usually finished.
        if (!config().mRace)
            world->vitaWaitPhysics();
        MWWorld::Ptr player = world->moveObject(world->getPlayerPtr(), sPending.mPos);
        world->rotateObject(player, sPending.mRot, MWBase::RotationFlag_none);
    }

    void benchTick(float frameSeconds)
    {
        switch (config().mMode)
        {
            case Mode::Off:
                return;
            case Mode::Record:
                sRecorder.tick();
                return;
            case Mode::Replay:
                sPlayer.tick(frameSeconds);
                // "race": move now, on main after the sim join, where the
                // async physics step may still run (the original crash site).
                if (config().mRace)
                    benchApplyMove();
                return;
        }
    }
}

#endif // __vita__
