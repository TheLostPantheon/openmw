#include "vitatouchzones.hpp"

#ifdef __vita__

#include <SDL_gamecontroller.h>

#include <components/sdlutil/events.hpp>

#include "../mwbase/environment.hpp"
#include "../mwbase/statemanager.hpp"
#include "../mwbase/windowmanager.hpp"
#include "../mwbase/luamanager.hpp"
#include "../mwbase/world.hpp"

#include "../mwworld/ptr.hpp"

#include "../mwgui/mode.hpp"
#include "../mwgui/vitatouchoverlay.hpp"
#include "../mwgui/windowbase.hpp"

#include "actionmanager.hpp"
#include "bindingsmanager.hpp"
#include "actions.hpp"

namespace MWInput
{
    // Corners hold system actions; edge slots are quick keys 1-6.
    // Quicksave stays top: a stray palm must not overwrite the slot.
    const VitaTouchZones::Zone VitaTouchZones::sZones[VitaTouchZones::kZoneCount] = {
        { 0.00f, 0.00f, 0.19f, 0.20f, A_Rest, true, false, "Wait" },
        { 0.81f, 0.00f, 1.00f, 0.20f, A_QuickSave, true, false, "Quick Save" },
        { 0.00f, 0.80f, 0.19f, 1.00f, A_Sneak, true, false, "Sneak" },
        { 0.81f, 0.80f, 1.00f, 1.00f, A_TogglePOV, true, true, "Camera" },
        { 0.00f, 0.35f, 0.08f, 0.65f, A_QuickKeysMenu, true, false, "Assign" },
        { 0.92f, 0.35f, 1.00f, 0.65f, A_QuickKey7, true, false, "7" },
        { 0.22f, 0.00f, 0.39f, 0.13f, A_QuickKey1, true, false, "1" },
        { 0.415f, 0.00f, 0.585f, 0.13f, A_QuickKey2, true, false, "2" },
        { 0.61f, 0.00f, 0.78f, 0.13f, A_QuickKey3, true, false, "3" },
        { 0.22f, 0.87f, 0.39f, 1.00f, A_QuickKey4, true, false, "4" },
        { 0.415f, 0.87f, 0.585f, 1.00f, A_QuickKey5, true, false, "5" },
        { 0.61f, 0.87f, 0.78f, 1.00f, A_QuickKey6, true, false, "6" },
    };

    VitaTouchZones::VitaTouchZones(ActionManager& actionManager, BindingsManager& bindingsManager)
        : mActionManager(actionManager)
        , mBindingsManager(bindingsManager)
    {
    }

    int VitaTouchZones::zoneAt(float x, float y, bool guiMode) const
    {
        for (int i = 0; i < kZoneCount; ++i)
        {
            const Zone& z = sZones[i];
            if (!(guiMode ? z.menu : z.gameplay))
                continue;
            if (x >= z.x0 && x < z.x1 && y >= z.y0 && y < z.y1)
                return i;
        }
        return -1;
    }

    void VitaTouchZones::onTouchDown(const SDLUtil::TouchEvent& e)
    {
        MWBase::StateManager* state = MWBase::Environment::get().getStateManager();
        if (state->getState() != MWBase::StateManager::State_Running)
            return;
        const bool guiMode = MWBase::Environment::get().getWindowManager()->isGuiMode();
        const int z = zoneAt(e.mX, e.mY, guiMode);
        if (z < 0)
            return;
        mActiveFinger = e.mFinger;
        mHighlight = z;
        if (!guiMode && sZones[z].action == A_TogglePOV)
        {
            // Drive the real action channel: tap toggles, hold previews —
            // the Lua camera script sees exactly a held binding.
            mBindingsManager.vitaSetActionValue(A_TogglePOV, true);
            mPovHeld = true;
        }
        showOverlay();
    }

    void VitaTouchZones::onTouchMove(const SDLUtil::TouchEvent& e)
    {
        if (e.mFinger != mActiveFinger)
            return;
        const bool guiMode = MWBase::Environment::get().getWindowManager()->isGuiMode();
        mHighlight = zoneAt(e.mX, e.mY, guiMode);
        if (mOverlayShown)
            if (MWGui::VitaTouchOverlay* ov = MWBase::Environment::get().getWindowManager()->getVitaTouchOverlay())
                ov->setHighlight(mHighlight);
    }

    void VitaTouchZones::onTouchUp(const SDLUtil::TouchEvent& e)
    {
        if (e.mFinger != mActiveFinger)
            return;
        mActiveFinger = -1;
        if (mPovHeld)
        {
            mBindingsManager.vitaSetActionValue(A_TogglePOV, false);
            mPovHeld = false;
            mFadeTimer = 0.6f;
            return;
        }
        const bool guiMode = MWBase::Environment::get().getWindowManager()->isGuiMode();
        const int z = zoneAt(e.mX, e.mY, guiMode);
        if (z >= 0)
            fire(z);
        mFadeTimer = 0.6f; // peek lingers, then fades
    }

    void VitaTouchZones::update(float dt)
    {
        if (mIntroPending
            && MWBase::Environment::get().getStateManager()->getState() == MWBase::StateManager::State_Running
            && !MWBase::Environment::get().getWindowManager()->isGuiMode())
        {
            mIntroPending = false;
            mHighlight = -1;
            showOverlay();
            mFadeTimer = mIntroSeconds;
        }
        // Assignment aid: while the quick keys menu is up, keep the zone
        // layout visible; captions re-read so fresh assignments show.
        if (MWBase::Environment::get().getWindowManager()->containsMode(MWGui::GM_QuickKeysMenu))
        {
            mQkRefresh -= dt;
            if (!mQkMenuOpen || mQkRefresh <= 0.f)
            {
                mHighlight = -1;
                showOverlay();
                mQkRefresh = 0.5f;
            }
            mQkMenuOpen = true;
            mFadeTimer = 0.6f;
        }
        else
            mQkMenuOpen = false;
        if (!mOverlayShown || mActiveFinger != -1)
            return;
        mFadeTimer -= dt;
        if (mFadeTimer <= 0.f)
            hideOverlay();
    }

    void VitaTouchZones::fire(int zone)
    {
        const Zone& z = sZones[zone];
        MWBase::WindowManager* winMgr = MWBase::Environment::get().getWindowManager();
        if (winMgr->isGuiMode())
        {
            if (z.menu)
            {
                // Menu context: R3 = the info/description toggle windows expect.
                if (MWGui::WindowBase* topWin = winMgr->getActiveControllerWindow())
                {
                    SDL_ControllerButtonEvent btn = {};
                    btn.type = SDL_CONTROLLERBUTTONDOWN;
                    btn.button = SDL_CONTROLLER_BUTTON_RIGHTSTICK;
                    btn.state = SDL_PRESSED;
                    topWin->onControllerButtonEvent(btn);
                }
            }
            return;
        }
        if (z.action == A_TogglePOV)
            return; // handled by channel press/release
        if (z.action == A_Sneak)
        {
            // Touch is always a toggle; same field Lua's ToggleSneak flips.
            const MWWorld::Ptr player = MWBase::Environment::get().getWorld()->getPlayerPtr();
            if (MWBase::LuaManager::ActorControls* controls
                = MWBase::Environment::get().getLuaManager()->getActorControls(player))
                controls->mSneak = !controls->mSneak;
            return;
        }
        mActionManager.executeAction(z.action);
    }

    void VitaTouchZones::showIntro(float seconds)
    {
        // Deferred: loading screens render GUI layers, so showing now
        // paints over them and burns the timer. update() arms it on the
        // first live gameplay frame.
        mIntroPending = true;
        mIntroSeconds = seconds;
    }

    void VitaTouchZones::showOverlay()
    {
        mFadeTimer = 0.6f;
        if (MWGui::VitaTouchOverlay* ov = MWBase::Environment::get().getWindowManager()->getVitaTouchOverlay())
        {
            ov->setVisible(true);
            ov->setHighlight(mHighlight);
            mOverlayShown = true;
        }
    }

    void VitaTouchZones::hideOverlay()
    {
        if (MWGui::VitaTouchOverlay* ov = MWBase::Environment::get().getWindowManager()->getVitaTouchOverlay())
            ov->setVisible(false);
        mOverlayShown = false;
    }
}

#endif
