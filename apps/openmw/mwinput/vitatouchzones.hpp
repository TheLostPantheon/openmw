#ifndef MWINPUT_VITATOUCHZONES_H
#define MWINPUT_VITATOUCHZONES_H

#ifdef __vita__

namespace SDLUtil
{
    struct TouchEvent;
}

namespace MWInput
{
    class ActionManager;
    class BindingsManager;

    /// Front-panel touch zones: tap reveals the layout, hold keeps it,
    /// release inside a zone fires its action.
    class VitaTouchZones
    {
    public:
        struct Zone
        {
            float x0, y0, x1, y1; // normalized front-panel rect
            int action; // A_* id, or kMenuInfo
            bool gameplay;
            bool menu;
            const char* label;
        };
        static constexpr int kMenuInfo = -2; // synthesize R3 for GUI windows
        static constexpr int kZoneCount = 12;
        static const Zone sZones[kZoneCount];

        VitaTouchZones(ActionManager& actionManager, BindingsManager& bindingsManager);

        void onTouchDown(const SDLUtil::TouchEvent& e);
        void onTouchMove(const SDLUtil::TouchEvent& e);
        void onTouchUp(const SDLUtil::TouchEvent& e);
        void update(float dt);
        /// Reveal the layout unprompted (post-load discoverability).
        void showIntro(float seconds);

    private:
        int zoneAt(float x, float y, bool guiMode) const;
        void fire(int zone);
        void showOverlay();
        void hideOverlay();

        ActionManager& mActionManager;
        BindingsManager& mBindingsManager;
        bool mPovHeld = false;
        long long mActiveFinger = -1;
        int mHighlight = -1;
        float mFadeTimer = 0.f;
        bool mOverlayShown = false;
        bool mIntroPending = false;
        float mIntroSeconds = 0.f;
        bool mQkMenuOpen = false;
        float mQkRefresh = 0.f;
    };
}

#endif

#endif
