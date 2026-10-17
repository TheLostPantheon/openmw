#ifndef MWGUI_VITATOUCHOVERLAY_H
#define MWGUI_VITATOUCHOVERLAY_H

#ifdef __vita__

#include <string>
#include <vector>

namespace MyGUI
{
    class ImageBox;
    class Widget;
    class TextBox;
}

namespace MWGui
{
    class ItemWidget;
    class QuickKeysMenu;

    /// Tiles revealed while a touch-zone gesture is active: quick keys show
    /// their item or spell icon, the tile under the finger names it.
    ///
    /// Any widget change makes the GUI re-walk every layer that frame, so
    /// widgets are only touched when something changed: a still overlay,
    /// a finger moving within one zone or an open quick keys menu cost
    /// nothing; hiding takes two steps.
    class VitaTouchOverlay
    {
    public:
        explicit VitaTouchOverlay(QuickKeysMenu* quickKeys);
        ~VitaTouchOverlay();

        void setVisible(bool visible);
        void setHighlight(int zone);
        void update(float dt);

    private:
        struct Tile
        {
            MyGUI::Widget* root = nullptr;
            MyGUI::ImageBox* fill = nullptr;
            MyGUI::TextBox* text = nullptr;
            ItemWidget* icon = nullptr; // quick keys only
            int key = 0; // quick key number, 0 for system actions
            std::string signature; // quick key assignment last shown
            bool assigned = false;
        };

        void refreshKeys();
        void setHot(int zone, bool hot);
        void showCaption(int zone);

        QuickKeysMenu* mQuickKeys;
        MyGUI::Widget* mRoot = nullptr;
        std::vector<Tile> mTiles;
        MyGUI::Widget* mCaption = nullptr;
        MyGUI::TextBox* mCaptionText = nullptr;
        int mUnit = 1; // tile size in GUI pixels
        int mHighlight = -1;
        bool mShown = false;
        float mHideTimer = -1.f; // >= 0 while the hide's dim step shows
        // [TouchOverlay] (guiprof.txt): frames that changed a widget per showing
        bool mChanged = false;
        unsigned mChangedFrames = 0;
        float mShownSeconds = 0.f;
    };
}

#endif

#endif
