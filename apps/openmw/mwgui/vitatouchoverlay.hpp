#ifndef MWGUI_VITATOUCHOVERLAY_H
#define MWGUI_VITATOUCHOVERLAY_H

#ifdef __vita__

#include <vector>

namespace MyGUI
{
    class Widget;
    class TextBox;
}

namespace MWGui
{
    class QuickKeysMenu;

    /// Translucent buttons revealed while a touch-zone gesture is active.
    class VitaTouchOverlay
    {
    public:
        explicit VitaTouchOverlay(QuickKeysMenu* quickKeys);
        ~VitaTouchOverlay();

        void setVisible(bool visible);
        void setHighlight(int zone);

    private:
        struct Btn
        {
            MyGUI::Widget* box = nullptr;
            MyGUI::TextBox* text = nullptr;
        };
        std::vector<Btn> mButtons;
        QuickKeysMenu* mQuickKeys;
        int mHighlight = -1;
    };
}

#endif

#endif
