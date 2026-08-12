#include "vitatouchoverlay.hpp"

#ifdef __vita__

#include <MyGUI_Gui.h>
#include <MyGUI_RenderManager.h>
#include <MyGUI_TextBox.h>
#include <MyGUI_Widget.h>

#include "../mwinput/actions.hpp"
#include "../mwinput/vitatouchzones.hpp"

#include "quickkeysmenu.hpp"

namespace MWGui
{
    namespace
    {
        constexpr float kIdleAlpha = 0.45f;
        constexpr float kHotAlpha = 0.85f;
    }

    VitaTouchOverlay::VitaTouchOverlay(QuickKeysMenu* quickKeys)
        : mQuickKeys(quickKeys)
    {
        const MyGUI::IntSize view = MyGUI::RenderManager::getInstance().getViewSize();
        MyGUI::Gui& gui = MyGUI::Gui::getInstance();
        mButtons.resize(MWInput::VitaTouchZones::kZoneCount);
        for (int i = 0; i < MWInput::VitaTouchZones::kZoneCount; ++i)
        {
            const auto& z = MWInput::VitaTouchZones::sZones[i];
            const MyGUI::IntCoord coord((int)(z.x0 * view.width), (int)(z.y0 * view.height),
                (int)((z.x1 - z.x0) * view.width), (int)((z.y1 - z.y0) * view.height));
            Btn& btn = mButtons[i];
            btn.box = gui.createWidget<MyGUI::Widget>("BlackBG", coord, MyGUI::Align::Default, "Notification");
            btn.box->setNeedMouseFocus(false);
            btn.text = btn.box->createWidget<MyGUI::TextBox>(
                "SandText", MyGUI::IntCoord(0, 0, coord.width, coord.height), MyGUI::Align::Stretch);
            btn.text->setTextAlign(MyGUI::Align::Center);
            btn.text->setNeedMouseFocus(false);
            btn.text->setCaption(z.label);
            btn.box->setVisible(false);
        }
    }

    VitaTouchOverlay::~VitaTouchOverlay()
    {
        for (Btn& btn : mButtons)
            if (btn.box != nullptr)
                MyGUI::Gui::getInstance().destroyWidget(btn.box);
    }

    void VitaTouchOverlay::setVisible(bool visible)
    {
        for (int i = 0; i < (int)mButtons.size(); ++i)
        {
            const auto& z = MWInput::VitaTouchZones::sZones[i];
            if (visible && z.action >= MWInput::A_QuickKey1 && z.action <= MWInput::A_QuickKey10 && mQuickKeys)
            {
                const std::string name = mQuickKeys->getKeyName(z.action - MWInput::A_QuickKey1 + 1);
                mButtons[i].text->setCaption(name.empty() ? z.label : std::string(z.label) + "\n" + name);
            }
            mButtons[i].box->setAlpha(i == mHighlight ? kHotAlpha : kIdleAlpha);
            mButtons[i].box->setVisible(visible);
        }
    }

    void VitaTouchOverlay::setHighlight(int zone)
    {
        mHighlight = zone;
        for (int i = 0; i < (int)mButtons.size(); ++i)
            mButtons[i].box->setAlpha(i == zone ? kHotAlpha : kIdleAlpha);
    }
}

#endif
