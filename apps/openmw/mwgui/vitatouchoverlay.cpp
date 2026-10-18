#include "vitatouchoverlay.hpp"

#ifdef __vita__

#include <algorithm>
#include <cstdio>

#include <psp2/io/stat.h>

#include <MyGUI_Gui.h>
#include <MyGUI_ImageBox.h>
#include <MyGUI_RenderManager.h>
#include <MyGUI_TextBox.h>
#include <MyGUI_Widget.h>

#include "../mwinput/actions.hpp"
#include "../mwinput/vitatouchzones.hpp"

#include "../vita/VitaInit.h"

#include "itemwidget.hpp"
#include "quickkeysmenu.hpp"

namespace MWGui
{
    namespace
    {
        // Solid black at an alpha of our own: the menu-transparency
        // background (BlackBG) rendered as a light veil over the scene.
        const MyGUI::Colour kIdleFill(0.f, 0.f, 0.f, 0.6f);
        const MyGUI::Colour kHotFill(0.42f, 0.32f, 0.12f, 0.88f);
        constexpr float kDimAlpha = 0.45f; // the hide's single dim step
        constexpr float kDimSeconds = 0.12f;

        using Zones = MWInput::VitaTouchZones;

        bool guiProf()
        {
            static const bool sOn = [] {
                SceIoStat st;
                return sceIoGetstat("ux0:data/openmw/guiprof.txt", &st) >= 0;
            }();
            return sOn;
        }

        MyGUI::ImageBox* makeFill(MyGUI::Widget* parent, const MyGUI::IntSize& size)
        {
            MyGUI::ImageBox* fill = parent->createWidget<MyGUI::ImageBox>(
                "ImageBox", MyGUI::IntCoord(0, 0, size.width, size.height), MyGUI::Align::Stretch);
            fill->setImageTexture("white");
            fill->setColour(kIdleFill);
            fill->setAlpha(kIdleFill.alpha);
            fill->setNeedMouseFocus(false);
            parent->createWidget<MyGUI::Widget>("MW_Box_Overlay", MyGUI::IntCoord(0, 0, size.width, size.height),
                MyGUI::Align::Stretch);
            return fill;
        }

        MyGUI::TextBox* makeText(MyGUI::Widget* parent, const MyGUI::IntCoord& coord)
        {
            MyGUI::TextBox* text = parent->createWidget<MyGUI::TextBox>("SandText", coord, MyGUI::Align::Default);
            text->setTextAlign(MyGUI::Align::Center);
            text->setTextShadow(true);
            text->setNeedMouseFocus(false);
            return text;
        }
    }

    VitaTouchOverlay::VitaTouchOverlay(QuickKeysMenu* quickKeys)
        : mQuickKeys(quickKeys)
    {
        const MyGUI::IntSize view = MyGUI::RenderManager::getInstance().getViewSize();
        const int w = view.width;
        const int h = view.height;
        mUnit = std::max(32, h * 60 / 544);
        const int margin = std::max(3, mUnit / 15);
        const int sysHeight = mUnit * 3 / 5;
        const int pad = mUnit / 4;

        mRoot = MyGUI::Gui::getInstance().createWidget<MyGUI::Widget>(
            "", MyGUI::IntCoord(0, 0, w, h), MyGUI::Align::Stretch, "Notification");
        mRoot->setNeedMouseFocus(false);
        mRoot->setVisible(false);

        mTiles.resize(Zones::kZoneCount);
        for (int i = 0; i < Zones::kZoneCount; ++i)
        {
            const Zones::Zone& z = Zones::sZones[i];
            Tile& tile = mTiles[i];
            if (z.action >= MWInput::A_QuickKey1 && z.action <= MWInput::A_QuickKey10)
                tile.key = z.action - MWInput::A_QuickKey1 + 1;

            // Compact tiles against the screen edge the zone touches; the
            // hit zones themselves stay as large as they were. Bottom corner
            // tiles sit at their zone's top, clear of the HUD bars and map.
            int tw = mUnit;
            int th = tile.key ? mUnit : sysHeight;
            if (!tile.key)
            {
                MyGUI::TextBox* probe = makeText(mRoot, MyGUI::IntCoord(0, 0, w, th));
                probe->setCaption(z.label);
                tw = std::max(mUnit, probe->getTextSize().width + 2 * pad);
                MyGUI::Gui::getInstance().destroyWidget(probe);
            }
            const int x0 = static_cast<int>(z.x0 * w), x1 = static_cast<int>(z.x1 * w);
            const int y0 = static_cast<int>(z.y0 * h), y1 = static_cast<int>(z.y1 * h);
            int x = (x0 + x1 - tw) / 2;
            if (z.x0 <= 0.f)
                x = margin;
            else if (z.x1 >= 1.f)
                x = w - tw - margin;
            int y = (y0 + y1 - th) / 2;
            if (z.y0 <= 0.f)
                y = margin;
            else if (z.y1 >= 1.f)
                y = tile.key ? h - th - margin : y0 + margin;

            tile.root = mRoot->createWidget<MyGUI::Widget>("", MyGUI::IntCoord(x, y, tw, th), MyGUI::Align::Default);
            tile.root->setNeedMouseFocus(false);
            tile.fill = makeFill(tile.root, MyGUI::IntSize(tw, th));
            if (tile.key)
            {
                const int icon = mUnit * 4 / 5;
                tile.icon = tile.root->createWidget<ItemWidget>("MW_ItemIcon",
                    MyGUI::IntCoord((tw - icon) / 2, (th - icon) / 2, icon, icon), MyGUI::Align::Default);
                tile.icon->setNeedMouseFocus(false);
            }
            tile.text = makeText(tile.root, MyGUI::IntCoord(0, 0, tw, th));
            tile.text->setCaption(
                tile.key ? MyGUI::UString(MyGUI::utility::toString(tile.key)) : MyGUI::UString(z.label));
        }

        mCaption = mRoot->createWidget<MyGUI::Widget>(
            "", MyGUI::IntCoord(0, 0, mUnit, sysHeight), MyGUI::Align::Default);
        mCaption->setNeedMouseFocus(false);
        makeFill(mCaption, MyGUI::IntSize(mUnit, sysHeight))->setAlpha(0.85f);
        mCaptionText = makeText(mCaption, MyGUI::IntCoord(0, 0, mUnit, sysHeight));
        mCaptionText->setAlign(MyGUI::Align::Stretch);
        mCaption->setVisible(false);
    }

    VitaTouchOverlay::~VitaTouchOverlay()
    {
        if (mRoot != nullptr)
            MyGUI::Gui::getInstance().destroyWidget(mRoot);
    }

    void VitaTouchOverlay::refreshKeys()
    {
        if (mQuickKeys == nullptr)
            return;
        for (int i = 0; i < static_cast<int>(mTiles.size()); ++i)
        {
            Tile& tile = mTiles[i];
            if (!tile.key)
                continue;
            std::string signature = mQuickKeys->vitaKeySignature(tile.key);
            if (signature == tile.signature)
                continue;
            tile.signature = std::move(signature);
            mChanged = true;
            mQuickKeys->vitaMirrorKey(tile.key, tile.icon);
            const bool assigned = !mQuickKeys->getKeyName(tile.key).empty();
            if (assigned != tile.assigned)
            {
                // Assigned: the number becomes a corner badge over the icon.
                tile.assigned = assigned;
                const int size = tile.root->getWidth();
                tile.text->setCoord(assigned ? MyGUI::IntCoord(size / 12, 0, size / 2, size / 2)
                                             : MyGUI::IntCoord(0, 0, size, size));
                tile.text->setTextAlign(assigned ? MyGUI::Align::Left | MyGUI::Align::Top : MyGUI::Align::Center);
            }
            if (i == mHighlight)
                showCaption(i);
        }
    }

    void VitaTouchOverlay::setVisible(bool visible)
    {
        if (visible)
        {
            refreshKeys();
            if (!mShown)
            {
                mRoot->setAlpha(1.f);
                mRoot->setVisible(true);
                mShown = true;
                mChanged = true;
                mChangedFrames = 0;
                mShownSeconds = 0.f;
            }
            else if (mHideTimer >= 0.f)
            {
                mRoot->setAlpha(1.f);
                mChanged = true;
            }
            mHideTimer = -1.f;
            return;
        }
        if (!mShown || mHideTimer >= 0.f)
            return;
        mRoot->setAlpha(kDimAlpha);
        mHideTimer = kDimSeconds;
        mChanged = true;
    }

    void VitaTouchOverlay::update(float dt)
    {
        if (!mShown)
            return;
        mShownSeconds += dt;
        if (mChanged)
        {
            ++mChangedFrames;
            mChanged = false;
        }
        if (mHideTimer < 0.f)
            return;
        mHideTimer -= dt;
        if (mHideTimer < 0.f)
        {
            mRoot->setVisible(false);
            mShown = false;
            if (guiProf())
            {
                char b[96];
                snprintf(b, sizeof(b), "[TouchOverlay] shown %.1f s, widgets changed in %u frames (incl. hide)",
                    mShownSeconds, mChangedFrames + 1);
                Vita::breadcrumb(b);
            }
        }
    }

    void VitaTouchOverlay::setHot(int zone, bool hot)
    {
        if (zone < 0 || zone >= static_cast<int>(mTiles.size()))
            return;
        mChanged = true;
        const MyGUI::Colour& colour = hot ? kHotFill : kIdleFill;
        mTiles[zone].fill->setColour(colour);
        mTiles[zone].fill->setAlpha(colour.alpha);
    }

    void VitaTouchOverlay::setHighlight(int zone)
    {
        if (zone == mHighlight)
            return;
        setHot(mHighlight, false);
        setHot(zone, true);
        mHighlight = zone;
        showCaption(zone);
    }

    void VitaTouchOverlay::showCaption(int zone)
    {
        // Names the quick key under the finger; system tiles carry their
        // own label.
        if (zone < 0 || zone >= static_cast<int>(mTiles.size()) || !mTiles[zone].key || mQuickKeys == nullptr)
        {
            if (mCaption->getVisible())
            {
                mCaption->setVisible(false);
                mChanged = true;
            }
            return;
        }
        const Tile& tile = mTiles[zone];
        const std::string name = mQuickKeys->getKeyName(tile.key);
        mCaptionText->setCaption(name.empty() ? std::string("Empty") : name);

        const MyGUI::IntSize view = mRoot->getSize();
        const int margin = std::max(3, mUnit / 15);
        const int ch = mCaption->getHeight();
        const int cw = std::min(view.width - 2 * margin, mCaptionText->getTextSize().width + mUnit / 2);
        const MyGUI::IntCoord t = tile.root->getCoord();
        const Zones::Zone& z = Zones::sZones[zone];
        int x = t.left + (t.width - cw) / 2;
        int y = t.top + t.height + margin; // below a top-edge tile
        if (z.y1 >= 1.f)
            y = t.top - ch - margin;
        else if (z.y0 > 0.f)
        {
            // Side tiles: beside, toward the screen centre.
            x = z.x0 <= 0.f ? t.left + t.width + margin : t.left - cw - margin;
            y = t.top + (t.height - ch) / 2;
        }
        x = std::clamp(x, margin, view.width - cw - margin);
        mCaption->setCoord(x, y, cw, ch);
        mChanged = true;
        if (!mCaption->getVisible())
            mCaption->setVisible(true);
    }
}

#endif
