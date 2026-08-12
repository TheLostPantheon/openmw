#include "controllermanager.hpp"

#include <MyGUI_Button.h>
#include <MyGUI_InputManager.h>

#include <SDL.h>

#include <components/debug/debuglog.hpp>
#include <components/esm/refid.hpp>
#include <components/files/conversion.hpp>
#include <components/sdlutil/sdlmappings.hpp>
#include <components/settings/values.hpp>

#include "../mwbase/environment.hpp"
#include "../mwbase/inputmanager.hpp"
#include "../mwbase/luamanager.hpp"
#include "../mwbase/statemanager.hpp"
#include "../mwbase/windowmanager.hpp"
#include "../mwgui/windowbase.hpp"

#include "actions.hpp"
#include "bindingsmanager.hpp"
#ifdef __vita__
#include "vitatouchzones.hpp"
#endif
#include "mousemanager.hpp"

namespace MWInput
{
    ControllerManager::ControllerManager(BindingsManager* bindingsManager, MouseManager* mouseManager,
        const std::filesystem::path& userControllerBindingsFile, const std::filesystem::path& controllerBindingsFile)
        : mBindingsManager(bindingsManager)
        , mMouseManager(mouseManager)
        , mGyroAvailable(false)
        , mGamepadGuiCursorEnabled(true)
        , mGuiCursorEnabled(true)
        , mJoystickLastUsed(false)
        , mGamepadMousePressed(false)
    {
        if (!controllerBindingsFile.empty())
        {
            const int result
                = SDL_GameControllerAddMappingsFromFile(Files::pathToUnicodeString(controllerBindingsFile).c_str());
            if (result < 0)
                Log(Debug::Error) << "Failed to add game controller mappings from file \"" << controllerBindingsFile
                                  << "\": " << SDL_GetError();
        }

        if (!userControllerBindingsFile.empty())
        {
            const int result
                = SDL_GameControllerAddMappingsFromFile(Files::pathToUnicodeString(userControllerBindingsFile).c_str());
            if (result < 0)
                Log(Debug::Error) << "Failed to add game controller mappings from user file \""
                                  << userControllerBindingsFile << "\": " << SDL_GetError();
        }

        // Open all presently connected sticks
        const int numSticks = SDL_NumJoysticks();
        if (numSticks < 0)
            Log(Debug::Error) << "Failed to get number of joysticks: " << SDL_GetError();

        for (int i = 0; i < numSticks; i++)
        {
            if (SDL_IsGameController(i))
            {
                SDL_ControllerDeviceEvent evt;
                evt.which = i;
                static const int fakeDeviceID = 1;
                ControllerManager::controllerAdded(fakeDeviceID, evt);
                if (const char* name = SDL_GameControllerNameForIndex(i))
                    Log(Debug::Info) << "Detected game controller: " << name;
                else
                    Log(Debug::Warning) << "Detected game controller without a name: " << SDL_GetError();
            }
            else
            {
                if (const char* name = SDL_JoystickNameForIndex(i))
                    Log(Debug::Info) << "Detected unusable controller: " << name;
                else
                    Log(Debug::Warning) << "Detected unusable controller without a name: " << SDL_GetError();
            }
        }

        mBindingsManager->setJoystickDeadZone(Settings::input().mJoystickDeadZone);
    }

    void ControllerManager::update(float dt)
    {
        if (mGuiCursorEnabled)
        {
            MWBase::WindowManager* winMgr = MWBase::Environment::get().getWindowManager();
            // Cursor movement from the left stick is only allowed when the
            // current window permits a gamepad-driven cursor.
            const bool allowCursorMove = !(mJoystickLastUsed && !mGamepadGuiCursorEnabled);
            // Wheel events should still fire whenever a cursor is visible — even
            // in windows that disable gamepad cursor (e.g. dialogue), the user
            // may have summoned the cursor via touch / left-stick on a window
            // that does allow it, or via an external mouse. Without this,
            // hovering a sibling scrollable and right-stick'ing does nothing.
            const bool allowWheel = allowCursorMove || winMgr->getCursorVisible();

            if (allowCursorMove || allowWheel)
            {
                float xAxis = mBindingsManager->getActionValue(A_MoveLeftRight) * 2.0f - 1.0f;
                float yAxis = mBindingsManager->getActionValue(A_MoveForwardBackward) * 2.0f - 1.0f;
                float zAxis = mBindingsManager->getActionValue(A_LookUpDown) * 2.0f - 1.0f;

                float xMove = 0.f;
                float yMove = 0.f;
                if (allowCursorMove)
                {
                    xAxis *= (1.5f - mBindingsManager->getActionValue(A_Use));
                    yAxis *= (1.5f - mBindingsManager->getActionValue(A_Use));
                    const float uiScale = winMgr->getScalingFactor();
                    const float gamepadCursorSpeed = Settings::input().mGamepadCursorSpeed;
                    xMove = xAxis * dt * 1500.0f / uiScale * gamepadCursorSpeed;
                    yMove = yAxis * dt * 1500.0f / uiScale * gamepadCursorSpeed;
                }

                float mouseWheelMove = 0.f;
                if (allowWheel)
                {
#ifdef __vita__
                    mouseWheelMove = -zAxis * dt * 300.0f;
#else
                    mouseWheelMove = -zAxis * dt * 1500.0f;
#endif
                }

                if (xMove != 0 || yMove != 0 || mouseWheelMove != 0)
                {
                    mMouseManager->injectMouseMove(xMove, yMove, mouseWheelMove);
                    // Only re-warp the OS cursor when the cursor actually moved.
                    // Wheel-only frames (right-stick scrolling) shouldn't warp; SDL's
                    // WarpMouse can emit a synthetic SDL_MOUSEMOTION event that
                    // makes MyGUI re-evaluate the mouse-focus widget, causing the
                    // scroll target to flip between sibling scrollables.
                    if (xMove != 0 || yMove != 0)
                        mMouseManager->warpMouse();
                    winMgr->setCursorActive(true);
                }
            }
        }

        if (!MWBase::Environment::get().getWindowManager()->isGuiMode()
            && MWBase::Environment::get().getStateManager()->getState() == MWBase::StateManager::State_Running
            && MWBase::Environment::get().getInputManager()->getControlSwitch("playercontrols"))
        {
            float xAxis = mBindingsManager->getActionValue(A_MoveLeftRight);
            float yAxis = mBindingsManager->getActionValue(A_MoveForwardBackward);
            if (xAxis != 0.5 || yAxis != 0.5)
            {
                mJoystickLastUsed = true;
                MWBase::Environment::get().getInputManager()->resetIdleTime();
            }
        }
    }

    void ControllerManager::buttonPressed(int deviceID, const SDL_ControllerButtonEvent& arg)
    {
        if (!Settings::input().mEnableController || mBindingsManager->isDetectingBindingState())
            return;

        MWBase::Environment::get().getLuaManager()->inputEvent(
            { MWBase::LuaManager::InputEvent::ControllerPressed, arg.button });

#ifdef __vita__
        // Select+Start combo: toggle console
        if (arg.button == SDL_CONTROLLER_BUTTON_BACK) // Select
        {
            mSelectHeld = true;
            mSelectUsedAsModifier = false;
            return; // Defer Journal action until release
        }
        if (arg.button == SDL_CONTROLLER_BUTTON_START && mSelectHeld)
        {
            mSelectUsedAsModifier = true;
            if (!MyGUI::InputManager::getInstance().isModalAny())
                MWBase::Environment::get().getWindowManager()->toggleConsole();
            return;
        }
#endif

        mJoystickLastUsed = true;
        if (MWBase::Environment::get().getWindowManager()->isGuiMode())
        {
            if (gamepadToGuiControl(arg))
                return;

            if (mGamepadGuiCursorEnabled)
            {
                // Temporary mouse binding until keyboard controls are available:
                if (arg.button == SDL_CONTROLLER_BUTTON_A) // We'll pretend that A is left click.
                {
                    bool mousePressSuccess = mMouseManager->injectMouseButtonPress(SDL_BUTTON_LEFT);
                    mGamepadMousePressed = true;
                    if (MyGUI::InputManager::getInstance().getMouseFocusWidget())
                    {
                        MyGUI::Button* b
                            = MyGUI::InputManager::getInstance().getMouseFocusWidget()->castType<MyGUI::Button>(false);
                        if (b && b->getEnabled())
                            MWBase::Environment::get().getWindowManager()->playSound(
                                ESM::RefId::stringRefId("Menu Click"));
                    }

                    mBindingsManager->setPlayerControlsEnabled(!mousePressSuccess);
                }
            }
        }
        else
            mBindingsManager->setPlayerControlsEnabled(true);

        // esc, to leave initial movie screen
        auto kc = SDLUtil::sdlKeyToMyGUI(SDLK_ESCAPE);
        mBindingsManager->setPlayerControlsEnabled(!MyGUI::InputManager::getInstance().injectKeyPress(kc, 0));

        if (!MWBase::Environment::get().getInputManager()->controlsDisabled())
            mBindingsManager->controllerButtonPressed(deviceID, arg);
    }

    void ControllerManager::buttonReleased(int deviceID, const SDL_ControllerButtonEvent& arg)
    {
        if (mBindingsManager->isDetectingBindingState())
        {
            mBindingsManager->controllerButtonReleased(deviceID, arg);
            return;
        }

        if (Settings::input().mEnableController)
        {
            MWBase::Environment::get().getLuaManager()->inputEvent(
                { MWBase::LuaManager::InputEvent::ControllerReleased, arg.button });
        }

#ifdef __vita__
        if (arg.button == SDL_CONTROLLER_BUTTON_BACK) // Select released
        {
            bool wasModifier = mSelectUsedAsModifier;
            mSelectHeld = false;
            mSelectUsedAsModifier = false;
            if (!wasModifier && Settings::input().mEnableController
                && !MWBase::Environment::get().getInputManager()->controlsDisabled())
            {
                // Select tapped alone — fire Journal
                mBindingsManager->controllerButtonPressed(deviceID, arg);
                mBindingsManager->controllerButtonReleased(deviceID, arg);
            }
            return;
        }
#endif

        if (!Settings::input().mEnableController || MWBase::Environment::get().getInputManager()->controlsDisabled())
            return;

        mJoystickLastUsed = true;
        if (MWBase::Environment::get().getWindowManager()->isGuiMode())
        {
            if (mGamepadGuiCursorEnabled && (!Settings::gui().mControllerMenus || mGamepadMousePressed))
            {
                // Temporary mouse binding until keyboard controls are available:
                if (arg.button == SDL_CONTROLLER_BUTTON_A) // We'll pretend that A is left click.
                {
                    bool mousePressSuccess = mMouseManager->injectMouseButtonRelease(SDL_BUTTON_LEFT);
                    mGamepadMousePressed = false;
                    if (mBindingsManager->isDetectingBindingState()) // If the player just triggered binding, don't let
                                                                     // button release bind.
                        return;

                    mBindingsManager->setPlayerControlsEnabled(!mousePressSuccess);
                }
            }
        }
        else
            mBindingsManager->setPlayerControlsEnabled(true);

        // esc, to leave initial movie screen
        auto kc = SDLUtil::sdlKeyToMyGUI(SDLK_ESCAPE);
        mBindingsManager->setPlayerControlsEnabled(!MyGUI::InputManager::getInstance().injectKeyRelease(kc));

        mBindingsManager->controllerButtonReleased(deviceID, arg);
    }

    void ControllerManager::axisMoved(int deviceID, const SDL_ControllerAxisEvent& arg)
    {
        if (mBindingsManager->isDetectingBindingState())
        {
            mBindingsManager->controllerAxisMoved(deviceID, arg);
            return;
        }

        if (!Settings::input().mEnableController || MWBase::Environment::get().getInputManager()->controlsDisabled())
            return;

        mJoystickLastUsed = true;
        if (MWBase::Environment::get().getWindowManager()->isGuiMode())
        {
            if (gamepadToGuiControl(arg))
                return;
        }
        else if (mBindingsManager->actionIsActive(A_TogglePOV)
            && (arg.axis == SDL_CONTROLLER_AXIS_TRIGGERRIGHT || arg.axis == SDL_CONTROLLER_AXIS_TRIGGERLEFT))
        {
            // Preview Mode Gamepad Zooming; do not propagate to mBindingsManager
            return;
        }
        mBindingsManager->controllerAxisMoved(deviceID, arg);
    }

    void ControllerManager::controllerAdded(int deviceID, const SDL_ControllerDeviceEvent& arg)
    {
        mBindingsManager->controllerAdded(deviceID, arg);
        enableGyroSensor();
    }

    void ControllerManager::controllerRemoved(const SDL_ControllerDeviceEvent& arg)
    {
        mBindingsManager->controllerRemoved(arg);
    }

    bool ControllerManager::gamepadToGuiControl(const SDL_ControllerButtonEvent& arg)
    {
        MWBase::WindowManager* winMgr = MWBase::Environment::get().getWindowManager();

        if (Settings::gui().mControllerMenus)
        {
            // Decide the role of this press before touching cursor state. If A is
            // going to fall through as a left-click, keep the cursor visible — the
            // user is in cursor-driven mode. Any other button means the user is
            // using menu navigation, so hide the cursor.
            const bool cursorVisible = winMgr->getCursorVisible();
            MWGui::WindowBase* topWin = winMgr->getActiveControllerWindow();

            if (topWin && topWin->isVisible())
            {
                bool treatAsMouse = cursorVisible;
                // When the inventory tooltip is visible, we don't actually want the A button to
                // act like a mouse button; it should act normally.
                if (treatAsMouse && arg.button == SDL_CONTROLLER_BUTTON_A && winMgr->getControllerTooltipVisible())
                    treatAsMouse = false;

                mGamepadGuiCursorEnabled = topWin->isGamepadCursorAllowed();

                // Fall through to mouse click — leave cursor active for the next press.
                if (mGamepadGuiCursorEnabled && treatAsMouse && arg.button == SDL_CONTROLLER_BUTTON_A)
                    return false;

                // Switching to menu navigation; hide the cursor.
                winMgr->setCursorActive(false);

                if (topWin->onControllerButtonEvent(arg))
                    return true;
            }
            else
            {
                winMgr->setCursorActive(false);
            }
        }

        // Presumption of GUI mode will be removed in the future.
        // MyGUI KeyCodes *may* change.
        MyGUI::KeyCode key = MyGUI::KeyCode::None;
        switch (arg.button)
        {
            case SDL_CONTROLLER_BUTTON_DPAD_UP:
                key = MyGUI::KeyCode::ArrowUp;
                break;
            case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
                key = MyGUI::KeyCode::ArrowRight;
                break;
            case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
                key = MyGUI::KeyCode::ArrowDown;
                break;
            case SDL_CONTROLLER_BUTTON_DPAD_LEFT:
                key = MyGUI::KeyCode::ArrowLeft;
                break;
            case SDL_CONTROLLER_BUTTON_A:
                // If we are using the joystick as a GUI mouse, A must be handled via mouse.
                if (mGamepadGuiCursorEnabled)
                    return false;
                key = MyGUI::KeyCode::Space;
                break;
            case SDL_CONTROLLER_BUTTON_B:
                if (MyGUI::InputManager::getInstance().isModalAny())
                    winMgr->exitCurrentModal();
                else
                    winMgr->exitCurrentGuiMode();
                return true;
            case SDL_CONTROLLER_BUTTON_X:
                key = MyGUI::KeyCode::Semicolon;
                break;
            case SDL_CONTROLLER_BUTTON_Y:
                key = MyGUI::KeyCode::Apostrophe;
                break;
            case SDL_CONTROLLER_BUTTON_LEFTSHOULDER:
                MyGUI::InputManager::getInstance().injectKeyPress(MyGUI::KeyCode::LeftShift);
                winMgr->injectKeyPress(MyGUI::KeyCode::Tab, 0, false);
                MyGUI::InputManager::getInstance().injectKeyRelease(MyGUI::KeyCode::LeftShift);
                return true;
            case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER:
                MWBase::Environment::get().getWindowManager()->injectKeyPress(MyGUI::KeyCode::Tab, 0, false);
                return true;
            case SDL_CONTROLLER_BUTTON_LEFTSTICK:
                mGamepadGuiCursorEnabled = !mGamepadGuiCursorEnabled;
                winMgr->setCursorActive(mGamepadGuiCursorEnabled);
                return true;
            default:
                return false;
        }

        // Some keys will work even when Text Input windows/modals are in focus.
        if (SDL_IsTextInputActive())
            return false;

        winMgr->injectKeyPress(key, 0, false);
        return true;
    }

    bool ControllerManager::gamepadToGuiControl(const SDL_ControllerAxisEvent& arg)
    {
        MWBase::WindowManager* winMgr = MWBase::Environment::get().getWindowManager();

        if (Settings::gui().mControllerMenus)
        {
            // Left and right triggers toggle through open GUI windows.
            if (arg.axis == SDL_CONTROLLER_AXIS_TRIGGERRIGHT)
            {
                if (arg.value == 32767) // Treat like a button.
                    winMgr->cycleActiveControllerWindow(true);
                return true;
            }
            else if (arg.axis == SDL_CONTROLLER_AXIS_TRIGGERLEFT)
            {
                if (arg.value == 32767) // Treat like a button.
                    winMgr->cycleActiveControllerWindow(false);
                return true;
            }

            MWGui::WindowBase* topWin = winMgr->getActiveControllerWindow();
            if (topWin && topWin->isVisible())
            {
                // Update cursor state. Only hide the cursor for cursor-driving
                // (left-stick) input — right-stick events are scroll-only and
                // shouldn't yank a touch- or left-stick-summoned cursor away
                // mid-scroll. Without this guard, dialogue (which sets
                // mDisableGamepadCursor=true) hides the cursor on every right-stick
                // tick and the scroll target can flip between sibling scrollables.
                mGamepadGuiCursorEnabled = topWin->isGamepadCursorAllowed();
                const bool isCursorStick = (arg.axis == SDL_CONTROLLER_AXIS_LEFTX
                                            || arg.axis == SDL_CONTROLLER_AXIS_LEFTY);
                if (!mGamepadGuiCursorEnabled && isCursorStick)
                    winMgr->setCursorActive(false);

                // Deadzone check
                if (std::abs(arg.value) < 2000)
                    return !mGamepadGuiCursorEnabled;

                if (mGamepadGuiCursorEnabled
                    && (arg.axis == SDL_CONTROLLER_AXIS_LEFTX || arg.axis == SDL_CONTROLLER_AXIS_LEFTY))
                {
                    // Treat the left stick like a cursor, which is the default behavior.
                    if (winMgr->getControllerTooltipVisible())
                    {
                        winMgr->setControllerTooltipVisible(false);
                        winMgr->setCursorVisible(true);
                    }
                    else if (mGamepadGuiCursorEnabled)
                    {
                        winMgr->setCursorVisible(true);
                    }
                    return false;
                }

                // Some windows have a specific widget to scroll with the right stick.
                // Only auto-warp the cursor to that widget in controller-only mode —
                // when the cursor is already visible (mouse-driven), scrolling should
                // target whatever's under the cursor via the wheel-injection path in
                // update(). Otherwise users hovering a sibling scrollable (e.g. the
                // dialogue topics list) would have the cursor yanked away to the
                // designated widget on every right-stick tick.
                if (arg.axis == SDL_CONTROLLER_AXIS_RIGHTY && topWin->getControllerScrollWidget() != nullptr
                    && !winMgr->getCursorVisible())
                {
                    mMouseManager->warpMouseToWidget(topWin->getControllerScrollWidget());
                    winMgr->setCursorVisible(false);
                }

                if (topWin->onControllerThumbstickEvent(arg))
                {
                    // Window handled the event.
                    return true;
                }
                else if (arg.axis == SDL_CONTROLLER_AXIS_RIGHTX || arg.axis == SDL_CONTROLLER_AXIS_RIGHTY)
                {
                    // Only right-stick scroll if mouse is visible or there's a widget to scroll.
                    return !winMgr->getCursorVisible() && topWin->getControllerScrollWidget() == nullptr;
                }
            }
        }

        switch (arg.axis)
        {
            case SDL_CONTROLLER_AXIS_TRIGGERRIGHT:
                if (arg.value == 32767) // Treat like a button.
                    winMgr->injectKeyPress(MyGUI::KeyCode::Minus, 0, false);
                break;
            case SDL_CONTROLLER_AXIS_TRIGGERLEFT:
                if (arg.value == 32767) // Treat like a button.
                    winMgr->injectKeyPress(MyGUI::KeyCode::Equals, 0, false);
                break;
            case SDL_CONTROLLER_AXIS_LEFTX:
            case SDL_CONTROLLER_AXIS_LEFTY:
            case SDL_CONTROLLER_AXIS_RIGHTX:
            case SDL_CONTROLLER_AXIS_RIGHTY:
                // If we are using the joystick as a GUI mouse, process mouse movement elsewhere.
                if (mGamepadGuiCursorEnabled)
                    return false;
                break;
            default:
                return false;
        }

        return true;
    }

    float ControllerManager::getAxisValue(SDL_GameControllerAxis axis) const
    {
        SDL_GameController* cntrl = mBindingsManager->getControllerOrNull();
        constexpr float axisMaxAbsoluteValue = 32768;
        if (cntrl != nullptr)
            return SDL_GameControllerGetAxis(cntrl, axis) / axisMaxAbsoluteValue;
        return 0;
    }

    bool ControllerManager::isButtonPressed(SDL_GameControllerButton button) const
    {
        SDL_GameController* cntrl = mBindingsManager->getControllerOrNull();
        if (cntrl)
            return SDL_GameControllerGetButton(cntrl, button) > 0;
        else
            return false;
    }

    void ControllerManager::enableGyroSensor()
    {
        mGyroAvailable = false;
        SDL_GameController* cntrl = mBindingsManager->getControllerOrNull();
        if (!cntrl)
            return;
        if (!SDL_GameControllerHasSensor(cntrl, SDL_SENSOR_GYRO))
            return;
        if (const int result = SDL_GameControllerSetSensorEnabled(cntrl, SDL_SENSOR_GYRO, SDL_TRUE); result < 0)
        {
            Log(Debug::Error) << "Failed to enable game controller sensor: " << SDL_GetError();
            return;
        }
        mGyroAvailable = true;
    }

    bool ControllerManager::isGyroAvailable() const
    {
        return mGyroAvailable;
    }

    std::array<float, 3> ControllerManager::getGyroValues() const
    {
        float gyro[3] = { 0.f };
        SDL_GameController* cntrl = mBindingsManager->getControllerOrNull();
        if (cntrl && mGyroAvailable)
        {
            const int result = SDL_GameControllerGetSensorData(cntrl, SDL_SENSOR_GYRO, gyro, 3);
            if (result < 0)
                Log(Debug::Error) << "Failed to get game controller sensor data: " << SDL_GetError();
        }
        return std::array<float, 3>({ gyro[0], gyro[1], gyro[2] });
    }

    int ControllerManager::getControllerType()
    {
        SDL_GameController* cntrl = mBindingsManager->getControllerOrNull();
        if (cntrl)
            return SDL_GameControllerGetType(cntrl);
        return 0;
    }

    std::string ControllerManager::getControllerButtonIcon(int button)
    {
        int controllerType = ControllerManager::getControllerType();

        bool isXbox = controllerType == SDL_CONTROLLER_TYPE_XBOX360 || controllerType == SDL_CONTROLLER_TYPE_XBOXONE;
#ifdef __vita__
        bool isPsx = true;
#else
        bool isPsx = controllerType == SDL_CONTROLLER_TYPE_PS3 || controllerType == SDL_CONTROLLER_TYPE_PS4
            || controllerType == SDL_CONTROLLER_TYPE_PS5;
#endif
        bool isSwitch = controllerType == SDL_CONTROLLER_TYPE_NINTENDO_SWITCH_PRO;

        switch (button)
        {
            case SDL_CONTROLLER_BUTTON_A:
                if (isPsx)
                    return "textures/omw_psx_button_x.dds";
                return "textures/omw_steam_button_a.dds";
            case SDL_CONTROLLER_BUTTON_B:
                if (isPsx)
                    return "textures/omw_psx_button_circle.dds";
                return "textures/omw_steam_button_b.dds";
            case SDL_CONTROLLER_BUTTON_BACK:
                return "textures/omw_steam_button_view.dds";
            case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
            case SDL_CONTROLLER_BUTTON_DPAD_LEFT:
            case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
            case SDL_CONTROLLER_BUTTON_DPAD_UP:
                if (isPsx)
                    return "textures/omw_psx_button_dpad.dds";
                return "textures/omw_steam_button_dpad.dds";
            case SDL_CONTROLLER_BUTTON_LEFTSHOULDER:
                if (isXbox)
                    return "textures/omw_xbox_button_lb.dds";
                else if (isSwitch)
                    return "textures/omw_switch_button_l.dds";
                return "textures/omw_steam_button_l1.dds";
            case SDL_CONTROLLER_BUTTON_LEFTSTICK:
                return "textures/omw_steam_button_l3.dds";
            case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER:
                if (isXbox)
                    return "textures/omw_xbox_button_rb.dds";
                else if (isSwitch)
                    return "textures/omw_switch_button_r.dds";
                return "textures/omw_steam_button_r1.dds";
            case SDL_CONTROLLER_BUTTON_RIGHTSTICK:
                return "textures/omw_steam_button_r3.dds";
            case SDL_CONTROLLER_BUTTON_START:
                return "textures/omw_steam_button_menu.dds";
            case SDL_CONTROLLER_BUTTON_X:
                if (isPsx)
                    return "textures/omw_psx_button_square.dds";
                return "textures/omw_steam_button_x.dds";
            case SDL_CONTROLLER_BUTTON_Y:
                if (isPsx)
                    return "textures/omw_psx_button_triangle.dds";
                return "textures/omw_steam_button_y.dds";
            case SDL_CONTROLLER_BUTTON_GUIDE:
            case SDL_CONTROLLER_BUTTON_MISC1:
            case SDL_CONTROLLER_BUTTON_PADDLE1:
            case SDL_CONTROLLER_BUTTON_PADDLE2:
            case SDL_CONTROLLER_BUTTON_PADDLE3:
            case SDL_CONTROLLER_BUTTON_PADDLE4:
            case SDL_CONTROLLER_BUTTON_TOUCHPAD:
            default:
                return {};
        }
    }

    std::string ControllerManager::getControllerAxisIcon(int axis)
    {
        int controllerType = ControllerManager::getControllerType();

        bool isXbox = controllerType == SDL_CONTROLLER_TYPE_XBOX360 || controllerType == SDL_CONTROLLER_TYPE_XBOXONE;
        bool isSwitch = controllerType == SDL_CONTROLLER_TYPE_NINTENDO_SWITCH_PRO;

        switch (axis)
        {
            case SDL_CONTROLLER_AXIS_LEFTX:
            case SDL_CONTROLLER_AXIS_LEFTY:
                return "textures/omw_steam_button_lstick.dds";
            case SDL_CONTROLLER_AXIS_RIGHTX:
            case SDL_CONTROLLER_AXIS_RIGHTY:
                return "textures/omw_steam_button_rstick.dds";
            case SDL_CONTROLLER_AXIS_TRIGGERLEFT:
                if (isXbox)
                    return "textures/omw_xbox_button_lt.dds";
                else if (isSwitch)
                    return "textures/omw_switch_button_zl.dds";
                return "textures/omw_steam_button_l2.dds";
            case SDL_CONTROLLER_AXIS_TRIGGERRIGHT:
                if (isXbox)
                    return "textures/omw_xbox_button_rt.dds";
                else if (isSwitch)
                    return "textures/omw_switch_button_zr.dds";
                return "textures/omw_steam_button_r2.dds";
            default:
                return {};
        }
    }

    void ControllerManager::touchpadMoved(int deviceId, const SDLUtil::TouchEvent& arg)
    {
#ifdef __vita__
        if (mVitaTouchZones != nullptr)
            mVitaTouchZones->onTouchMove(arg);
#endif
        MWBase::Environment::get().getLuaManager()->inputEvent({ MWBase::LuaManager::InputEvent::TouchMoved, arg });
    }

    void ControllerManager::touchpadPressed(int deviceId, const SDLUtil::TouchEvent& arg)
    {
#ifdef __vita__
        if (mVitaTouchZones != nullptr)
            mVitaTouchZones->onTouchDown(arg);
#endif
        MWBase::Environment::get().getLuaManager()->inputEvent({ MWBase::LuaManager::InputEvent::TouchPressed, arg });
    }

    void ControllerManager::touchpadReleased(int deviceId, const SDLUtil::TouchEvent& arg)
    {
#ifdef __vita__
        if (mVitaTouchZones != nullptr)
            mVitaTouchZones->onTouchUp(arg);
#endif
        MWBase::Environment::get().getLuaManager()->inputEvent({ MWBase::LuaManager::InputEvent::TouchReleased, arg });
    }
}
