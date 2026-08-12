#include "sdlinputwrapper.hpp"

#ifdef __vita__
#include <cstdio>
#include <psp2/kernel/processmgr.h>
extern "C" void vitaBreadcrumb(const char* msg); // apps/openmw/vita/VitaInit.cpp
// Capture sub-region timings; read by [Slow] in engine.cpp.
extern "C" uint32_t vitastat_input_pump_us;
uint32_t vitastat_input_pump_us = 0;
extern "C" uint32_t vitastat_input_q_us, vitastat_input_loop_us, vitastat_input_ax_us, vitastat_input_nevt;
uint32_t vitastat_input_q_us = 0, vitastat_input_loop_us = 0, vitastat_input_ax_us = 0, vitastat_input_nevt = 0;
namespace
{
    // Axis dispatch cost tracking, reported by the 1Hz [Input] crumb.
    unsigned s_dispatchN = 0;
    unsigned s_dispatchUs = 0;
    unsigned s_dispatchMaxUs = 0;
}
#endif

#include <components/debug/debuglog.hpp>
#include <components/settings/values.hpp>

#include <osgViewer/Viewer>

namespace SDLUtil
{

    InputWrapper::InputWrapper(SDL_Window* window, osg::ref_ptr<osgViewer::Viewer> viewer, bool grab)
        : mSDLWindow(window)
        , mViewer(std::move(viewer))
        , mMouseListener(nullptr)
        , mSensorListener(nullptr)
        , mKeyboardListener(nullptr)
        , mWindowListener(nullptr)
        , mConListener(nullptr)
        , mWarpX(0)
        , mWarpY(0)
        , mWarpCompensate(false)
        , mWrapPointer(false)
        , mAllowGrab(grab)
        , mWantMouseVisible(false)
        , mWantGrab(false)
        , mWantRelative(false)
        , mGrabPointer(false)
        , mMouseRelative(false)
        , mFirstMouseMove(true)
        , mMouseZ(0)
        , mMouseX(0)
        , mMouseY(0)
        , mWindowHasFocus(true)
        , mMouseInWindow(true)
    {
        Uint32 flags = SDL_GetWindowFlags(mSDLWindow);
        mWindowHasFocus = (flags & SDL_WINDOW_INPUT_FOCUS);
        mMouseInWindow = (flags & SDL_WINDOW_MOUSE_FOCUS);
        _setWindowScale();
    }

    InputWrapper::~InputWrapper() = default;

    void InputWrapper::_setWindowScale()
    {
        int w, h;
        SDL_GetWindowSize(mSDLWindow, &w, &h);
        int dw, dh;
        SDL_GL_GetDrawableSize(mSDLWindow, &dw, &dh);
        mScaleX = static_cast<Uint16>(dw / w);
        mScaleY = static_cast<Uint16>(dh / h);
    }

    void InputWrapper::capture(bool windowEventsOnly)
    {
#ifdef __vita__
        {
            const unsigned long long q0 = sceKernelGetProcessTimeWide();
            mViewer->getEventQueue()->frame(0.f);
            vitastat_input_q_us = (uint32_t)(sceKernelGetProcessTimeWide() - q0);
        }
#else
        mViewer->getEventQueue()->frame(0.f);
#endif

#ifdef __vita__
        {
            const unsigned long long p0 = sceKernelGetProcessTimeWide();
            SDL_PumpEvents();
            vitastat_input_pump_us = (uint32_t)(sceKernelGetProcessTimeWide() - p0);
        }
#else
        SDL_PumpEvents();
#endif

#ifdef __vita__
        // ~1 Hz input health line.
        {
            static unsigned s_calls = 0;
            static unsigned s_events = 0;
            static unsigned s_padEvents = 0;
            SDL_Event peek[16];
            const int pending
                = SDL_PeepEvents(peek, 16, SDL_PEEKEVENT, SDL_FIRSTEVENT, SDL_LASTEVENT);
            s_events += pending > 0 ? pending : 0;
            for (int i = 0; i < (pending > 0 ? pending : 0); ++i)
                if (peek[i].type == SDL_CONTROLLERBUTTONDOWN || peek[i].type == SDL_CONTROLLERBUTTONUP
                    || peek[i].type == SDL_CONTROLLERAXISMOTION)
                    ++s_padEvents;
            if ((++s_calls & 31) == 0)
            {
                char buf[160];
                snprintf(buf, sizeof(buf), "[Input] calls=%u pending=%d pad=%u focus=%d disp=%u avg=%uus max=%uus",
                    s_calls, pending, s_padEvents, mWindowHasFocus ? 1 : 0, s_dispatchN,
                    s_dispatchN ? s_dispatchUs / s_dispatchN : 0, s_dispatchMaxUs);
                vitaBreadcrumb(buf);
                s_dispatchN = 0;
                s_dispatchUs = 0;
                s_dispatchMaxUs = 0;
            }
        }
#endif

        SDL_Event evt;

#ifdef __vita__
        // Coalesce axis motion: only last value per axis matters.
        SDL_ControllerAxisEvent vitaPendingAxis[SDL_CONTROLLER_AXIS_MAX];
        bool vitaHaveAxis[SDL_CONTROLLER_AXIS_MAX] = {};
#endif

        if (windowEventsOnly)
        {
            // During loading, handle window events, discard button presses and mouse movement and keep others for later
            while (SDL_PeepEvents(&evt, 1, SDL_GETEVENT, SDL_WINDOWEVENT, SDL_WINDOWEVENT) > 0)
                handleWindowEvent(evt);

            SDL_FlushEvent(SDL_KEYDOWN);
            SDL_FlushEvent(SDL_CONTROLLERBUTTONDOWN);
            SDL_FlushEvent(SDL_MOUSEBUTTONDOWN);
            SDL_FlushEvent(SDL_MOUSEMOTION);
            SDL_FlushEvent(SDL_MOUSEWHEEL);
#ifdef __vita__
            SDL_FlushEvent(SDL_FINGERDOWN);
            SDL_FlushEvent(SDL_FINGERUP);
            SDL_FlushEvent(SDL_FINGERMOTION);
#endif

            return;
        }

#ifdef __vita__
        const unsigned long long vitaLoop0 = sceKernelGetProcessTimeWide();
        vitastat_input_nevt = 0;
#endif
        while (SDL_PollEvent(&evt))
        {
#ifdef __vita__
            ++vitastat_input_nevt;
#endif
#if SDL_VERSION_ATLEAST(2, 30, 50)
            // SDL2-compat may pass us SDL3 display and window events alongside the SDL2 events for funsies
            // Until we are ready to move to SDL3, we'll want to prevent the noise

            // Silence 0x151 to 0x1FF range
            if (evt.type > SDL_DISPLAYEVENT && evt.type < SDL_WINDOWEVENT)
                continue;

            // Silence 0x202 to 0x2FF range
            if (evt.type > SDL_SYSWMEVENT && evt.type < SDL_KEYDOWN)
                continue;
#endif
            switch (evt.type)
            {
                case SDL_MOUSEMOTION:
                    // Ignore this if it happened due to a warp
                    if (!_handleWarpMotion(evt.motion))
                    {
                        // If in relative mode, don't trigger events unless window has focus
                        if (!mWantRelative || mWindowHasFocus)
                            mMouseListener->mouseMoved(_packageMouseMotion(evt));

                        // Try to keep the mouse inside the window
                        if (mWindowHasFocus)
                            _wrapMousePointer(evt.motion);
                    }
                    break;
                case SDL_MOUSEWHEEL:
                    mMouseListener->mouseMoved(_packageMouseMotion(evt));
                    mMouseListener->mouseWheelMoved(evt.wheel);
                    break;
                case SDL_SENSORUPDATE:
                    mSensorListener->sensorUpdated(evt.sensor);
                    break;
                case SDL_MOUSEBUTTONDOWN:
                    mMouseListener->mousePressed(evt.button, evt.button.button);
                    break;
                case SDL_MOUSEBUTTONUP:
                    mMouseListener->mouseReleased(evt.button, evt.button.button);
                    break;
                case SDL_KEYDOWN:
                    mKeyboardListener->keyPressed(evt.key);

                    if (!isModifierHeld(KMOD_ALT) && evt.key.keysym.sym >= SDLK_F1 && evt.key.keysym.sym <= SDLK_F12)
                    {
                        mViewer->getEventQueue()->keyPress(
                            osgGA::GUIEventAdapter::KEY_F1 + (evt.key.keysym.sym - SDLK_F1));
                    }

                    break;
                case SDL_KEYUP:
                    if (!evt.key.repeat)
                    {
                        mKeyboardListener->keyReleased(evt.key);

                        if (!isModifierHeld(KMOD_ALT) && evt.key.keysym.sym >= SDLK_F1
                            && evt.key.keysym.sym <= SDLK_F12)
                            mViewer->getEventQueue()->keyRelease(
                                osgGA::GUIEventAdapter::KEY_F1 + (evt.key.keysym.sym - SDLK_F1));
                    }

                    break;
                case SDL_TEXTEDITING:
                    break;
                case SDL_TEXTINPUT:
                    mKeyboardListener->textInput(evt.text);
                    break;
                case SDL_KEYMAPCHANGED:
                    break;
                case SDL_JOYHATMOTION: // As we manage everything with GameController, don't even bother with these.
                case SDL_JOYAXISMOTION:
                case SDL_JOYBUTTONDOWN:
                case SDL_JOYBUTTONUP:
                case SDL_JOYDEVICEADDED:
                case SDL_JOYDEVICEREMOVED:
                    break;
                case SDL_CONTROLLERDEVICEADDED:
                    if (mConListener)
                        mConListener->controllerAdded(
                            1, evt.cdevice); // We only support one joystick, so give everything a generic deviceID
                    break;
                case SDL_CONTROLLERDEVICEREMOVED:
                    if (mConListener)
                        mConListener->controllerRemoved(evt.cdevice);
                    break;
                case SDL_CONTROLLERBUTTONDOWN:
                    if (mConListener)
                        mConListener->buttonPressed(1, evt.cbutton);
                    break;
                case SDL_CONTROLLERBUTTONUP:
                    if (mConListener)
                        mConListener->buttonReleased(1, evt.cbutton);
                    break;
                case SDL_CONTROLLERAXISMOTION:
#ifdef __vita__
                    if (evt.caxis.axis < SDL_CONTROLLER_AXIS_MAX)
                    {
                        vitaPendingAxis[evt.caxis.axis] = evt.caxis;
                        vitaHaveAxis[evt.caxis.axis] = true;
                        break;
                    }
#endif
                    if (mConListener)
                        mConListener->axisMoved(1, evt.caxis);
                    break;
                case SDL_CONTROLLERSENSORUPDATE:
                    // controller sensor data is received on demand
                    break;
                case SDL_CONTROLLERTOUCHPADDOWN:
                    mConListener->touchpadPressed(1, TouchEvent(evt.ctouchpad));
                    break;
                case SDL_CONTROLLERTOUCHPADMOTION:
                    mConListener->touchpadMoved(1, TouchEvent(evt.ctouchpad));
                    break;
                case SDL_CONTROLLERTOUCHPADUP:
                    mConListener->touchpadReleased(1, TouchEvent(evt.ctouchpad));
                    break;
                case SDL_WINDOWEVENT:
                    handleWindowEvent(evt);
                    break;
                case SDL_QUIT:
                    if (mWindowListener)
                        mWindowListener->windowClosed();
                    break;
                case SDL_DISPLAYEVENT:
                    switch (evt.display.event)
                    {
                        case SDL_DISPLAYEVENT_ORIENTATION:
                            if (mSensorListener
                                && evt.display.display == static_cast<Uint32>(Settings::video().mScreen))
                            {
                                mSensorListener->displayOrientationChanged();
                            }
                            break;
                        default:
                            break;
                    }
                    break;
                case SDL_CLIPBOARDUPDATE:
                    break; // We don't need this event, clipboard is retrieved on demand

                case SDL_FINGERDOWN:
                case SDL_FINGERUP:
                case SDL_FINGERMOTION:
#ifdef __vita__
                    // Back touchpad: left/right halves emulate L2/R2 triggers.
                    // Margins on the outer edges and the centre seam keep the
                    // hit zones away from where users naturally rest fingers.
                    if (evt.tfinger.touchId == 2 && evt.type != SDL_FINGERMOTION && mConListener)
                    {
                        constexpr float kEdgeMargin = 0.10f; // left/right outer dead zones
                        constexpr float kSeamMargin = 0.05f; // centre dead zone (each side)
                        const float x = evt.tfinger.x;
                        const bool isL2Zone = (x >= kEdgeMargin && x < 0.5f - kSeamMargin);
                        const bool isR2Zone = (x > 0.5f + kSeamMargin && x <= 1.0f - kEdgeMargin);
                        if (isL2Zone || isR2Zone)
                        {
                            const bool pressed = (evt.type == SDL_FINGERDOWN);
                            bool& state = isL2Zone ? mBackTouchL2 : mBackTouchR2;
                            if (pressed != state)
                            {
                                state = pressed;
                                SDL_ControllerAxisEvent axEvt = {};
                                axEvt.type = SDL_CONTROLLERAXISMOTION;
                                axEvt.axis = isL2Zone ? SDL_CONTROLLER_AXIS_TRIGGERLEFT
                                                      : SDL_CONTROLLER_AXIS_TRIGGERRIGHT;
                                axEvt.value = pressed ? 32767 : 0;
                                mConListener->axisMoved(1, axEvt);
                            }
                        }
                        break;
                    }
                    if (evt.tfinger.touchId == 1)
                    {
                        const bool guiMode = mTouchCursorEnabled && mTouchCursorEnabled();

                        // App-level touch zones consume these (device 0 = front).
                        if (mConListener)
                        {
                            const SDLUtil::TouchEvent te(evt.tfinger, 0);
                            if (evt.type == SDL_FINGERDOWN)
                                mConListener->touchpadPressed(0, te);
                            else if (evt.type == SDL_FINGERUP)
                                mConListener->touchpadReleased(0, te);
                            else
                                mConListener->touchpadMoved(0, te);
                        }

                        // GUI mode: touch is absolute cursor position + click/drag.
                        if (guiMode && mMouseListener)
                        {
                            int winW = 0, winH = 0;
                            SDL_GetWindowSize(mSDLWindow, &winW, &winH);
                            const Sint32 px = static_cast<Sint32>(evt.tfinger.x * winW);
                            const Sint32 py = static_cast<Sint32>(evt.tfinger.y * winH);

                            if (evt.type == SDL_FINGERDOWN && !mCursorFingerActive)
                            {
                                mCursorFingerActive = true;
                                mCursorFingerId = evt.tfinger.fingerId;
                                mMouseX = px;
                                mMouseY = py;
                                MouseMotionEvent mm = {};
                                mm.type = SDL_MOUSEMOTION;
                                mm.which = SDL_TOUCH_MOUSEID;
                                mm.x = px;
                                mm.y = py;
                                mm.state = SDL_BUTTON_LMASK;
                                mMouseListener->mouseMoved(mm);
                                SDL_MouseButtonEvent mb = {};
                                mb.type = SDL_MOUSEBUTTONDOWN;
                                mb.which = SDL_TOUCH_MOUSEID;
                                mb.button = SDL_BUTTON_LEFT;
                                mb.state = SDL_PRESSED;
                                mb.clicks = 1;
                                mb.x = px;
                                mb.y = py;
                                mMouseListener->mousePressed(mb, SDL_BUTTON_LEFT);
                            }
                            else if (evt.type == SDL_FINGERMOTION && mCursorFingerActive
                                && evt.tfinger.fingerId == mCursorFingerId)
                            {
                                MouseMotionEvent mm = {};
                                mm.type = SDL_MOUSEMOTION;
                                mm.which = SDL_TOUCH_MOUSEID;
                                mm.x = px;
                                mm.y = py;
                                mm.xrel = px - mMouseX;
                                mm.yrel = py - mMouseY;
                                mm.state = SDL_BUTTON_LMASK;
                                mMouseX = px;
                                mMouseY = py;
                                mMouseListener->mouseMoved(mm);
                            }
                            else if (evt.type == SDL_FINGERUP && mCursorFingerActive
                                && evt.tfinger.fingerId == mCursorFingerId)
                            {
                                mCursorFingerActive = false;
                                SDL_MouseButtonEvent mb = {};
                                mb.type = SDL_MOUSEBUTTONUP;
                                mb.which = SDL_TOUCH_MOUSEID;
                                mb.button = SDL_BUTTON_LEFT;
                                mb.state = SDL_RELEASED;
                                mb.x = px;
                                mb.y = py;
                                mMouseListener->mouseReleased(mb, SDL_BUTTON_LEFT);
                            }
                        }
                    }
                    break;
#endif
                case SDL_DOLLARGESTURE:
                case SDL_DOLLARRECORD:
                case SDL_MULTIGESTURE:
                    break;

                case SDL_APP_WILLENTERBACKGROUND:
                case SDL_APP_WILLENTERFOREGROUND:
                case SDL_APP_DIDENTERBACKGROUND:
                case SDL_APP_DIDENTERFOREGROUND:
                    // We do not need background/foreground switch event for mobile devices so far
                    break;

                case SDL_APP_TERMINATING:
                    // There is nothing we can do here.
                    break;

                case SDL_APP_LOWMEMORY:
                    Log(Debug::Warning) << "System reports that free RAM on device is running low. You may encounter "
                                           "an unexpected behaviour.";
                    break;

                default:
                    Log(Debug::Info) << "Unhandled SDL event of type 0x" << std::hex << evt.type;
                    break;
            }
        }

#ifdef __vita__
        vitastat_input_loop_us = (uint32_t)(sceKernelGetProcessTimeWide() - vitaLoop0);
        const unsigned long long vitaAx0 = sceKernelGetProcessTimeWide();
        if (mConListener)
        {
            for (int a = 0; a < SDL_CONTROLLER_AXIS_MAX; ++a)
                if (vitaHaveAxis[a])
                {
                    const unsigned long long d0 = sceKernelGetProcessTimeWide();
                    mConListener->axisMoved(1, vitaPendingAxis[a]);
                    const uint32_t dUs = (uint32_t)(sceKernelGetProcessTimeWide() - d0);
                    ++s_dispatchN;
                    s_dispatchUs += dUs;
                    if (dUs > s_dispatchMaxUs)
                        s_dispatchMaxUs = dUs;
                }
        }
        vitastat_input_ax_us = (uint32_t)(sceKernelGetProcessTimeWide() - vitaAx0);
#endif
    }

    void InputWrapper::handleWindowEvent(const SDL_Event& evt)
    {
        switch (evt.window.event)
        {
            case SDL_WINDOWEVENT_ENTER:
                mMouseInWindow = true;
                updateMouseSettings();
                break;
            case SDL_WINDOWEVENT_LEAVE:
                mMouseInWindow = false;
                updateMouseSettings();
                break;
            case SDL_WINDOWEVENT_MOVED:
                // I'm not sure what OSG is using the window position for, but I don't think it's needed,
                // so we ignore window moved events (improves window movement performance)
                break;
            case SDL_WINDOWEVENT_SIZE_CHANGED:
                int w, h;
                SDL_GL_GetDrawableSize(mSDLWindow, &w, &h);
                int x, y;
                SDL_GetWindowPosition(mSDLWindow, &x, &y);

                // Happens when you Alt-Tab out of game
                if (w == 0 && h == 0)
                    return;

                mViewer->getCamera()->getGraphicsContext()->resized(x, y, w, h);

                mViewer->getEventQueue()->windowResize(x, y, w, h);

                if (mWindowListener)
                    mWindowListener->windowResized(w, h);

                _setWindowScale();

                break;

            case SDL_WINDOWEVENT_RESIZED:
                // This should also fire SIZE_CHANGED, so no need to handle
                break;

            case SDL_WINDOWEVENT_FOCUS_GAINED:
                mWindowHasFocus = true;
                updateMouseSettings();
                break;
            case SDL_WINDOWEVENT_FOCUS_LOST:
                mWindowHasFocus = false;
                updateMouseSettings();
                break;
            case SDL_WINDOWEVENT_CLOSE:
                break;
            case SDL_WINDOWEVENT_SHOWN:
            case SDL_WINDOWEVENT_RESTORED:
                if (mWindowListener)
                    mWindowListener->windowVisibilityChange(true);
                break;
            case SDL_WINDOWEVENT_HIDDEN:
            case SDL_WINDOWEVENT_MINIMIZED:
                if (mWindowListener)
                    mWindowListener->windowVisibilityChange(false);
                break;
        }
    }

    bool InputWrapper::isModifierHeld(int mod)
    {
        return (SDL_GetModState() & mod) != 0;
    }

    bool InputWrapper::isKeyDown(SDL_Scancode key)
    {
        return (SDL_GetKeyboardState(nullptr)[key]) != 0;
    }

    /// \brief Moves the mouse to the specified point within the viewport
    void InputWrapper::warpMouse(int x, int y)
    {
        SDL_WarpMouseInWindow(mSDLWindow, x, y);
        mWarpCompensate = true;
        mWarpX = static_cast<Uint16>(x);
        mWarpY = static_cast<Uint16>(y);
    }

    /// \brief Locks the pointer to the window
    void InputWrapper::setGrabPointer(bool grab)
    {
        mWantGrab = grab;
        updateMouseSettings();
    }

    /// \brief Set the mouse to relative positioning. Doesn't move the cursor
    ///        and disables mouse acceleration.
    void InputWrapper::setMouseRelative(bool relative)
    {
        mWantRelative = relative;
        updateMouseSettings();
    }

    void InputWrapper::setMouseVisible(bool visible)
    {
        mWantMouseVisible = visible;
        updateMouseSettings();
    }

    void InputWrapper::updateMouseSettings()
    {
        mGrabPointer = mWantGrab && mMouseInWindow && mWindowHasFocus;
        SDL_SetWindowGrab(mSDLWindow, mGrabPointer && mAllowGrab ? SDL_TRUE : SDL_FALSE);

        SDL_ShowCursor(mWantMouseVisible || !mWindowHasFocus);

        bool relative = mWantRelative && mMouseInWindow && mWindowHasFocus;
        if (mMouseRelative == relative)
            return;

        mMouseRelative = relative;

        mWrapPointer = false;

        // eep, wrap the pointer manually if the input driver doesn't support
        // relative positioning natively
        // also use wrapping if no-grab was specified in options (SDL_SetRelativeMouseMode
        // appears to eat the mouse cursor when pausing in a debugger)
        bool success = mAllowGrab && SDL_SetRelativeMouseMode(relative ? SDL_TRUE : SDL_FALSE) == 0;
        if (relative && !success)
            mWrapPointer = true;

        // now remove all mouse events using the old setting from the queue
        SDL_PumpEvents();
        SDL_FlushEvent(SDL_MOUSEMOTION);
    }

    /// \brief Internal method for ignoring relative motions as a side effect
    ///        of warpMouse()
    bool InputWrapper::_handleWarpMotion(const SDL_MouseMotionEvent& evt)
    {
        if (!mWarpCompensate)
            return false;

        // this was a warp event, signal the caller to eat it.
        if (evt.x == mWarpX && evt.y == mWarpY)
        {
            mWarpCompensate = false;
            return true;
        }

        return false;
    }

    /// \brief Wrap the mouse to the viewport
    void InputWrapper::_wrapMousePointer(const SDL_MouseMotionEvent& evt)
    {
        // don't wrap if we don't want relative movements, support relative
        // movements natively, or aren't grabbing anyways
        if (!mMouseRelative || !mWrapPointer || !mGrabPointer)
            return;

        int width = 0;
        int height = 0;

        SDL_GetWindowSize(mSDLWindow, &width, &height);

        const int fudgeFactorX = width / 4;
        const int fudgeFactorY = height / 4;

        // warp the mouse if it's about to go outside the window
        if (evt.x - fudgeFactorX < 0 || evt.x + fudgeFactorX > width || evt.y - fudgeFactorY < 0
            || evt.y + fudgeFactorY > height)
        {
            warpMouse(width / 2, height / 2);
        }
    }

    /// \brief Package mouse and mousewheel motions into a single event
    MouseMotionEvent InputWrapper::_packageMouseMotion(const SDL_Event& evt)
    {
        MouseMotionEvent packEvt = {};
        packEvt.x = mMouseX * mScaleX;
        packEvt.y = mMouseY * mScaleY;
        packEvt.z = mMouseZ;

        if (evt.type == SDL_MOUSEMOTION)
        {
            packEvt.x = mMouseX = evt.motion.x * mScaleX;
            packEvt.y = mMouseY = evt.motion.y * mScaleY;
            packEvt.xrel = evt.motion.xrel * mScaleX;
            packEvt.yrel = evt.motion.yrel * mScaleY;
            packEvt.type = SDL_MOUSEMOTION;
            if (mFirstMouseMove)
            {
                // first event should be treated as non-relative, since there's no point of reference
                // SDL then (incorrectly) uses (0,0) as point of reference, on Linux at least...
                packEvt.xrel = packEvt.yrel = 0;
                mFirstMouseMove = false;
            }
        }
        else if (evt.type == SDL_MOUSEWHEEL)
        {
            mMouseZ += packEvt.zrel = (evt.wheel.y * 120);
            packEvt.z = mMouseZ;
            packEvt.type = SDL_MOUSEWHEEL;
        }
        else
        {
            throw std::runtime_error("Tried to package non-motion event!");
        }

        return packEvt;
    }
}
