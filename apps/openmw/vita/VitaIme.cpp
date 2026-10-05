#include "VitaIme.h"

#ifdef __vita__

#include <algorithm>
#include <cstring>

#include <psp2/sysmodule.h>
#include <psp2/ime_dialog.h>
#include <psp2/kernel/threadmgr.h>
#include <vitaGL.h>

#include <MyGUI_EditBox.h>
#include <MyGUI_UString.h>

#include "VitaGLWorker.h"
#include "VitaInit.h"

namespace
{
    void utf8ToUtf16(const char* src, SceWChar16* dst, int maxOut)
    {
        int o = 0;
        const auto* s = reinterpret_cast<const unsigned char*>(src);
        while (*s && o < maxOut - 1)
        {
            uint32_t ch;
            if (*s < 0x80)
            {
                ch = *s++;
            }
            else if ((*s & 0xE0) == 0xC0)
            {
                ch = (*s++ & 0x1F) << 6;
                if (*s && (*s & 0xC0) == 0x80) ch |= (*s++ & 0x3F);
            }
            else if ((*s & 0xF0) == 0xE0)
            {
                ch = (*s++ & 0x0F) << 12;
                if (*s && (*s & 0xC0) == 0x80) { ch |= (*s++ & 0x3F) << 6; }
                if (*s && (*s & 0xC0) == 0x80) { ch |= (*s++ & 0x3F); }
            }
            else
            {
                // Skip non-BMP (4-byte) sequences
                s++;
                if (*s && (*s & 0xC0) == 0x80) s++;
                if (*s && (*s & 0xC0) == 0x80) s++;
                if (*s && (*s & 0xC0) == 0x80) s++;
                continue;
            }
            dst[o++] = static_cast<SceWChar16>(ch);
        }
        dst[o] = 0;
    }

    std::string utf16ToUtf8(const SceWChar16* src)
    {
        std::string result;
        while (*src)
        {
            uint16_t ch = *src++;
            if (ch < 0x80)
            {
                result += static_cast<char>(ch);
            }
            else if (ch < 0x800)
            {
                result += static_cast<char>(0xC0 | (ch >> 6));
                result += static_cast<char>(0x80 | (ch & 0x3F));
            }
            else
            {
                result += static_cast<char>(0xE0 | (ch >> 12));
                result += static_cast<char>(0x80 | ((ch >> 6) & 0x3F));
                result += static_cast<char>(0x80 | (ch & 0x3F));
            }
        }
        return result;
    }
}

namespace Vita
{
    void initIme()
    {
        int ret = sceSysmoduleLoadModule(SCE_SYSMODULE_IME);
        if (ret < 0 && ret != static_cast<int>(0x80800003)) // already loaded is OK
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "[VitaIME] sceSysmoduleLoadModule(IME) failed: 0x%08X", ret);
            debugLog(buf);
        }
        else
        {
            debugLog("[VitaIME] IME module loaded.");
        }
    }

    std::string openTextDialog(const char* title, const char* initial, int maxLen)
    {
        maxLen = std::min(maxLen, static_cast<int>(SCE_IME_DIALOG_MAX_TEXT_LENGTH));

        // Convert title and initial text to UTF-16
        SceWChar16 titleUtf16[SCE_IME_DIALOG_MAX_TITLE_LENGTH + 1];
        utf8ToUtf16(title, titleUtf16, SCE_IME_DIALOG_MAX_TITLE_LENGTH + 1);

        SceWChar16 initialUtf16[SCE_IME_DIALOG_MAX_TEXT_LENGTH + 1];
        utf8ToUtf16(initial, initialUtf16, SCE_IME_DIALOG_MAX_TEXT_LENGTH + 1);

        // Output buffer
        SceWChar16 resultUtf16[SCE_IME_DIALOG_MAX_TEXT_LENGTH + 1];
        memset(resultUtf16, 0, sizeof(resultUtf16));

        // Copy initial text to result buffer (IME dialog reads from it)
        int initLen = 0;
        while (initialUtf16[initLen]) ++initLen;
        memcpy(resultUtf16, initialUtf16, (initLen + 1) * sizeof(SceWChar16));

        SceImeDialogParam param;
        sceImeDialogParamInit(&param);
        param.supportedLanguages = 0; // all
        param.languagesForced = SCE_FALSE;
        param.type = SCE_IME_TYPE_BASIC_LATIN;
        param.option = 0;
        param.dialogMode = SCE_IME_DIALOG_DIALOG_MODE_WITH_CANCEL;
        param.textBoxMode = SCE_IME_DIALOG_TEXTBOX_MODE_DEFAULT;
        param.title = titleUtf16;
        param.maxTextLength = static_cast<SceUInt32>(maxLen);
        param.initialText = initialUtf16;
        param.inputTextBuffer = resultUtf16;

        breadcrumb("[VitaIME] Calling sceImeDialogInit...");
        int ret = sceImeDialogInit(&param);
        if (ret < 0)
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "[VitaIME] sceImeDialogInit failed: 0x%08X", ret);
            breadcrumb(buf);
            return {};
        }

        breadcrumb("[VitaIME] Dialog opened, entering poll loop.");

        // Poll until dialog finishes. Render a real cleared scene each
        // swap: the common dialog composites onto the frame, and a swap
        // with no scene leaves it nothing valid to draw into.
        // The swap must happen on the GL owner thread (vitaGL is unlocked).
        while (sceImeDialogGetStatus() == SCE_COMMON_DIALOG_STATUS_RUNNING)
        {
            callOnGL(
                [] {
                    glClearColor(0.f, 0.f, 0.f, 1.f);
                    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                    vglSwapBuffers(GL_TRUE);
                },
                "ime");
        }

        // Get result
        SceImeDialogResult dialogResult;
        memset(&dialogResult, 0, sizeof(dialogResult));
        sceImeDialogGetResult(&dialogResult);
        sceImeDialogTerm();

        if (dialogResult.button == SCE_IME_DIALOG_BUTTON_ENTER)
        {
            std::string text = utf16ToUtf8(resultUtf16);
            char buf[256];
            snprintf(buf, sizeof(buf), "[VitaIME] Dialog confirmed: \"%s\"", text.c_str());
            breadcrumb(buf);
            return text;
        }

        {
            // Diagnose: which button closed it, and what was typed.
            const std::string text = utf16ToUtf8(resultUtf16);
            char buf[256];
            snprintf(buf, sizeof(buf), "[VitaIME] Dialog cancelled (button=%d, buffer=\"%s\")",
                (int)dialogResult.button, text.c_str());
            breadcrumb(buf);
        }
        return {};
    }

    bool fillEditBoxFromIme(MyGUI::EditBox* edit, const char* title, int maxLen)
    {
        if (edit == nullptr)
            return false;
        // Read existing text so the user can edit rather than re-type.
        const std::string current = edit->getCaption().asUTF8();
        const std::string result = openTextDialog(title, current.c_str(), maxLen);
        if (result.empty())
            return false;
        edit->setCaption(result);
        return true;
    }
}

#endif // __vita__
