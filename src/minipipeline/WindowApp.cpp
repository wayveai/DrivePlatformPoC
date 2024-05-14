/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2022-2023 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#ifdef VIBRANTE
#include <EGL/egl.h>
#include <EGL/eglext.h>
#endif

#include "WindowApp.hpp"
#include <exception>

void* gUserCallbackData = 0;
void (*gUserKeyPressCallback)(void*, int, int, int, int)      = 0;
void (*gUserMouseDownCallback)(void*, int, float, float, int) = 0;
void (*gUserMouseUpCallback)(void*, int, float, float, int)   = 0;
void (*gUserMouseMoveCallback)(void*, float, float)  = 0;
void (*gUserMouseWheelCallback)(void*, float, float) = 0;
void (*gUserToggleCameraViewMode)(void*) = 0;

WindowBase* gWindow = nullptr;
bool gRun           = false;

// toggle camera view mode
static void toggleCameraViewMode()
{
    if (gUserToggleCameraViewMode)
        gUserToggleCameraViewMode(gUserCallbackData);
}

void handlePress(int key, int scancode, int action, int mods)
{
    (void)scancode;
    (void)action;
    switch (key)
    {
    case GLFW_KEY_ESCAPE:
        gRun = false;
        break;
    case GLFW_KEY_F3:
        toggleCameraViewMode();
        break;
    default:
        break;
    }
}

void keyPressCallback(int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS)
    {
        handlePress(key, scancode, action, mods);
    }

    if (gUserKeyPressCallback)
        gUserKeyPressCallback(gUserCallbackData, key, scancode, action, mods);
}

void mouseDownCallback(int button, float x, float y, int mods)
{
    if (gUserMouseDownCallback)
        gUserMouseDownCallback(gUserCallbackData, button, x, y, mods);
}

void mouseUpCallback(int button, float x, float y, int mods)
{
    if (gUserMouseUpCallback)
        gUserMouseUpCallback(gUserCallbackData, button, x, y, mods);
}

void mouseMoveCallback(float x, float y)
{
    if (gUserMouseMoveCallback)
        gUserMouseMoveCallback(gUserCallbackData, x, y);
}

void mouseWheelCallback(float dx, float dy)
{
    if (gUserMouseWheelCallback)
        gUserMouseWheelCallback(gUserCallbackData, dx, dy);
}

bool initWindowApp(bool fullScreen, bool offscreen, int32_t width, int32_t height,
                   void* userCallbackData,
                   void (*userKeyPressCallback)(void*, int, int, int, int),
                   void (*userMouseDownCallback)(void*, int, float, float, int),
                   void (*userMouseUpCallback)(void*, int, float, float, int),
                   void (*userMouseMoveCallback)(void*, float, float),
                   void (*userMouseWheelCallback)(void*, float, float))
{
    gUserCallbackData       = userCallbackData;
    gUserKeyPressCallback   = userKeyPressCallback;
    gUserMouseDownCallback  = userMouseDownCallback;
    gUserMouseUpCallback    = userMouseUpCallback;
    gUserMouseMoveCallback  = userMouseMoveCallback;
    gUserMouseWheelCallback = userMouseWheelCallback;

    gRun = true;

    try
    {
        gWindow = WindowBase::create("Mini Pipeline", width, height, offscreen, 0, true, fullScreen);
    }
    catch (const std::exception& /*ex*/)
    {
        gWindow = nullptr;
    }
    if ((!offscreen) && gWindow == nullptr)
        return false;

    if (!offscreen)
    {
        gWindow->makeCurrent();
        gWindow->setOnKeypressCallback(keyPressCallback);
        gWindow->setOnMouseDownCallback(mouseDownCallback);
        gWindow->setOnMouseUpCallback(mouseUpCallback);
        gWindow->setOnMouseMoveCallback(mouseMoveCallback);
        gWindow->setOnMouseWheelCallback(mouseWheelCallback);
    }

    return true;
}

void releaseWindowApp()
{
    // Shutdown
    delete gWindow;
    gWindow = nullptr;
}
