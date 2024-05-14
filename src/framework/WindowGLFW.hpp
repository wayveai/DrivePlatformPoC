/////////////////////////////////////////////////////////////////////////////////////////
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA CORPORATION & AFFILIATES assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA CORPORATION & AFFILIATES. No third party distribution is allowed unless
// expressly authorized by NVIDIA. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA CORPORATION & AFFILIATES products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA CORPORATION & AFFILIATES.
//
// SPDX-FileCopyrightText: Copyright (c) 2014-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//
// NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
// property and proprietary rights in and to this material, related
// documentation and any modifications thereto. Any use, reproduction,
// disclosure or distribution of this material and related documentation
// without an express license agreement from NVIDIA CORPORATION or
// its affiliates is strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef SAMPLES_COMMON_WINDOWGLFW_HPP_
#define SAMPLES_COMMON_WINDOWGLFW_HPP_

#include "Window.hpp"

#ifdef DW_USE_EGL
#define GLFW_INCLUDE_ES3
#ifndef VIBRANTE_V5Q
#ifndef GLFW_USE_WAYLAND
#define GLFW_EXPOSE_NATIVE_X11
#endif
#endif
#define GLFW_EXPOSE_NATIVE_EGL
#include <EGL/egl.h>
#endif

#include <GLFW/glfw3.h>

// Status is defined by some X11/EGL headers
// This is needed by GLFW/glfw3.h but can be undefined
// after it is included to avoid clashes with
// other things called Status elsewhere in the code.
#ifdef Status
#undef Status
#endif

// ... and similarly with Success
// defined by the X11/EGL headers
#ifdef Success
#undef Success
#endif

#include <memory>

class WindowGLFW : public WindowBase
{
public:
    // create an X11 window
    //   width: width of window
    //   height: height of window
    //   offscreen: rendering to this window is not intended to present to the display
    //   samples: specifies the desired number of samples to use for subsampling
    //   initIvisible: make the window initially invisible, setWindowVisibility can be
    //                 used to change visibility some time after window creation
    //   fullScreen: create a full screen window
    WindowGLFW(const char* title, int width, int height, bool offscreen, int samples = 0,
               bool initInvisible = false, bool fullScreen = false);
    WindowGLFW(int w, int h, bool offscreen = false, int samples = 0)
        : WindowGLFW("DriveWorks", w, h, offscreen, samples)
    {
    }

    // release window
    ~WindowGLFW() override;

    // poll events and swap back and front buffers
    bool swapBuffers() override;

    // swap back and front buffers ONLY, does not poll events
    bool swapBuffersOnly() override;

    // release the current context
    bool releaseContext() override;

    // reset EGL context
    void resetContext() override;

#ifdef DW_USE_EGL
    EGLContext createSharedContext() const override;
#else
    GLFWwindow* createSharedContext() const override;
#endif

    // make window context current to the calling thread
    bool makeCurrent() override;

    // remove current window context from the calling thread
    bool resetCurrent() override;

    bool shouldClose() override { return glfwWindowShouldClose(m_hWindow) != 0; }
    bool isOffscreen() const override { return m_offscreen; }

#ifdef DW_USE_EGL
    bool isEGLEnabled() const override
    {
        return true;
    }
#endif

    // Set the window size
    bool setWindowSize(int width, int height) override;

    // Get the current desktop resolution
    bool getDesktopResolution(int& width, int& height) override;

    // Set windowed mode window to full screen
    bool setFullScreen() override;

    bool setWindowPosCentered() override;

    bool setWindowVisibility(bool visible) override;

    bool setWindowTitle(const char* title);

    // get EGL display
    EGLDisplay getEGLDisplay(void) override;
    EGLContext getEGLContext(void) override;

    GLFWwindow* getGLFW() const { return m_hWindow; }

    virtual void onKeyCallback(int key, int scancode, int action, int mods);
    virtual void onMouseButtonCallback(int button, int action, int mods);
    virtual void onMouseMoveCallback(double x, double y);
    virtual void onMouseWheelCallback(double dx, double dy);
    virtual void onCharModsCallback(uint32_t codepoint, int32_t mods);
    virtual void onResizeWindowCallback(int width, int height);
    virtual void onDropCallback(int count, const char** paths);

protected:
    GLFWwindow* m_hWindow;

    bool m_offscreen;

#ifdef DW_USE_EGL
    EGLDisplay m_display;
    EGLContext m_context;
    std::unique_ptr<EGLConfig[]> m_config;
#endif
};

#endif // SAMPLES_COMMON_WINDOWGLFW_HPP_
