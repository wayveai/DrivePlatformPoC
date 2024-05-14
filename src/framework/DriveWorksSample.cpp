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
// SPDX-FileCopyrightText: Copyright (c) 2015-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "DriveWorksSample.hpp"

#include <signal.h>

#include <framework/ChecksExt.hpp>
#include <framework/WindowGLFW.hpp>
#include <framework/MathUtils.hpp>
#include <framework/Log.hpp>
#include <framework/SamplesDataPath.hpp>
#include <framework/SimpleRenderer.hpp>
#include <dwtrace/dw/trace/GlobalTracer.hpp>

// System includes
#include <thread>
#include <condition_variable>

#if (!WINDOWS)
#include <termios.h>
#include <unistd.h>
#include <csignal>
#endif

namespace dw_samples
{
namespace common
{

//------------------------------------------------------------------------------
DriveWorksSample* DriveWorksSample::g_instance = nullptr;

//------------------------------------------------------------------------------
DriveWorksSample::DriveWorksSample(const ProgramArguments& args)
    : m_profiler()
    , m_args(args)
    , m_run(true)
    , m_pause(false)
    , m_playSingleFrame(false)
    , m_reset(false)
    , m_processPeriod(0)
    , m_renderPeriod(convertFrequencyToPeriod(60))
    , m_frameIdx(0)
    , m_stopFrameIdx(0)
    , m_commandLineInputActive(false)
{
    // ----------- Singleton -----------------
    if (g_instance)
        throw std::runtime_error("Can only create one app in the process.");
    g_instance = this;

    // ----------- Signals -----------------
    struct sigaction action = {};
    action.sa_handler       = DriveWorksSample::globalSigHandler;

    sigaction(SIGHUP, &action, NULL);  // controlling terminal closed, Ctrl-D
    sigaction(SIGINT, &action, NULL);  // Ctrl-C
    sigaction(SIGQUIT, &action, NULL); // Ctrl-\, clean quit with core dump
    sigaction(SIGABRT, &action, NULL); // abort() called.
    sigaction(SIGTERM, &action, NULL); // kill command
    sigaction(SIGSTOP, &action, NULL); // kill command

#if (VIBRANTE_PDK_DECIMAL == 6000400)
    // Some sensors rely on FSI Coms which requires that SIGRTMIN is blocked by client application
    // threads as the signal is used by the NvFsiCom daemon.
    sigset_t lSigSet;
    (void)sigemptyset(&lSigSet);
    sigaddset(&lSigSet, SIGRTMIN);
    if (pthread_sigmask(SIG_BLOCK, &lSigSet, NULL) != 0)
        std::cout << "pthread_sigmask failed" << std::endl;
#endif

    // ----------- Initialization -----------------
    cudaFree(0);

    m_doProfile = atoi(getArgument("profiling").c_str());
    if (!m_doProfile)
        std::cout << "Sample profiling is disabled\n";

    if (!getArgument("dwTracePath").empty())
    {
        std::string tracePath = getArgument("dwTracePath").c_str();
        initDWTrace(tracePath);
    }
}

//------------------------------------------------------------------------------
void DriveWorksSample::initializeWindow(const char* title, int width, int height, bool offscreen, int samples)
{
    // -------------------------------------------
    // Initialize GL
    // -------------------------------------------
    m_window.reset(WindowBase::create(title, width, height, offscreen, samples));
    m_window->makeCurrent();
    m_window->setOnResizeWindowCallback(resizeCb);

    // Some window implementations (e.g., on QNX) like to go fullscreen and ignore the arguments,
    // so query the actual final window size after creation
    m_width  = m_window->width();
    m_height = m_window->height();
    m_title  = title;

    glClearColor(0, 0, 0, 0);

    // Start window init thread
    m_windowInitThreadActive = true;
    m_windowInitThread       = std::thread([&] {
        static uint32_t SLEEP_INTERVAL_US = 15000;

        std::string tempTitle = "Loading";
        m_window->setWindowTitle(tempTitle.c_str());

        int32_t dotCount = 0;
        while (m_windowInitThreadActive)
        {
            {
                if (dotCount > 199)
                    dotCount = 0;

                std::string newTitle = tempTitle;
                for (int32_t i = 0; i < dotCount / 50; i++)
                    newTitle += ".";

                dotCount++;
                m_window->setWindowTitle(newTitle.c_str());
            }

            usleep(SLEEP_INTERVAL_US);
        }

        m_window->setWindowTitle(m_title.c_str());
        m_window->setOnKeyDownCallback(keyDownCb);
        m_window->setOnKeyUpCallback(keyUpCb);
        m_window->setOnKeyRepeatCallback(keyRepeatCb);
        m_window->setOnCharModsCallback(charModsCb);
        m_window->setOnMouseUpCallback(mouseUpCb);
        m_window->setOnMouseDownCallback(mouseDownCb);
        m_window->setOnMouseMoveCallback(mouseMoveCb);
        m_window->setOnMouseWheelCallback(mouseWheelCb);
        m_window->setOnDropCallback(dropCb);
    });

    CHECK_GL_ERROR();
}

//------------------------------------------------------------------------------
void DriveWorksSample::stopWindowInitThread()
{
    if (m_windowInitThreadActive)
    {
        m_windowInitThreadActive = false;

        if (m_windowInitThread.joinable())
            m_windowInitThread.join();

        // It's possible the window was resized while we
        // were spinning here, so call the resize callback
        // again.
        resize(m_width, m_height);
    }
}

//------------------------------------------------------------------------------
DriveWorksSample::~DriveWorksSample()
{
    DW_TRACE_FLUSH(true);

    g_instance = nullptr;
    stopWindowInitThread();
}
//------------------------------------------------------------------------------
void DriveWorksSample::initializeCommandLineInput()
{
    m_commandLineInputActive = true;
    m_commandLineInputThread = std::thread(&DriveWorksSample::readCLIKeyPressLoop, this);
}

//------------------------------------------------------------------------------
char DriveWorksSample::readCommandLineChar()
{
    char buf           = 0;
    struct termios old = {0};
    // tcgetattr may have error on some shells
    // if so, use cin which requires user to
    // press return
    if (tcgetattr(0, &old) < 0)
    {
        buf = static_cast<char>(std::cin.get());
    }
    else
    {
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN]  = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
            perror("tcsetattr ICANON");
        if (read(STDIN_FILENO, &buf, 1) < 0)
            perror("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
            perror("tcsetattr ~ICANON");
    }
    return buf;
}

//------------------------------------------------------------------------------
void DriveWorksSample::readCLIKeyPressLoop()
{

    int32_t key     = 0;
    int32_t lastKey = -1;

    timepoint_t start                     = myclock_t::now();
    const uint64_t MAX_KEY_REPEAT_TIME_MS = 800;

    while (shouldRun())
    {
        key = readCommandLineChar();
        if (lastKey == -1)
            start = myclock_t::now();

        // Convert to ascii lower case if
        // upper case character
        if (key >= 'A' && key <= 'Z')
        {
            key -= ('A' - 'a');
        }
        // If key code was escape, convert to public escape
        if (key == 27)
        {
            key = GLFW_KEY_ESCAPE;
        }

        uint64_t elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  myclock_t::now() - start)
                                  .count();
        if (elapsed_ms < MAX_KEY_REPEAT_TIME_MS && key == lastKey)
        {
            keyRepeat(key, 0, 0);
        }
        else
        {
            keyDown(key, 0, 0);
        }

        lastKey = key;
        start   = myclock_t::now();
    }
}

//------------------------------------------------------------------------------
bool DriveWorksSample::shouldRun()
{
    if (m_window)
    {
        return !m_window->shouldClose() && m_run &&
               (m_frameIdx < m_stopFrameIdx || m_stopFrameIdx == 0);
    }

    return m_run;
}

//------------------------------------------------------------------------------
void DriveWorksSample::resume()
{
    m_pause = false;
}

//------------------------------------------------------------------------------
void DriveWorksSample::pause()
{
    m_pause = true;
}

//------------------------------------------------------------------------------
bool DriveWorksSample::isPaused() const
{
    return m_pause;
}

//------------------------------------------------------------------------------
void DriveWorksSample::reset()
{
    m_reset = true;
}

//------------------------------------------------------------------------------
void DriveWorksSample::stop()
{
    m_run = false;
}

//------------------------------------------------------------------------------
void DriveWorksSample::stopWindowCallbacks()
{
    // stop any callbacks
    if (m_window)
    {
        m_window->setOnKeyDownCallback(nullptr);
        m_window->setOnKeyUpCallback(nullptr);
        m_window->setOnKeyRepeatCallback(nullptr);
        m_window->setOnCharModsCallback(nullptr);
        m_window->setOnMouseUpCallback(nullptr);
        m_window->setOnMouseDownCallback(nullptr);
        m_window->setOnMouseMoveCallback(nullptr);
        m_window->setOnMouseWheelCallback(nullptr);
        m_window->setOnCharModsCallback(nullptr);
        m_window->setOnResizeWindowCallback(nullptr);
        m_window->setOnDropCallback(nullptr);
    }
}

//------------------------------------------------------------------------------
auto DriveWorksSample::convertFrequencyToPeriod(int loopsPerSecond) -> myclock_t::duration
{
    using ns = std::chrono::nanoseconds;

    if (loopsPerSecond <= 0)
        return std::chrono::milliseconds(0);
    else
        return ns(static_cast<ns::rep>(1e9 / loopsPerSecond));
}

//------------------------------------------------------------------------------
void DriveWorksSample::setProcessRate(int loopsPerSecond)
{
    m_processPeriod = convertFrequencyToPeriod(loopsPerSecond);
}

//------------------------------------------------------------------------------
void DriveWorksSample::setRenderRate(int loopsPerSecond)
{
    m_renderPeriod = convertFrequencyToPeriod(loopsPerSecond);
}

//------------------------------------------------------------------------------
void DriveWorksSample::setStopFrame(uint32_t stopFrame)
{
    m_stopFrameIdx = stopFrame;
}

//------------------------------------------------------------------------------
void DriveWorksSample::onSignal(int)
{
    if (m_commandLineInputActive)
        log("Press any key to exit...\n");
    stop();
}

//------------------------------------------------------------------------------
float32_t DriveWorksSample::getCurrentFPS() const
{
    return m_currentFPS;
}

//------------------------------------------------------------------------------
dwPrecision DriveWorksSample::getLowestSupportedFloatPrecision(dwContextHandle_t ctx)
{
    const char* gpuArch = nullptr;
    dwStatus status     = dwContext_getGPUArchitecture(&gpuArch, ctx);
    if (status != DW_SUCCESS)
    {
        throw std::runtime_error("Unable to get GPU architecture.");
    }

    // Pascal GPU should set precision to FP32
    if (strcmp(gpuArch, "gp1xx-discrete") == 0)
    {
        dwLogger_log(ctx, DW_LOG_WARN, "Warning: Current GPU does not support FP16 inference. The FP32 DNN model is selected instead.\n");
        return DW_PRECISION_FP32;
    }
    else
    {
        return DW_PRECISION_FP16;
    }
}

//------------------------------------------------------------------------------
void DriveWorksSample::keyDown(int key, int scancode, int mods)
{

    // stop application
    if (key == GLFW_KEY_ESCAPE)
        m_run = false;
    else if (key == GLFW_KEY_SPACE)
        m_pause = !m_pause;
    else if (key == GLFW_KEY_F5)
    {
        m_playSingleFrame = !m_playSingleFrame;
        m_pause           = m_playSingleFrame;
    }
    else if (key == GLFW_KEY_R)
        m_reset = true;

    onKeyDown(key, scancode, mods);
}

//------------------------------------------------------------------------------
void DriveWorksSample::keyUp(int key, int scancode, int mods)
{
    onKeyUp(key, scancode, mods);
}

//------------------------------------------------------------------------------
void DriveWorksSample::keyRepeat(int key, int scancode, int mods)
{
    onKeyRepeat(key, scancode, mods);
}

//------------------------------------------------------------------------------
void DriveWorksSample::charMods(unsigned int codepoint, int mods)
{
    onCharMods(codepoint, mods);
}

//------------------------------------------------------------------------------
void DriveWorksSample::mouseDown(int button, float x, float y, int mods)
{
    onMouseDown(button, x, y, mods);
}

//------------------------------------------------------------------------------
void DriveWorksSample::mouseUp(int button, float x, float y, int mods)
{
    onMouseUp(button, x, y, mods);
}

//------------------------------------------------------------------------------
void DriveWorksSample::mouseMove(float x, float y)
{
    onMouseMove(x, y);
}

//------------------------------------------------------------------------------
void DriveWorksSample::mouseWheel(float x, float y)
{
    onMouseWheel(x, y);
}

//------------------------------------------------------------------------------
void DriveWorksSample::keyDownCb(int key, int scancode, int mods)
{
    instance()->keyDown(key, scancode, mods);
}

//------------------------------------------------------------------------------
void DriveWorksSample::keyUpCb(int key, int scancode, int mods)
{
    instance()->keyUp(key, scancode, mods);
}

//------------------------------------------------------------------------------
void DriveWorksSample::keyRepeatCb(int key, int scancode, int mods)
{
    instance()->keyRepeat(key, scancode, mods);
}

//------------------------------------------------------------------------------
void DriveWorksSample::charModsCb(unsigned int codepoint, int mods)
{
    instance()->charMods(codepoint, mods);
}

//------------------------------------------------------------------------------
void DriveWorksSample::mouseDownCb(int button, float x, float y, int mods)
{
    instance()->mouseDown(button, x, y, mods);
}

//------------------------------------------------------------------------------
void DriveWorksSample::mouseUpCb(int button, float x, float y, int mods)
{
    instance()->mouseUp(button, x, y, mods);
}

//------------------------------------------------------------------------------
void DriveWorksSample::mouseMoveCb(float x, float y)
{
    instance()->mouseMove(x, y);
}

//------------------------------------------------------------------------------
void DriveWorksSample::mouseWheelCb(float x, float y)
{
    instance()->mouseWheel(x, y);
}

//------------------------------------------------------------------------------
void DriveWorksSample::resizeCb(int width, int height)
{
    instance()->resize(width, height);
}

//------------------------------------------------------------------------------
void DriveWorksSample::dropCb(int count, const char** paths)
{
    instance()->drop(count, paths);
}

//------------------------------------------------------------------------------
void DriveWorksSample::resize(int width, int height)
{
    m_mouseView.setWindowAspect((float)width / height);

    if (m_initialized)
        onResizeWindow(width, height);

    m_width  = width;
    m_height = height;
}

//------------------------------------------------------------------------------
void DriveWorksSample::drop(int count, const char** paths)
{
    onDrop(count, paths);
}

//------------------------------------------------------------------------------
EGLDisplay DriveWorksSample::getEGLDisplay() const
{
    if (m_window)
        return m_window->getEGLDisplay();
    return 0;
}

//------------------------------------------------------------------------------
void DriveWorksSample::createSharedContext() const
{
    if (m_window)
        m_window->createSharedContext();
}

//------------------------------------------------------------------------------
WindowBase* DriveWorksSample::getWindow() const
{
    return m_window.get();
}

//------------------------------------------------------------------------------
int DriveWorksSample::getWindowWidth() const
{
    if (m_window)
        return m_window->width();
    return 0;
}

//------------------------------------------------------------------------------
int DriveWorksSample::getWindowHeight() const
{
    if (m_window)
        return m_window->height();
    return 0;
}

//------------------------------------------------------------------------------
void DriveWorksSample::setWindowSize(int width, int height)
{
    if (m_window)
        m_window->setWindowSize(width, height);
}

//------------------------------------------------------------------------------
bool DriveWorksSample::isOffscreen() const
{
    if (!m_window)
        return true;
    return m_window->isOffscreen();
}

//------------------------------------------------------------------------------
const std::string& DriveWorksSample::getArgument(const char* name) const
{
    return m_args.get(name);
}

//------------------------------------------------------------------------------
ProfilerCUDA* DriveWorksSample::getProfilerCUDA()
{
    return &m_profiler;
}

//------------------------------------------------------------------------------
MouseView3D& DriveWorksSample::getMouseView()
{
    return m_mouseView;
}

ProgramArguments& DriveWorksSample::getArgs()
{
    return m_args;
}

uint32_t DriveWorksSample::getFrameIndex() const
{
    return m_frameIdx;
}

void DriveWorksSample::calculateFPS()
{
    // This is the time that the previous iteration took
    auto timeSinceUpdate = myclock_t::now() - m_lastFPSRunTime;

    // Count FPS
    if (!m_pause)
    {
        m_lastFPSRunTime            = myclock_t::now();
        m_fpsBuffer[m_fpsSampleIdx] = timeSinceUpdate.count();
        m_fpsSampleIdx              = (m_fpsSampleIdx + 1) % FPS_BUFFER_SIZE;

        float32_t totalTime = 0;
        for (uint32_t i = 0; i < FPS_BUFFER_SIZE; i++)
            totalTime += m_fpsBuffer[i];

        myclock_t::duration meanTime(static_cast<myclock_t::duration::rep>(
            totalTime / FPS_BUFFER_SIZE));
        m_currentFPS = 1e6f / static_cast<float32_t>(
                                  std::chrono::duration_cast<std::chrono::microseconds>(
                                      meanTime)
                                      .count());
    }
}

//------------------------------------------------------------------------------
void DriveWorksSample::tryToSleep(timepoint_t lastRunTime)
{
    // This is the time that the previous iteration took
    auto timeSinceUpdate = myclock_t::now() - lastRunTime;

    // Decide which is the period for the master run loop
    myclock_t::duration runPeriod;
    if (!m_window)
    {
        // UI disabled, only process
        runPeriod = m_processPeriod;
    }
    if (m_pause)
    {
        // Process disabled, only render
        runPeriod = m_renderPeriod;
    }
    else
    {
        // Use minimum of both to ensure smooth UI
        if (m_renderPeriod < m_processPeriod)
            runPeriod = m_renderPeriod;
        else
            runPeriod = m_processPeriod;
    }

    // Limit framerate, sleep if necessary
    if (timeSinceUpdate < runPeriod)
    {
        auto sleepDuration = runPeriod - timeSinceUpdate;
        std::this_thread::sleep_for(sleepDuration);
    }
}

//------------------------------------------------------------------------------
DriveWorksSample* DriveWorksSample::instance()
{
    return g_instance;
}

//------------------------------------------------------------------------------
void DriveWorksSample::globalSigHandler(int sig)
{
    instance()->onSignal(sig);
}

//------------------------------------------------------------------------------
void DriveWorksSample::onReset()
{
    m_frameIdx = 0;
}

//------------------------------------------------------------------------------
int DriveWorksSample::run()
{
    // Main program loop
    m_run = true;

    if (!onInitialize())
        return -1;

    // Set initialization flag
    m_initialized = true;

    timepoint_t lastRunIterationTime = myclock_t::now() - m_renderPeriod - m_processPeriod;
    timepoint_t nextOnProcessTime    = myclock_t::now();
    timepoint_t nextOnRenderTime     = myclock_t::now();

    while (shouldRun())
    {
        if (m_reset)
        {
            onReset();
            m_reset = false;
        }

        // Iteration
        if (!m_pause && myclock_t::now() >= nextOnProcessTime)
        {
            nextOnProcessTime = myclock_t::now() + m_processPeriod;

            if (m_doProfile)
                m_profiler.tic("onProcess", true);
            onProcess();
            if (m_doProfile)
                m_profiler.toc();

            if (m_playSingleFrame)
                m_pause = true;
        }
        else
        {
            onPause();
        }

        // Sleep here in the middle so the rendering
        // is as regular as possible
        tryToSleep(lastRunIterationTime);
        lastRunIterationTime = myclock_t::now();

        if (shouldRun() && !m_reset && m_window && myclock_t::now() >= nextOnRenderTime)
        {
            nextOnRenderTime = myclock_t::now() + m_renderPeriod;

            if (m_clearOnRender)
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            if (m_doProfile)
                m_profiler.tic("onRender", true);

            stopWindowInitThread();
            onRender();
            calculateFPS();
            if (m_doProfile)
                m_profiler.toc();
            m_window->swapBuffers();
        }

        if (m_doProfile)
            m_profiler.collectTimers();

        if (!m_pause)
            m_frameIdx++;
    }

    // Show timings
    if (m_doProfile)
        if (!m_profiler.empty())
        {
            m_profiler.collectTimers();

            std::stringstream ss;
            ss << "Timing results:\n"
               << m_profiler << "\n";
            std::cout << ss.str();
        }

    // stop any window calls before destroying resources
    stopWindowCallbacks();
    onRelease();

    if (m_commandLineInputActive)
        m_commandLineInputThread.join();

    return 0;
}

void DriveWorksSample::initDWTrace(std::string& path)
{
    const uint64_t& channelMask          = dw::trace::DW_TRACE_CHAN_MASK(static_cast<uint32_t>(DW_TRACE_CHANNEL));
    const dw::trace::Level& tracingLevel = dw::trace::Level::LEVEL_50; // Driveworks API calls are at this level
    const bool& memTraceEnabled          = false;
    const bool& fullMemUsage             = false;
    const bool& diskIOStatsEnabled       = false;

    dw::trace::TracerConfig traceConfig(true, path.c_str(),
                                        channelMask, true, 8192, false,
                                        "", 0, true /*nvtx enabled */, (uint32_t)tracingLevel,
                                        20, false, memTraceEnabled,
                                        fullMemUsage, diskIOStatsEnabled, "0");
    DW_TRACE_INIT(traceConfig);
}
}
}
