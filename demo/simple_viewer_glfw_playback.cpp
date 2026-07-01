#include "simple_viewer.h"

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <GLFW/glfw3.h>

#include <cmath>
#include <vector>
#include <mutex>
#include <thread>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <atomic>
#include <algorithm>

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "ImGuiFileDialog.h"

// ---------------- GLOBALS ----------------

static std::vector<SimplePoint> g_points;
static std::mutex g_mutex;

static float zoom = 5.0f;
static float rotX = 10.0f;
static float rotY = 90.0f;

static float panX = 0.0f;
static float panY = 0.0f;

static bool mouseDown = false;
static bool rightMouseDown = false;

static double lastX = 0;
static double lastY = 0;

// Playback
static std::vector<std::filesystem::path> g_bin_files;
static std::atomic<bool> g_playback(false);
static std::thread g_playback_thread;
static size_t g_current_frame = 0;
static std::atomic<bool> g_pause(false);
static int g_step_size = 10; // Default: 10 Frames springen
static int g_ms_delay = 100; // Default: 100 ms Framedauer (je höher, desto niedriger die playback-speed)

// ---------------- UPDATE ----------------

void updatePointCloud(const std::vector<SimplePoint>& pts)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    g_points = pts;
}

// ---------------- BIN LOADER ----------------

std::vector<SimplePoint> loadBinFile(const std::string& path)
{
    std::vector<SimplePoint> pts;

    std::ifstream in(path, std::ios::binary);
    if (!in) return pts;

    in.seekg(0, std::ios::end);
    size_t size = in.tellg();
    in.seekg(0);

    std::vector<float> buffer(size / sizeof(float));
    in.read(reinterpret_cast<char*>(buffer.data()), size);

    for (size_t i = 0; i + 3 < buffer.size(); i += 4)
    {
        pts.push_back({
            buffer[i + 0],
            buffer[i + 1],
            buffer[i + 2],
            buffer[i + 3]
        });
    }

    return pts;
}

// ---------------- PLAYBACK THREAD ----------------

void playbackThreadFunc()
{
    while (g_playback)
    {
        
		if (g_pause)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

		
		if (g_bin_files.empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(g_ms_delay));
            continue;
        }

        if (g_current_frame >= g_bin_files.size())
            g_current_frame = 0;

        auto pts = loadBinFile(g_bin_files[g_current_frame].string());
        updatePointCloud(pts);

        g_current_frame++;

        std::this_thread::sleep_for(std::chrono::milliseconds(g_ms_delay));
    }
}


void jumpFrames(int offset)
{
    if (g_bin_files.empty()) return;

    int new_index = static_cast<int>(g_current_frame) + offset;

    if (new_index < 0)
        new_index = 0;

    if (new_index >= (int)g_bin_files.size())
        new_index = (int)g_bin_files.size() - 1;

    g_current_frame = (size_t)new_index;

    // sofort anzeigen
    auto pts = loadBinFile(g_bin_files[g_current_frame].string());
    updatePointCloud(pts);
}


// ---------------- RENDER ----------------

void render()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();

    glTranslatef(panX, panY, 0.0f);
    glTranslatef(0.0f, 0.0f, -zoom);

    glRotatef(rotX, 1, 0, 0);
    glRotatef(rotY, 0, 1, 0);

    glBegin(GL_POINTS);

    std::lock_guard<std::mutex> lock(g_mutex);

    for (const auto& p : g_points)
    {
        float i = logf(1.0f + p.intensity) / logf(256.0f);

        float r = i;
        float g = 1.0f - fabs(i - 0.5f) * 2.0f;
        float b = 1.0f - i;

        glColor3f(r, g, b);
        glVertex3f(p.x * 0.1f, p.z * 0.1f, -p.y * 0.1f);
    }

    glEnd();
}

// ---------------- INPUT ----------------

void processInput(GLFWwindow* window)
{
    ImGuiIO& io = ImGui::GetIO();

    if (!io.WantCaptureMouse)
    {
        double x, y;
        glfwGetCursorPos(window, &x, &y);

        int lmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
        int rmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);

        if (lmb == GLFW_PRESS)
        {
            if (!mouseDown)
            {
                mouseDown = true;
                lastX = x;
                lastY = y;
            }

            rotY += (x - lastX) * 0.5f;
            rotX += (y - lastY) * 0.5f;

            rotX = std::clamp(rotX, -89.0f, 89.0f);
        }
        else mouseDown = false;

        if (rmb == GLFW_PRESS)
        {
            if (!rightMouseDown)
            {
                rightMouseDown = true;
                lastX = x;
                lastY = y;
            }

            float panSpeed = 0.01f * zoom;
            panX += (x - lastX) * panSpeed;
            panY -= (y - lastY) * panSpeed;
        }
        else rightMouseDown = false;

        lastX = x;
        lastY = y;
    }
}

void scroll_callback(GLFWwindow*, double, double yoffset)
{
    if (!ImGui::GetIO().WantCaptureMouse)
        zoom -= yoffset * 0.5f;
}

void framebuffer_size_callback(GLFWwindow*, int width, int height)
{
    if (height == 0) height = 1;

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (float)width / height, 0.1, 500.0);

    glMatrixMode(GL_MODELVIEW);
}

// ---------------- VIEWER THREAD ----------------

void viewerThread()
{
    glfwInit();
    GLFWwindow* window = glfwCreateWindow(1024, 768, "LiDAR Playback Viewer", NULL, NULL);
    glfwMakeContextCurrent(window);

    glfwSetScrollCallback(window, scroll_callback);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    glEnable(GL_DEPTH_TEST);
    glPointSize(2.0f);
    glClearColor(0.05f, 0.05f, 0.1f, 1.0f);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    framebuffer_size_callback(window, width, height);

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();
        processInput(window);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Playback Controls");

        IGFD::FileDialogConfig config;
        config.path = ".";

        if (ImGui::Button("Select Folder"))
        {
            ImGuiFileDialog::Instance()->OpenDialog("ChooseFolder", "Choose Folder", nullptr, config);
        }

        if (ImGuiFileDialog::Instance()->Display("ChooseFolder"))
        {
            if (ImGuiFileDialog::Instance()->IsOk())
            {
                std::string folder = ImGuiFileDialog::Instance()->GetCurrentPath();

                g_bin_files.clear();

                for (auto& e : std::filesystem::directory_iterator(folder))
                {
                    if (e.path().extension() == ".bin")
                        g_bin_files.push_back(e.path());
                }

                std::sort(g_bin_files.begin(), g_bin_files.end());
                g_current_frame = 0;
            }
            ImGuiFileDialog::Instance()->Close();
        }

        ImGui::Text("Files: %d", (int)g_bin_files.size());
        ImGui::Text("Frame: %d", (int)g_current_frame);

        ImGui::Separator();
        ImGui::Text("Playback Controls");

        if (!g_playback)
        {
            if (ImGui::Button("Start"))
            {
                if (!g_bin_files.empty())
                {
                    g_playback = true;
                    g_playback_thread = std::thread(playbackThreadFunc);
                }
            }
        }
        else
        {
            if (ImGui::Button("Stop"))
            {
                g_playback = false;
                if (g_playback_thread.joinable())
                    g_playback_thread.join();
            }
        }
		
        ImGui::SameLine();

		// Pause / Resume
		if (!g_pause)
		{
			if (ImGui::Button("Pause"))
				g_pause = true;
		}
		else
		{
			if (ImGui::Button("Resume"))
				g_pause = false;
		}

        
        ImGui::SliderInt("Frame Time (ms)", &g_ms_delay, 10, 500);


        ImGui::InputInt("Step Size", &g_step_size);

        if (g_step_size < 1)
            g_step_size = 1;


		// Vor / Zurück Buttons
		if (ImGui::Button("<< Back"))
		{
			g_pause = true;
			jumpFrames(-g_step_size);
		}

		ImGui::SameLine();

		if (ImGui::Button("Forward >>"))
		{
			g_pause = true;
			jumpFrames(g_step_size);
		}


        ImGui::SliderInt("Frame",
            (int*)&g_current_frame,
            0,
            (int)g_bin_files.size() - 1);

        ImGui::Separator();

        ImGui::SliderFloat("Zoom", &zoom, 0.5f, 50.0f);

        ImGui::End();

        render();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    g_playback = false;
    if (g_playback_thread.joinable())
        g_playback_thread.join();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
}

// ---------------- START ----------------

void startViewer()
{
    std::thread(viewerThread).detach();
}
