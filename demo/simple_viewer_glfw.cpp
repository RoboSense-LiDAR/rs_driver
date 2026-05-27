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

#include <cmath>

#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <vector>
#include <mutex>
#include <thread>
#include <iostream>
#include <filesystem>

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl2.h"

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

// ---------------- UPDATE ----------------

void updatePointCloud(const std::vector<SimplePoint>& pts)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    g_points = pts;
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

            double dx = x - lastX;
            double dy = y - lastY;

            rotY += dx * 0.5f;
            rotX += dy * 0.5f;

            if (rotX > 89.0f) rotX = 89.0f;
            if (rotX < -89.0f) rotX = -89.0f;
        }
        else
        {
            mouseDown = false;
        }

        if (rmb == GLFW_PRESS)
        {
            if (!rightMouseDown)
            {
                rightMouseDown = true;
                lastX = x;
                lastY = y;
            }

            double dx = x - lastX;
            double dy = y - lastY;

            float panSpeed = 0.01f * zoom;

            panX += dx * panSpeed;
            panY -= dy * panSpeed;
        }
        else
        {
            rightMouseDown = false;
        }

        lastX = x;
        lastY = y;
    }
}

// Scroll Callback
void scroll_callback(GLFWwindow*, double, double yoffset)
{
    ImGuiIO& io = ImGui::GetIO();
    if (!io.WantCaptureMouse)
        zoom -= yoffset * 0.5f;
}

// Resize Callback
void framebuffer_size_callback(GLFWwindow*, int width, int height)
{
    if (height == 0) height = 1;

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(60.0, (float)width / height, 0.1, 500.0);

    glMatrixMode(GL_MODELVIEW);
}

// ---------------- THREAD ----------------

void viewerThread()
{
    if (!glfwInit())
    {
        std::cerr << "GLFW init failed\n";
        return;
    }
    std::cout << "GLFW init OK" << std::endl;

    GLFWwindow* window = glfwCreateWindow(1024, 768, "LiDAR Viewer", NULL, NULL);

    if (!window)
    {
        std::cerr << "GLFW window creation failed!" << std::endl;
        glfwTerminate();
        return;
    }
    std::cout << "Window OK" << std::endl;

    glfwMakeContextCurrent(window);

    glfwSetScrollCallback(window, scroll_callback);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    glEnable(GL_DEPTH_TEST);
    glPointSize(2.0f);

    glClearColor(0.05f, 0.05f, 0.1f, 1.0f);

    // ImGui Setup
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    if (ImGui::GetCurrentContext() == nullptr)
    {
        std::cerr << "ImGui context failed!" << std::endl;
        return;
    }


    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL2_Init();

    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    framebuffer_size_callback(window, width, height);

    // ---------------- LOOP ----------------
    
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        processInput(window);

        
        ImGui_ImplGlfw_NewFrame();
        ImGui_ImplOpenGL2_NewFrame();
        ImGui::NewFrame();

        // UI
        ImGui::Begin("LiDAR Controls");

        IGFD::FileDialogConfig config;
        config.path = ".";

        if (ImGui::Button("Select Output Folder"))
        {
            ImGuiFileDialog::Instance()->OpenDialog(
                "ChooseFolder",
                "Choose Folder",
                nullptr,
                config
            );
        }

        if (ImGuiFileDialog::Instance()->Display("ChooseFolder"))
        {
            if (ImGuiFileDialog::Instance()->IsOk())
            {
                g_output_folder =
                    ImGuiFileDialog::Instance()->GetCurrentPath();

                std::filesystem::create_directories(g_output_folder);
                g_frame_counter = 0;
            }
            ImGuiFileDialog::Instance()->Close();
        }

        ImGui::Text("Folder:");
        ImGui::TextWrapped("%s", g_output_folder.c_str());

        if (!g_recording)
        {
            if (ImGui::Button("Start Recording"))
            {
                if (!g_output_folder.empty())
                {
                    g_frame_counter = 0;
                    g_recording = true;
                }
            }
        }
        else
        {
            if (ImGui::Button("Stop Recording"))
                g_recording = false;
        }

        ImGui::Text("Recording: %s", g_recording ? "ON" : "OFF");
        ImGui::Text("Frames saved: %llu", g_frame_counter);

        ImGui::Text("Points: %d", (int)g_points.size());
        ImGui::SliderFloat("Zoom", &zoom, 0.5f, 50.0f);
        ImGui::SliderFloat("RotX", &rotX, -90.0f, 90.0f);
        ImGui::SliderFloat("RotY", &rotY, -180.0f, 180.0f);

        ImGui::Text("Pan: %.2f %.2f", panX, panY);

        ImGui::End();

        render();

        ImGui::Render();
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }
    

    

    // Cleanup
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
}

// ---------------- START ----------------

void startViewer()
{
    //std::thread(viewerThread).detach();
    viewerThread();   // direkt im Main Thread
}
