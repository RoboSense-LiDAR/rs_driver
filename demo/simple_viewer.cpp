#include "simple_viewer.h"

#define NOMINMAX
#include <windows.h>
#include <gl/GL.h>
#include <vector>
#include <mutex>
#include <thread>
#include <iostream>

#include <gl/GLU.h>

#include "imgui.h"
#include "backends/imgui_impl_win32.h"
#include "backends/imgui_impl_opengl2.h"

extern LRESULT ImGui_ImplWin32_WndProcHandler(HWND, UINT, WPARAM, LPARAM);

static std::vector<SimplePoint> g_points;
static std::mutex g_mutex;

static float zoom = 1.0f;
static float rotX = 10.0f;
static float rotY = 90.0f;

static bool mouseDown = false;
static int lastX = 0;
static int lastY = 0;

static float panX = 0.0f;
static float panY = 0.0f;

static bool rightMouseDown = false;

//float g_min_intensity = 0.0f;
//float g_max_intensity = 255.0f;

// --- Update Points ---
void updatePointCloud(const std::vector<SimplePoint>& pts)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    //std::cout << "updatePointCloud: " << pts.size() << std::endl;

    g_points = pts;
}

// --- Render ---
void render()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    
    glLoadIdentity();

    // Pan zuerst
    glTranslatef(panX, panY, 0.0f);

    // dann Zoom
    glTranslatef(0.0f, 0.0f, -zoom);
    //glTranslatef(0, -0.5, -zoom);
    
    // dann Rotation
    glRotatef(rotX, 1, 0, 0);
    glRotatef(rotY, 0, 1, 0);

    //glPointSize(3.0f);
    
    glBegin(GL_POINTS);

    std::lock_guard<std::mutex> lock(g_mutex);

    for (const auto& p : g_points) {
        //glColor3f(1.0f, 1.0f, 1.0f);

        //float intensity = p.intensity / 255.0f;
        //glColor3f(intensity, intensity, intensity);


        //float i = p.intensity / 255.0f;
        //float i = (p.intensity - g_min_intensity) /
        //    (g_max_intensity - g_min_intensity + 1e-6f);
        float i = logf(1.0f + p.intensity) / logf(256.0f);

        float r = i;
        float g = 1.0f - fabs(i - 0.5f) * 2.0f;
        float b = 1.0f - i;

        glColor3f(r, g, b);


        //glVertex3f(p.x, p.y, p.z);
        glVertex3f(p.x * 0.1f, p.z * 0.1f, -p.y * 0.1f);
    }
    glColor3f(1.0f, 1.0f, 1.0f);

    glEnd();
    
    /*
    glPointSize(3.0f);

    glBegin(GL_POINTS);

    std::lock_guard<std::mutex> lock(g_mutex);

    for (const auto& p : g_points)
    {
        glColor3f(1.0f, 1.0f, 1.0f);
        glVertex3f(p.x * 0.1f, p.z * 0.1f, -p.y * 0.1f);
    }

    glEnd();
    */
}

// --- Window Proc ---
LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{

    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam))
        return true;
    
    ImGuiIO& io = ImGui::GetIO(); // determine if the ImGui "wants" the mouse

    switch (msg)
    {     
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;

    case WM_KEYDOWN:
        if (wParam == 'R')
        {
            panX = panY = 0;
            zoom = 5.0f;
            rotX = 10.0f;
            rotY = 90.0f;
        }
        if (wParam == VK_UP) rotX += 2;
        if (wParam == VK_DOWN) rotX -= 2;
        if (wParam == VK_LEFT) rotY -= 2;
        if (wParam == VK_RIGHT) rotY += 2;
        if (wParam == VK_OEM_PLUS) zoom += 2;
        if (wParam == VK_OEM_MINUS) zoom -= 2;
        return 0;

    case WM_LBUTTONDOWN:

        if (!io.WantCaptureMouse)
        {
            mouseDown = true;
            lastX = LOWORD(lParam);
            lastY = HIWORD(lParam);
        }
        return 0;

    case WM_LBUTTONUP:
        mouseDown = false;
        return 0;

    /*case WM_MOUSEMOVE:
    {
        if (mouseDown)
        {
            int x = LOWORD(lParam);
            int y = HIWORD(lParam);

            int dx = x - lastX;
            int dy = y - lastY;

            rotY += dx * 0.5f;   // horizontal drehen
            rotX += dy * 0.5f;   // vertikal drehen

            lastX = x;
            lastY = y;
        }
        return 0;
    }*/

    case WM_MOUSEWHEEL:
    {

        if (!io.WantCaptureMouse)
        {
            int delta = GET_WHEEL_DELTA_WPARAM(wParam);
            zoom += delta * 0.001f;   // fein einstellbar
        }
        return 0;
    }

    case WM_RBUTTONDOWN:

        if (!io.WantCaptureMouse)
        {
            rightMouseDown = true;
            lastX = LOWORD(lParam);
            lastY = HIWORD(lParam);
        }
        return 0;

    case WM_RBUTTONUP:

        
        rightMouseDown = false;
        
        return 0;


    case WM_MOUSEMOVE:
    {
        if (!io.WantCaptureMouse)
        {

            int x = LOWORD(lParam);
            int y = HIWORD(lParam);

            int dx = x - lastX;
            int dy = y - lastY;

            if (mouseDown)
            {
                rotY += dx * 0.5f;
                rotX += dy * 0.5f;

                // begrenzen
                if (rotX > 89.0f) rotX = 89.0f;
                if (rotX < -89.0f) rotX = -89.0f;
            }

            if (rightMouseDown)
            {
                float panSpeed = 0.01f * zoom;  // skaliert mit Entfernung

                panX += dx * panSpeed;
                panY -= dy * panSpeed;
            }

            lastX = x;
            lastY = y;

            return 0;
        }
    }
    /**/
    case WM_SIZE:
    {

        
        int width = LOWORD(lParam);
        int height = HIWORD(lParam);

        if (height == 0)
            height = 1;

        // Viewport anpassen
        glViewport(0, 0, width, height);

        // Projection neu berechnen
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        gluPerspective(
            60.0,
            (float)width / (float)height,
            0.1,
            500.0
        );

        glMatrixMode(GL_MODELVIEW);
        
        return 0;
    }
    /**/

    }

    return DefWindowProc(hWnd, msg, wParam, lParam);
}

// --- Thread ---
void viewerThread()
{
    HINSTANCE hInstance = GetModuleHandle(NULL);

    WNDCLASS wc = {};
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInstance;
    wc.lpszClassName = "LidarViewer";
    RegisterClass(&wc);

    HWND hwnd = CreateWindow("LidarViewer", "LiDAR Viewer",
        WS_OVERLAPPEDWINDOW | WS_VISIBLE,
        100, 100, 1024, 768,
        nullptr, nullptr, hInstance, nullptr);

    HDC hdc = GetDC(hwnd);

    PIXELFORMATDESCRIPTOR pfd = {};
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;

    int pf = ChoosePixelFormat(hdc, &pfd);
    SetPixelFormat(hdc, pf, &pfd);

    HGLRC glrc = wglCreateContext(hdc);
    wglMakeCurrent(hdc, glrc);

    glClearColor(0.05f, 0.05f, 0.1f, 1.0f);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGuiIO& io = ImGui::GetIO();

    ImGui::StyleColorsDark();

    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplOpenGL2_Init();


    // Für tiefe
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(
        60.0,           // Sichtfeld (Grad)
        1024.0 / 768.0, // Aspect Ratio
        0.1,            // near plane
        500.0           // far plane (WICHTIG!)
    );

    glMatrixMode(GL_MODELVIEW);
    // Ende für tiefe

    glEnable(GL_DEPTH_TEST);
    glPointSize(2.0f);

    MSG msg;


    while (true)
    {
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE))
        {
            if (msg.message == WM_QUIT)
                return;

            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

        // ----------------------------
        // 1. Neues ImGui Frame starten
        // ----------------------------
        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        // ----------------------------
        // 2. UI definieren
        // ----------------------------
        ImGui::Begin("LiDAR Controls");

        ImGui::Text("Points: %d", (int)g_points.size());
        ImGui::SliderFloat("Zoom", &zoom, 0.5f, 50.0f);
        ImGui::SliderFloat("RotX", &rotX, -90.0f, 90.0f);
        ImGui::SliderFloat("RotY", &rotY, -180.0f, 180.0f);

        ImGui::Text("Pan: %.2f %.2f", panX, panY);

        ImGui::End();

        // ----------------------------
        // 3. 3D Szene rendern
        // ----------------------------
        render();

        // ----------------------------
        // 4. UI rendern (IMMER zuletzt!)
        // ----------------------------
        ImGui::Render();
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

        // ----------------------------
        // 5. Display aktualisieren
        // ----------------------------
        SwapBuffers(hdc);
    }

}

void startViewer()
{
    std::thread(viewerThread).detach();
}
