// minimal Win32 + WGL viewer for AVBD demo (no SDL2 / no ImGui)
// Build: link opengl32.lib; gdi32.lib; user32.lib

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "user32.lib")

#include <stdio.h>
#include <windows.h>
#include <stdint.h>
#include <cstdio>
#include <algorithm>    // std::max, std::min
#include <windowsx.h>   // GET_X_LPARAM, GET_Y_LPARAM, GET_WHEEL_DELTA_WPARAM
#include <cwchar>   // swprintf

#include "maths.h"
#include "solver.h"
#include "scenes.h"

// -------------------- globals --------------------
static const int WinWidth = 1280;
static const int WinHeight = 720;

static HINSTANCE hInst = nullptr;
static HWND      hWnd = nullptr;
static HDC       hDC = nullptr;
static HGLRC     hGLRC = nullptr;
static bool      gRunning = true;

static Solver* solver = new Solver();
static Joint* drag = nullptr;

static float   camZoom = 25.0f;
static float2  camPos = { 0, 5 };
static int     currScene = 6; // rope scene index가 4라면 그대로 사용
static bool    paused = false;

// 박스 만들기 옵션(원본과 유사)
static float2  boxSize = { 1, 1 };
static float2  boxVelocity = { 0, 0 };
static float   boxFriction = 0.5f;
static float   boxDensity = 1.0f;

// 입력 상태
static bool    mbLeft = false;
static bool    mbMiddle = false;
static POINT   prevMouse{ 0,0 };

// -------------------- tiny helpers --------------------
static float2 worldFromScreen(int sx, int sy)
{
    RECT rc; GetClientRect(hWnd, &rc);
    int w = rc.right - rc.left, h = rc.bottom - rc.top;
    float2 ss = { (float)sx, (float)(h - sy) };
    float2 center = { w * 0.5f, h * 0.5f };
    return camPos + (ss - center) / camZoom;
}

static void setupGL()
{
    RECT rc; GetClientRect(hWnd, &rc);
    int w = rc.right - rc.left, h = rc.bottom - rc.top;

    glViewport(0, 0, w, h);
    glClearColor(1, 1, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(camPos.x - w * 0.5f / camZoom, camPos.x + w * 0.5f / camZoom,
        camPos.y - h * 0.5f / camZoom, camPos.y + h * 0.5f / camZoom, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_LINE_SMOOTH);
    glLineWidth(2.0f);
    glPointSize(3.0f);
}

static void drawHUD()
{
    // 간단 텍스트는 생략(고정 파이프라인에 텍스트 유틸 없음).
    // 필요하면 wglUseFontBitmaps 등으로 글리프 만들면 됨.
}

// -------------------- Win32 boilerplate --------------------
static void createGLContext(HWND hwnd)
{
    hDC = GetDC(hwnd);

    PIXELFORMATDESCRIPTOR pfd = { sizeof(PIXELFORMATDESCRIPTOR) };
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 24;
    pfd.cDepthBits = 0;  // 2D라 생략
    pfd.cStencilBits = 0;
    pfd.iLayerType = PFD_MAIN_PLANE;

    int pf = ChoosePixelFormat(hDC, &pfd);
    SetPixelFormat(hDC, pf, &pfd);

    hGLRC = wglCreateContext(hDC);
    wglMakeCurrent(hDC, hGLRC);
}

static void destroyGL()
{
    if (hGLRC) { wglMakeCurrent(nullptr, nullptr); wglDeleteContext(hGLRC); hGLRC = nullptr; }
    if (hDC) { ReleaseDC(hWnd, hDC); hDC = nullptr; }
}

static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    switch (msg)
    {
    case WM_DESTROY:
        gRunning = false;
        PostQuitMessage(0);
        return 0;

    case WM_SIZE:
        // 창 크기 바뀌면 투영 갱신은 매 프레임 setupGL에서 처리
        return 0;

    case WM_RBUTTONDOWN:
    {
        POINT p; p.x = GET_X_LPARAM(lParam); p.y = GET_Y_LPARAM(lParam);
        prevMouse = p;

        float2 mouseW = worldFromScreen(p.x, p.y);

        new Rigid(solver, boxSize, boxDensity, boxFriction, float3{ mouseW.x, mouseW.y, 0.0f },
            float3{ boxVelocity.x, boxVelocity.y, 0.0f });
        return 0;
    }

    case WM_MOUSEWHEEL:
    {
        int delta = GET_WHEEL_DELTA_WPARAM(wParam);
        float zoomFactor = (delta > 0) ? 1.1f : 1.0f / 1.1f;

        // 마우스 위치 기준 줌인/아웃
        POINT p; GetCursorPos(&p); ScreenToClient(hwnd, &p);
        float2 before = worldFromScreen(p.x, p.y);
        camZoom *= zoomFactor;
        camZoom = std::max(1.0f, std::min(200.0f, camZoom));
        float2 after = worldFromScreen(p.x, p.y);
        camPos += (before - after);
        return 0;
    }

    case WM_MBUTTONDOWN: mbMiddle = true;  SetCapture(hwnd); prevMouse.x = GET_X_LPARAM(lParam); prevMouse.y = GET_Y_LPARAM(lParam); return 0;
    case WM_MBUTTONUP:   mbMiddle = false; ReleaseCapture(); return 0;

    case WM_LBUTTONDOWN:
    {
        mbLeft = true; SetCapture(hwnd);
        POINT p; p.x = GET_X_LPARAM(lParam); p.y = GET_Y_LPARAM(lParam);
        prevMouse = p;

        float2 mouseW = worldFromScreen(p.x, p.y);
        if (!drag)
        {
            float2 local;
            if (Rigid* b = solver->pick(mouseW, local))
            {
                // 마우스-바디 드래그 조인트 (원본과 동일한 강성 벡터 사용)
                drag = new Joint(solver, /*A=*/nullptr, /*B=*/b,
                    /*rA(world)*/ mouseW, /*rB(local)*/ local,
                    /*stiff*/ float3{ 1000.0f,1000.0f,0.0f });
            }
        }
        return 0;
    }
    case WM_LBUTTONUP:
        mbLeft = false; ReleaseCapture();
        if (drag) { delete drag; drag = nullptr; }
        return 0;

    case WM_MOUSEMOVE:
    {
        POINT p; p.x = GET_X_LPARAM(lParam); p.y = GET_Y_LPARAM(lParam);
        if (mbMiddle)
        {
            // 팬(중클릭 드래그)
            RECT rc; GetClientRect(hwnd, &rc);
            float dx = float(p.x - prevMouse.x);
            float dy = float(p.y - prevMouse.y);
            camPos -= float2{ dx, -dy } / camZoom;
        }
        if (mbLeft && drag)
        {
            float2 mouseW = worldFromScreen(p.x, p.y);
            drag->rA = mouseW; // 마우스 위치로 조인트 이동
        }
        prevMouse = p;
        return 0;
    }

    case WM_KEYDOWN:
        if (wParam == VK_ESCAPE) { PostMessage(hwnd, WM_CLOSE, 0, 0); }
        if (wParam == 'R') { scenes[currScene](solver); } // reset
        if (wParam == 'P') { paused = !paused; }
        if (wParam == '1' || wParam == '2' || wParam == '3' || wParam == '4' || wParam == '5' || wParam == '6' || wParam == '7' || wParam == '8' || wParam == '9')
        {
            int idx = int(wParam - '0') - 1;
            if (idx >= 0 && idx < sceneCount) { currScene = idx; scenes[currScene](solver); }
        }
        if (wParam == 'Q') camZoom /= 1.05f;
        if (wParam == 'E') camZoom *= 1.05f;
        if (wParam == 'W') camPos.y += 10.0f / camZoom;
        if (wParam == 'S') camPos.y -= 10.0f / camZoom;
        if (wParam == 'A') camPos.x -= 10.0f / camZoom;
        if (wParam == 'D') camPos.x += 10.0f / camZoom;
        if (wParam == 'B') // 우클릭 대체: 박스 생성
        {
            POINT sp; GetCursorPos(&sp); ScreenToClient(hwnd, &sp);
            float2 pos = worldFromScreen(sp.x, sp.y);
            new Rigid(solver, boxSize, boxDensity, boxFriction,
                float3{ pos.x,pos.y,0 }, float3{ boxVelocity.x,boxVelocity.y,0 });
        }
        return 0;
    }
    return DefWindowProc(hwnd, msg, wParam, lParam);
}

// -------------------- main loop --------------------
static void frame()
{
    setupGL();

    int bucketSize = 0;
    if (!paused)
    {
        bucketSize = solver->step();
    }
    solver->draw();

    // --- 타이틀에 디버그 숫자 표시 ---
    // 간단한 FPS 추정 + 주기적 업데이트(0.25s)
    static bool inited = false;
    static LARGE_INTEGER freq, tPrev;
    static double fps = 0.0, accTitle = 0.0;

    LARGE_INTEGER tNow;
    if (!inited) {
        QueryPerformanceFrequency(&freq);
        QueryPerformanceCounter(&tPrev);
        inited = true;
    }
    QueryPerformanceCounter(&tNow);
    double dt = double(tNow.QuadPart - tPrev.QuadPart) / double(freq.QuadPart);
    tPrev = tNow;

    double instFps = (dt > 0.0) ? (1.0 / dt) : 0.0;
    fps = 0.9 * fps + 0.1 * instFps;   // 부드럽게

    accTitle += dt;
    if (accTitle > 0.25) {
        wchar_t title[256];
        swprintf(title, 256, L"AVBD 2D  | BucketSize : %d , FPS %.1f , Zoom %.2f  Cam (%.1f, %.1f)",
            bucketSize, fps, camZoom, camPos.x, camPos.y);
        SetWindowTextW(hWnd, title);
        accTitle = 0.0;
    }
    // -----------------------------------

    SwapBuffers(hDC);
}


int APIENTRY WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, int)
{
    hInst = hInstance;

    // Win32 창 만들기
    WNDCLASS wc = {};
    wc.style = CS_OWNDC | CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInst;
    wc.lpszClassName = L"AVBD_MinWin";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    RegisterClass(&wc);

    RECT wr{ 0,0,WinWidth,WinHeight };
    AdjustWindowRect(&wr, WS_OVERLAPPEDWINDOW, FALSE);
    hWnd = CreateWindow(wc.lpszClassName, L"AVBD 2D (Win32+WGL minimal)",
        WS_OVERLAPPEDWINDOW | WS_VISIBLE,
        CW_USEDEFAULT, CW_USEDEFAULT,
        wr.right - wr.left, wr.bottom - wr.top,
        nullptr, nullptr, hInst, nullptr);

    createGLContext(hWnd);

    // 초기 파라미터는 solver 기본값 그대로 사용
    solver->defaultParams();
    // 원하는 scene 로드
    scenes[currScene](solver);

    // 메시지루프 + 프레임 타이밍(고정 dt로 충분하면 단순 루프)
    MSG msg;
    LARGE_INTEGER freq, t0, t1; QueryPerformanceFrequency(&freq); QueryPerformanceCounter(&t0);
    double acc = 0.0;

    while (gRunning)
    {
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE))
        {
            if (msg.message == WM_QUIT) gRunning = false;
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

        // 고정 스텝(원 코드가 solver->dt를 쓰므로 그에 맞춰 진행)
        QueryPerformanceCounter(&t1);
        double dt = double(t1.QuadPart - t0.QuadPart) / double(freq.QuadPart);
        t0 = t1;
        acc += dt;

        // 너무 빠르게 돌면 60FPS로 억제(임시)
        if (acc < 1.0 / 120.0) { Sleep(1); continue; }

        frame();
        acc = 0.0;
    }

    destroyGL();
    return 0;
}
