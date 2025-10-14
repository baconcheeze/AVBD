// main.cpp
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
#include <cwchar>       // swprintf

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

// 카메라
static float   camZoom = 25.0f;             // 커질수록 더 "줌 인"되도록 FOV 조절에 사용
static float3  camPos = { -7.3f, 10.1f, 7.9f }; // +Z에서 -Z로 바라보는 OpenGL 규약 기준
static int     currScene = 1;               
static bool    paused = false;
// globals
static bool gCullCW = true;

// FPS식 회전(카메라 고정, 방향만 회전)
static float   camYawDeg = 140.7f;   // Yaw  (좌우)
static float   camPitchDeg = -9.3f;   // Pitch(위아래)
static bool    mbRight = false;   // 우클릭 드래그로 회전

// 입력 상태(팬)
static bool  mbMiddle = false;
static POINT prevMouse{ 0,0 };

// -------------------- helpers --------------------
static inline float rad(float d) { return d * 3.1415926535f / 180.0f; }

static inline float3 normalize3(float3 v) {
    float L = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (L <= 1e-12f) return { 0,0,0 };
    return { v.x / L, v.y / L, v.z / L };
}

static inline float3 cross3(float3 a, float3 b) {
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

// yaw/pitch → forward/right/up
static void cameraAxes(float3& fwd, float3& right, float3& up) {
    const float cy = std::cos(rad(camYawDeg)), sy = std::sin(rad(camYawDeg));
    const float cp = std::cos(rad(camPitchDeg)), sp = std::sin(rad(camPitchDeg));

    // OpenGL 기준(-Z가 기본 시선)이지만 우린 lookAt으로 정리하니 일반적 정의 사용
    fwd = normalize3({ cp * sy, sp, cp * cy });            // 전방
    const float3 worldUp = { 0,1,0 };
    right = normalize3(cross3(fwd, worldUp));          // 오른쪽
    up = normalize3(cross3(right, fwd));            // 위쪽(정규직교)
}

// LookAt(view) 로드 (gluLookAt 없이)
static void loadLookAt(const float3& eye, const float3& target, const float3& upW)
{
    float3 f = normalize3({ target.x - eye.x, target.y - eye.y, target.z - eye.z });
    float3 s = normalize3(cross3(f, upW));
    float3 u = cross3(s, f);

    // rotation만 column-major로 적재
    float M[16] = {
        s.x, u.x, -f.x, 0.0f,
        s.y, u.y, -f.y, 0.0f,
        s.z, u.z, -f.z, 0.0f,
        0.0f,0.0f, 0.0f, 1.0f
    };
    glMultMatrixf(M);                     // 회전
    glTranslatef(-eye.x, -eye.y, -eye.z); // 이동(역변환)
}






// 간단한 투영행렬(퍼스펙티브) 설정 + 기본 렌더 스테이트
static void setupGL()
{
    RECT rc; GetClientRect(hWnd, &rc);
    int w = rc.right - rc.left;
    int h = rc.bottom - rc.top;
    if (w <= 0) w = 1;
    if (h <= 0) h = 1;

    glViewport(0, 0, w, h);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    //glDisable(GL_CULL_FACE);
    // ✅ 백페이스 컬링 켜기
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);     // 뒤쪽 면 버림
    glFrontFace(GL_CCW);     // 정면은 CCW(반시계)로 간주 (모델이 CW면 GL_CW로 바꿔요)


    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glLineWidth(2.0f);
    glPointSize(3.0f);

    glClearColor(1, 1, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // ---- Projection (Perspective) ----
    const float nearZ = 0.1f, farZ = 1000.0f;
    const float baseFovDeg = 45.0f;
    float fovDeg = std::clamp(baseFovDeg * (25.0f / camZoom), 10.0f, 90.0f);
    float aspect = (float)w / (float)h;
    float fovRad = rad(fovDeg);
    float top = nearZ * std::tan(fovRad * 0.5f);
    float right = top * aspect;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-right, right, -top, top, nearZ, farZ);

    // ----- ModelView (Camera) -----
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    float3 fwd, rgt, up;
    cameraAxes(fwd, rgt, up);
    float3 target = { camPos.x + fwd.x, camPos.y + fwd.y, camPos.z + fwd.z };
    loadLookAt(camPos, target, { 0,1,0 }); // up은 worldUp 써도 OK

    //glFrontFace(gCullCW ? GL_CW : GL_CCW);
}


static void drawGrid(float y = 0.0f, int half = 20, float step = 1.0f)
{
    glColor4f(0.7f, 0.7f, 0.7f, 0.25f);
    glBegin(GL_LINES);
    for (int i = -half; i <= half; ++i) {
        // X 방향 라인
        glVertex3f(-half * step, y, i * step);
        glVertex3f(+half * step, y, i * step);
        // Z 방향 라인
        glVertex3f(i * step, y, -half * step);
        glVertex3f(i * step, y, +half * step);
    }
    glEnd();

    // 축 표시
    glLineWidth(3.0f);
    // X-red
    glColor3f(0.9f, 0.2f, 0.2f); glBegin(GL_LINES);
    glVertex3f(0, y, 0); glVertex3f(2, y, 0); glEnd();
    // Y-green
    glColor3f(0.2f, 0.8f, 0.2f); glBegin(GL_LINES);
    glVertex3f(0, y, 0); glVertex3f(0, y + 2, 0); glEnd();
    // Z-blue
    glColor3f(0.2f, 0.3f, 0.9f); glBegin(GL_LINES);
    glVertex3f(0, y, 0); glVertex3f(0, y, 2); glEnd();
    glLineWidth(2.0f);
}

// WGL 컨텍스트 생성
static void createGLContext(HWND hwnd)
{
    hDC = GetDC(hwnd);

    PIXELFORMATDESCRIPTOR pfd = { sizeof(PIXELFORMATDESCRIPTOR) };
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 24;
    pfd.cDepthBits = 24;   // 3D 깊이 테스트용
    pfd.cStencilBits = 8;
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

// 화면 좌표 이동량을 카메라 공간으로 대충 맵핑(팬 속도)
static void panByPixels(int dx, int dy)
{
    float scale = (25.0f / camZoom) * 0.02f;

    float3 fwd, right, up;
    cameraAxes(fwd, right, up);

    // 화면에서 +X는 오른쪽, +Y는 아래 → world에선 Up은 반대로(+Y 위)
    camPos.x += (-dx) * scale * right.x + (dy)*scale * up.x;
    camPos.y += (-dx) * scale * right.y + (dy)*scale * up.y;
    camPos.z += (-dx) * scale * right.z + (dy)*scale * up.z;
}



// -------------------- WndProc --------------------
static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    switch (msg)
    {
    case WM_DESTROY:
        gRunning = false;
        PostQuitMessage(0);
        return 0;

    case WM_SIZE:
        // 리사이즈는 다음 frame의 setupGL에서 처리
        return 0;

    case WM_MOUSEWHEEL:
    {
        const int delta = GET_WHEEL_DELTA_WPARAM(wParam);
        float factor = (delta > 0) ? 1.1f : (1.0f / 1.1f);
        camZoom = std::clamp(camZoom * factor, 5.0f, 120.0f);
        return 0;
    }

    case WM_MBUTTONDOWN:
        mbMiddle = true;
        SetCapture(hwnd);
        prevMouse.x = GET_X_LPARAM(lParam);
        prevMouse.y = GET_Y_LPARAM(lParam);
        return 0;

    case WM_MBUTTONUP:
        mbMiddle = false;
        ReleaseCapture();
        return 0;

    case WM_MOUSEMOVE:
    {
        int mx = GET_X_LPARAM(lParam);
        int my = GET_Y_LPARAM(lParam);
        int dx = mx - prevMouse.x;
        int dy = my - prevMouse.y;

        if (mbMiddle) {
            panByPixels(dx, dy);
        }
        if (mbRight) {
            camYawDeg -= dx * 0.1f;        // 좌우 회전
            camPitchDeg -= dy * 0.1f;       // 위아래(마우스 올리면 pitch 증가)
            camPitchDeg = std::clamp(camPitchDeg, -89.0f, 89.0f);
        }

        prevMouse.x = mx;
        prevMouse.y = my;
        return 0;
    }

    case WM_KEYDOWN:
    {
        if (wParam == VK_ESCAPE) { PostMessage(hwnd, WM_CLOSE, 0, 0); return 0; }
        if (wParam == 'P') { paused = !paused; return 0; }
        if (wParam == 'R') { scenes[currScene](solver); return 0; }

        // FOV 줌
        if (wParam == 'Q') camZoom = std::clamp(camZoom / 1.05f, 5.0f, 120.0f);
        if (wParam == 'E') camZoom = std::clamp(camZoom * 1.05f, 5.0f, 120.0f);

        // 선택: WASD 로컬 이동
        float3 fwd, right, up;
        cameraAxes(fwd, right, up);
        float move = 0.3f * (25.0f / camZoom);  // FOV에 따라 보폭 스케일
        if (wParam == 'F') { gCullCW = !gCullCW; return 0; }
        if (wParam == 'W') { camPos.x += fwd.x * move; camPos.y += fwd.y * move; camPos.z += fwd.z * move; }
        if (wParam == 'S') { camPos.x -= fwd.x * move; camPos.y -= fwd.y * move; camPos.z -= fwd.z * move; }
        if (wParam == 'A') { camPos.x -= right.x * move; camPos.y -= right.y * move; camPos.z -= right.z * move; }
        if (wParam == 'D') { camPos.x += right.x * move; camPos.y += right.y * move; camPos.z += right.z * move; }
        if (wParam == VK_SPACE) { camPos.x += up.x * move; camPos.y += up.y * move; camPos.z += up.z * move; }
        if (wParam == VK_CONTROL) { camPos.x -= up.x * move; camPos.y -= up.y * move; camPos.z -= up.z * move; }
        return 0;
    }

    case WM_RBUTTONDOWN:
        mbRight = true;
        SetCapture(hwnd);
        prevMouse.x = GET_X_LPARAM(lParam);
        prevMouse.y = GET_Y_LPARAM(lParam);
        return 0;

    case WM_RBUTTONUP:
        mbRight = false;
        ReleaseCapture();
        return 0;
    }
    return DefWindowProc(hwnd, msg, wParam, lParam);
}

// -------------------- main loop --------------------
static void frame()
{
    setupGL();
    drawGrid();

    SolverProcessOutput solverProcessOutput;   
    if (!paused)
        solverProcessOutput = solver->step();
    solver->draw();

    // --- 타이틀에 디버그 숫자 표시 ---
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
    fps = 0.9 * fps + 0.1 * instFps;

    accTitle += dt;
    if (accTitle > 0.25) {
        wchar_t title[256];
        swprintf(title, 256, L"bucketSize : %d | JointCount : %d | ManifoldCount : %d | FPS: %.1f | Zoom: %.2f | Pos(%.1f,%.1f,%.1f) | Yaw %.1f Pitch %.1f",
            solverProcessOutput.bucketSize, solverProcessOutput.jointCount, solverProcessOutput.manifoldCount,fps, camZoom, camPos.x, camPos.y, camPos.z, camYawDeg, camPitchDeg);
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
    wc.lpfnWndProc = WndProc;                // ← WndProc 지정
    wc.hInstance = hInst;
    wc.lpszClassName = L"AVBD_MinWin";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    RegisterClass(&wc);

    RECT wr{ 0,0,WinWidth,WinHeight };
    AdjustWindowRect(&wr, WS_OVERLAPPEDWINDOW, FALSE);
    hWnd = CreateWindow(wc.lpszClassName, L"AVBD 3D (Win32+WGL minimal)",
        WS_OVERLAPPEDWINDOW | WS_VISIBLE,
        CW_USEDEFAULT, CW_USEDEFAULT,
        wr.right - wr.left, wr.bottom - wr.top,
        nullptr, nullptr, hInst, nullptr);

    createGLContext(hWnd);

    // 초기 파라미터는 solver 기본값 그대로 사용
    solver->defaultParams();
    // 원하는 scene 로드 (지금은 0번 하나)
    scenes[currScene](solver);

    // 메시지루프 + 프레임 타이밍
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

        QueryPerformanceCounter(&t1);
        double dt = double(t1.QuadPart - t0.QuadPart) / double(freq.QuadPart);
        t0 = t1;
        acc += dt;

        // 너무 빠르게 돌면 120FPS로 억제(임시)
        if (acc < 1.0 / 120.0) { Sleep(1); continue; }

        frame();
        acc = 0.0;
    }

    destroyGL();
    return 0;
}
