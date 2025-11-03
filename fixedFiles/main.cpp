/*
 * main.cpp – minimal example rendering several 3D cuboids using AVBD Rigid structs
 */

#include "render/engine.h"
#include "solver.h"
#include "scenes.h"
#include "util/random.h"
#include <vector>
#include <cstdlib>
#include <chrono>
#include <cstdio>
#include <GLFW/glfw3.h> // GLFW 키 입력 폴링용

static void (*scenes[])(Solver*) =
{
    sceneEmpty,
    sceneBox,
    sceneBoxStack,
    sceneRope,
    scenePyramid,
    sceneRopeAndPyramid,
    sceneNet,
    sceneNetAndBoxes
};

static int currScene = 7;

static std::vector<int> Keys = {
    GLFW_KEY_0,
    GLFW_KEY_1,
    GLFW_KEY_2,
    GLFW_KEY_3,
    GLFW_KEY_4,
    GLFW_KEY_5,
    GLFW_KEY_6,    
    GLFW_KEY_7,
    GLFW_KEY_R
};

static void processInput(GLFWwindow* window, Solver& solver)
{
    static std::vector<bool> KeyPressed(Keys.size(), false);

    const int count = Keys.size();
    for (int i = 0; i < count; ++i) {
        int key = Keys[i];
        bool pressed = (glfwGetKey(window, key) == GLFW_PRESS);
        if (pressed && KeyPressed[i] == false) {
            // 숫자키로 씬 전환 — Keys 벡터 순서에 맞추어 i가 씬 인덱스가 됨
            if (i < count-1 ) { 
                solver.clear();
                solver.useCollision = true;
                sceneGround(&solver); // 바닥은 항상 추가
                currScene = i;
                scenes[currScene](&solver);
            }
            // 리셋 처리 (R)
            else if (key == GLFW_KEY_R) {
                solver.clear();
                sceneGround(&solver); // 바닥은 항상 추가
                scenes[currScene](&solver); // 현재 씬 재생성(리셋)
            }
        }
        KeyPressed[i] = pressed;
    }
}

int main() {
    Solver solver;
    solver.gravity = vec3(0, -9.8f, 0);
    solver.iterations = 10;

    // 초기 씬 설정
    solver.clear();
    sceneGround(&solver); // 바닥은 항상 추가
    scenes[currScene](&solver);

    // create engine and clock
    Engine* engine = new Engine(1920, 1080, "AVBD Cuboids", "shaders/vertex.glsl", "shaders/fragment.glsl", solver.bodies, solver.forces);
    std::chrono::steady_clock::time_point lastFrameTime = std::chrono::steady_clock::now();

    // set Camera Position
    engine->getCamera().position = vec3(0.0f, 4.0f, 8.0f);

    static float elapsedTime = 0.0f;
    static const float fixedDeltaTime = 1.0f / 60.0f;
        
    static StepResult stepResult;

    // main loop
    while (!engine->shouldClose()) {
        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<float> dt = currentTime - lastFrameTime;

        elapsedTime += dt.count();

        // 물리 고정 스텝
        while (elapsedTime > fixedDeltaTime)
        {
            elapsedTime -= fixedDeltaTime;
            stepResult = solver.step(fixedDeltaTime);
            engine->stepResult = stepResult;
        }        

        // 키 입력 폴링
        GLFWwindow* window = engine->getWindow(); // 만약 private이면 아래에 제안된 getWindow() 사용
        processInput(window, solver);
       
        {
            engine->setBodiesAndForces(solver.bodies, solver.forces);
            engine->render();
            engine->update();
        }        

        lastFrameTime = currentTime;
    }
}