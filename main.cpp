/*
 * main.cpp – minimal example rendering several 3D cuboids using AVBD Rigid structs
 */

#include "render/engine.h"
#include "solver.h"
#include "util/random.h"
#include <vector>
#include <cstdlib>
#include <chrono>

const bool STACK = true;
const bool JOINT = true;
const bool JOINT2 = false;

int main() {
    Solver solver;
    solver.gravity = vec3(0, -9.8f, 0);
    solver.iterations = 10;

    // create ground plane (large flat box)
    new Rigid(&solver, {15, 0.25f, 15}, -1.0f, 0.5f, {0, -1.0f, 0});
      

        if (STACK) {
            for (int i = 0; i < 7; ++i) {
                new Rigid(&solver, vec3(0.5f), 10.0f, 0.4f, vec3(0.0f, i * 0.5f - 0.5f, 0.0f), quat(1, 0, 0, 0), vec6());
            }
        }

        if (JOINT) {
            // ====== 가로 막대 N개 체인 ======
            const int   N = 10;                 // <- 원하는 개수
            const float linkLen = 0.5f;              // X방향 길이
            const float thick = 0.12f;             // Y,Z 두께
            const vec3  size = vec3(linkLen, thick, thick);
            const float density = 10.0f;
            const float friction = 0.4f;

            // 힌지 강성 (발산하면 posK↓, swingK↓ 또는 iterations↑)
            const vec3  posK = vec3(3e5f);        // 위치 3행
            const float swingK = 5.0f;              // 스윙 2행(힌지 제약)
            const float frac = INFINITY;

            // 힌지 축/기준: Z축 힌지면 XY평면으로 흔들림
            const vec3 axisW(0, 0, 1);               // 회전축(힌지)
            const vec3 refW(1, 0, 0);               // 스윙 평면 기준축

            // 월드 피벗(천장 고정점)
            const vec3 pivotW = vec3(0.0f, 5.0f, 0.0f);

            // 로컬 앵커들: 오른쪽/왼쪽 끝
            const vec3 r_right(+0.5f * linkLen, 0, 0);
            const vec3 r_left(-0.5f * linkLen, 0, 0);

            // 링크 생성
            std::vector<Rigid*> links; links.reserve(N);
            for (int i = 0; i < N; ++i) {
                auto* L = new Rigid(&solver, size, density, friction, pivotW - vec3(0.5f* linkLen + i * linkLen,0,0), quat(1, 0, 0, 0), vec6());
                // 보기 좋은 컬러 그라데이션
                float t = (N > 1) ? float(i) / float(N - 1) : 0.0f;
                L->color = vec4(0.85f * (1 - t) + 0.30f * t, 0.45f * (1 - t) + 0.70f * t, 0.45f, 0.75f);
                links.push_back(L);
            }

            // 1) 첫 막대를 월드 피벗에 걸기: L0의 +X 끝이 pivotW에 오도록 배치
            //links[0]->position = pivotW - links[0]->rotation * r_right;
            if (true)
            {
                new Joint(&solver,
                    /*A*/ nullptr, /*B*/ links[0],
                    /*rA*/ pivotW, /*rB*/ r_right,
                    /*axisA*/ axisW, /*axisB*/ axisW,
                    /*refA*/  refW,  /*refB*/  refW,
                    /*posK*/  posK,  /*swingK*/ 0.1f,
                    /*frac*/  frac);
            }
            

            // 2) 나머지 막대들을 체인으로 연결: A(left) <-> B(right)
            for (int i = 1; i < N; ++i) {
                Rigid* A = links[i - 1];
                Rigid* B = links[i];

                // A의 왼쪽 끝 월드 위치(힌지 점)
                vec3 hingeW = A->position + A->rotation * r_left;

                // B의 오른쪽 끝이 그 힌지 점에 오도록 배치
                //B->position = hingeW - B->rotation * r_right;

                // 힌지 조인트 생성
               
                if (true)
                {
                    new Joint(&solver,
                        /*A*/ A, /*B*/ B,
                        /*rA*/ r_left, /*rB*/ r_right,
                        /*axisA*/ axisW, /*axisB*/ axisW,
                        /*refA*/  refW,  /*refB*/  refW,
                        /*posK*/  posK,  /*swingK*/ swingK,
                        /*frac*/  frac);
                }
                
                    
            }

            // 초기 속도 0
            for (auto* L : links) L->velocity = vec6(0);

            // (선택) 더 단단하게 조이려면 반복 수 늘리기
            solver.iterations = 10;  // 30~60 권장
        }



    // create engine and clock
    Engine engine(800, 600, "AVBD Cuboids", "shaders/vertex.glsl", "shaders/fragment.glsl", solver.bodies, solver.forces);
    std::chrono::steady_clock::time_point lastFrameTime = std::chrono::steady_clock::now();

    static float elapsedTime = 0.0f;

    // main loop
    while (!engine.shouldClose()) {
        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<float> dt = currentTime - lastFrameTime;

        elapsedTime += dt.count();

        if (elapsedTime > 1.0f / 60.0f)
        {
            while (elapsedTime > 1.0f / 60.0f)
            {
                elapsedTime -= 1.0f / 60.0f;
                solver.step(1.0f / 60.0f);
            }            
        }
        
        engine.render();
        engine.update();

        lastFrameTime = currentTime;
    }
}
