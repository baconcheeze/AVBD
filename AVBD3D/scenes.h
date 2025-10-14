// scenes.h

#pragma once

#include "maths.h"
#include "solver.h"


static void sceneBox(Solver* solver)
{
    solver->clear();  
    Rigid* box0 = new Rigid(solver, { 1.0f, 0.2f , 0.2f }, 1.0f , 0.5f, float3{ 0.0f, 10.0f, 0.0f } , float3{ 0.0f, 0.0f, 0.0f }, float3{ 0.0f, 0.0f, 0.0f }, float3{ 0.0f, 0.0f, 0.0f });
    //Rigid* box1 = new Rigid(solver, { 1.0f, 0.2f , 0.2f }, 1.0f, 0.5f, float3{ 1.0f, 10.0f, 0.0f }, float3{ 0.0f, 0.0f, 0.0f }, float3{ 0.0f, 0.0f, 0.0f }, float3{ 0.0f, 0.0f, 0.0f });
}

// scenes.h (또는 당신의 scene 구현 파일)

static void sceneRope3D(Solver* s)
{
    s->clear();
    s->gravity = -10.0f;

    s->iterations = 50;
    s->gamma = 0.95f;
    s->beta = 200.0f;

    const int   N = 5;                        // 링크 개수 
    const float3 size{ 0.5f, 0.12f, 0.12f };   // 가늘고 긴 박스
    const float  hx = size.x * 0.5f;

    // 강성 설정
    const float3 posK = float3{ INFINITY, INFINITY, INFINITY }; // 위치 3행: 핀처럼 딱 맞추기
    const float  swingK = 1e6f;   // 스윙(축과 직교 성분 2행) — 축 정렬 강성
    const float  twistK = 0.0f;   // 트위스트 자유(힌지 1DOF), 잠그고 싶으면 1e6f 같은 큰 값
    const float  restTwist = 0.0f;

    //ground 추가

    if (true)
    {
        Rigid* ground = new Rigid(
            s,
            /*size*/     float3{ 1.0f,0.2f,1.0f },
            /*density*/  0.0f,
            /*friction*/ 0.5f,
            /*position*/ float3{ 0,8.0f,0 },
            /*rotation*/ float3{ 0,0,0 }
        );
    }
    

    Rigid* prev = nullptr;

    for (int i = 0; i < N; ++i)
    {
        const float3 pos{ 0.5f+i * size.x, 10.0f, 0.0f };
        const float  density = 1.0f;

        Rigid* curr = new Rigid(
            s,
            /*size*/     size,
            /*density*/  density,
            /*friction*/ 0.5f,
            /*position*/ pos,
            /*rotation*/ float3{ 0,0,0 }
            , float3{0,0,0}
            ,float3{0,0,0}
        );


        const bool attachOnAir = true;
        if (!prev && attachOnAir)
        {
            // 첫 조각: 월드 고정점에 힌지(위치3 + 스윙2 + (옵션)트위스트1)
            const float3 anchorW{ pos.x - hx, pos.y, pos.z };
            
            if (false)
            {
                new Joint(s,
                    nullptr, curr,
                    anchorW, float3{ -hx, 0, 0 },
                    posK,
                    10);
            }

            else
            {
                new Joint(
                    s,
                    /*A=*/ nullptr,
                    /*B=*/ curr,
                    /*rA(world)*/ anchorW,
                    /*rB(local)*/ float3{ -hx, 0, 0 },
                    /*axisA(world)*/ float3{ 0,0,1 },   // 힌지 축(Z)
                    /*axisB(local)*/ float3{ 0,0,1 },
                    /*refA(world)*/  float3{ 0,1,0 },   // 축과 직교 기준
                    /*refB(local)*/  float3{ 0,1,0 },
                    /*posK*/   posK,
                    /*swingK*/ swingK,
                    /*twistK*/ twistK,                  // 0이면 자유, 큰 값이면 고정
                    /*restTwistRadians*/ restTwist,
                    /*fracture*/ INFINITY
                );
            }
            
        }
        else if(prev != nullptr)
        {
            if (false)
            {
                new Joint(s,
                    prev, curr,
                    float3{ +hx, 0, 0 }, float3{ -hx, 0, 0 },
                    posK,
                    10);
            }
            else
            {
                // 인접 조각끼리 힌지 연결
                new Joint(
                    s,
                    /*A=*/ prev,
                    /*B=*/ curr,
                    /*rA_local*/ float3{ +hx, 0, 0 },
                    /*rB_local*/ float3{ -hx, 0, 0 },
                    /*axisA_local*/ float3{ 0,0,1 },   // 두 바디 로컬에서 같은 힌지축(Z)
                    /*axisB_local*/ float3{ 0,0,1 },
                    /*refA_local*/  float3{ 0,1,0 },   // 축과 직교 기준(예: X)
                    /*refB_local*/  float3{ 0,1,0 },
                    /*posK*/   posK,
                    /*swingK*/ swingK,
                    /*twistK*/ twistK,
                    /*restTwistRadians*/ restTwist,
                    /*fracture*/ INFINITY
                );
            }
        }
       
        prev = curr;
    }

   
}

static void sceneBox3D(Solver* s)
{
    s->clear();
    s->gravity = -10.0f;   
    Rigid* box1 = new Rigid(
            s,
            /*size*/     float3{ 1.0f, 1.0f, 1.0f },
            /*density*/  1.0f,
            /*friction*/ 0.5f,
            /*position*/ float3{ 0.9f,10.0f,0.0f },
            /*rotation*/ float3{ 0,0,0 }
    );

    Rigid* box2 = new Rigid(
        s,
        /*size*/     float3{ 1.0f,1.0f,1.0f },
        /*density*/  0.0f,
        /*friction*/ 0.5f,
        /*position*/ float3{ 0,8.0f,0 },
        /*rotation*/ float3{ 0,0,0 }
    );

    Rigid* box3 = new Rigid(
        s,
        /*size*/     float3{ 1.0f,1.0f,1.0f },
        /*density*/  1.0f,
        /*friction*/ 0.5f,
        /*position*/ float3{ 0.5f,12.0f,0.5f },
        /*rotation*/ float3{ 0,0,0 }
    );
}

static void scenePyramid3D(Solver* s)
{
    s->clear();
    s->gravity = -10.0f;

    // (선택) 피라미드가 잘 버티도록 반복 수 약간 올리고 감쇠 보정
    s->iterations = 15;
    s->gamma      = 0.99f;
    s->beta       = 1500.0f;

    // --- Ground (static box) ---
    {
        const float3 size = { 20.0f, 1.0f, 20.0f };    // 넓은 바닥
        const float3 pos = { 0.0f, -0.5f, 0.0f };     // 윗면이 y=0에 오도록
        const float3 rot = { 0.0f, 0.0f, 0.0f };
        const float  dens = 0.0f;                      // 0 => 정적
        const float  fric = 0.7f;

        new Rigid(s, size, dens, fric, pos, rot);
    }

    // --- Bricks (dynamic) ---
    const float3 brickSize = { 0.6f, 0.2f, 0.35f };      // 가로 x, 높이 y, 깊이 z
    const float  dens = 1.0f;
    const float  fric = 0.6f;

    const int baseCount = 6;                             // 6층 → 21개 (대충 20개)
    const float gapX = 0.005f;                           // 옆 벽돌과 아주 얇은 간격
    const float gapY = 0.005f;                           // 층간 아주 얇은 간격

    const float y0 = brickSize.y * 0.5f + 0.001f;        // 바닥에서 살짝 띄워 관통 방지

    for (int layer = 0; layer < baseCount; ++layer)
    {
        const int count = baseCount - layer;             // 이 층에서의 벽돌 개수
        const float y = y0 + layer * (brickSize.y + gapY);

        // 가운데 정렬: 첫 벽돌의 x 시작점
        const float span = (brickSize.x + gapX) * (count - 1);
        float xStart = -0.5f * span;

        for (int i = 0; i < count; ++i)
        {
            const float x = xStart + i * (brickSize.x + gapX);
            const float3 pos = { x, y, 0.0f };
            const float3 rot = { 0.0f, 0.0f, 0.0f };     // 필요하면 아주 약간의 랜덤 회전도 가능

            new Rigid(s, brickSize, dens, fric, pos, rot);
        }
    }
}


static void (*scenes[])(Solver*) =
{
    sceneBox3D,
    sceneRope3D,
    scenePyramid3D,
};

static const char* sceneNames[] = {
    "Box3D",
    "Rope3D",   
    "Pyramid3D",
};

static const int sceneCount = 1;
#pragma once
