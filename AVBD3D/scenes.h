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

    const int   N = 5;                        // 링크 개수 
    const float3 size{ 0.5f, 0.12f, 0.12f };   // 가늘고 긴 박스
    const float  hx = size.x * 0.5f;

    // 강성 설정
    const float3 posK = float3{ INFINITY, INFINITY, INFINITY }; // 위치 3행: 핀처럼 딱 맞추기
    const float  swingK = 1e6f;   // 스윙(축과 직교 성분 2행) — 축 정렬 강성
    const float  twistK = 0.0f;   // 트위스트 자유(힌지 1DOF), 잠그고 싶으면 1e6f 같은 큰 값
    const float  restTwist = 0.0f;

    Rigid* prev = nullptr;

    for (int i = 0; i < N; ++i)
    {
        const float3 pos{ i * size.x, 10.0f, 0.0f };
        const float  density = 1.0f;

        Rigid* curr = new Rigid(
            s,
            /*size*/     size,
            /*density*/  density,
            /*friction*/ 0.5f,
            /*position*/ pos,
            /*rotation*/ float3{ 0,0,0 }
        );

        if (!prev)
        {
            // 첫 조각: 월드 고정점에 힌지(위치3 + 스윙2 + (옵션)트위스트1)
            const float3 anchorW{ pos.x - hx, pos.y, pos.z };
            
            new Joint(
                s,
                /*A=*/ nullptr,
                /*B=*/ curr,
                /*rA(world)*/ anchorW,
                /*rB(local)*/ float3{ -hx, 0, 0 },
                /*axisA(world)*/ float3{ 0,0,1 },   // 힌지 축(Z)
                /*axisB(local)*/ float3{ 0,0,1 },
                /*refA(world)*/  float3{ 1,0,0 },   // 축과 직교 기준
                /*refB(local)*/  float3{ 1,0,0 },
                /*posK*/   posK,
                /*swingK*/ swingK,
                /*twistK*/ twistK,                  // 0이면 자유, 큰 값이면 고정
                /*restTwistRadians*/ restTwist,
                /*fracture*/ INFINITY
            ); 
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
                /*refA_local*/  float3{ 1,0,0 },   // 축과 직교 기준(예: X)
                /*refB_local*/  float3{ 1,0,0 },
                /*posK*/   posK,
                /*swingK*/ swingK,
                /*twistK*/ twistK,
                /*restTwistRadians*/ restTwist,
                /*fracture*/ INFINITY
            );
        }

        prev = curr;
    }

    //ground 추가

    Rigid* ground = new Rigid(
        s,
        /*size*/     float3{ 1.0f,1.0f,1.0f },
        /*density*/  0.0f,
        /*friction*/ 0.5f,
        /*position*/ float3{ 0,8.0f,0 },
        /*rotation*/ float3{ 0,0,0 }
    );
}

static void sceneBox3D(Solver* s)
{
    s->clear();
    s->gravity = -10.0f;   
    Rigid* box = new Rigid(
            s,
            /*size*/     float3{ 0.5f, 0.12f, 0.12f },
            /*density*/  1.0f,
            /*friction*/ 0.5f,
            /*position*/ float3{ 0.0f,10.0f,0.0f },
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
}

static void (*scenes[])(Solver*) =
{
    sceneBox3D,
    sceneRope3D,
};

static const char* sceneNames[] = {
    "Box3D",
    "Rope3D",    
};

static const int sceneCount = 1;
#pragma once
