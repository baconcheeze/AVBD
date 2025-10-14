// solver.h

#pragma once
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

// OpenMP는 선택사항
//#include <omp.h>
#include <vector>
#include <unordered_map>
#include <algorithm>

#ifdef TARGET_OS_MAC
#include <OpenGL/GL.h>
#else
#include <GL/gl.h>
#endif

#include "maths.h"

#define MAX_CONTACTS 4
#define MAX_ROWS 12
#define PENALTY_MIN 1.0f
#define PENALTY_MAX 1000000000.0f
#define COLLISION_MARGIN 0.0005f
#define STICK_THRESH 0.01f
#define SHOW_CONTACTS true

struct Rigid;
struct Force;
struct Manifold;
struct Solver;

struct Rigid {
    Solver* solver{};
    Rigid* next{};   // ← 한 번만
    Force* forces{};

    float radius;
    float3 size;
    float friction = 0;    

    // 상태
    float3 x{};   // 위치
    quat   q{};   // 자세
    float3 v{};   // 선속
    float3 w{};   // 각속

    float3 v_prev{};

    float  mass{ 1.0f };
    float3 Ibody{ 1,1,1 };
    float3x3 Iworld{}; // 필요 시 갱신

    // warmstart/solver용
    float3 x_inertial{}, x_initial{};
    quat   q_inertial{}, q_initial{ 1,0,0,0 };

    float3x3 R() const { return q.R(); }

    // p_world = x + R * r_local
    float3 worldPoint(float3 rL) const {
        float3 rW = R() * rL;
        return x + rW;
    }
    // d(x + R r)/dθ ≈ rW  (소각에서 J의 각속도 항에 cross(rW,dir)로 사용)
    float3 dWorldPoint_dtheta(float3 rL) const {
        return R() * rL;
    }

    Rigid(Solver* solver, float3 size, float density, float friction, float3 position, float3 rotation, float3 linearVelocity = float3{ 0, 0, 0 }, float3 angularVelocity = float3{0,0,0});
    ~Rigid();

    void draw();

    bool constrainedTo(Rigid* other) const;
};

// A에 대한 행: [ dir,  rAw × dir ]
inline float6 Jrow_A_pointDir(const Rigid& A, float3 rA_local, float3 dir_world) {
    float3 rAw = A.R() * rA_local;
    float3 jang = cross(rAw, dir_world);
    float6 J{};
    J.v[0] = dir_world.x; J.v[1] = dir_world.y; J.v[2] = dir_world.z;
    J.v[3] = jang.x;      J.v[4] = jang.y;      J.v[5] = jang.z;
    return J;
}

// B에 대한 행: [ -dir,  -(rBw × dir) ]
inline float6 Jrow_B_pointDir(const Rigid& B, float3 rB_local, float3 dir_world) {
    float3 rBw = B.R() * rB_local;
    float3 jang = cross(rBw, dir_world);
    float6 J{};
    J.v[0] = -dir_world.x; J.v[1] = -dir_world.y; J.v[2] = -dir_world.z;
    J.v[3] = -jang.x;      J.v[4] = -jang.y;      J.v[5] = -jang.z;
    return J;
}

struct Solver {
    float dt{ 1.0f / 60.0f };
    float gravity{ -10.0f };
    int   iterations{ 10 };

    float alpha{ 0.99f };
    float beta{ 100000.0f };
    float gamma{ 0.99f };
    bool  postStabilize{ true };

    Rigid* bodies{};
    Force* forces{};

    Solver();
    ~Solver();

    void clear();
    void defaultParams();
    int  step();
    void draw();
};

// Holds all user defined and derived constraint parameters, and provides a common interface for all forces.
struct Force
{
    Solver* solver;
    Rigid* bodyA;
    Rigid* bodyB;
    Force* nextA;
    Force* nextB;
    Force* next;

    float6 J[MAX_ROWS];
    float6x6 H[MAX_ROWS];
    float C[MAX_ROWS];
    float fmin[MAX_ROWS];
    float fmax[MAX_ROWS];
    float stiffness[MAX_ROWS];
    float fracture[MAX_ROWS];
    float penalty[MAX_ROWS];
    float lambda[MAX_ROWS];

    Force(Solver* solver, Rigid* bodyA, Rigid* bodyB);
    virtual ~Force();

    void disable();

    virtual int rows() const = 0;
    virtual bool initialize() = 0;
    virtual void computeConstraint(float alpha) = 0;
    virtual void computeDerivatives(Rigid* body) = 0;
    virtual void draw() const {}
};

// 힌지 조인트: 위치 3행 + 스윙 2행(+ 선택 트위스트 1행) = 5~6행
struct Joint : Force {
    Rigid* A;
    Rigid* B;

    // 위치 일치용 앵커(로컬)
    float3 rA_local;
    float3 rB_local;

    // 스윙/트위스트 설정
    bool   enableTwist = false;   // true면 twist 행 활성화 (마지막 행)
    // 로컬 힌지 축 및 기준(축과 직교) — A/B 로컬 좌표계에서 주어짐
    float3 axisA_local{ 0,0,1 };
    float3 axisB_local{ 0,0,1 };
    float3 refA_local{ 1,0,0 };
    float3 refB_local{ 1,0,0 };
    float  restTwist = 0.0f;      // 목표 트위스트 (라디안)

    // 스윙/트위스트에서 쓸 고정 월드 기준들 (initialize에서 고정)
    float3 axisW_fixed{ 0,0,1 };    // 힌지 축(월드, 고정)
    float3 swingU{ 1,0,0 };         // 축에 수직인 첫 기준(월드, 고정)
    float3 swingV{ 0,1,0 };         // 축에 수직인 두번째 기준(월드, 고정)

    // 스텝 시작 시 C(x-) 저장(안정화용)
    float  C0[6]{};

    // --- 생성자: 볼(위치 3행)만 ---
    Joint(Solver* solver,
        Rigid* bodyA, Rigid* bodyB,
        float3 rA, float3 rB,
        float3 posStiffness = float3{ INFINITY, INFINITY, INFINITY },
        float  fracture = INFINITY);

    // --- 생성자: 힌지(스윙 2행 + (선택) 트위스트 1행 포함) ---
    // twistK==0이면 트위스트 비활성 (총 5행), >0이면 활성(총 6행)
    Joint(Solver* solver,
        Rigid* bodyA, Rigid* bodyB,
        float3 rA, float3 rB,
        float3 axisA, float3 axisB,
        float3 refA, float3 refB,
        float3 posK,          // 위치 3행 강성
        float  swingK,        // 스윙 2행 강성(두 행 동일)
        float  twistK = 0.0f, // 트위스트(마지막 행) 강성(0이면 비활성)
        float  restTwistRadians = 0.0f,
        float  frac = INFINITY);

    bool initialize() override;
    int  rows() const override { return enableTwist ? 6 : 5; }  // 위치3 + 스윙2 + (옵션)트위스트1

    void computeConstraint(float alpha) override;
    void computeDerivatives(Rigid* body) override;
    void draw() const override;
};

// 3D 충돌 매니폴드(접촉점당: 법선1 + 마찰2 = 3행)
struct Manifold : Force
{
    // 프레임 간 feature 매칭용 (간단히 id만 보관)
    union FeaturePair {
        struct { int a, b, c, d; } e;
        uint64_t value;
    };

    struct Contact {
        FeaturePair feature{};
        // 로컬 앵커: A/B의 로컬 좌표
        float3 rA{};
        float3 rB{};

        // 월드 기저
        float3 n{};  // normal (A->B, unit)
        float3 u{};  // tangent 1 (unit, n⊥u)
        float3 v{};  // tangent 2 (unit, n⊥v, u×v=n)

        // 선형화용 J (각 6벡터)
        float6 JAn{}, JBn{};
        float6 JAu{}, JBu{};
        float6 JAv{}, JBv{};

        // C(x-) 저장 (법선/마찰 3성분)
        float3 C0{};

        // 정지마찰 판단
        bool stickU{ false };
        bool stickV{ false };
    };

    // 최대 접촉점 수 (박스-박스면 보통 ≤4)
    
    Contact contacts[MAX_CONTACTS];
    int     numContacts{ 0 };
    float   mu{ 0.0f }; // 마찰계수 μ

    Manifold(Solver* solver, Rigid* A, Rigid* B);

    int  rows() const override { return numContacts * 3; }
    bool initialize() override;
    void computeConstraint(float alpha) override;
    void computeDerivatives(Rigid* body) override;
    void draw() const override;

    // 좁은단계: A,B OBB 충돌을 찾아 contacts를 채워야 함 (아래 정의 참고)
    static int collide(Rigid* A, Rigid* B, Contact* out, int maxOut);
};
