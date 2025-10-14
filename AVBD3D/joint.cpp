// ----- Joint.cpp -----
#include "solver.h"
#include <algorithm>
#include <cmath>

// A의 로컬(=qA 기준) 회전벡터를 월드로 변환: v_world = R(A) * v_local
static inline float3 toWorldFromA(const Rigid* A, float3 v_local) {
    if (!A) return v_local; // A==월드(항등)
    return A->R() * v_local;
}

// ====== Joint 구현 ======

/* 1) 위치 3행만(볼) */
Joint::Joint(Solver* solver,
    Rigid* bodyA, Rigid* bodyB,
    float3 rA, float3 rB,
    float3 posStiffness,
    float  frac)
    : Force(solver, bodyA, bodyB)
    , A(bodyA), B(bodyB)
    , rA_local(rA), rB_local(rB)
{
    // 위치 3행 강성
    stiffness[0] = posStiffness.x; stiffness[1] = posStiffness.y; stiffness[2] = posStiffness.z;
    for (int i = 0; i < 3; ++i) {
        penalty[i] = 0; lambda[i] = 0; fracture[i] = frac;
        if (std::isfinite(frac)) { fmin[i] = -frac; fmax[i] = +frac; }
        else { fmin[i] = -INFINITY; fmax[i] = +INFINITY; }
    }
    // 스윙/트위스트 비활성 → 나머지 행들 0
    for (int i = 3; i < 6; ++i) {
        stiffness[i] = 0; penalty[i] = 0; lambda[i] = 0; fracture[i] = INFINITY;
        fmin[i] = -INFINITY; fmax[i] = +INFINITY;
    }
    enableTwist = false;
    // 스윙U/V 기본값 (쓰지 않지만 초기화)
    swingU = { 1,0,0 }; swingV = { 0,1,0 };
    axisW_fixed = { 0,0,1 };
}

/* 2) 힌지(스윙 2행 + (옵션) 트위스트 1행) */
Joint::Joint(Solver* solver,
    Rigid* bodyA, Rigid* bodyB,
    float3 rA, float3 rB,
    float3 axisA, float3 axisB,
    float3 refA, float3 refB,
    float3 posK,
    float  swingK,
    float  twistK,
    float  restTwistRadians,
    float  frac)
    : Force(solver, bodyA, bodyB)
    , A(bodyA), B(bodyB)
    , rA_local(rA), rB_local(rB)
    , axisA_local(axisA), axisB_local(axisB)
    , refA_local(refA), refB_local(refB)
    , restTwist(restTwistRadians)
{
    // 위치 3행
    stiffness[0] = posK.x; stiffness[1] = posK.y; stiffness[2] = posK.z;
    for (int i = 0; i < 3; ++i) {
        penalty[i] = 0; lambda[i] = 0; fracture[i] = frac;
        if (std::isfinite(frac)) { fmin[i] = -frac; fmax[i] = +frac; }
        else { fmin[i] = -INFINITY; fmax[i] = +INFINITY; }
    }
    // 스윙 2행 (행 3,4)
    stiffness[3] = swingK; stiffness[4] = swingK;
    for (int i = 3; i <= 4; ++i) {
        penalty[i] = 0; lambda[i] = 0; fracture[i] = frac;
        if (std::isfinite(frac)) { fmin[i] = -frac; fmax[i] = +frac; }
        else { fmin[i] = -INFINITY; fmax[i] = +INFINITY; }
    }
    // 트위스트 1행 (행 5) — twistK==0이면 비활성
    enableTwist = (twistK > 0.0f);
    stiffness[5] = enableTwist ? twistK : 0.0f;
    penalty[5] = 0; lambda[5] = 0; fracture[5] = frac;
    if (std::isfinite(frac)) { fmin[5] = -frac; fmax[5] = +frac; }
    else { fmin[5] = -INFINITY; fmax[5] = +INFINITY; }

    // 기준 초기값(실제 값은 initialize에서 확정)
    axisW_fixed = { 0,0,1 };
    swingU = { 1,0,0 }; swingV = { 0,1,0 };
}

bool Joint::initialize()
{
    // --- 위치 3행 C0 ---
    float3 pA0 = A ? A->worldPoint(rA_local) : rA_local;
    float3 pB0 = B ? B->worldPoint(rB_local) : rB_local;
    float3 e0 = pA0 - pB0;
    C0[0] = e0.x; C0[1] = e0.y; C0[2] = e0.z;

    // --- 스윙/트위스트 기준을 고정(월드기준) ---
    // 월드 힌지축: aAw, aBw 의 평균 정규화; 휘청거리면 한쪽을 사용
    float3 aAw = A ? (A->R() * axisA_local) : axisA_local;
    float3 aBw = B ? (B->R() * axisB_local) : axisB_local;
    float3 ax = aAw + aBw;
    if (ax.x * ax.x + ax.y * ax.y + ax.z * ax.z < 1e-10f) ax = aAw; // fallback
    axisW_fixed = normalize(ax);

    // 스윙 평면 기준(swingU, swingV) — refA를 기반으로 축 직교화
    float3 tA0w = A ? (A->R() * refA_local) : refA_local;
    float3 u = projectOnPlane(tA0w, axisW_fixed);
    if (u.x * u.x + u.y * u.y + u.z * u.z < 1e-10f) u = { 1,0,0 }; // degenerate fallback
    swingU = normalize(u);
    swingV = normalize(cross(axisW_fixed, swingU)); // 축 × U

    // --- 스윙 2행 & (옵션) 트위스트 1행 안정화용 C0 저장 ---
    // 상대 자세: q_rel = conj(qA) * qB  (A없으면 conj(I)=I)
    quat qA = A ? A->q : quat{ 1,0,0,0 };
    quat qB = B ? B->q : quat{ 1,0,0,0 };
    quat qrel = qmul(qconj(qA), qB);      // A기준 로컬
    float3 vrel_local = quat_log_vec_local(qrel);   // A로컬 회전벡터
    float3 vrel_world = toWorldFromA(A, vrel_local);// 월드로

    // 스윙 2행: v_rel의 축에 수직 성분(=U/V축 성분)을 0으로
    C0[3] = vrel_world.x * swingU.x + vrel_world.y * swingU.y + vrel_world.z * swingU.z; // U성분
    C0[4] = vrel_world.x * swingV.x + vrel_world.y * swingV.y + vrel_world.z * swingV.z; // V성분
    // 트위스트: 축 성분 - restTwist
    C0[5] = (vrel_world.x * axisW_fixed.x + vrel_world.y * axisW_fixed.y + vrel_world.z * axisW_fixed.z) - restTwist;

    // 활성 조건: 위치 3행 중 1개라도, 또는 스윙(둘), 또는 트위스트가 유효
    bool posActive = (stiffness[0] != 0 || stiffness[1] != 0 || stiffness[2] != 0);
    bool swingActive = (stiffness[3] != 0 || stiffness[4] != 0);
    bool twistActive = (enableTwist && stiffness[5] != 0);
    return posActive || swingActive || twistActive;
}

/*
void Joint::computeConstraint(float alpha)
{
    // --- 위치 3행 ---
    float3 pA = A ? A->worldPoint(rA_local) : rA_local;
    float3 pB = B ? B->worldPoint(rB_local) : rB_local;
    float3 e = pA - pB;
    for (int i = 0; i < 3; ++i) {
        const float ei = (i == 0 ? e.x : (i == 1 ? e.y : e.z));
        C[i] = std::isinf(stiffness[i]) ? (ei - alpha * C0[i]) : ei;
    }

    // --- 스윙/트위스트 ---
    quat qA = A ? A->q : quat{ 1,0,0,0 };
    quat qB = B ? B->q : quat{ 1,0,0,0 };
    quat qrel = qmul(qconj(qA), qB);
    float3 vrel_local = quat_log_vec_local(qrel);
    float3 vrel_world = toWorldFromA(A, vrel_local);

    // 스윙 2행: U, V 성분
    float CnU = vrel_world.x * swingU.x + vrel_world.y * swingU.y + vrel_world.z * swingU.z;
    float CnV = vrel_world.x * swingV.x + vrel_world.y * swingV.y + vrel_world.z * swingV.z;

    C[3] = std::isinf(stiffness[3]) ? (CnU - alpha * C0[3]) : CnU;
    C[4] = std::isinf(stiffness[4]) ? (CnV - alpha * C0[4]) : CnV;

    // 트위스트 1행: 축 성분 - restTwist
    if (enableTwist) {
        float CnTw = (vrel_world.x * axisW_fixed.x + vrel_world.y * axisW_fixed.y + vrel_world.z * axisW_fixed.z) - restTwist;
        C[5] = std::isinf(stiffness[5]) ? (CnTw - alpha * C0[5]) : CnTw;
    }
    else {
        C[5] = 0.0f;
    }
}
*/


void Joint::computeConstraint(float alpha)
{
    // Δx, Δθ (초기 상태 대비)
    const float3 dpA = A ? (A->x - A->x_initial) : float3{ 0,0,0 };
    const float3 dtA = A ? rotVec_fromTo(A->q_initial, A->q) : float3{ 0,0,0 };
    const float3 dpB = B ? (B->x - B->x_initial) : float3{ 0,0,0 };
    const float3 dtB = B ? rotVec_fromTo(B->q_initial, B->q) : float3{ 0,0,0 };

    // --- 위치 3행: contact와 동일한 선형화 ---
    // Ji_A = [ dir,  rA×dir ], Ji_B = [ -dir, -(rB×dir) ]
    // C = C0*(1-α) + Ji_A·[dpA,dtA] + Ji_B·[dpB,dtB]
    static const float3 ex{ 1,0,0 }, ey{ 0,1,0 }, ez{ 0,0,1 };

    auto addPosRow = [&](int row, const float3& dir) {
        float c = C0[row] * (1.0f - alpha);
        if (A) c += dot6(Jrow_A_pointDir(*A, rA_local, dir), dpA, dtA);
        if (B) c += dot6(Jrow_B_pointDir(*B, rB_local, dir), dpB, dtB);
        C[row] = c;
    };
    addPosRow(0, ex);
    addPosRow(1, ey);
    addPosRow(2, ez);

    // --- 스윙 2행 & 트위스트 1행: 순수 회전 증분 ---
    // J_A = [0, -basis], J_B = [0, +basis] → C = C0*(1-α) + basis·(dtB - dtA)
    auto addRotRow = [&](int row, const float3& basisW) {
        float c = C0[row] * (1.0f - alpha);
        c += basisW.x * (dtB.x - dtA.x) + basisW.y * (dtB.y - dtA.y) + basisW.z * (dtB.z - dtA.z);
        C[row] = c;
    };
    addRotRow(3, swingU);
    addRotRow(4, swingV);
    if (enableTwist) addRotRow(5, axisW_fixed);
    else             C[5] = 0.0f;
}


void Joint::computeDerivatives(Rigid* body)
{
    // 위치 3행은 “포인트-방향” 자코비안 사용
    static const float3 ex{ 1,0,0 }, ey{ 0,1,0 }, ez{ 0,0,1 };
    if (body == A) {
        J[0] = Jrow_A_pointDir(*A, rA_local, ex);
        J[1] = Jrow_A_pointDir(*A, rA_local, ey);
        J[2] = Jrow_A_pointDir(*A, rA_local, ez);
    }
    else {
        J[0] = Jrow_B_pointDir(*B, rB_local, ex);
        J[1] = Jrow_B_pointDir(*B, rB_local, ey);
        J[2] = Jrow_B_pointDir(*B, rB_local, ez);
    }

    // 스윙 2행: 회전만 관여 (선형=0, 각=±U/±V)
    auto setPureRotRow = [&](int row, float3 basisW, bool isA) {
        float6 Jr{}; // [0 0 0 | ±basis]
        Jr.v[0] = Jr.v[1] = Jr.v[2] = 0.0f;
        Jr.v[3] = (isA ? -basisW.x : +basisW.x);
        Jr.v[4] = (isA ? -basisW.y : +basisW.y);
        Jr.v[5] = (isA ? -basisW.z : +basisW.z);
        J[row] = Jr;
    };
    setPureRotRow(3, swingU, body == A);
    setPureRotRow(4, swingV, body == A);

    // 트위스트 1행: 회전만 관여 (선형=0, 각=±axisW_fixed)
    if (enableTwist) setPureRotRow(5, axisW_fixed, body == A);
    else {
        // 비활성일 때도 안전하게 0 채움
        J[5] = float6{};
    }

    // Hessian은 0으로 (필요하면 대각 근사만 넣어도 됨)
    auto zeroH = [&](int i) { for (int r = 0; r < 6; ++r) for (int c = 0; c < 6; ++c) H[i].m[r][c] = 0.0f; };
    zeroH(0); zeroH(1); zeroH(2); zeroH(3); zeroH(4); if (enableTwist) zeroH(5);
}

void Joint::draw() const
{
    // 위치 앵커 라인
    float3 pA = A ? A->worldPoint(rA_local) : rA_local;
    float3 pB = B ? B->worldPoint(rB_local) : rB_local;

    glColor3f(0.75f, 0.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(pA.x, pA.y, pA.z);
    glVertex3f(pB.x, pB.y, pB.z);
    glEnd();

    // 축/스윙 기준을 빠르게 디버그 표시하고 싶다면:
    // (축)
    glColor3f(0.1f, 0.6f, 0.9f);
    glBegin(GL_LINES);
    float3 m = axisW_fixed * 0.5f;
    glVertex3f(pB.x, pB.y, pB.z);
    glVertex3f(pB.x + m.x, pB.y + m.y, pB.z + m.z);
    glEnd();
    // (U/V)
    glColor3f(0.2f, 0.8f, 0.2f);
    glBegin(GL_LINES);
    float3 u = swingU * 0.4f;
    glVertex3f(pB.x, pB.y, pB.z);
    glVertex3f(pB.x + u.x, pB.y + u.y, pB.z + u.z);
    glEnd();
    glColor3f(0.9f, 0.6f, 0.2f);
    glBegin(GL_LINES);
    float3 v = swingV * 0.4f;
    glVertex3f(pB.x, pB.y, pB.z);
    glVertex3f(pB.x + v.x, pB.y + v.y, pB.z + v.z);
    glEnd();
}
