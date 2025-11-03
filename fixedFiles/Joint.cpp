// Joint.cpp
#include "solver.h"
#include <algorithm>
#include <cmath>

// ===== 유틸 =====
static inline vec3 normalizeSafe(const vec3& v, const vec3& fb) {
    float L = length(v);
    return (L > 1e-8f) ? (v / L) : fb;
}

static inline void makeOrthonormalBasis(const vec3& n, vec3& u, vec3& v) {
    vec3 a = (std::fabs(n.x) < 0.9f) ? vec3(1, 0, 0) : vec3(0, 1, 0);
    u = normalizeSafe(cross(a, n), (std::fabs(n.z) < 0.9f ? vec3(0, 0, 1) : vec3(1, 0, 0)));
    v = normalizeSafe(cross(n, u), vec3(0, 1, 0));
}

// A 기준 로컬 회전로그 → 월드
static inline vec3 rotLog_toWorld(const Rigid* A, const vec3& vLocal) {
    return A ? rotate(A->rotation, vLocal) : vLocal;
}

// 월드 앵커 rW = R(A)*r_local (+scale)
static inline vec3 anchor_world(const Rigid* body, const vec3& rLocal) {
    return rotateWithoutScale(rLocal, const_cast<Rigid*>(body));
}

// 위치행 자코비안 (A/B)
static inline vec6 Jrow_point_A(const Rigid& A, const vec3& rA_local, const vec3& dirW) {
    vec3 rAw = anchor_world(&A, rA_local);
    vec3 ang = cross(rAw, dirW);
    return vec6(dirW.x, dirW.y, dirW.z, ang.x, ang.y, ang.z);
}
static inline vec6 Jrow_point_B(const Rigid& B, const vec3& rB_local, const vec3& dirW) {
    vec3 rBw = anchor_world(&B, rB_local);
    vec3 ang = cross(rBw, dirW);
    return vec6(-dirW.x, -dirW.y, -dirW.z, -ang.x, -ang.y, -ang.z);
}

// ===== Joint 구현 =====
Joint::Joint(Solver* solver,
    Rigid* bodyA, Rigid* bodyB,
    vec3 rA, vec3 rB,
    vec3 axisA, vec3 axisB,
    vec3 refA, vec3 refB,
    vec3 posK,
    float swingK,
    float frac,
    float twistK)
    : Force(solver, bodyA, bodyB),
    A(bodyA), B(bodyB),
    rA_local(rA), rB_local(rB),
    axisA_local(axisA), axisB_local(axisB),
    refA_local(refA), refB_local(refB)
{
    // 공통 초기화
    for (int i = 0; i < MAX_ROWS; ++i) {
        C[i] = 0; fmin[i] = -INFINITY; fmax[i] = +INFINITY;
        stiffness[i] = 0; motor[i] = 0; fracture[i] = INFINITY;
        penalty[i] = 0; lambda[i] = 0;
    }
    // 위치 3행
    stiffness[0] = posK.x; stiffness[1] = posK.y; stiffness[2] = posK.z;
    if (std::isfinite(frac)) {
        for (int i = 0; i < 3; ++i) { fracture[i] = frac; fmin[i] = -frac; fmax[i] = +frac; }
    }
    // 스윙 2행 (3,4)
    stiffness[3] = swingK; stiffness[4] = swingK;
    if (std::isfinite(frac)) {
        for (int i = 3; i <= 4; ++i) { fracture[i] = frac; fmin[i] = -frac; fmax[i] = +frac; }
    }

    // 트위스트 1행 (5) — 기본 비활성, twistK>0일 때만 사용
    stiffness[5] = twistK;
    if (twistK != 0.0f && std::isfinite(frac)) {
        fracture[5] = frac; fmin[5] = -frac; fmax[5] = +frac;        
    }
    
    // 총 행수 결정: 스윙만(5) , 트위스트 포함(6)
    nrows = (twistK == 0.0f) ? 5 : 6;

    // 행 수(=5)에 맞춰 컨테이너 리사이즈
    J.resize(nrows);
    H.resize(nrows);
    // 0으로 클리어 (mat6x6(float) 없음 → 수동 0화)
    for (int i = 0; i < nrows; ++i) {
        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 6; ++c)
                H[i][r][c] = 0.0f;
    }
}

bool Joint::initialize()
{
    // 월드 힌지축 고정
    vec3 aAw = A ? rotate(A->rotation, axisA_local) : axisA_local;
    vec3 aBw = B ? rotate(B->rotation, axisB_local) : axisB_local;
    vec3 axTry = A ? aAw + aBw : aAw;
    axisW_fixed = normalizeSafe(axTry, normalizeSafe(aAw, vec3(0, 0, 1)));

    // 스윙 평면 U/V
    makeOrthonormalBasis(axisW_fixed, swingU, swingV);
    // refA 사영으로 U 보정
    vec3 tA0w = A ? rotate(A->rotation, refA_local) : refA_local;
    vec3 uProj = tA0w - axisW_fixed * dot(axisW_fixed, tA0w);
    if (length(uProj) > 1e-6f) {
        swingU = normalize(uProj);
        swingV = normalize(cross(axisW_fixed, swingU));
    }

    bool posActive = (stiffness[0] != 0 || stiffness[1] != 0 || stiffness[2] != 0);
    bool swingActive = (stiffness[3] != 0 || stiffness[4] != 0);
    bool twistActive = (nrows > 5);
    return posActive || swingActive || twistActive;
}

void Joint::computeConstraint(float /*alpha*/)
{
    // 위치 3행: pA - pB
    vec3 pA = A ? (A->position + anchor_world(A, rA_local)) : rA_local;
    vec3 pB = B ? (B->position + anchor_world(B, rB_local)) : rB_local;
    vec3 e = pA - pB;
    C[0] = e.x; C[1] = e.y; C[2] = e.z;

    // 스윙 2행: 상대 회전의 U/V 성분
    quat qA = A ? A->rotation : quat(1, 0, 0, 0);
    quat qB = B ? B->rotation : quat(1, 0, 0, 0);
    quat qrel = conjugate(qA) * qB;   // A 기준
    quat l = glm::log(glm::normalize(qrel));   // l.w≈0, l.xyz = axis*(θ/2)
    vec3 vrel_local = vec3(l.x, l.y, l.z) * 2.0f;  // 회전벡터 = 2 * Im(log)
    vec3 vrel_world = rotLog_toWorld(A, vrel_local);

    C[3] = dot(vrel_world, swingU);
    C[4] = dot(vrel_world, swingV);

    // 트위스트(축에 대한 상대 회전)
    if (nrows > 5) {
        C[5] = dot(vrel_world, axisW_fixed);        
    }
}

void Joint::computeDerivatives(Rigid* body)
{
    static const vec3 ex(1, 0, 0), ey(0, 1, 0), ez(0, 0, 1);

    // 위치 3행 J
    if (body == A) {
        J[0] = Jrow_point_A(*A, rA_local, ex);
        J[1] = Jrow_point_A(*A, rA_local, ey);
        J[2] = Jrow_point_A(*A, rA_local, ez);
    }
    else {
        J[0] = Jrow_point_B(*B, rB_local, ex);
        J[1] = Jrow_point_B(*B, rB_local, ey);
        J[2] = Jrow_point_B(*B, rB_local, ez);
    }

    // 스윙 2행 J (순수 회전)
    auto pureRotRow = [&](const vec3& basisW, bool isA)->vec6 {
        return isA ? vec6(0, 0, 0, -basisW.x, -basisW.y, -basisW.z)
            : vec6(0, 0, 0, +basisW.x, +basisW.y, +basisW.z);
        };
    J[3] = pureRotRow(swingU, body == A);
    J[4] = pureRotRow(swingV, body == A);

    // 트위스트 J: 힌지축 방향 순수 회전
    if (nrows > 5) {    
        J[5] = pureRotRow(axisW_fixed, body == A);        
    }

    // H를 0으로 (mat6x6(float) 없음 → 수동 0화)
    for (int i = 0; i < rows(); ++i)
        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 6; ++c)
                H[i][r][c] = 0.0f;

    // 각도 대각 감쇠(회전행에만)
    const float kd = 0.2f; // 0.02~0.3
    H[3][3][3] += kd; H[3][4][4] += kd; H[3][5][5] += kd;
    H[4][3][3] += kd; H[4][4][4] += kd; H[4][5][5] += kd;

    // 트위스트 감쇠
    if (nrows > 5) {
        H[5][3][3] += kd; H[5][4][4] += kd; H[5][5][5] += kd;         
    }

    // 위치행에 θθ 랭크-1(SPD) 추가 → 고속 회전 안정화
    auto addAxisAlignedThetaThetaSPD = [&](int row, const vec3& rW, const vec3& dir) {
        vec3 u = cross(rW, dir);
        float L = length(u);
        if (L < 1e-8f) return;
        vec3 n = u / L;
        float w = 0.6f * (L * L); // 0.3~1.0 튠
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                H[row][3 + i][3 + j] += w * n[i] * n[j];
        };

    if (body == A) {
        vec3 rAw = anchor_world(A, rA_local);
        addAxisAlignedThetaThetaSPD(0, rAw, ex);
        addAxisAlignedThetaThetaSPD(1, rAw, ey);
        addAxisAlignedThetaThetaSPD(2, rAw, ez);
    }
    else {
        vec3 rBw = anchor_world(B, rB_local);
        addAxisAlignedThetaThetaSPD(0, rBw, ex);
        addAxisAlignedThetaThetaSPD(1, rBw, ey);
        addAxisAlignedThetaThetaSPD(2, rBw, ez);
    }
}