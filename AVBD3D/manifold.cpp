#include "solver.h"
#include <cmath>
#include <cstring>

// 행 인덱싱 헬퍼
static inline int rowN(int i) { return i * 3 + 0; }
static inline int rowU(int i) { return i * 3 + 1; }
static inline int rowV(int i) { return i * 3 + 2; }

Manifold::Manifold(Solver* solver, Rigid* A, Rigid* B)
    : Force(solver, A, B) , numContacts(0)
{
    // 법선행은 “미는 힘만” 허용(= 당김 금지) → fmax=0, fmin=-INF  (부호는 구현 관례에 맞추세요)
    // 일단 모든 행 초기화
    for (int i = 0; i < MAX_ROWS; ++i) {
        const bool isNormal = (i % 3) == 0;
        fmin[i] = isNormal ? 0.0f : -INFINITY;  // λ_n ≥ 0만 허용(미는 힘만)
        fmax[i] = INFINITY;                         // 나머지는 일단 ∞ (마찰은 이후 프레임별로 제한)
        penalty[i] = 0.0f;
        lambda[i] = 0.0f;
    }
}

bool Manifold::initialize()
{
    mu = std::sqrt(std::max(0.0f, bodyA->friction) * std::max(0.0f, bodyB->friction));

    // 이전 프레임 저장
    Contact old[MAX_CONTACTS];
    std::memcpy(old, contacts, sizeof(contacts));
    float   oldPen[MAX_ROWS], oldLam[MAX_ROWS];
    std::memcpy(oldPen, penalty, sizeof(oldPen));
    std::memcpy(oldLam, lambda, sizeof(oldLam));
    int oldN = numContacts;

    // 새로운 접촉 검출
    numContacts = collide(bodyA, bodyB, contacts, MAX_CONTACTS);
    if (numContacts <= 0) return false;

    // 병합(웜스타트/스틱 유지)
    for (int i = 0; i < numContacts; i++) {
        // 기본 초기화
        penalty[rowN(i)] = penalty[rowU(i)] = penalty[rowV(i)] = 0.0f;
        lambda[rowN(i)] = lambda[rowU(i)] = lambda[rowV(i)] = 0.0f;
        contacts[i].stickU = contacts[i].stickV = false;

        for (int j = 0; j < oldN; j++) {
            if (contacts[i].feature.value == old[j].feature.value) {
                // 같은 feature이면 warmstart 이어 받기
                penalty[rowN(i)] = oldPen[rowN(j)];
                penalty[rowU(i)] = oldPen[rowU(j)];
                penalty[rowV(i)] = oldPen[rowV(j)];
                lambda[rowN(i)] = oldLam[rowN(j)];
                lambda[rowU(i)] = oldLam[rowU(j)];
                lambda[rowV(i)] = oldLam[rowV(j)];

                // stick 유지 시, 앵커를 이전 로컬로 유지(정지마찰 안정화)
                if (old[j].stickU && old[j].stickV) {
                    contacts[i].rA = old[j].rA;
                    contacts[i].rB = old[j].rB;
                }
            }
        }
    }

    // 각 접촉점에 대해 기저/자코비안/C0 구성
    for (int i = 0; i < numContacts; i++) {
        auto& c = contacts[i];

        // 월드 앵커
        float3 rAw = bodyA->R() * c.rA;
        float3 rBw = bodyB->R() * c.rB;
        float3 pAw = bodyA->x + rAw;
        float3 pBw = bodyB->x + rBw;

        // 법선 n (이미 collide에서 단위벡터로 채웠다고 가정)
        c.n = normalize(c.n);

        // 접선 u,v 생성 (임의 ONB)
        makeONB(c.n, c.u, c.v);

        // 사전계산 J (C(x-)에서의 선형화: Sec.4 방식)
        c.JAn = Jrow_A_pointDir(*bodyA, c.rA, c.n);
        c.JBn = Jrow_B_pointDir(*bodyB, c.rB, c.n);

        c.JAu = Jrow_A_pointDir(*bodyA, c.rA, c.u);
        c.JBu = Jrow_B_pointDir(*bodyB, c.rB, c.u);

        c.JAv = Jrow_A_pointDir(*bodyA, c.rA, c.v);
        c.JBv = Jrow_B_pointDir(*bodyB, c.rB, c.v);

        // C0 (법선은 여유 간격 추가, 마찰은 0 목표)
        float3 d0 = pAw - pBw;
        c.C0.x = (c.n.x * d0.x + c.n.y * d0.y + c.n.z * d0.z) - COLLISION_MARGIN; // 관통이면 +margin>0
        c.C0.y = (c.u.x * d0.x + c.u.y * d0.y + c.u.z * d0.z); // 접선목표 0
        c.C0.z = (c.v.x * d0.x + c.v.y * d0.y + c.v.z * d0.z);

        // 법선행은 “미는 힘만” (λ_n ≥ 0)
        fmin[rowN(i)] = 0.0f;
        fmax[rowN(i)] = INFINITY;
    }

    return true;
}

void Manifold::computeConstraint(float alpha)
{
    for (int i = 0; i < numContacts; i++) {
        auto& c = contacts[i];

        // Δx, Δθ (x - x_initial, log(q_initial→q))
        float3 dpA = bodyA->x - bodyA->x_initial;
        float3 dtA = rotVec_fromTo(bodyA->q_initial, bodyA->q);
        float3 dpB = bodyB->x - bodyB->x_initial;
        float3 dtB = rotVec_fromTo(bodyB->q_initial, bodyB->q);

        // 법선
        C[rowN(i)] = c.C0.x * (1 - alpha)
            + dot6(c.JAn, dpA, dtA)
            + dot6(c.JBn, dpB, dtB);

        // 접선 u
        C[rowU(i)] = c.C0.y * (1 - alpha)
            + dot6(c.JAu, dpA, dtA)
            + dot6(c.JBu, dpB, dtB);

        // 접선 v
        C[rowV(i)] = c.C0.z * (1 - alpha)
            + dot6(c.JAv, dpA, dtA)
            + dot6(c.JBv, dpB, dtB);

        // 최신 λ_n으로 마찰 경계 갱신 (pyramid 근사): |λ_t| ≤ μ |λ_n|
        float bound = mu * std::max(0.0f, lambda[rowN(i)]);
        fmax[rowU(i)] = +bound;  fmin[rowU(i)] = -bound;
        fmax[rowV(i)] = +bound;  fmin[rowV(i)] = -bound;

        // 정지마찰 판정(다음 프레임 앵커 유지 판단)
        c.stickU = (std::fabs(lambda[rowU(i)]) < bound) && (std::fabs(c.C0.y) < STICK_THRESH);
        c.stickV = (std::fabs(lambda[rowV(i)]) < bound) && (std::fabs(c.C0.z) < STICK_THRESH);
    }
}

void Manifold::computeDerivatives(Rigid* body)
{
    for (int i = 0; i < numContacts; i++) {
        auto& c = contacts[i];
        if (body == bodyA) {
            J[rowN(i)] = c.JAn;
            J[rowU(i)] = c.JAu;
            J[rowV(i)] = c.JAv;
        }
        else {
            J[rowN(i)] = c.JBn;
            J[rowU(i)] = c.JBu;
            J[rowV(i)] = c.JBv;
        }
        // H는 접촉에서 생략(0) — 필요 시 대각 근사만 넣어도 OK
        for (int r = 0; r < 6; r++) for (int cc = 0; cc < 6; cc++) {
            H[rowN(i)].m[r][cc] = 0.0f;
            H[rowU(i)].m[r][cc] = 0.0f;
            H[rowV(i)].m[r][cc] = 0.0f;
        }
    }
}

void Manifold::draw() const
{
#if SHOW_CONTACTS
    for (int i = 0; i < numContacts; i++) {
        const auto& c = contacts[i];
        float3 pAw = bodyA->x + bodyA->R() * c.rA;
        float3 pBw = bodyB->x + bodyB->R() * c.rB;
        float3 pc = (pAw + pBw) * 0.5f;

        glPointSize(6.0f);
        glBegin(GL_POINTS);
        glColor3f(0.9f, 0.1f, 0.1f);
        glVertex3f(pc.x, pc.y, pc.z);
        glEnd();

        // normal 시각화
        glBegin(GL_LINES);
        glColor3f(0.9f, 0.1f, 0.1f);
        glVertex3f(pc.x, pc.y, pc.z);
        float3 q = pc + c.n * 0.25f;
        glVertex3f(q.x, q.y, q.z);
        glEnd();
    }
#endif
}
