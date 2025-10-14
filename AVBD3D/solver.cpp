// solver.cpp  (3D AVBD 버전)
#include <algorithm>
#include <cmath>
#include <cstring>
#include "solver.h"
#include "graghColor.h"

// ======== Solver ========
Solver::Solver() : bodies(0) { defaultParams(); }
Solver::~Solver() { clear(); }

void Solver::clear()
{
    while (bodies) delete bodies;
    while (forces) delete forces;
}

void Solver::defaultParams()
{
    dt = 1.0f / 60.0f;
    gravity = -9.8f;    // 3D 시작값: 원하면 -9.8로
    iterations = 10;

    beta = 200.0f;
    alpha = 0.99f;
    gamma = 0.95f;
    postStabilize = true;
}

// 3D 6×6 프라이멀/듀얼 업데이트
int Solver::step()
{
    // Perform broadphase collision detection
    // This is a naive O(n^2) approach, but it is sufficient for small numbers of bodies in this sample.
    for (Rigid* bodyA = bodies; bodyA != 0; bodyA = bodyA->next)
    {
        for (Rigid* bodyB = bodyA->next; bodyB != 0; bodyB = bodyB->next)
        {
            float3 dp = bodyA->x - bodyB->x;
            float r = bodyA->radius + bodyB->radius;
            if (dot(dp, dp) <= r * r && bodyA->constrainedTo(bodyB) == false)
                new Manifold(this, bodyA, bodyB);
        }
    }

    // ----- Force warmstart / 활성 필터 -----
    for (Force* f = forces; f != nullptr; )
    {
        if (!f->initialize()) {
            Force* nxt = f->next;
            delete f;
            f = nxt;
            continue;
        }

        for (int i = 0; i < f->rows(); ++i) {
            if (postStabilize) {
                f->penalty[i] = std::clamp(f->penalty[i] * gamma, PENALTY_MIN, PENALTY_MAX);
            }
            else {
                f->lambda[i] = f->lambda[i] * alpha * gamma;
                f->penalty[i] = std::clamp(f->penalty[i] * gamma, PENALTY_MIN, PENALTY_MAX);
            }
            // 재질 강성으로 상한
            f->penalty[i] = std::min(f->penalty[i], f->stiffness[i]);
        }
        f = f->next;
    }

    // ----- Body/Force 목록 -----
    std::vector<Rigid*> bodyList; bodyList.reserve(1024);
    for (Rigid* b = bodies; b; b = b->next) bodyList.push_back(b);

    std::vector<Force*> forceList; forceList.reserve(1024);
    for (Force* f = forces; f; f = f->next) forceList.push_back(f);

    std::vector<std::vector<Rigid*>> buckets;
    std::unordered_map<Rigid*, int> colorOf;
    std::vector<Rigid*> nodes;
    int bucketSize = buildBodyColors(bodies, buckets, colorOf);

    // ----- Bodies warmstart (x-, q-) -----
    const float3 gravityVector{ 0.0f, gravity, 0.0f };

#pragma omp parallel for schedule(static) 
    for (int bi = 0; bi < (int)bodyList.size(); ++bi)
    {
        Rigid* b = bodyList[bi];

        // 각속 제한
        b->w = clamp3(b->w, -50.0f, 50.0f);

        // 저장
        b->x_initial = b->x;
        b->q_initial = b->q;

        // 관성 예측
        b->x_inertial = b->x + b->v * dt;
        if (b->mass > 0.0f) b->x_inertial = b->x_inertial + gravityVector * (dt * dt);
        b->q_inertial = qnorm(qmul(b->q, expQuat(b->w * dt)));

        // Adaptive warmstart (선택)
        float accelWeight = 0.0f;
        float3 a = (b->v - b->v_prev) * (1.0f / dt);
        if (std::abs(gravity) > 1e-8f) {
            float aExt = a.y * (gravity >= 0.0f ? 1.0f : -1.0f);
            accelWeight = std::clamp(aExt / std::abs(gravity), 0.0f, 1.0f);
            if (!std::isfinite(accelWeight)) accelWeight = 0.0f;
        }

        // warm-start 추정
        b->x = b->x + b->v * dt + gravityVector * (accelWeight * dt * dt);
        b->q = qnorm(qmul(b->q, expQuat(b->w * dt)));

        b->v_prev = b->v;
    }

    // ----- 메인 반복 -----
    const int totalIterations = iterations + (postStabilize ? 1 : 0);

    for (int it = 0; it < totalIterations; ++it)
    {
        const float currentAlpha = postStabilize ? (it < iterations ? 1.0f : 0.0f) : alpha;

        // Primal update
        for (const std::vector<Rigid*>& bucket : buckets)
        {
#pragma omp parallel for schedule(static)
            for (int bi = 0; bi < (int)bucket.size(); ++bi)
            {
                Rigid* body = bucket[bi];

                if (body->mass <= 0.0f) continue;

                // LHS = M/dt^2
                float6x6 lhs; zero6x6(lhs);
                addMassInertiaBlock(lhs, body->mass, body->Iworld, 1.0f / (dt * dt));

                // RHS = (M/dt^2)*[ x - x_inertial ; Δθ ]
                float3 dpos = body->x - body->x_inertial;
                float3 dtheta = rotVec_fromTo(body->q_inertial, body->q); // (inertial → current)

                float6 rhs; zero6(rhs);
                // 선형
                rhs.v[0] = (body->mass / (dt * dt)) * dpos.x;
                rhs.v[1] = (body->mass / (dt * dt)) * dpos.y;
                rhs.v[2] = (body->mass / (dt * dt)) * dpos.z;
                // 각
                float3 ang = mul33(body->Iworld, dtheta) * (1.0f / (dt * dt));
                rhs.v[3] = ang.x; rhs.v[4] = ang.y; rhs.v[5] = ang.z;

                // 바디에 붙은 force 누적
                for (Force* f = body->forces; f != nullptr; f = (f->bodyA == body) ? f->nextA : f->nextB)
                {
                    // 제약/도함수
                    f->computeConstraint(currentAlpha);
                    f->computeDerivatives(body);

                    for (int i = 0; i < f->rows(); ++i)
                    {
                        const float lambda = std::isinf(f->stiffness[i]) ? f->lambda[i] : 0.0f;
                        const float fi = std::clamp(f->penalty[i] * f->C[i] + lambda, f->fmin[i], f->fmax[i]);

                        // LHS += ρ JᵢᵀJᵢ
                        addJTJ(lhs, f->J[i], f->penalty[i]);

                        // G(대각 근사): diag(||H.col(k)|| * |f|)
                        // (computeDerivatives에서 H=0이면 효과 없음)
                        float gdiag[6] = {
                            colL2(f->H[i], 0) * std::fabs(fi),
                            colL2(f->H[i], 1) * std::fabs(fi),
                            colL2(f->H[i], 2) * std::fabs(fi),
                            colL2(f->H[i], 3) * std::fabs(fi),
                            colL2(f->H[i], 4) * std::fabs(fi),
                            colL2(f->H[i], 5) * std::fabs(fi)
                        };
                        addDiag(lhs, gdiag);

                        // RHS += Jᵢ * f
                        rhs = rhs + f->J[i] * fi; 
                    }
                }

                // 6×6 풀기
                float6 d = solveSPD6x6(lhs, rhs);

                // 적용: x -= dp, q = q * exp(-dtheta)
                float3 dp{ d.v[0], d.v[1], d.v[2] };
                float3 dth{ d.v[3], d.v[4], d.v[5] };
                body->x = body->x - dp;
                body->q = qnorm(qmul(body->q, expQuat({ -dth.x, -dth.y, -dth.z })));

                // Iworld 갱신(다음 반복을 위해)
                const float3x3 Rm = body->q.R();
                const float3x3 D = diagonal(body->Ibody.x, body->Ibody.y, body->Ibody.z);
                body->Iworld = Rm * D * transpose3x3(Rm);
            }
        }

        // === Dual update (원래 식과 동일) ===
        if (it < iterations)
        {
#pragma omp parallel for schedule(static) 
            for (int k = 0; k < (int)forceList.size(); ++k)
            {
                Force* f = forceList[k];

                f->computeConstraint(currentAlpha);
                for (int i = 0; i < f->rows(); ++i)
                {
                    const float lambda = std::isinf(f->stiffness[i]) ? f->lambda[i] : 0.0f;

                    // λ ← clamp(ρC + λ, [fmin,fmax])
                    f->lambda[i] = std::clamp(f->penalty[i] * f->C[i] + lambda, f->fmin[i], f->fmax[i]);

                    // 파단
                    if (std::fabs(f->lambda[i]) >= f->fracture[i]) f->disable();

                    // 페널티 램핑
                    if (f->lambda[i] > f->fmin[i] && f->lambda[i] < f->fmax[i]) {
                        f->penalty[i] = std::min(f->penalty[i] + beta * std::fabs(f->C[i]),
                            std::min(PENALTY_MAX, f->stiffness[i]));
                    }
                }
            }
        }

        // 속도 업데이트 (마지막 정상 반복 직후)
        if (it == iterations - 1)
        {
#pragma omp parallel for schedule(static) 
            for (int bi = 0; bi < (int)bodyList.size(); ++bi)
            {
                Rigid* b = bodyList[bi];

                // 선속
                b->v = (b->x - b->x_initial) * (1.0f / dt);
                // 각속 ~ Δθ/dt
                float3 dth = rotVec_fromTo(b->q_initial, b->q);
                b->w = dth * (1.0f / dt);
            }
        }
    }

    return (int)bodyList.size();
}

void Solver::draw()
{
    for (Rigid* body = bodies; body != 0; body = body->next) body->draw();
    for (Force* f = forces; f != nullptr; f = f->next) f->draw();
}
