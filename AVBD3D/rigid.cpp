//rigid.cpp
#pragma once

#include "solver.h"

Rigid::Rigid(Solver* solver_,
    float3 size_,
    float density,
    float friction_,
    float3 position,
    float3 rotation,          // 라디안 Euler XYZ
    float3 linearVelocity,
    float3 angularVelocity)
{
    // solver 등록 (단순 head 삽입)
    solver = solver_;
    next = solver->bodies;
    solver->bodies = this;

    // 기하/재질
    size = size_;          // ← Rigid에 size 필드가 있어야 합니다
    friction = friction_;      // ← Rigid에 friction 필드가 있어야 합니다

    // 질량 및 경계 반경
    mass = size.x * size.y * size.z * density;     // 균질 박스
    radius = length3({ size.x * 0.5f, size.y * 0.5f, size.z * 0.5f }); // AABB 반대각선의 절반

    // 박스 관성(중심, 로컬 축 기준; 대각 가정)
    const float xx = size.x, yy = size.y, zz = size.z;
    const float Ix = mass * (yy * yy + zz * zz) / 12.0f;
    const float Iy = mass * (xx * xx + zz * zz) / 12.0f;
    const float Iz = mass * (xx * xx + yy * yy) / 12.0f;
    Ibody = { Ix, Iy, Iz };

    // 초기 상태
    x = position;
    v = linearVelocity;
    w = angularVelocity;

    q = quatFromEulerXYZ(rotation);   // 자세
    q = qnorm(q);                     // 수치 안전

    // 월드 관성 (Iworld = R * Ibody * R^T)
    const float3x3 Rm = q.R();
    const float3x3 D = diagonal(Ix, Iy, Iz);
    Iworld = Rm * D * transpose3x3(Rm);

    // 웜스타트/관성 상태
    x_initial = x;    q_initial = q;
    x_inertial = x;   q_inertial = q;

    v_prev = v;               // ← Rigid에 prevVelocity가 있다면
    // prevAngularVelocity 같은 게 있으면 여기서 w로 세팅
    // forces 헤드 초기화가 필요하면:
    // forces = nullptr;
}

Rigid::~Rigid()
{
    // Remove from linked list
    Rigid** p = &solver->bodies;
    while (*p != this)
        p = &(*p)->next;
    *p = next;
}

bool Rigid::constrainedTo(Rigid* other) const
{
    for (Force* f = forces; f != 0; f = (f->bodyA == this) ? f->nextA : f->nextB)
        if ((f->bodyA == this && f->bodyB == other) || (f->bodyA == other && f->bodyB == this))
            return true;
    return false;
}


void Rigid::draw()
{
    const float hx = size.x * 0.5f;
    const float hy = size.y * 0.5f;
    const float hz = size.z * 0.5f;

    const float3x3 Rm = R();
    auto X = [&](float3 rl)->float3 { return x + (Rm * rl); };

    // 8 corners (local → world)
    float3 p[8] = {
        X({-hx,-hy,-hz}), X({+hx,-hy,-hz}), X({+hx,+hy,-hz}), X({-hx,+hy,-hz}),
        X({-hx,-hy,+hz}), X({+hx,-hy,+hz}), X({+hx,+hy,+hz}), X({-hx,+hy,+hz})
    };

    static const int edges[12][2] = {
        {0,1},{1,2},{2,3},{3,0}, // bottom
        {4,5},{5,6},{6,7},{7,4}, // top
        {0,4},{1,5},{2,6},{3,7}  // pillars
    };

    glColor3f(0.15f, 0.2f, 0.85f);
    glBegin(GL_LINES);
    for (int e = 0; e < 12; ++e) {
        const int a = edges[e][0], b = edges[e][1];
        glVertex3f(p[a].x, p[a].y, p[a].z);
        glVertex3f(p[b].x, p[b].y, p[b].z);
    }
    glEnd();
}
