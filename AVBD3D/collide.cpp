#pragma once

#include "solver.h"


// ===================== 3D OBB-OBB SAT + Face Clipping + Edge-Edge =====================

// 박스의 월드축(열벡터)
static inline void getAxes(const float3x3& R, float3 ax[3]) {
    ax[0] = { R.m[0][0], R.m[1][0], R.m[2][0] };
    ax[1] = { R.m[0][1], R.m[1][1], R.m[2][1] };
    ax[2] = { R.m[0][2], R.m[1][2], R.m[2][2] };
}

// 축 L에 대한 박스 반폭 투영값: sum_i h[i] * |a_i·L|
static inline float projectExtentOnAxis(const float3 h, const float3 a[3], const float3& L) {
    return h.x * std::fabs(dot(a[0], L))
        + h.y * std::fabs(dot(a[1], L))
        + h.z * std::fabs(dot(a[2], L));
}

// 선분-평면 클립(Sutherland–Hodgman 한 스텝)
static int clipPolygonAgainstPlane(const Plane3& P, const float3* vin, int nIn, float3* vout) {
    int nOut = 0;
    if (nIn <= 0) return 0;
    float3 prev = vin[nIn - 1];
    float  prevDist = dot(P.n, prev) - P.d;

    for (int i = 0; i < nIn; i++) {
        float3 curr = vin[i];
        float  currDist = dot(P.n, curr) - P.d;

        bool prevIn = (prevDist <= 0.0f);
        bool currIn = (currDist <= 0.0f);

        if (prevIn && currIn) {
            // keep current
            vout[nOut++] = curr;
        }
        else if (prevIn && !currIn) {
            // leaving: add intersection
            float t = prevDist / (prevDist - currDist);
            vout[nOut++] = prev + (curr - prev) * t;
        }
        else if (!prevIn && currIn) {
            // entering: add intersection + current
            float t = prevDist / (prevDist - currDist);
            vout[nOut++] = prev + (curr - prev) * t;
            vout[nOut++] = curr;
        }
        prev = curr; prevDist = currDist;
    }
    return nOut;
}

// incident face(사각형) 가져오기
// box B의 로컬축 b[3], 반폭 hb, 중심 cB, 법선 nRef에 가장 반대방향인 면을 고름
// outV[4]에 월드 정점들 채움(시계/반시계 무관)
static void buildIncidentFace(const float3 cB, const float3 b[3], const float3 hb,
    const float3& nRef, float3 outV[4], int& faceIdxOut)
{
    // nRef와 가장 anti-parallel인 축 index
    float dots[3] = { dot(b[0], nRef), dot(b[1], nRef), dot(b[2], nRef) };
    int   m = 0; if (std::fabs(dots[1]) > std::fabs(dots[m])) m = 1;
    if (std::fabs(dots[2]) > std::fabs(dots[m])) m = 2;

    float s = (dots[m] > 0.0f) ? -1.0f : +1.0f; // anti-parallel
    faceIdxOut = m;

    // sat_obb.h (incident face 만들 때)
    int iu = (m + 1) % 3;
    int iv = (m + 2) % 3;
    float3 fc = cB + b[m] * (s * (&hb.x)[m]);
    float3 u = b[iu], v = b[iv];
    float  hu = (&hb.x)[iu], hv = (&hb.x)[iv];


    // 사각형 4점
    outV[0] = fc + u * hu + v * hv;
    outV[1] = fc - u * hu + v * hv;
    outV[2] = fc - u * hu - v * hv;
    outV[3] = fc + u * hu - v * hv;
}

// 참조면 사변형 영역을 정의하는 4개 side plane (inside: n·p <= d)
// ref 박스 중심 cR, 축 r[3], 반폭 hR, 참조 face index k
static void buildReferenceSidePlanes(const float3 cR, const float3 r[3], const float3 hR, int k,
    Plane3 side[4])
{
    // k면을 제외한 두 축(u,v)에 대한 ± plane
    int iu = (k + 1) % 3;
    int iv = (k + 2) % 3;

    // u+:  r[iu]·p <= r[iu]·cR + hR[iu]
    side[0].n = r[iu]; side[0].d = dot(side[0].n, cR) + (&hR.x)[iu];
    // u-: -r[iu]·p <= -r[iu]·cR + hR[iu]
    side[1].n = -r[iu]; side[1].d = dot(side[1].n, cR) + (&hR.x)[iu];

    // v+:  r[iv]·p <= r[iv]·cR + hR[iv]
    side[2].n = r[iv]; side[2].d = dot(side[2].n, cR) + (&hR.x)[iv];
    // v-: -r[iv]·p <= -r[iv]·cR + hR[iv]
    side[3].n = -r[iv]; side[3].d = dot(side[3].n, cR) + (&hR.x)[iv];
}

// 다각형을 4개의 side plane으로 클립
static int clipPolygonToRefRect(const Plane3 side[4], const float3* polyIn, int nIn, float3* out)
{
    float3 tmp1[16], tmp2[16];
    int n = nIn;
    for (int i = 0; i < nIn; i++) tmp1[i] = polyIn[i];

    int nTmp = n;
    float3* vin = tmp1;
    float3* vout = tmp2;

    for (int s = 0; s < 4; s++) {
        nTmp = clipPolygonAgainstPlane(side[s], vin, n, vout);
        if (nTmp == 0) return 0;
        // swap buffers
        n = nTmp;
        float3* t = vin; vin = vout; vout = t;
    }
    for (int i = 0; i < n; i++) out[i] = vin[i];
    return n;
}

// 가장 작은 겹침축을 찾는다(분리 있으면 음수 반환)
// 반환: bestType: 0..2(A face k), 3..5(B face k-3), 6..14(edge i,j: i*3+j + 6)
// bestAxis: unit axis, overlap depth(+)와 sign(법선이 A→B가 되도록)
static float findBestAxisOBB(const float3 cA, const float3 a[3], const float3 hA,
    const float3 cB, const float3 b[3], const float3 hB,
    int& bestType, int& bestI, int& bestJ, float3& bestAxis)
{
    const float3 t = { cB.x - cA.x, cB.y - cA.y, cB.z - cA.z };

    float minSep = INFINITY;
    bestType = -1; bestI = -1; bestJ = -1; bestAxis = { 0,0,0 };

    auto testAxis = [&](const float3& L, int type, int i, int j) {
        float3 n = L;
        float nLen = length3(n);
        if (nLen < 1e-9f) return;   // ill-defined(평행축)
        n = { n.x / nLen, n.y / nLen, n.z / nLen };

        float ra = projectExtentOnAxis(hA, a, n);
        float rb = projectExtentOnAxis(hB, b, n);
        float dist = std::fabs(dot(t, n));
        float sep = (ra + rb) - dist;   // overlap depth
        if (sep < minSep) {
            minSep = sep;
            bestType = type; bestI = i; bestJ = j;
            // n은 A에서 B로 향하도록
            bestAxis = (dot(t, n) >= 0.0f) ? n : float3{ -n.x, -n.y, -n.z };
        }
    };

    // 3 face axes of A
    testAxis(a[0], 0, 0, -1);
    testAxis(a[1], 1, 1, -1);
    testAxis(a[2], 2, 2, -1);

    // 3 face axes of B
    testAxis(b[0], 3, 0, -1);
    testAxis(b[1], 4, 1, -1);
    testAxis(b[2], 5, 2, -1);

    // 9 edge-edge axes (ai × bj)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float3 L = cross(a[i], b[j]);
            testAxis(L, 6 + i * 3 + j, i, j);
        }
    }

    return minSep; // <0이면 분리
}
int Manifold::collide(Rigid* A, Rigid* B, Contact* out, int maxOut)
{
    if (maxOut <= 0) return 0;

    const float3x3 RA = A->R();
    const float3x3 RB = B->R();

    float3 a[3], b[3];
    getAxes(RA, a);
    getAxes(RB, b);

    const float3 hA = A->size * 0.5f;
    const float3 hB = B->size * 0.5f;
    const float3 cA = A->x;
    const float3 cB = B->x;

    int bestType = -1, iBest = -1, jBest = -1;
    float3 nBest{ 0,0,0 };
    float sep = findBestAxisOBB(cA, a, hA, cB, b, hB, bestType, iBest, jBest, nBest);
    if (sep < 0.0f) return 0; // 분리 상태

    // ---------------- Face-클리핑 경로 ----------------
    if (bestType >= 0 && bestType <= 5)
    {
        const bool   refIsA = (bestType <= 2);
        const int    k = refIsA ? bestType : (bestType - 3);

        // 참조/incident 박스 데이터
        const float3  cR = refIsA ? cA : cB;
        const float3* r = refIsA ? a : b;
        const float3  hR = refIsA ? hA : hB;

        const float3  cI = refIsA ? cB : cA;
        const float3* ii = refIsA ? b : a;
        const float3  hI = refIsA ? hB : hA;

        // 참조면 법선 (이미 A→B 방향)
        const float3 nRef = nBest;

        // 참조 face의 전면 평면값 dFront (inside: nRef·p <= dFront)
        const float sgn = (dot(nRef, r[k]) >= 0.0f ? +1.0f : -1.0f);
        const float dFront = dot(nRef, cR + r[k] * ((&hR.x)[k] * sgn));

        // incident face 사각형
        float3 incVerts[4]; int incFace = -1;
        buildIncidentFace(cI, ii, hI, nRef, incVerts, incFace);

        // 참조 사변형의 사이드 4평면으로 클립
        Plane3 side[4];
        buildReferenceSidePlanes(cR, r, hR, k, side);
        float3 clipped[16];
        int nClip = clipPolygonToRefRect(side, incVerts, 4, clipped);
        if (nClip <= 0) return 0;

        // 후보 점들 필터링: 참조면 앞(관통)만, 깊이 큰 순으로 정렬 후 상위 maxOut만 채우기
        struct Cand { float3 p; float pen; int idx; };
        Cand cand[16]; int nc = 0;
        for (int i = 0; i < nClip; ++i) {
            float s = dot(nRef, clipped[i]) - dFront; // >0이면 면 앞(관통)
            if (s <= 0.0f) {
                // 면 위로 정사영한 점 (Box2D-lite 방식)
                float3 pRef = clipped[i] - nRef * s;
                cand[nc++] = { pRef, -s, i };
            }
        }
        if (nc == 0) return 0;

        std::sort(cand, cand + nc, [](const Cand& A, const Cand& B) { return A.pen > B.pen; });
        const int nOut = std::min(nc, maxOut);

        // 접촉점 기록
        for (int i = 0; i < nOut; ++i) {
            const float3 pW = cand[i].p;      // pRef(참조면 위의 점)
            const float3x3 RAT = transpose3x3(RA);
            const float3x3 RBT = transpose3x3(RB);

            out[i].n = nRef;                 // 항상 A→B
            out[i].rA = RAT * (pW - cA);      // 둘 다 같은 pW를 로컬로 변환
            out[i].rB = RBT * (pW - cB);

            // feature id (간단 태깅)
            uint64_t fid = 0;
            fid |= (uint64_t)(refIsA ? 1 : 2) << 60;
            fid |= (uint64_t)(k & 0x7) << 56;
            fid |= (uint64_t)(incFace & 0x7) << 52;
            fid |= (uint64_t)(cand[i].idx & 0xFF);
            out[i].feature.value = fid;
        }
        return nOut;
    }

    // ---------------- Edge-Edge 경로 ----------------
    // bestType = 6 + ia*3 + jb
    const int ia = iBest;
    const int jb = jBest;

    const float3 ea = a[ia]; // 단위
    const float3 eb = b[jb]; // 단위

    // 두 보조축 인덱스
    const int au = (ia + 1) % 3, av = (ia + 2) % 3;
    const int bu = (jb + 1) % 3, bv = (jb + 2) % 3;

    // 법선은 SAT에서 얻은 A→B 방향
    const float3 n = nBest;

    // 에지 선택 부호 (면을 마주보는 가장 가까운 에지)
    const float suA = signf(dot(n, a[au]));
    const float svA = signf(dot(n, a[av]));
    const float suB = -signf(dot(n, b[bu]));
    const float svB = -signf(dot(n, b[bv]));

    // 각 에지의 중심점
    const float3 pA0 = cA + a[au] * (suA * (&hA.x)[au]) + a[av] * (svA * (&hA.x)[av]);
    const float3 pB0 = cB + b[bu] * (suB * (&hB.x)[bu]) + b[bv] * (svB * (&hB.x)[bv]);

    // 두 에지(무한선) 최근접 매개변수
    const float3 r = pB0 - pA0;
    const float Aee = 1.0f;               // |ea|=1
    const float Bee = dot(ea, eb);
    const float Cee = 1.0f;               // |eb|=1
    const float Dee = dot(ea, r);
    const float Eee = dot(eb, r);
    const float denom = Aee * Cee - Bee * Bee;

    float s = 0.0f, t = 0.0f;
    if (std::fabs(denom) > 1e-8f) {
        s = (Bee * Eee - Cee * Dee) / denom;
        t = (Aee * Eee - Bee * Dee) / denom;
    }

    // 세그먼트 길이로 클램프
    s = std::clamp(s, -(&hA.x)[ia], +(&hA.x)[ia]);
    t = std::clamp(t, -(&hB.x)[jb], +(&hB.x)[jb]);

    const float3 pA = pA0 + ea * s;
    const float3 pB = pB0 + eb * t;

    const float3x3 RAT = transpose3x3(RA);
    const float3x3 RBT = transpose3x3(RB);

    const float3 pc = (pA + pB) * 0.5f;
    out[0].n = n;
    out[0].rA = RAT * (pc - cA);
    out[0].rB = RBT * (pc - cB);

    uint64_t fid = 0;
    fid |= (uint64_t)3 << 60; // edge-edge
    fid |= (uint64_t)(ia & 0x7) << 56;
    fid |= (uint64_t)(jb & 0x7) << 52;
    out[0].feature.value = fid;

    return (maxOut > 0) ? 1 : 0;
}

