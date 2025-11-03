#pragma once
#include "solver.h"
#include "util/random.h"

static void sceneEmpty(Solver* solver)
{
    solver->clear();
}

static void sceneGround(Solver* solver)
{
    // create ground plane (large flat box)
    Rigid* ground = new Rigid(solver, { 15, 0.25f, 15 }, -1.0f, 0.5f, { 0, -1.0f, 0 });
	ground->rigidType = RIGID_GROUND;
}

static void sceneBox(Solver* solver)
{
    float diff = 0.5f;
    for (int i = 0; i < 5; ++i) 
    {
        new Rigid(solver, vec3(uniform(0.5f, 1.0f), uniform(0.5f, 1.0f), uniform(0.5f, 1.0f)), 10.0f, 1.0f, vec3(0.0f, 5.5f + i * 0.5 - 0.5, 0.0f) + vec3(uniform(-diff, diff), uniform(-diff, diff), uniform(-diff, diff)), quat(uniform(-diff, diff), uniform(-diff, diff), uniform(-diff, diff), uniform(-diff, diff)), vec6());
    }

    solver->useCollision = true;
    solver->useSelfCollision = false;
}

static void sceneBoxStack(Solver* solver)
{
    for (int i = 0; i < 7; ++i) 
    {
        new Rigid(solver, vec3(0.5f), 15.0f, 0.4f, vec3(0.0f, i * 0.5 - 0.5, 0.0f), quat(1, 0, 0, 0), vec6());
    }

    solver->useCollision = true;
    solver->useSelfCollision = false;
}

static void sceneRope(Solver* solver)
{
    // ====== 가로 막대 N개 체인 ======
    const int   N = 10;                     // <- 원하는 개수
    const float linkLen = 0.5f;             // X방향 길이
    const float thick = 0.12f;              // Y,Z 두께
    const vec3  size = vec3(linkLen, thick, thick);
    const float density = 100.0f;
    const float friction = 0.4f;

    // 힌지 강성 (발산하면 posK↓, swingK↓ 또는 iterations↑)
    const vec3  posK = vec3(3e5f);          // 위치 3행
    const float swingK = 10.0f;             // 스윙 2행(힌지 제약)
	const float twistK = 0.52f * swingK;    // 트위스트 1행(힌지 제약)
    const float frac = INFINITY;

    // 힌지 축/기준: Z축 힌지면 XY평면으로 흔들림
    const vec3 axisW(1, 0, 0);              // 회전축(힌지)
    const vec3 refW(0, 0, 1);               // 스윙 평면 기준축

    // 월드 피벗(천장 고정점)
    const vec3 pivotW = vec3(0.0f, 5.0f, 0.0f);

    // 로컬 앵커들: 오른쪽/왼쪽 끝
    const vec3 r_right(+0.5f * linkLen, 0, 0);
    const vec3 r_left(-0.5f * linkLen, 0, 0);

    // 링크 생성
    std::vector<Rigid*> links; links.reserve(N);
    for (int i = 0; i < N; ++i) {
        auto* L = new Rigid(solver, size, density, friction, pivotW - vec3(0.5f * linkLen + i * linkLen, 0, 0), quat(1, 0, 0, 0), vec6());
        // 보기 좋은 컬러 그라데이션
        float t = (N > 1) ? float(i) / float(N - 1) : 0.0f;
        L->color = vec4(0.85f * (1 - t) + 0.30f * t, 0.45f * (1 - t) + 0.70f * t, 0.45f, 0.75f);
		L->rigidType = RIGID_ROPE;
        links.push_back(L);
    }

    // 1) 첫 막대를 월드 피벗에 걸기: L0의 +X 끝이 pivotW에 오도록 배치
    //links[0]->position = pivotW - links[0]->rotation * r_right;
    if (true)
    {
        new Joint(solver,
            /*A*/ nullptr, /*B*/ links[0],
            /*rA*/ pivotW, /*rB*/ r_right,
            /*axisA*/ axisW, /*axisB*/ axisW,
            /*refA*/  refW,  /*refB*/  refW,
            /*posK*/  posK,  /*swingK*/ 1.0f,
            /*frac*/  frac,
            /*twistK*/ twistK);
    }


    // 2) 나머지 막대들을 체인으로 연결: A(left) <-> B(right)
    for (int i = 1; i < N; ++i) {
        Rigid* A = links[i - 1];
        Rigid* B = links[i];

        // A의 왼쪽 끝 월드 위치(힌지 점)
        //vec3 hingeW = A->position + A->rotation * r_left;

        // B의 오른쪽 끝이 그 힌지 점에 오도록 배치
        //B->position = hingeW - B->rotation * r_right;

        // 힌지 조인트 생성

        if (true)
        {
            new Joint(solver,
                /*A*/ A, /*B*/ B,
                /*rA*/ r_left, /*rB*/ r_right,
                /*axisA*/ axisW, /*axisB*/ axisW,
                /*refA*/  refW,  /*refB*/  refW,
                /*posK*/  posK,  /*swingK*/ swingK,
                /*frac*/  frac,
                /*twistK*/ twistK);
        }


    }

    // 초기 속도 0
    for (auto* L : links) L->velocity = vec6(0);

    // (선택) 더 단단하게 조이려면 반복 수 늘리기
    solver->iterations = 10;  // 30~60 권장

    solver->useCollision = false;
    solver->useSelfCollision = false;
}

static void scenePyramid(Solver* solver)
{
    // Pyramid parameters
    const int baseCount = 4;         // 밑면 박스 개수 (한 변)
    const float boxSize = 0.5f;      // 한 박스의 한 변 길이
    const float gap = 0.02f;         // 박스 사이 작은 간격
    const vec3 boxScale = vec3(boxSize); // 박스 크기
    const float density = 10.0f;
    const float friction = 0.4f;

    // 층 수 = baseCount
    for (int layer = 0; layer < baseCount; ++layer) {
        int count = baseCount - layer; // 현재 층의 한 변당 박스 수
        if (count <= 0) break;

        // 높이: 바닥 위로 층 쌓기 (각 박스의 중심 y 좌표)
        float y = (boxSize * 0.5f) + layer * (boxSize + gap);

        // 한 변의 전체 길이 (간격 포함)
        float span = count * boxSize + (count - 1) * gap;

        // 가운데 정렬을 위한 오프셋
        float originOffset = (span - boxSize) * 0.5f;

        for (int i = 0; i < count; ++i) {
            for (int j = 0; j < count; ++j) {
                float x = (i * (boxSize + gap)) - originOffset;
                float z = (j * (boxSize + gap)) - originOffset;

                // 살짝 랜덤한 위치 보정(너무 불안정하면 0으로 설정)
                float rx = uniform(-0.01f, 0.01f);
                float rz = uniform(-0.01f, 0.01f);

                // 색상: 층에 따라 그라데이션
                float t = (baseCount > 1) ? float(layer) / float(baseCount - 1) : 0.0f;
                vec4 color = vec4(0.6f * (1.0f - t) + 0.2f * t, 0.5f * (1.0f - t) + 0.8f * t, 0.4f + 0.1f * t, 1.0f);

                // 생성 (rotation 기본, velocity 기본, color 전달)
                new Rigid(solver, boxScale, density, friction, vec3(x + rx, y, z + rz), quat(1, 0, 0, 0), vec6(), color);
            }
        }
    }

    // 추천: 안정성을 위해 반복 수 약간 증가
    solver->iterations = 12;

    solver->useCollision = true;
    solver->useSelfCollision = false;
}

static void sceneRopeAndPyramid(Solver* solver)
{
    sceneRope(solver);
	scenePyramid(solver);

    solver->useCollision = true;
    solver->useSelfCollision = false;
}

static void sceneNet(Solver* solver)
{
    // ===== 파라미터 =====
    const int   ROWS = 4;              // 세로 노드 수
    const int   COLS = 4;              // 가로 노드 수
    const float cell = 0.6f;            // 노드 간격(격자 간격)
    const float thick = 0.06f;           // 바의 두께 (Y,Z 방향)
    const float density = 80.0f;           // 바 밀도
    const float friction = 0.45f;           // 마찰
    const float y0 = 4.5f;            // 초기 높이

    // 물리 강성(조인트)
    const vec3  posK = vec3(3e5f);      // 위치 3행 (빡빡)
    const float swingK = 100.0f;           // 스윙 2행 (볼조인트 느낌이면 0~작게)
    const float twistK = 50.0f;           // 트위스트 1행 (볼조인트면 0)
    const float frac = INFINITY;       // 파단 X

    // 힌지용 축/기준 (큰 의미는 없음; 회전 자유로 둘 거면 0이라 축 영향 적음)
    const vec3 axisW(0, 1, 0);
    const vec3 refW(0, 0, 1);

    const vec3 axisH(1, 0, 0);
	const vec3 refH(0, 1, 0);
	const vec3 axisV(0, 0, 1);
	const vec3 refV(1, 0, 0);

    // 수평/수직 바의 로컬 끝점
    const vec3 h_left = vec3(-0.5f * cell, 0, 0);
    const vec3 h_right = vec3(+0.5f * cell, 0, 0);
    const vec3 v_top = vec3(0, 0, -0.5f * cell);
    const vec3 v_bot = vec3(0, 0, +0.5f * cell);

    // 바 크기
    const vec3 sizeH(cell, thick, thick);   // 수평 바 (X 방향 길이)
    const vec3 sizeV(thick, thick, cell);   // 수직 바 (Z 방향 길이)

    // 격자 중앙 정렬
    const float x0 = -(COLS - 1) * cell * 0.5f;
    const float z0 = -(ROWS - 1) * cell * 0.5f;

    // ===== 바 생성: H(수평), V(수직) =====
    std::vector<std::vector<Rigid*>> H(ROWS, std::vector<Rigid*>(COLS - 1, nullptr));
    std::vector<std::vector<Rigid*>> V(ROWS - 1, std::vector<Rigid*>(COLS, nullptr));

    // 수평 바
    for (int r = 0; r < ROWS; ++r) {
        for (int c = 0; c < COLS - 1; ++c) {
            const float cx = x0 + (c + 0.5f) * cell;
            const float cz = z0 + r * cell;
            auto* bar = new Rigid(solver, sizeH, density, friction,
                vec3(cx, y0, cz), quat(1, 0, 0, 0), vec6());
            // 컬러 그라데이션 (가독)
            float t = (ROWS > 1 ? float(r) / (ROWS - 1) : 0.0f);
            bar->color = vec4(0.25f + 0.50f * t, 0.65f - 0.25f * t, 0.85f - 0.25f * t, 0.9f);
			bar->rigidType = RIGID_NET;
            H[r][c] = bar;
        }
    }

    // 수직 바
    for (int r = 0; r < ROWS - 1; ++r) {
        for (int c = 0; c < COLS; ++c) {
            const float cx = x0 + c * cell;
            const float cz = z0 + (r + 0.5f) * cell;
            auto* bar = new Rigid(solver, sizeV, density, friction,
                vec3(cx, y0, cz), quat(1, 0, 0, 0), vec6());
            float t = (COLS > 1 ? float(c) / (COLS - 1) : 0.0f);
            bar->color = vec4(0.85f - 0.55f * t, 0.45f + 0.40f * t, 0.45f, 0.9f);
            bar->rigidType = RIGID_NET;
            V[r][c] = bar;
        }
    }

	// ===== 같은 노드를 공유하는 바들을 ignoreCollision 처리 =====
    // === [IGNORE LIST 구축] : 같은 노드를 공유하는 모든 bar 쌍을 서로 ignore ===
    auto addIgnorePair = [](Rigid* a, Rigid* b) {
        if (!a || !b || a == b) return;
        auto& va = a->ignoreCollisionList;
        auto& vb = b->ignoreCollisionList;
        if (std::find(va.begin(), va.end(), b) == va.end()) va.push_back(b);
        if (std::find(vb.begin(), vb.end(), a) == vb.end()) vb.push_back(a);
        };

    // node(r,c)에 접속하는 bar 모으기:
    //  - 수평: H[r][c]의 left, H[r][c-1]의 right
    //  - 수직: V[r][c]의 top,  V[r-1][c]의 bottom
    std::vector<Rigid*> nodeBars;
    nodeBars.reserve(4);

    auto gatherBarsAtNode = [&](int r, int c) {
        nodeBars.clear();
        if (c < COLS - 1 && H[r][c])       nodeBars.push_back(H[r][c]);     // left end
        if (c > 0 && H[r][c - 1])           nodeBars.push_back(H[r][c - 1]);   // right end
        if (r < ROWS - 1 && V[r][c])      nodeBars.push_back(V[r][c]);     // top end
        if (r > 0 && V[r - 1][c])           nodeBars.push_back(V[r - 1][c]);   // bottom end
        };

    for (int r = 0; r < ROWS; ++r) {
        for (int c = 0; c < COLS; ++c) {
            gatherBarsAtNode(r, c);
            // 같은 노드에 연결된 모든 bar를 서로 ignore
            for (size_t i = 0; i < nodeBars.size(); ++i) {
                for (size_t j = i + 1; j < nodeBars.size(); ++j) {
                    addIgnorePair(nodeBars[i], nodeBars[j]);
                }
            }
        }
    }

    // ===== 같은 행/열 바들 end-to-end 힌지 체인으로 연결 =====
    // (수평 체인) H[r][c-1].right  <->  H[r][c].left
    for (int r = 0; r < ROWS; ++r) {
        for (int c = 1; c < COLS - 1; ++c) {
            new Joint(solver,
                /*A*/ H[r][c - 1], /*B*/ H[r][c],
                /*rA*/ h_right,  /*rB*/ h_left,
                /*axisA*/ axisH, /*axisB*/ axisH,
                /*refA*/  refH,  /*refB*/  refH,
                /*posK*/  posK,  /*swingK*/ swingK,
                /*frac*/  frac,  /*twistK*/ twistK);
        }
    }

    // (수직 체인) V[r-1][c].bot  <->  V[r][c].top
    for (int r = 1; r < ROWS - 1; ++r) {
        for (int c = 0; c < COLS; ++c) {
            new Joint(solver,
                /*A*/ V[r - 1][c], /*B*/ V[r][c],
                /*rA*/ v_bot,    /*rB*/ v_top,
                /*axisA*/ axisV, /*axisB*/ axisV,
                /*refA*/  refV,  /*refB*/  refV,
                /*posK*/  posK,  /*swingK*/ swingK,
                /*frac*/  frac,  /*twistK*/ twistK);
        }
    }

    // ===== 교차점(H/V) 묶기: 각 노드에서 수평/수직 바의 끝점을 하나로 결속 =====
    // node(r,c)에서 사용할 대표 수평/수직 바와 그 끝점을 고르고, 서로 Joint로 묶는다.
    auto bindNodeHV = [&](int r, int c) {
        // 대표 수평바 선택 (우선 오른쪽 바의 left, 없으면 왼쪽 바의 right)
        Rigid* hBar = nullptr; vec3 hEnd;
        if (c < COLS - 1 && H[r][c]) { hBar = H[r][c];      hEnd = h_left; }
        else if (c > 0 && H[r][c - 1]) { hBar = H[r][c - 1];    hEnd = h_right; }
        else return; // 수평바 없음

        // 대표 수직바 선택 (우선 아래쪽 바의 top, 없으면 위쪽 바의 bot)
        Rigid* vBar = nullptr; vec3 vEnd;
        if (r < ROWS - 1 && V[r][c]) { vBar = V[r][c];      vEnd = v_top; }
        else if (r > 0 && V[r - 1][c]) { vBar = V[r - 1][c];    vEnd = v_bot; }
        else return; // 수직바 없음

        // 두 끝점이 같은 월드 위치에 오도록 포지션 3행 중심으로 묶기
        new Joint(solver,
            /*A*/ hBar, /*B*/ vBar,
            /*rA*/ hEnd, /*rB*/ vEnd,
            /*axisA*/ axisH, /*axisB*/ axisV,
            /*refA*/  refH,  /*refB*/  refV,
            /*posK*/  posK,  /*swingK*/ 0.0f,
            /*frac*/  frac,  /*twistK*/ 100.0f);
        };

    for (int r = 0; r < ROWS; ++r) {
        for (int c = 0; c < COLS; ++c) {
            bindNodeHV(r, c);
        }
    }

    // ===== 네 귀퉁이 공중 고정 =====
    auto pinNode = [&](int r, int c) {
        // node(r,c)의 위치(월드)
        const vec3 pivotW = vec3(x0 + c * cell, y0, z0 + r * cell);

        // 이 노드에서 쓸 대표 바/끝점을 재활용 (bind와 동일 로직)
        Rigid* bar = nullptr; vec3 rEnd;
        if (c < COLS - 1 && H[r][c]) { bar = H[r][c];   rEnd = h_left; }
        else if (c > 0 && H[r][c - 1]) { bar = H[r][c - 1]; rEnd = h_right; }
        else if (r < ROWS - 1 && V[r][c]) { bar = V[r][c];   rEnd = v_top; }
        else if (r > 0 && V[r - 1][c]) { bar = V[r - 1][c]; rEnd = v_bot; }
        else return;

        // 월드 고정: A=nullptr, rA=pivotW(월드 좌표), B=bar, rB=rEnd(로컬)
        new Joint(solver,
            /*A*/ nullptr, /*B*/ bar,
            /*rA*/ pivotW, /*rB*/ rEnd,
            /*axisA*/ axisW, /*axisB*/ axisW,
            /*refA*/  refW,  /*refB*/  refW,
            /*posK*/  posK,  /*swingK*/ 1.0f,   // 살짝 회전 억제(옵션)
            /*frac*/  frac,  /*twistK*/ 0.5f);
        };

    pinNode(0, 0);
    pinNode(0, COLS - 1);
    pinNode(ROWS - 1, 0);
    pinNode(ROWS - 1, COLS - 1);

    // 초기 속도 0
    for (int r = 0; r < ROWS; ++r)
        for (int c = 0; c < COLS - 1; ++c)
            if (H[r][c]) H[r][c]->velocity = vec6(0);
    for (int r = 0; r < ROWS - 1; ++r)
        for (int c = 0; c < COLS; ++c)
            if (V[r][c]) V[r][c]->velocity = vec6(0);

    // 수렴성 보완
    solver->iterations = 10;   // 20~40 권장
    solver->useCollision = true;
    solver->useSelfCollision = true;
}

static void sceneNetAndBoxes(Solver* solver)
{
    sceneBox(solver);
	sceneNet(solver);

    // 수렴성 보완
    solver->iterations = 10;   // 20~40 권장
    solver->useCollision = true;
    solver->useSelfCollision = true;
}
