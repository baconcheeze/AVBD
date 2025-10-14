#pragma once
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <numeric>

#include "solver.h"

// bodiesHead : solver가 가진 바디 단일 연결리스트(head)
// buckets    : 색깔별 바디 묶음 (출력)
// colorOf    : 바디 포인터 -> 색 인덱스 (출력)
// return     : 색 개수
int buildBodyColors(Rigid* bodiesHead,
    std::vector<std::vector<Rigid*>>& buckets,
    std::unordered_map<Rigid*, int>& colorOf)
{
    // 1) 동적 바디 수집
    std::vector<Rigid*> nodes;
    nodes.reserve(256);
    for (Rigid* b = bodiesHead; b; b = b->next) {
        if (b->mass > 0) nodes.push_back(b); // 정적/키네매틱은 스킵
    }
    const int n = (int)nodes.size();
    if (n == 0) { buckets.clear(); colorOf.clear(); return 0; }

    // 포인터 -> 인덱스 매핑
    std::unordered_map<Rigid*, int> idx;
    idx.reserve(n * 2);
    for (int i = 0; i < n; ++i) idx[nodes[i]] = i;

    // 2) 인접 리스트 생성(간선: Force가 양쪽 바디를 연결하는 경우)
    std::vector<std::vector<int>> adj(n);
    for (int i = 0; i < n; ++i) {
        Rigid* bi = nodes[i];
        for (Force* f = bi->forces; f; f = (f->bodyA == bi) ? f->nextA : f->nextB) {
            Rigid* A = f->bodyA;
            Rigid* B = f->bodyB;
            // 두 바디 모두 존재하고 동적일 때만 간선
            if (!A || !B) continue;
            if (A->mass <= 0 || B->mass <= 0) continue;

            // bi의 반대편 찾기
            Rigid* bj = (A == bi) ? B : A;
            auto it = idx.find(bj);
            if (it == idx.end()) continue; // 동적집합에 없으면 스킵

            int j = it->second;
            if (i == j) continue;
            adj[i].push_back(j);
            adj[j].push_back(i);
        }
    }
    // 중복 제거
    for (int i = 0; i < n; ++i) {
        auto& v = adj[i];
        std::sort(v.begin(), v.end());
        v.erase(std::unique(v.begin(), v.end()), v.end());
    }

    // 3) 차수 내림차순 순서로 그리디 컬러링
    std::vector<int> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
        [&](int a, int b) { return adj[a].size() > adj[b].size(); });

    std::vector<int> color(n, -1);
    int maxColor = -1;
    std::vector<char> used; // 인접 색 사용표

    for (int k = 0; k < n; ++k) {
        int u = order[k];
        used.assign(static_cast<char>(maxColor + 2), 0);
        for (int v : adj[u]) if (color[v] >= 0) {
            if (color[v] < (int)used.size()) used[color[v]] = 1;
        }
        int c = 0;
        while (c < (int)used.size() && used[c]) ++c;
        color[u] = c;
        if (c > maxColor) maxColor = c;
    }

    // 4) 결과 묶음 & 맵 채우기
    buckets.assign(static_cast<char>(maxColor + 1), {});
    buckets.shrink_to_fit();
    colorOf.clear(); colorOf.reserve(n * 2);

    for (int i = 0; i < n; ++i) {
        buckets[color[i]].push_back(nodes[i]);
        colorOf[nodes[i]] = color[i];
    }

    return maxColor + 1;
}
