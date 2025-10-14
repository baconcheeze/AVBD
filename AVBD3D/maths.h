// maths.h
#pragma once
#include <cmath>
#include <algorithm> // std::min, std::max, std::clamp

// ===== Constants =====
constexpr float PI = 3.14159265358979323846f;
constexpr float TAU = 6.2831853071795864769f;

// ===== float3 =====
struct float3 {
    float x{ 0 }, y{ 0 }, z{ 0 };

    // Accessors
    inline float  v(int i) const { return (i == 0) ? x : (i == 1) ? y : z; }
    inline float& operator[](int i) { return (i == 0) ? x : (i == 1) ? y : z; }
    inline float  operator[](int i) const { return (i == 0) ? x : (i == 1) ? y : z; }
};

// ---- basic ops ----
inline float3 operator+(float3 a, float3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
inline float3 operator-(float3 a, float3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
inline float3 operator-(float3 v) { return { -v.x, -v.y, -v.z }; }
inline float3 operator*(float3 a, float s) { return { a.x * s, a.y * s, a.z * s }; }
inline float3 operator*(float s, float3 a) { return { a.x * s, a.y * s, a.z * s }; }
inline float3 operator/(float3 a, float s) { return { a.x / s, a.y / s, a.z / s }; }

inline float3& operator+=(float3& a, const float3& b) { a.x += b.x; a.y += b.y; a.z += b.z; return a; }
inline float3& operator-=(float3& a, const float3& b) { a.x -= b.x; a.y -= b.y; a.z -= b.z; return a; }
inline float3& operator*=(float3& a, float s) { a.x *= s;  a.y *= s;  a.z *= s;  return a; }
inline float3& operator/=(float3& a, float s) { a.x /= s;  a.y /= s;  a.z /= s;  return a; }

// ---- vector math ----
inline float  dot(float3 a, float3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float3 cross(float3 a, float3 b) {
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}
inline float  length3(float3 v) { return std::sqrt(dot(v, v)); }
inline float  lengthSq3(float3 v) { return dot(v, v); }
inline float3 abs3(float3 v) { return { std::fabs(v.x), std::fabs(v.y), std::fabs(v.z) }; }

inline float3 normalize(float3 v) {
    float L = length3(v);
    if (L <= 1e-12f) return { 0,0,0 };
    return v / L;
}
inline float3 clamp3(const float3& a, float lo, float hi) {
    return { std::clamp(a.x, lo, hi), std::clamp(a.y, lo, hi), std::clamp(a.z, lo, hi) };
}
inline float3 projectOnPlane(float3 v, float3 n) { // v - (v·n)n
    float d = dot(v, n);
    return { v.x - d * n.x, v.y - d * n.y, v.z - d * n.z };
}
inline float  wrapPi(float a) {
    while (a > PI) a -= TAU;
    while (a < -PI) a += TAU;
    return a;
}
inline void   makeONB(const float3& n, float3& u, float3& v) {
    if (std::fabs(n.x) > std::fabs(n.z)) u = normalize(float3{ -n.y, n.x, 0.0f });
    else                                 u = normalize(float3{ 0.0f, -n.z, n.y });
    v = cross(n, u);
}

// ===== float3x3 =====
struct float3x3 {
    float m[3][3]{}; // row-major storage

    inline float3 row(int r) const { return { m[r][0], m[r][1], m[r][2] }; }
    inline float3 col(int c) const { return { m[0][c], m[1][c], m[2][c] }; }

    inline float3   operator*(const float3& v) const {
        return { m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                 m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                 m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z };
    }
    inline float3x3 operator*(const float3x3& B) const {
        float3x3 C{};
        for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) {
            C.m[r][c] = m[r][0] * B.m[0][c] + m[r][1] * B.m[1][c] + m[r][2] * B.m[2][c];
        }
        return C;
    }
    inline float3x3& operator*=(const float3x3& B) { *this = (*this) * B; return *this; }
};

inline float3x3 identity3x3() { float3x3 I{}; I.m[0][0] = I.m[1][1] = I.m[2][2] = 1.0f; return I; }
inline float3x3 diagonal(float a, float b, float c) { float3x3 D{}; D.m[0][0] = a; D.m[1][1] = b; D.m[2][2] = c; return D; }
inline float3x3 transpose3x3(const float3x3& A) {
    float3x3 T{};
    T.m[0][0] = A.m[0][0]; T.m[0][1] = A.m[1][0]; T.m[0][2] = A.m[2][0];
    T.m[1][0] = A.m[0][1]; T.m[1][1] = A.m[1][1]; T.m[1][2] = A.m[2][1];
    T.m[2][0] = A.m[0][2]; T.m[2][1] = A.m[1][2]; T.m[2][2] = A.m[2][2];
    return T;
}
inline float3 mul33(const float3x3& M, const float3& v) { return M * v; }

// ===== float6 / float6x6 (for rigid-body dofs) =====
struct float6 { float v[6]{ 0,0,0,0,0,0 }; };
struct float6x6 { float m[6][6]{}; };

// ===== float6 basic ops =====
inline float6 operator+(const float6& a, const float6& b) {
    float6 r;
    for (int i = 0; i < 6; ++i) r.v[i] = a.v[i] + b.v[i];
    return r;
}
inline float6 operator-(const float6& a, const float6& b) {
    float6 r;
    for (int i = 0; i < 6; ++i) r.v[i] = a.v[i] - b.v[i];
    return r;
}
inline float6 operator-(const float6& a) {
    float6 r;
    for (int i = 0; i < 6; ++i) r.v[i] = -a.v[i];
    return r;
}
inline float6 operator*(const float6& a, float s) {
    float6 r;
    for (int i = 0; i < 6; ++i) r.v[i] = a.v[i] * s;
    return r;
}
inline float6 operator*(float s, const float6& a) { return a * s; }
inline float6 operator/(const float6& a, float s) {
    float6 r;
    for (int i = 0; i < 6; ++i) r.v[i] = a.v[i] / s;
    return r;
}

inline float6& operator+=(float6& a, const float6& b) {
    for (int i = 0; i < 6; ++i) a.v[i] += b.v[i];
    return a;
}
inline float6& operator-=(float6& a, const float6& b) {
    for (int i = 0; i < 6; ++i) a.v[i] -= b.v[i];
    return a;
}
inline float6& operator*=(float6& a, float s) {
    for (int i = 0; i < 6; ++i) a.v[i] *= s;
    return a;
}
inline float6& operator/=(float6& a, float s) {
    for (int i = 0; i < 6; ++i) a.v[i] /= s;
    return a;
}

// (옵션) 유틸 몇 개
inline float  dot(const float6& a, const float6& b) {
    float s = 0.0f; for (int i = 0; i < 6; ++i) s += a.v[i] * b.v[i]; return s;
}
inline float6 hadamard(const float6& a, const float6& b) {
    float6 r; for (int i = 0; i < 6; ++i) r.v[i] = a.v[i] * b.v[i]; return r;
}


inline void zero6(float6& a) { for (int i = 0; i < 6; ++i) a.v[i] = 0.0f; }
inline void zero6x6(float6x6& A) { for (int r = 0; r < 6; ++r) for (int c = 0; c < 6; ++c) A.m[r][c] = 0.0f; }

inline void addJTJ(float6x6& lhs, const float6& J, float rho) {
    for (int r = 0; r < 6; ++r) for (int c = 0; c < 6; ++c) lhs.m[r][c] += rho * J.v[r] * J.v[c];
}
inline void addOuter(float6x6& lhs, const float6& a, const float6& b, float s = 1.f) {
    for (int r = 0; r < 6; ++r) for (int c = 0; c < 6; ++c) lhs.m[r][c] += s * a.v[r] * b.v[c];
}
inline void addDiag(float6x6& A, const float d[6]) { for (int i = 0; i < 6; ++i) A.m[i][i] += d[i]; }
inline float  colL2(const float6x6& H, int c) { float s = 0; for (int r = 0; r < 6; ++r) s += H.m[r][c] * H.m[r][c]; return std::sqrt(s); }
inline float6 mul(const float6x6& A, const float6& x) {
    float6 y; for (int r = 0; r < 6; ++r) { float acc = 0; for (int c = 0; c < 6; ++c) acc += A.m[r][c] * x.v[c]; y.v[r] = acc; } return y;
}
inline float  dot6(const float6& J, const float3& dp, const float3& dth) {
    return J.v[0] * dp.x + J.v[1] * dp.y + J.v[2] * dp.z + J.v[3] * dth.x + J.v[4] * dth.y + J.v[5] * dth.z;
}

// Cholesky (SPD 6x6). 아주 작은 대각 정칙화 포함.
inline bool cholesky6(const float6x6& A, float L[6][6]) {
    float M[6][6];
    for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) M[i][j] = A.m[i][j];
    for (int i = 0; i < 6; ++i) M[i][i] += 1e-8f;

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j <= i; ++j) {
            float sum = M[i][j];
            for (int k = 0; k < j; ++k) sum -= L[i][k] * L[j][k];
            if (i == j) {
                if (sum <= 0.0f) return false;
                L[i][i] = std::sqrt(sum);
            }
            else {
                L[i][j] = sum / L[j][j];
            }
        }
        for (int j = i + 1; j < 6; ++j) L[i][j] = 0.0f;
    }
    return true;
}
inline float6 solveSPD6x6(const float6x6& A, const float6& b) {
    float L[6][6];
    float6 x; zero6(x);
    if (!cholesky6(A, L)) return x;
    // Ly=b
    float y[6];
    for (int i = 0; i < 6; ++i) {
        float sum = b.v[i];
        for (int k = 0; k < i; ++k) sum -= L[i][k] * y[k];
        y[i] = sum / L[i][i];
    }
    // L^T x = y
    for (int i = 5; i >= 0; --i) {
        float sum = y[i];
        for (int k = i + 1; k < 6; ++k) sum -= L[k][i] * x.v[k];
        x.v[i] = sum / L[i][i];
    }
    return x;
}

// 질량-관성 블록: diag(m I3, Iworld)
inline void addMassInertiaBlock(float6x6& A, float m, const float3x3& Iw, float s) {
    for (int i = 0; i < 3; ++i) A.m[i][i] += s * m;
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) A.m[3 + r][3 + c] += s * Iw.m[r][c];
}

// ===== Quaternion =====
struct quat {
    float w{ 1 }, x{ 0 }, y{ 0 }, z{ 0 };

    inline float3x3 R() const {
        float3x3 Rm{};
        const float n = w * w + x * x + y * y + z * z;
        if (n <= 1e-12f) return identity3x3();
        const float s = 2.0f / n;
        const float xx = x * x * s, yy = y * y * s, zz = z * z * s;
        const float xy = x * y * s, xz = x * z * s, yz = y * z * s;
        const float wx = w * x * s, wy = w * y * s, wz = w * z * s;
        Rm.m[0][0] = 1 - (yy + zz); Rm.m[0][1] = xy - wz;     Rm.m[0][2] = xz + wy;
        Rm.m[1][0] = xy + wz;       Rm.m[1][1] = 1 - (xx + zz); Rm.m[1][2] = yz - wx;
        Rm.m[2][0] = xz - wy;       Rm.m[2][1] = yz + wx;     Rm.m[2][2] = 1 - (xx + yy);
        return Rm;
    }
};

inline quat qmul(quat a, quat b) {
    return { a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
             a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
             a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
             a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w };
}
inline quat qnorm(quat q) {
    float s = 1.0f / std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return { q.w * s, q.x * s, q.y * s, q.z * s };
}
inline quat qconj(quat q) { return { q.w, -q.x, -q.y, -q.z }; }

inline quat expQuat(float3 dtheta) {
    float ang = length3(dtheta);
    if (ang < 1e-9f) return { 1,0,0,0 };
    float s = std::sin(0.5f * ang) / ang;
    return qnorm({ std::cos(0.5f * ang), dtheta.x * s, dtheta.y * s, dtheta.z * s });
}

inline quat quatFromEulerXYZ(float3 e) {
    const float cx = std::cos(0.5f * e.x), sx = std::sin(0.5f * e.x);
    const float cy = std::cos(0.5f * e.y), sy = std::sin(0.5f * e.y);
    const float cz = std::cos(0.5f * e.z), sz = std::sin(0.5f * e.z);
    quat qx{ cx, sx, 0, 0 };
    quat qy{ cy, 0, sy, 0 };
    quat qz{ cz, 0, 0, sz };
    return qnorm(qmul(qmul(qz, qy), qx));
}

// dtheta = log( conj(q_from) * q_to )  (so that q_to ≈ q_from * exp(0.5*dtheta))
inline float3 rotVec_fromTo(const quat& q_from, const quat& q_to) {
    quat qd = qmul(qconj(q_from), q_to); // delta
    float w = std::clamp(qd.w, -1.0f, 1.0f);
    float s2 = std::sqrt(std::max(0.0f, 1.0f - w * w));
    if (s2 < 1e-8f) return { 2.0f * qd.x, 2.0f * qd.y, 2.0f * qd.z }; // small-angle
    float angle = 2.0f * std::acos(w);
    float inv = angle / s2;
    return { qd.x * inv, qd.y * inv, qd.z * inv };
}

// 쿼터니언 → 회전벡터(log) (단위 쿼터니언 가정; 작은 각 보정)
static inline float3 quat_log_vec_local(quat q)
{
    // 정규화 (안전)
    q = qnorm(q);
    float vnorm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    float ang = 2.0f * std::atan2(vnorm, q.w);
    if (vnorm < 1e-12f) return { 0,0,0 };
    float s = ang / vnorm;
    return { q.x * s, q.y * s, q.z * s };
}

struct Plane3 {
    float3 n; // unit normal, inside: n·p <= d
    float  d;
};

static inline float  signf(float x) { return (x >= 0.0f) ? 1.0f : -1.0f; }