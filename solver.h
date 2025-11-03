#ifndef SOLVER_H
#define SOLVER_H

#include "util/includes.h"
#include "collision/face.h"
#include "linalg/ldlt.h"
#include "debug_utils/debug.h"
#include "linalg/linalg.h"
#include <array>

#define MAX_ROWS 12           // Max scalar rows an individual constraint can have (3D contact = 3n)
#define PENALTY_MIN 1000.0f   // Minimum penalty parameter
#define PENALTY_MAX 1e9f      // Maximum penalty parameter
#define COLLISION_MARGIN 0.04f
#define STICK_THRESH 0.02f
#define SHOW_CONTACTS true

// early declare structs
struct Rigid;
struct Force;
struct Manifold;
struct Solver;
struct Mesh;
struct StackFace;

// contains data for a single rigid body
struct Rigid {
    Solver* solver;
    Force* forces;
    Rigid* next;

    // position and rotation stored seperately since rotation is quaternion
    vec3 position; 
    quat rotation = quat(1, 0, 0, 0); 

    vec6 velocity = vec6(0); // linear 3, angular 3
    vec6 prevVelocity = vec6(0);

    vec3 initialPosition;
    quat initialRotation;
    vec3 inertialPosition;
    quat inertialRotation;

    vec3 scale;
    float mass;
    mat3x3 inertiaTensor;
    float friction;
    float radius;

    // visual attributes
    vec4 color;

    Rigid(Solver* solver, vec3 size, float density, float friction, vec3 position, quat rotation = quat(1, 0, 0, 0),
          vec6 velocity = vec6(), vec4 color = vec4(0.8, 0.8, 0.8, 0.5));
    ~Rigid();

    bool constrainedTo(Rigid* other) const;

    mat3x3 getInertiaTensor() const;
    mat6x6 getMassMatrix() const;

    vec3 deltaWInitial() const;
    vec3 deltaWInertial() const;

    // static
    static int globalID;
};

// Provides constraint parameters and common interface for all forces.
struct Force {
    Solver* solver;
    Rigid* bodyA;
    Rigid* bodyB;

    Force* nextA;
    Force* nextB;
    Force* next;

    std::vector<vec6> J; // Jacobian rows for bodyA (bodyB are -J)
    std::vector<mat6x6> H; // Hassian/approx for complaint constraints

    float C[MAX_ROWS]; // Constraint error per row;
    float fmin[MAX_ROWS]; // Lower force/impulse limits
    float fmax[MAX_ROWS]; // Upper force/impulse limits
    float stiffness[MAX_ROWS];
    float motor[MAX_ROWS];
    float fracture[MAX_ROWS];
    float penalty[MAX_ROWS];
    float lambda[MAX_ROWS]; // Accumulated impulses (warm-start)

    Force(Solver* solver, Rigid* bodyA, Rigid* bodyB);
    virtual ~Force();

    void disable();

    virtual int rows() const = 0; // # of scalar constraint equations
    virtual bool initialize() = 0; // called once when added to solver
    virtual void computeConstraint(float alpha) = 0; // C and limits per row
    virtual void computeDerivatives(Rigid* body) = 0; // J and H per body

    // static
    static int globalID;
};

struct Manifold : Force {
    struct Contact {
        vec3 rA;
        vec3 rB;
        vec3 normal; // world space contact normal A -> B
        float depth; 
        vec3 t1;
        vec3 t2;
        vec6 JAn, JBn; // normal Jacobian rows
        vec6 JAt1, JBt1; // tangent Jacobian rows
        vec6 JAt2, JBt2; // tangent Jacobian rows in the other direction
        vec3 C0; // accumulated positional error (n, t1, t2)
        bool stick; // static vs dynamic friction
        StackFace face; // saves contact data
        int type;

        Contact() : rA(), rB(), normal(), depth(0.0), t1(), t2(), JAn(), JBn(), JAt1(), JBt1(), JAt2(), JBt2(), C0(), stick(true), face() {}

        // only considers face indices
        bool operator==(const Contact& rhs) {
            for (int i = 0; i < 3; i++) 
                if (face.sps[i].indexA != rhs.face.sps[i].indexA || face.sps[i].indexB != rhs.face.sps[i].indexB)
                    return false;
            return true;
        }
    }; 

    Contact contacts[4];
    int numContacts;

    // friction variables, later change to mus and mud
    float friction;

    Manifold(Solver* solver, Rigid* bodyA, Rigid* bodyB);

    int rows() const override { return numContacts * 3; }
    bool initialize() override;
    void computeConstraint(float alpha) override;
    void computeDerivatives(Rigid* body) override;
    bool isContactStillValid(const Contact& oldContact, Rigid* bodyA, Rigid* bodyB);

    static int collide(Rigid* bodyA, Rigid* bodyB, Contact* contacts);
};

struct Joint : Force {
    Rigid* A;
    Rigid* B;

    // 로컬 앵커/축/기준
    vec3 rA_local, rB_local;   // 로컬 앵커
    vec3 axisA_local, axisB_local; // 힌지 축(로컬)
    vec3 refA_local, refB_local;  // 스윙 기준 벡터(로컬)

    // 월드 기준 (스텝 동안 고정)
    vec3 axisW_fixed; // 힌지 축 (월드)
    vec3 swingU;      // 스윙 평면 기저 U (월드)
    vec3 swingV;      // 스윙 평면 기저 V (월드)

    // 생성자: 위치강성 posK(각축), 스윙강성 swingK, 파단한계 frac
    Joint(Solver* solver,
        Rigid* bodyA, Rigid* bodyB,
        vec3 rA, vec3 rB,
        vec3 axisA, vec3 axisB,
        vec3 refA, vec3 refB,
        vec3 posK,
        float swingK,
        float frac);

    int rows() const override { return 5; } // 3(pos) + 2(swing)
    bool initialize() override;
    void computeConstraint(float alpha) override;
    void computeDerivatives(Rigid* body) override;
};

// 🟢 추가: Joint2 선언 (ball/hinge + optional twist)
struct Joint2 : Force {
    Rigid* A;
    Rigid* B;

    // local anchors & axes
    vec3 rA_local, rB_local;
    vec3 axisA_local, axisB_local;
    vec3 refA_local, refB_local;

    // world-fixed hinge frame (frozen at initialize)
    vec3 axisW_fixed;
    vec3 swingU, swingV;

    // twist
    float restTwist = 0.0f;
    bool  enableTwist = false;

    // cached initial error (for compliant/infinite stiffness style if needed)
    float C0[6] = { 0,0,0,0,0,0 };

    // per-instance row count (3=ball, 5=hinge(no twist), 6=hinge+twist)
    int nrows = 5;

    // --- ctors ---
    // 1) Ball (position only: 3 rows)
    Joint2(Solver* solver,
        Rigid* bodyA, Rigid* bodyB,
        vec3 rA, vec3 rB,
        vec3 posStiffness,
        float frac = INFINITY);

    // 2) Hinge (pos 3 + swing 2 + [optional twist 1])
    Joint2(Solver* solver,
        Rigid* bodyA, Rigid* bodyB,
        vec3 rA, vec3 rB,
        vec3 axisA, vec3 axisB,
        vec3 refA, vec3 refB,
        vec3 posK,
        float swingK,
        float twistK = 0.0f,
        float restTwistRadians = 0.0f,
        float frac = INFINITY);

    // --- Force overrides ---
    int  rows() const override { return nrows; }
    bool initialize() override;
    void computeConstraint(float alpha) override;
    void computeDerivatives(Rigid* body) override;
};




struct Solver {
    vec3 gravity;
    int iterations;

    float alpha; 
    float beta;
    float gamma;

    Rigid* bodies;
    Force* forces;
    Mesh* meshes;

    Solver();
    ~Solver();

    Rigid* pick(vec3 at, vec3& local); // ray-pick helper

    void clear();
    void defaultParams();
    void step(float dt);
};

// helper functions
vec3 rotateNScale(const vec3& vertex, Rigid* body);
vec3 rotateNScale(int index, Rigid* body);
vec3 rotateWithoutScale(const vec3& vertex, Rigid* body);
vec3 rotateWithoutScale(int index, Rigid* body);
mat6x6 diagonalLump(const mat6x6& mat);
SupportPoint getSupportPoint(Rigid* bodyA, Rigid* bodyB, const vec3& dir);

#endif