#ifndef BULLET_H
#define BULLET_H

#ifdef __cplusplus
extern "C" { 
#endif

typedef float	Vector3[3];
typedef float	Vector4[4];
typedef float	Mat4[16];

typedef struct bulletBoxShape {
    void *p;
} bulletBoxShape;

typedef struct bulletGImpactMeshShape {
    void *p;
} bulletGImpactMeshShape;

typedef struct bulletConvexHullShape {
    void *p;
} bulletConvexHullShape;

typedef struct bulletCollisionObject {
    void *p;
} bulletCollisionObject;

typedef struct bulletRigidBody {
    void *p;
} bulletRigidBody;

typedef struct bulletCollisionShape {
    void *p;
} bulletCollisionShape;

typedef struct bulletStaticPlaneShape {
    void *p;
} bulletStaticPlaneShape;

typedef struct bulletDefaultCollisionConfiguration {
    void *p;
} bulletDefaultCollisionConfiguration;

typedef struct bulletCollisionConfiguration {
    void *p;
} bulletCollisionConfiguration;

typedef struct bulletCollisionDispatcher {
    void *p;
} bulletCollisionDispatcher;

typedef struct bulletDispatcher {
    void *p;
} bulletDispatcher;

typedef struct bulletAxisSweep3 {
    void *p;
} bulletAxisSweep3;

typedef struct bulletBroadphaseInterface {
    void *p;

} bulletBroadphaseInterface;

typedef struct bulletConstraintSolver {
    void *p;
} bulletConstraintSolver;

typedef struct bulletDynamicsWorld {
    void *p;
} bulletDynamicsWorld;

extern bulletCollisionShape* GetBoxCollisionShape(bulletBoxShape* shape);
extern bulletBoxShape* CreateBoxShape(float x, float y, float z);
extern bulletGImpactMeshShape* CreateGImpactMeshShape(int vcount, float *v, int fcount, int *i);
extern bulletCollisionShape* GetGImpactMeshShapeCollisionShape(bulletGImpactMeshShape* shape);
extern bulletConvexHullShape* CreateConvexHullShape(int vcount, float *v);
extern bulletCollisionShape* GetConvexHullCollisionShape(bulletConvexHullShape* shape);

extern bulletRigidBody* CreateRigidBody(void* user_data,  float mass, bulletCollisionShape* shape);
extern void RigidBodyGetOrigin(bulletRigidBody* body, float* v);
extern void RigidBodyGetRotation(bulletRigidBody* body, float* v);
extern void RigidBodySetFromOpenGLMatrix(bulletRigidBody* body, float* m);
extern void RigidBodyApplyCentralForce(bulletRigidBody* body, float* m);
extern void RigidBodyApplyForce(bulletRigidBody* b, float* m, float *rel);
extern void RigidBodyApplyImpulse(bulletRigidBody* b, float* m, float *rel);
extern void RigidBodyApplyTorque(bulletRigidBody* b, float* m);
extern void RigidBodyApplyTorqueImpulse(bulletRigidBody* b, float* m);
extern void RigidBodySetLinearVelocity(bulletRigidBody* b, float* m);

extern bulletStaticPlaneShape* CreateStaticPlaneShape(float *normals,  float planeConstant);
extern bulletCollisionShape* GetStaticPlaneCollisionShape(bulletStaticPlaneShape* shape);

extern bulletDefaultCollisionConfiguration* CreateDefaultCollisionConfiguration();
extern bulletCollisionConfiguration* GetCollisionConfiguration(bulletDefaultCollisionConfiguration* config);

extern bulletCollisionDispatcher* CreateCollisionDispatcher(bulletCollisionConfiguration* config);
extern bulletDispatcher* GetCollisionDispatcher(bulletCollisionDispatcher* d);

extern bulletAxisSweep3* CreateAxisSweep3(float* min, float* max);
extern bulletBroadphaseInterface* GetAxisSweep3BroadphaseInterface(bulletAxisSweep3* axis);

extern bulletConstraintSolver* CreateSequentialImpulseConstraintSolver();
extern bulletDynamicsWorld* CreateDiscreteDynamicsWorld(bulletDispatcher* dispatcher, bulletBroadphaseInterface *pairCache, bulletConstraintSolver *constraintSolver, bulletCollisionConfiguration *collisionConfiguration);

typedef struct DebugDraw {
    int count;
    float* verts;
    float* colors;
} DebugDraw;
extern DebugDraw DynamicsWorldDebugDrawWorld(bulletDynamicsWorld* world);
extern void DynamicsWorldSetGravity(bulletDynamicsWorld* world, float* gravity);
extern void DynamicsWorldAddRigidBody(bulletDynamicsWorld* world, bulletRigidBody *body);
extern void DynamicsWorldStepSimulation(bulletDynamicsWorld* world, float time);

#ifdef __cplusplus
}
#endif

#endif /*BULLET_H*/