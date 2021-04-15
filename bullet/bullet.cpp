#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "bullet.h"

class collisionShapeI {
public:
	collisionShapeI(btCollisionShape* shape) : collisionShape(shape)
	{
        bss.p = this;
	}
    bulletCollisionShape bss;
    btCollisionShape *collisionShape;
};

class collisionObjectI {
public:
	collisionObjectI(btCollisionObject* o) : collisionObject(o)
	{
        bco.p = this;
	}
    bulletCollisionObject bco;
    btCollisionObject *collisionObject;
};

class rigidBodyI : public collisionObjectI {
public:
	rigidBodyI(btRigidBody* o) : collisionObjectI(o), rigid(o)
	{
        brb.p = this;
	}
    bulletRigidBody brb;
    btRigidBody *rigid;
};

class boxShapeI : public collisionShapeI {
public:
	boxShapeI(btBoxShape* box) : collisionShapeI(box), box(box)
	{
        bsp.p = this;
	}
    bulletBoxShape bsp;
    btBoxShape *box;
};

bulletBoxShape* CreateBoxShape(float x, float y, float z)
{
    btBoxShape* box = new btBoxShape(btVector3(x, y, z));
    boxShapeI* impl = new boxShapeI(box);
    return &impl->bsp;
}

bulletCollisionShape* GetBoxCollisionShape(bulletBoxShape* shape) {
    boxShapeI *box = reinterpret_cast<boxShapeI*>(shape->p);
	return &box->bss;
}

class gImpactMeshShapeI : public collisionShapeI {
public:
	gImpactMeshShapeI(btGImpactMeshShape* box) : collisionShapeI(box), box(box)
	{
        bsp.p = this;
	}
    bulletGImpactMeshShape bsp;
    btGImpactMeshShape *box;
};


bulletGImpactMeshShape* CreateGImpactMeshShape(int vn, float *v, int in, int *i) {
	btTriangleIndexVertexArray* a = new btTriangleIndexVertexArray(in/3, i, 3*sizeof(int), vn, v,sizeof(float)*3);

	btGImpactMeshShape * trimesh = new btGImpactMeshShape(a);
	trimesh->setLocalScaling(btVector3(1.f,1.f,1.f));
	//trimesh->setMargin(0.0f);
	trimesh->setMargin(0.07f);
	trimesh->updateBound();

    gImpactMeshShapeI* impl = new gImpactMeshShapeI(trimesh);
    return &impl->bsp;
}

bulletCollisionShape* GetGImpactMeshShapeCollisionShape(bulletGImpactMeshShape* shape) {
    gImpactMeshShapeI *box = reinterpret_cast<gImpactMeshShapeI*>(shape->p);
	return &box->bss;
}

class convexHullShapeI : public collisionShapeI {
public:
	convexHullShapeI(btConvexHullShape* box) : collisionShapeI(box), box(box)
	{
        bsp.p = this;
	}
    bulletConvexHullShape bsp;
    btConvexHullShape *box;
};

bulletConvexHullShape* CreateConvexHullShape(int vn, float *v) {
	btConvexHullShape* hull = new 	btConvexHullShape(v, vn/3, 12);
	hull->optimizeConvexHull();

    convexHullShapeI* impl = new convexHullShapeI(hull);
    return &impl->bsp;
}

bulletCollisionShape* GetConvexHullCollisionShape(bulletConvexHullShape* shape) {
    gImpactMeshShapeI *box = reinterpret_cast<gImpactMeshShapeI*>(shape->p);
	return &box->bss;
}

bulletRigidBody* CreateRigidBody(void* user_data,  float mass, bulletCollisionShape* cshape)
{
	btTransform trans;
	trans.setIdentity();
	btVector3 localInertia(0,0,0);
	collisionShapeI* shape = reinterpret_cast<collisionShapeI*>(cshape->p);
	btAssert(shape);
	if (mass)
	{
		shape->collisionShape->calculateLocalInertia(mass,localInertia);
	}
	btRigidBody::btRigidBodyConstructionInfo rbci(mass, 0,shape->collisionShape,localInertia);
	btRigidBody* body = new btRigidBody(rbci);
	body->setWorldTransform(trans);
	body->setUserPointer(user_data);

    rigidBodyI* impl = new rigidBodyI(body);
	return &impl->brb;
}

void RigidBodyGetOrigin(bulletRigidBody* b, float* v) {
    rigidBodyI *rb = reinterpret_cast<rigidBodyI*>(b->p);
	btVector3 origin = rb->rigid->getWorldTransform().getOrigin();
	v[0] = origin.getX();
	v[1] = origin.getY();
	v[2] = origin.getZ();
}

void RigidBodyGetRotation(bulletRigidBody* b, float* v) {
    rigidBodyI *rb = reinterpret_cast<rigidBodyI*>(b->p);
	btQuaternion r = rb->rigid->getWorldTransform().getRotation();
	v[0] = r.getW();
	v[1] = r.getX();
	v[2] = r.getY();
	v[3] = r.getZ();
}

void RigidBodySetFromOpenGLMatrix(bulletRigidBody* b, float* m) {
    rigidBodyI *rb = reinterpret_cast<rigidBodyI*>(b->p);
	rb->rigid->getWorldTransform().setFromOpenGLMatrix(m);
}

void RigidBodyApplyCentralForce(bulletRigidBody* b, float* m) {
    rigidBodyI *rb = reinterpret_cast<rigidBodyI*>(b->p);
	rb->rigid->applyCentralForce(btVector3(m[0], m[1], m[2]));
}

void RigidBodyApplyForce(bulletRigidBody* b, float* m, float *rel) {
    rigidBodyI *rb = reinterpret_cast<rigidBodyI*>(b->p);
	rb->rigid->applyForce(btVector3(m[0], m[1], m[2]), btVector3(rel[0], rel[1], rel[2]));
}

void RigidBodyApplyImpulse(bulletRigidBody* b, float* m, float *rel) {
    rigidBodyI *rb = reinterpret_cast<rigidBodyI*>(b->p);
	rb->rigid->applyImpulse(btVector3(m[0], m[1], m[2]), btVector3(rel[0], rel[1], rel[2]));
}

void RigidBodyApplyTorque(bulletRigidBody* b, float* m) {
    rigidBodyI *rb = reinterpret_cast<rigidBodyI*>(b->p);
	rb->rigid->applyTorque(btVector3(m[0], m[1], m[2]));
}

void RigidBodyApplyTorqueImpulse(bulletRigidBody* b, float* m) {
    rigidBodyI *rb = reinterpret_cast<rigidBodyI*>(b->p);
	rb->rigid->applyTorqueImpulse(btVector3(m[0], m[1], m[2]));
}

void RigidBodySetLinearVelocity(bulletRigidBody* b, float* m) {
    rigidBodyI *rb = reinterpret_cast<rigidBodyI*>(b->p);
	rb->rigid->setLinearVelocity(btVector3(m[0], m[1], m[2]));
}

class staticPlaneShapeI : public collisionShapeI {
public:
	staticPlaneShapeI(btStaticPlaneShape* s) : collisionShapeI(s), sps(s)
	{
        bsp.p = this;
	}
    bulletStaticPlaneShape bsp;
    btStaticPlaneShape *sps;
};

bulletStaticPlaneShape* CreateStaticPlaneShape(float* normals,  float planeConstant) {
	btStaticPlaneShape *shape = new btStaticPlaneShape(btVector3(normals[0],normals[1],normals[2]), planeConstant);
    staticPlaneShapeI* impl = new staticPlaneShapeI(shape);
	return &impl->bsp;
}

bulletCollisionShape* GetStaticPlaneCollisionShape(bulletStaticPlaneShape* shape) {
    staticPlaneShapeI *box = reinterpret_cast<staticPlaneShapeI*>(shape->p);
	return &box->bss;
}

class collisionConfigurationI {
public:
	collisionConfigurationI(btCollisionConfiguration* o) : config(o)
	{
        bcc.p = this;
	}
    bulletCollisionConfiguration bcc;
    btCollisionConfiguration *config;
};

class defaultCollisionConfigurationI : public collisionConfigurationI {
public:
	defaultCollisionConfigurationI(btDefaultCollisionConfiguration* o) : collisionConfigurationI(o), dcc(o)
	{
        bdc.p = this;
	}
    bulletDefaultCollisionConfiguration bdc;
    btDefaultCollisionConfiguration *dcc;
};

bulletDefaultCollisionConfiguration* CreateDefaultCollisionConfiguration() {
	btDefaultCollisionConfiguration* config = new btDefaultCollisionConfiguration();
	defaultCollisionConfigurationI* impl = new defaultCollisionConfigurationI(config);
	return &impl->bdc;
}

bulletCollisionConfiguration* GetCollisionConfiguration(bulletDefaultCollisionConfiguration* config) {
    defaultCollisionConfigurationI *box = reinterpret_cast<defaultCollisionConfigurationI*>(config->p);
	return &box->bcc;
}

class bulletDispatcherI {
public:
	bulletDispatcherI(btDispatcher* o) : dispatcher(o)
	{
        bd.p = this;
	}
    bulletDispatcher bd;
    btDispatcher *dispatcher;
};

class bulletCollisionDispatcherI : public bulletDispatcherI {
public:
	bulletCollisionDispatcherI(btCollisionDispatcher* o) : bulletDispatcherI(o), collisionDispatcher(o)
	{
        bdc.p = this;
	}
    bulletCollisionDispatcher bdc;
    btCollisionDispatcher *collisionDispatcher;
};

bulletCollisionDispatcher* CreateCollisionDispatcher(bulletCollisionConfiguration* config) {
    collisionConfigurationI *box = reinterpret_cast<collisionConfigurationI*>(config->p);
	
	btCollisionDispatcher *dispatcher = new btCollisionDispatcher(box->config);
    bulletCollisionDispatcherI* impl = new bulletCollisionDispatcherI(dispatcher);
	return &impl->bdc;
}

bulletDispatcher* GetCollisionDispatcher(bulletCollisionDispatcher* d) {
    bulletCollisionDispatcherI *box = reinterpret_cast<bulletCollisionDispatcherI*>(d->p);
	return &box->bd;
}

class bulletBroadphaseInterfaceI {
public:
	bulletBroadphaseInterfaceI(btBroadphaseInterface* o) : broadphase(o)
	{
        bpi.p = this;
	}
    bulletBroadphaseInterface bpi;
    btBroadphaseInterface *broadphase;
};

class bulletAxisSweep3I : public bulletBroadphaseInterfaceI {
public:
	bulletAxisSweep3I(btAxisSweep3* o) : bulletBroadphaseInterfaceI(o), axis(o)
	{
        bas.p = this;
	}
    bulletAxisSweep3 bas;
    btAxisSweep3 *axis;
};
bulletAxisSweep3* CreateAxisSweep3(float* min, float* max) {
	btAxisSweep3 *axis = new btAxisSweep3(btVector3(min[0], min[1], min[2]), btVector3(max[0], max[1], max[2]));
    bulletAxisSweep3I* impl = new bulletAxisSweep3I(axis);
	return &impl->bas;

}

bulletBroadphaseInterface* GetAxisSweep3BroadphaseInterface(bulletAxisSweep3* axis) {
    bulletAxisSweep3I *box = reinterpret_cast<bulletAxisSweep3I*>(axis->p);
	return &box->bpi;
}

class bulletConstraintSolverI {
public:
	bulletConstraintSolverI(btSequentialImpulseConstraintSolver* o) : solver(o)
	{
        bpi.p = this;
	}
    bulletConstraintSolver bpi;
    btSequentialImpulseConstraintSolver *solver;
};

bulletConstraintSolver* CreateSequentialImpulseConstraintSolver() {
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
	bulletConstraintSolverI* impl = new bulletConstraintSolverI(solver);
	return &impl->bpi;
}

#include <vector>

class BulletDebugDrawer : public btIDebugDraw {
	int _debugMode;

public:

	BulletDebugDrawer() {

	}
	virtual ~BulletDebugDrawer() {

	}

	std::vector<float> vertices;
	std::vector<float> colors;

	void clear() {
		vertices.clear();
		colors.clear();
	}

	virtual void   drawLine(const btVector3& from, const btVector3& to, const btVector3& fromColor, const btVector3& toColor) {
		vertices.push_back(from.getX());
		vertices.push_back(from.getY());
		vertices.push_back(from.getZ());
		colors.push_back(fromColor.getX());
		colors.push_back(fromColor.getY());
		colors.push_back(fromColor.getZ());
		vertices.push_back(to.getX());
		vertices.push_back(to.getY());
		vertices.push_back(to.getZ());
		colors.push_back(toColor.getX());
		colors.push_back(toColor.getY());
		colors.push_back(toColor.getZ());
	}

	virtual void   drawLine(const btVector3& from, const btVector3& to, const btVector3& color) {
		vertices.push_back(from.getX());
		vertices.push_back(from.getY());
		vertices.push_back(from.getZ());

		colors.push_back(color.getX());
		colors.push_back(color.getY());
		colors.push_back(color.getZ());

		vertices.push_back(to.getX());
		vertices.push_back(to.getY());
		vertices.push_back(to.getZ());

		colors.push_back(color.getX());
		colors.push_back(color.getY());
		colors.push_back(color.getZ());
	}

#ifdef never
	virtual void   drawSphere(const btVector3& p, btScalar radius, const btVector3& color);

	virtual void   drawTriangle(const btVector3& a, const btVector3& b, const btVector3& c, const btVector3& color, btScalar alpha);
#endif
	virtual void   drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color) {

	}

	virtual void   reportErrorWarning(const char* warningString) {
		printf("warning %s\n", warningString);

	}

	virtual void   draw3dText(const btVector3& location, const char* textString) {
		printf("3d text at %f %f %f: %s\n", location.getX(), location.getY(), location.getZ(), textString);

	}

	virtual void   setDebugMode(int debugMode) {
		_debugMode = debugMode;
	}

	virtual int getDebugMode() const { return _debugMode; }
};

class bulletDynamicsWorldI {
public:
	bulletDynamicsWorldI(btDynamicsWorld* o, BulletDebugDrawer* drawer) : world(o), debugDrawer(drawer)
	{
        bcw.p = this;
	}
    bulletDynamicsWorld bcw;
    btDynamicsWorld *world;
	BulletDebugDrawer* debugDrawer;
};



bulletDynamicsWorld* CreateDiscreteDynamicsWorld(bulletDispatcher* dispatcher, bulletBroadphaseInterface *pairCache, bulletConstraintSolver *constraintSolver, bulletCollisionConfiguration *collisionConfiguration) {
	btDynamicsWorld *world = new btDiscreteDynamicsWorld(
		reinterpret_cast<bulletDispatcherI*>(dispatcher->p)->dispatcher,
		reinterpret_cast<bulletBroadphaseInterfaceI*>(pairCache->p)->broadphase,
		reinterpret_cast<bulletConstraintSolverI*>(constraintSolver->p)->solver,
		reinterpret_cast<collisionConfigurationI*>(collisionConfiguration->p)->config);

	BulletDebugDrawer * debugDrawer = new BulletDebugDrawer();
	debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
	//debugDrawer->setDebugMode(btIDebugDraw::DBG_NoDebug);

	world->setDebugDrawer(debugDrawer);

// Necessary if we're going to use the GImpactCollisionShape.
	//btGImpactCollisionAlgorithm::registerAlgorithm(static_cast<btCollisionDispatcher *>(reinterpret_cast<bulletDispatcherI*>(dispatcher->p)->dispatcher));

	bulletDynamicsWorldI* impl = new bulletDynamicsWorldI(world, debugDrawer);
	return &impl->bcw;
}

DebugDraw DynamicsWorldDebugDrawWorld(bulletDynamicsWorld* world) {
    bulletDynamicsWorldI *w = reinterpret_cast<bulletDynamicsWorldI*>(world->p);
	//printf("clear\n");
	w->debugDrawer->clear();
	//printf("draw\n");
	w->world->debugDrawWorld();

	//printf("return\n");
	DebugDraw d;
	d.count = int(w->debugDrawer->vertices.size());
	d.verts = &w->debugDrawer->vertices[0];
	d.colors = &w->debugDrawer->colors[0];
	return d;
}

void DynamicsWorldSetGravity(bulletDynamicsWorld* world, float* gravity) {
    bulletDynamicsWorldI *w = reinterpret_cast<bulletDynamicsWorldI*>(world->p);
	w->world->setGravity(btVector3(gravity[0], gravity[1], gravity[2]));
}

void DynamicsWorldAddRigidBody(bulletDynamicsWorld* world, bulletRigidBody* body) {
    bulletDynamicsWorldI *w = reinterpret_cast<bulletDynamicsWorldI*>(world->p);
    rigidBodyI *rb = reinterpret_cast<rigidBodyI*>(body->p);
	w->world->addRigidBody(rb->rigid);
}
void DynamicsWorldStepSimulation(bulletDynamicsWorld* world, float time) {
    bulletDynamicsWorldI *w = reinterpret_cast<bulletDynamicsWorldI*>(world->p);
	w->world->stepSimulation(time);
}