#include "btBulletDynamicsCommon.h"
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

class bulletDynamicsWorldI {
public:
	bulletDynamicsWorldI(btDynamicsWorld* o) : world(o)
	{
        bcw.p = this;
	}
    bulletDynamicsWorld bcw;
    btDynamicsWorld *world;
};

bulletDynamicsWorld* CreateDiscreteDynamicsWorld(bulletDispatcher* dispatcher, bulletBroadphaseInterface *pairCache, bulletConstraintSolver *constraintSolver, bulletCollisionConfiguration *collisionConfiguration) {
	btDynamicsWorld *world = new btDiscreteDynamicsWorld(
		reinterpret_cast<bulletDispatcherI*>(dispatcher->p)->dispatcher,
		reinterpret_cast<bulletBroadphaseInterfaceI*>(pairCache->p)->broadphase,
		reinterpret_cast<bulletConstraintSolverI*>(constraintSolver->p)->solver,
		reinterpret_cast<collisionConfigurationI*>(collisionConfiguration->p)->config);
	bulletDynamicsWorldI* impl = new bulletDynamicsWorldI(world);
	return &impl->bcw;
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