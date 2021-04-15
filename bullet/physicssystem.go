package bullet

// #cgo pkg-config: bullet
// #cgo windows LDFLAGS: -Wl,--allow-multiple-definition
// #include "bullet.h"
import "C"
import (
	"unsafe"

	"github.com/go-gl/mathgl/mgl32"
)

// convenience
func vec3ToBullet(vec mgl32.Vec3) (out C.Vector3) {
	out[0] = C.float(vec.X())
	out[1] = C.float(vec.Y())
	out[2] = C.float(vec.Z())

	return out
}

/*
func quatToBullet(quat mgl64.Quat) (out C.plQuaternion) {
	out[0] = C.plReal(quat.X())
	out[1] = C.plReal(quat.Y())
	out[2] = C.plReal(quat.Z())
	out[3] = C.plReal(quat.W)

	return out
}

func mat4ToBullet(mat mgl64.Mat4) (out [16]C.plReal) {
	for x := 0; x < 16; x++ {
		out[x] = C.plReal(mat[x])
	}
	return out
}

func mat4FromBullet(mat [16]C.plReal) (out mgl64.Mat4) {
	for x := 0; x < 16; x++ {
		out[x] = float64(mat[x])
	}
	return out
}
*/

type CollisionShape struct {
	handle *C.bulletCollisionShape
}

type BoxShape struct {
	handle *C.bulletBoxShape
}

func (b BoxShape) GetCollisionShape() CollisionShape {
	return CollisionShape{C.GetBoxCollisionShape(b.handle)}
}

func NewBoxShape(size mgl32.Vec3) BoxShape {
	return BoxShape{
		C.CreateBoxShape(C.float(size.X()), C.float(size.Y()), C.float(size.Z())),
	}
}

type GImpactMeshShape struct {
	handle *C.bulletGImpactMeshShape
}

func (b GImpactMeshShape) GetCollisionShape() CollisionShape {
	return CollisionShape{C.GetGImpactMeshShapeCollisionShape(b.handle)}
}

func NewGImpactMeshShape(v []float32, faces []int) GImpactMeshShape {
	return GImpactMeshShape{
		C.CreateGImpactMeshShape(C.int(len(v)), (*C.float)(unsafe.Pointer(&v[0])), C.int(len(faces)), (*C.int)(unsafe.Pointer(&faces[0]))),
	}
}

type ConvexHullShape struct {
	handle *C.bulletConvexHullShape
}

func (b ConvexHullShape) GetCollisionShape() CollisionShape {
	return CollisionShape{C.GetConvexHullCollisionShape(b.handle)}
}

func NewConvexHullShape(v []float32) ConvexHullShape {
	return ConvexHullShape{
		C.CreateConvexHullShape(C.int(len(v)), (*C.float)(unsafe.Pointer(&v[0]))),
	}
}

type StaticPlaneShape struct {
	handle *C.bulletStaticPlaneShape
}

func (b StaticPlaneShape) GetCollisionShape() CollisionShape {
	return CollisionShape{C.GetStaticPlaneCollisionShape(b.handle)}
}

// NewStaticPlaneShape implements the core.PhysicsSystem interface
func NewStaticPlaneShape(normal mgl32.Vec3, constant float64) StaticPlaneShape {
	vec := vec3ToBullet(normal)
	return StaticPlaneShape{
		C.CreateStaticPlaneShape(&vec[0], C.float(constant)),
	}
}

type RigidBody struct {
	handle *C.bulletRigidBody
}

func (c RigidBody) ApplyCentralForce(v mgl32.Vec3) {
	vec := vec3ToBullet(v)
	C.RigidBodyApplyCentralForce(c.handle, &vec[0])
}

func (c RigidBody) ApplyForce(v mgl32.Vec3, relativePosition mgl32.Vec3) {
	vec := vec3ToBullet(v)
	r := vec3ToBullet(relativePosition)
	C.RigidBodyApplyForce(c.handle, &vec[0], &r[0])
}

func (c RigidBody) ApplyImpulse(v mgl32.Vec3, relativePosition mgl32.Vec3) {
	vec := vec3ToBullet(v)
	r := vec3ToBullet(relativePosition)
	C.RigidBodyApplyImpulse(c.handle, &vec[0], &r[0])
}

func (c RigidBody) ApplyTorque(v mgl32.Vec3) {
	vec := vec3ToBullet(v)
	C.RigidBodyApplyTorque(c.handle, &vec[0])
}

func (c RigidBody) ApplyTorqueImpulse(v mgl32.Vec3) {
	vec := vec3ToBullet(v)
	C.RigidBodyApplyTorqueImpulse(c.handle, &vec[0])
}

func (c RigidBody) SetLinearVelocity(v mgl32.Vec3) {
	vec := vec3ToBullet(v)
	C.RigidBodySetLinearVelocity(c.handle, &vec[0])
}

func (c RigidBody) GetOrigin() mgl32.Vec3 {
	var out C.Vector3
	C.RigidBodyGetOrigin(c.handle, &out[0])
	return mgl32.Vec3{float32(out[0]), float32(out[1]), float32(out[2])}
}

func (c RigidBody) GetRotation() mgl32.Quat {
	var out C.Vector4
	C.RigidBodyGetRotation(c.handle, &out[0])
	return mgl32.Quat{float32(out[0]), mgl32.Vec3{float32(out[1]), float32(out[2]), float32(out[3])}}
}

func (c RigidBody) SetFromOpenGLMatrix(m mgl32.Mat4) {
	var i C.Mat4
	for x := 0; x < 16; x++ {
		i[x] = C.float(m[x])
	}
	C.RigidBodySetFromOpenGLMatrix(c.handle, &i[0])
}

func NewRigidBody(mass float32, shape CollisionShape) RigidBody {
	return RigidBody{C.CreateRigidBody(nil, C.float(mass), shape.handle)}
}

type DefaultCollisionConfiguration struct {
	handle *C.bulletDefaultCollisionConfiguration
}

func (c DefaultCollisionConfiguration) CollisionConfiguration() CollisionConfiguration {
	return CollisionConfiguration{
		C.GetCollisionConfiguration(c.handle),
	}
}

func NewDefaultCollisionConfiguration() DefaultCollisionConfiguration {
	return DefaultCollisionConfiguration{
		C.CreateDefaultCollisionConfiguration(),
	}
}

type CollisionConfiguration struct {
	handle *C.bulletCollisionConfiguration
}

type CollisionDispatcher struct {
	handle *C.bulletCollisionDispatcher
}

func (c CollisionDispatcher) Dispatcher() Dispatcher {
	return Dispatcher{
		C.GetCollisionDispatcher(c.handle),
	}
}

func NewCollisionDispatcher(config CollisionConfiguration) CollisionDispatcher {
	return CollisionDispatcher{
		C.CreateCollisionDispatcher(config.handle),
	}
}

type AxisSweep3 struct {
	handle *C.bulletAxisSweep3
}

func (c AxisSweep3) BroadphaseInterface() BroadphaseInterface {
	return BroadphaseInterface{
		C.GetAxisSweep3BroadphaseInterface(c.handle),
	}
}

func NewAxisSweep3(min mgl32.Vec3, max mgl32.Vec3) AxisSweep3 {
	mn := vec3ToBullet(min)
	mx := vec3ToBullet(max)
	return AxisSweep3{
		C.CreateAxisSweep3(&mn[0], &mx[0]),
	}
}

type BroadphaseInterface struct {
	handle *C.bulletBroadphaseInterface
}

type ConstraintSolver struct {
	handle *C.bulletConstraintSolver
}

func NewSequentialImpulseConstraintSolver() ConstraintSolver {
	return ConstraintSolver{
		C.CreateSequentialImpulseConstraintSolver(),
	}
}

type Dispatcher struct {
	handle *C.bulletDispatcher
}

type DynamicsWorld struct {
	handle *C.bulletDynamicsWorld
}

func (c DynamicsWorld) SetGravity(v mgl32.Vec3) {
	m := vec3ToBullet(v)
	C.DynamicsWorldSetGravity(c.handle, &m[0])
}

func (c DynamicsWorld) AddRigidBody(body RigidBody) {
	C.DynamicsWorldAddRigidBody(c.handle, body.handle)
}
func (c DynamicsWorld) StepSimulation(t float32) {
	C.DynamicsWorldStepSimulation(c.handle, C.float(t))
}
func (c DynamicsWorld) DebugDrawWorld() ([]float32, []float32) {
	d := C.DynamicsWorldDebugDrawWorld(c.handle)

	count := int(d.count)
	v := make([]float32, count)
	copy(v, (*[1 << 20]float32)(unsafe.Pointer(d.verts))[:])
	cc := make([]float32, count)
	copy(cc, (*[1 << 20]float32)(unsafe.Pointer(d.colors))[:])
	return v, cc
}

func NewDiscreteDynamicsWorld(dispatcher Dispatcher, pairCache BroadphaseInterface, solver ConstraintSolver, config CollisionConfiguration) DynamicsWorld {
	return DynamicsWorld{
		C.CreateDiscreteDynamicsWorld(dispatcher.handle, pairCache.handle, solver.handle, config.handle),
	}
}
