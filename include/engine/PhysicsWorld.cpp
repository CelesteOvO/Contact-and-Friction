#include "PhysicsWorld.h"

PhysicsWorld::PhysicsWorld() : m_preStepFunc(nullptr), m_resetFunc(nullptr)
{
    physicsWorldData._contactStiffness = 1e6f;
    physicsWorldData._contactDamping = 1e5f;
    physicsWorldData._frictionCoefficient = 0.4f;
    physicsWorldData._solverIter = 10;

    /*_collisionDetect = std::make_unique<CollisionDetect>(this);
    _solver = new SolverBoxPGS(this);*/
}

PhysicsWorld::~PhysicsWorld()
{
    clear();
}

void PhysicsWorld::addRigidBody(RigidBody *b) {
    _bodies.push_back(b);
}

void PhysicsWorld::step(float dt) {
    /// 1. 初始化系统，应用重力，重置角动量，清除上一步的接触点
    for (auto b : _bodies) {
        b->RigidBodyData._linearForce = b->RigidBodyData._mass * Eigen::Vector3f(0.f, -9.81f, 0.f);
        b->RigidBodyData._torque.setZero();
        b->RigidBodyData._linearConstraintForce.setZero();
        b->RigidBodyData._angularConstraintForce.setZero();
        b->RigidBodyData._contacts.clear();
    }

    /// 2. 计算所有刚体的惯性矩阵
    computeInertias();

    /// 执行每一步模拟之前的回调函数
    if( m_preStepFunc )
    {
        m_preStepFunc(_bodies);
    }

    /// 3. 检测碰撞，计算接触的雅可比矩阵
    //_collisionDetect->detectCollision();
    //_collisionDetect->computeContactJacobians();

    /// 更新所有接触的接触刚度和阻尼

}




