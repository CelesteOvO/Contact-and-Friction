#include "PhysicsWorld.h"

namespace Eigen
{
    typedef Matrix<float, 6, 1, ColMajor> Vector6f;
    typedef Matrix<float, 4, 3, ColMajor> KinematicMap;
}

namespace
{
    inline Eigen::Quaternionf kinematicMap(const Eigen::Quaternionf& q, const Eigen::Vector3f& omega)
    {
        return 0.5f * Eigen::Quaternionf(
                -q.x() * omega.x() - q.y() * omega.y() - q.z() * omega.z(),
                q.w() * omega.x() + q.z() * omega.y() - q.y() * omega.z(),
                -q.z() * omega.x() + q.w() * omega.y() + q.x() * omega.z(),
                -q.y() * omega.x() - q.x() * omega.y() + q.w() * omega.z()
        );
    }
}

PhysicsWorld::PhysicsWorld() : _preStepFunc(nullptr), _resetFunc(nullptr)
{
    physicsWorldData._contactStiffness = 1e6f;
    physicsWorldData._contactDamping = 1e5f;
    physicsWorldData._frictionCoefficient = 0.4f;
    physicsWorldData._solverIter = 10;

    _collisionDetect = std::make_unique<CollisionDetect>(this);
    //_solver = new SolverBoxPGS(this);
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
    if( _preStepFunc )
    {
        _preStepFunc(_bodies);
    }

    /// 3. 检测碰撞，计算接触的雅可比矩阵
    _collisionDetect->detectCollisions(); /// 把所有的接触点都存储在了 _contacts 中

    /// TODO 基于约束的
    //_collisionDetect->computeContactJacobians();

    /// TODO 基于脉冲的


    /// 4. 更新所有接触的接触刚度和阻尼
    for(auto c : _collisionDetect->_contacts)
    {
        c->_contactStiffness = physicsWorldData._contactStiffness;
        c->_contactDamping = physicsWorldData._contactDamping;
        c->_frictionCoefficient = physicsWorldData._frictionCoefficient;
    }

    for(auto b : _bodies)
    {
        b->RigidBodyData._linearConstraintForce.setZero();
        b->RigidBodyData._angularConstraintForce.setZero();
    }

    /// TODO 基于约束的
    /// 5. 计算约束力
    //calcConstraintForces(dt);

    /// TODO 基于脉冲的
    /// 5. 计算惩罚力

    /// TODO: 1. 更新刚体的位置和旋转
    /// 6. 更新刚体的线速度和角速度
    for(auto b : _bodies)
    {
        if(b->RigidBodyData._bodyTypes == STATIC)
            continue;
        b->RigidBodyData._linearVelocity += dt * b->RigidBodyData._linearForce / b->RigidBodyData._mass;
        b->RigidBodyData._angularVelocity += dt * b->RigidBodyData._worldIinv * (b->RigidBodyData._torque - b->RigidBodyData._angularVelocity.cross(b->RigidBodyData._worldI * b->RigidBodyData._angularVelocity));
        /// TODO: 2. 更新刚体的位置和旋转
        b->RigidBodyData._position += dt * b->RigidBodyData._linearVelocity;
        b->RigidBodyData._orientation = dt * kinematicMap(b->RigidBodyData._orientation, b->RigidBodyData._angularVelocity) * b->RigidBodyData._orientation;
    }

}

void PhysicsWorld::clear() {
    if( _resetFunc)
    {
        _resetFunc();
    }

    _collisionDetect->clear();

    for(auto b : _bodies)
    {
        delete b;
    }
    _bodies.clear();
}

void PhysicsWorld::computeInertias() {
    for(RigidBody* b : _bodies)
    {
        b->updateInertiaMatrix();
    }
}

void PhysicsWorld::calcConstraintForces(float dt) const {
    //_solver->_maxIter = physicsWorldData._solverIter;
    //_solver->solve(dt);

    for(const auto c : _collisionDetect->_contacts)
    {
        const Eigen::Vector6f f0 = c->_Jacobian0.transpose() * c->_lambda / dt;
        const Eigen::Vector6f f1 = c->_Jacobian1.transpose() * c->_lambda / dt;

        if (c->_body0->RigidBodyData._bodyTypes != STATIC)
        {
            c->_body0->RigidBodyData._linearConstraintForce += f0.head<3>();
            c->_body0->RigidBodyData._angularConstraintForce += f0.tail<3>();
        }
        if (c->_body1->RigidBodyData._bodyTypes != STATIC)
        {
            c->_body1->RigidBodyData._linearConstraintForce += f1.head<3>();
            c->_body1->RigidBodyData._angularConstraintForce += f1.tail<3>();
        }
    }
}










