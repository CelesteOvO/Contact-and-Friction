#ifndef CONTACT_PHYSICSWORLD_H
#define CONTACT_PHYSICSWORLD_H

#include "body/RigidBody.h"
#include "collision/CollisionDetect.h"

#include <Eigen/Dense>
#include <memory>
#include <utility>
#include <vector>

class PhysicsWorld
{
public:
    PhysicsWorld();
    virtual ~PhysicsWorld();

    void addRigidBody(RigidBody* b);

    void step(float dt);
    void clear();

public:
    // Callbacks
    typedef std::function<void(std::vector<RigidBody*>&)> PreStepFunc;
    typedef std::function<void()> ResetFunc;
    void setPreStepFunc(PreStepFunc _func) { _preStepFunc = std::move(_func); }
    void setResetFunc(ResetFunc _func) { _resetFunc = std::move(_func); }

public:
    /// 这部分属性在Contact.h中也有定义,如果那边没有用到的话，建议把那边的删除
    struct{
        float _contactStiffness; /// 接触刚度, 用于计算Baumgarte稳定化参数
        float _contactDamping; /// 接触阻尼, 用于计算Baumgarte稳定化参数
        float _frictionCoefficient; /// 摩擦系数
        int _solverIter; /// 求解器迭代次数
    }physicsWorldData;

private:
    void computeInertias(); /// 计算所有刚体的惯性矩阵, 这个函数也会更新旋转矩阵
    void calcConstraintForces(float dt) const; /// 计算约束力

public:
    std::vector<RigidBody*> _bodies;
    std::unique_ptr<CollisionDetect> _collisionDetect;
    //Solver* _solver;

    PreStepFunc _preStepFunc;      /// 用于在每一步模拟之前调用的函数
    ResetFunc _resetFunc;          /// 用于重置系统的函数
};

template<typename ScalarType, int Options>
Eigen::Quaternion<ScalarType, Options> operator*(ScalarType a, const Eigen::Quaternion<ScalarType, Options>& q)
{
    return Eigen::Quaternion<ScalarType, Options>(a*q.w(),a*q.x(),a*q.y(),a*q.z());
}

template<typename ScalarType, int Options>
Eigen::Quaternion<ScalarType, Options> operator+(const Eigen::Quaternion<ScalarType, Options>& q1, const Eigen::Quaternion<ScalarType, Options>& q2)
{
    return Eigen::Quaternion<ScalarType, Options>(q1.w()+q2.w(),q1.x()+q2.x(),q1.y()+q2.y(),q1.z()+q2.z());
}

#endif //CONTACT_PHYSICSWORLD_H
