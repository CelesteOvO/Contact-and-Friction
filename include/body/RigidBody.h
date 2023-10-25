#ifndef CONTACT_RIGIDBODY_H
#define CONTACT_RIGIDBODY_H

#include "Geometry.h"
#include "contact/Contact.h"
#include <Eigen/Dense>
#include <vector>

namespace polyscope
{
    class SurfaceMesh;
}

enum BodyType {
    STATIC,
    KINEMATIC,
    DYNAMIC
};

class RigidBody
{
public:
    RigidBody(float _mass, Geometry* _geometry, const std::string& meshFilename = "");

public:
    void updateInertiaMatrix();
    void addForceAtPos(const Eigen::Vector3f& pos, const Eigen::Vector3f& force);
    void getVelocityAtPos(const Eigen::Vector3f& pos, Eigen::Vector3f& vel);

public:
    // -------------------- Friendship -------------------- //
    friend class Contact;
public:
    struct
    {
        BodyType _bodyTypes; /// 0: static, 1: kinematic, 2: dynamic
        float _mass; /// 质量

        Eigen::Matrix3f _worldI, _worldIinv; // 冲量和逆冲量矩阵（全局）
        Eigen::Matrix3f _localI, _localIinv; // 冲量和逆冲量矩阵（局部）

        Eigen::Vector3f _position; // 位置
        Eigen::Quaternionf _orientation; // 方向

        Eigen::Vector3f _linearVelocity; // 线速度
        Eigen::Vector3f _angularVelocity; // 角速度

        Eigen::Vector3f _linearForce; // 线性力
        Eigen::Vector3f _torque; // 角力（扭矩）

        Eigen::Vector3f _linearConstraintForce; // 约束力
        Eigen::Vector3f _angularConstraintForce; // 角约束力

        std::unique_ptr<Geometry> _geometry; // 刚体的几何形状
        std::vector<Contact*> _contacts; // 指向刚体的接触约束的指针

        polyscope::SurfaceMesh* _mesh; // 用于渲染
    }RigidBodyData;

};

#endif //CONTACT_RIGIDBODY_H
