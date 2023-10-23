#ifndef CONTACT_CONTACT_H
#define CONTACT_CONTACT_H

#include <Eigen/Dense>
#include "body/RigidBody.h"

typedef Eigen::Matrix<float, -1, 6, Eigen::RowMajor> JBlock; /// Jacobian matrix
typedef Eigen::Matrix<float, 6, -1, Eigen::ColMajor> JBlockTranspose; /// Transpose of Jacobian matrix

class Contact
{
public:
    Contact(RigidBody* body0, RigidBody* body1, const Eigen::Vector3f& p, const Eigen::Vector3f& n, float penetration);
    virtual ~Contact() = default;

public:
    void computeContactFrame(); /// 计算接触坐标系, 使用接触法向量和提供的方向dir, dir与第一个切向量对齐, 结果存储在基向量n, t1, t2中
    virtual void computeJacobian();


public:
    // -------------------- Friendship -------------------- //
    friend class RigidBody;

public:

    Eigen::Vector3f _point; /// 接触点
    Eigen::Vector3f _normal; /// 接触法向量
    Eigen::Vector3f _tangent1, _tangent2; /// 接触切向量
    float _frictionCoefficient; /// 摩擦系数
    float _penetration; /// 接触深度


    unsigned int _index; /// 用于全局索引的辅助变量

    RigidBody* _body0; /// 第一个刚体
    RigidBody* _body1; /// 第二个刚体

    JBlock _Jacobian0; /// 第一个刚体的接触雅克比矩阵
    JBlock _Jacobian1; /// 第二个刚体的接触雅克比矩阵

    JBlock _Jacobian0Minv; /// 第一个刚体的接触雅克比矩阵乘以质量逆矩阵
    JBlock _Jacobian1Minv; /// 第二个刚体的接触雅克比矩阵乘以质量逆矩阵

    ///
    ///以后可不可以改成pair的形式？
    /*std::pair<RigidBody*, RigidBody*> _bodies; // 第一个刚体和第二个刚体
    std::pair<JBlock, JBlock> _Jacobians; // 第一个刚体的接触雅克比矩阵和第二个刚体的接触雅克比矩阵
    std::pair<JBlock, JBlock> _JacobiansMinv; // 第一个刚体的接触雅克比矩阵乘以质量逆矩阵和第二个刚体的接触雅克比矩阵乘以质量逆矩阵*/
    ///

    Eigen::VectorXf _phi; /// 接触约束误差
    Eigen::VectorXf _lambda; /// 接触约束冲量

    float _contactStiffness; /// 接触刚度（Baumgarte稳定化）
    float _contactDamping; /// 接触阻尼（Baumgarte稳定化）
protected:
    // Default constructor.
    explicit Contact();
};

#endif //CONTACT_CONTACT_H
