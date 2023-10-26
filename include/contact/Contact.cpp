#include "Contact.h"

Contact::Contact() : _point(), _normal(), _tangent1(), _tangent2(), _frictionCoefficient(0.4f),
                     _body0(nullptr), _body1(nullptr), _contactStiffness(1e6f), _contactDamping(1e5f), _index(-1), _penetration(0.0f)
{
}

Contact::Contact(RigidBody* body0, RigidBody* body1, const Eigen::Vector3f& p, const Eigen::Vector3f& n, float penetration): _point(p), _normal(n), _tangent1(), _tangent2(), _penetration(0.0f), _frictionCoefficient(0.4f),
                                                                                                                             _body0(body0), _body1(body1), _contactStiffness(1e6f), _contactDamping(1e5f), _index(-1)
{
    _Jacobian0.setZero(3, 6);
    _Jacobian1.setZero(3, 6);
    _Jacobian0Minv.setZero(3, 6);
    _Jacobian1Minv.setZero(3, 6);

    _phi.setZero(3);
    _phi(0) = penetration;
    _lambda.setZero(3);

    _body0->RigidBodyData._contacts.push_back(this);
    _body1->RigidBodyData._contacts.push_back(this);
}

/// TODO: 3. 计算接触点的切线方向
void Contact::computeContactFrame() {
    /// 计算切线方向
    Eigen::Vector3f tangent1 = Eigen::Vector3f::Zero();
    Eigen::Vector3f tangent2 = Eigen::Vector3f::Zero();
    Eigen::Vector3f n = _normal;

    if (n.x() == 0 && n.y() == 0) { /// 如果法向量的x和y分量都为0，那么法向量的z分量必然不为0, 所以切线方向为(1, 0, 0)
        tangent1 = Eigen::Vector3f(1, 0, 0);
    } else { /// 否则，切线方向为(n.y, -n.x, 0)
        tangent1 = Eigen::Vector3f(n.y(), -n.x(), 0);
    }
    tangent1.normalize();
    tangent2 = n.cross(tangent1);
    tangent2.normalize();

    _tangent1 = tangent1;
    _tangent2 = tangent2;

}

/// TODO: 4. 计算接触点的雅可比矩阵
void Contact::computeJacobian() {
    /// 计算刚体0的雅可比矩阵
    Eigen::Vector3f r0 = _point - _body0->RigidBodyData._x;
    Eigen::Vector3f r1 = _point - _body1->RigidBodyData._x;

    Eigen::Matrix3f skew0 = Eigen::Matrix3f::Zero();
    skew0 << 0, -r0.z(), r0.y(),
            r0.z(), 0, -r0.x(),
            -r0.y(), r0.x(), 0;

    Eigen::Matrix3f skew1 = Eigen::Matrix3f::Zero();
    skew1 << 0, -r1.z(), r1.y(),
            r1.z(), 0, -r1.x(),
            -r1.y(), r1.x(), 0;

    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();

    Eigen::Matrix3f R0 = _body0->RigidBodyData._q.toRotationMatrix();
    Eigen::Matrix3f R1 = _body1->RigidBodyData._q.toRotationMatrix();

    Eigen::Matrix3f Jv0 = -I;
    Eigen::Matrix3f Jv1 = I;

    Eigen::Matrix3f Jw0 = -R0 * skew0;
    Eigen::Matrix3f Jw1 = R1 * skew1;

    _Jacobian0.block(0, 0, 3, 3) = Jv0;
    _Jacobian0.block(0, 3, 3, 3) = Jw0;
    _Jacobian1.block(0, 0, 3, 3) = Jv1;
    _Jacobian1.block(0, 3, 3, 3) = Jw1;

    /// 计算刚体0的雅可比矩阵乘以质量逆矩阵
    _Jacobian0Minv.block(0,0,3,3) = (1.0f/_body0->RigidBodyData._mass) * _Jacobian0.block(0, 0, 3, 3);
    _Jacobian0Minv.block(0,3,3,3) = _Jacobian0.block(0, 3, 3, 3) * _body0->RigidBodyData._worldIinv;
    _Jacobian1Minv.block(0,0,3,3) = (1.0f/_body1->RigidBodyData._mass) * _Jacobian1.block(0, 0, 3, 3);
    _Jacobian1Minv.block(0,3,3,3) = _Jacobian1.block(0, 3, 3, 3) * _body1->RigidBodyData._worldIinv;
}




