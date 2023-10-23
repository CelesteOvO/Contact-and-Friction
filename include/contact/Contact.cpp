#include "Contact.h"

Contact::Contact() : _point(), _normal(), _tangent1(), _tangent2(), _frictionCoefficient(0.4f),
    _body0(nullptr), _body1(nullptr), _contactStiffness(1e6f), _contactDamping(1e5f), _index(-1), _penetration(0.0f)
{

}

Contact::Contact(RigidBody *body0, RigidBody *body1, const Eigen::Vector3f &p, const Eigen::Vector3f &n,
                 float penetration) : _point(p), _normal(n), _tangent1(), _tangent2(), _frictionCoefficient(0.4f),
                                      _body0(body0), _body1(body1), _contactStiffness(1e6f), _contactDamping(1e5f), _index(-1), _penetration(0.0f)
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

void Contact::computeContactFrame() {

}

void Contact::computeJacobian() {


}



