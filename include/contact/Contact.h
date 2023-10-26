#ifndef CONTACT_CONTACT_H
#define CONTACT_CONTACT_H

#include <Eigen/Dense>
#include "body/RigidBody.h"

typedef Eigen::Matrix<float, -1, 6, Eigen::RowMajor> JBlock; // Jacobian matrix
typedef Eigen::Matrix<float, 6, -1, Eigen::ColMajor> JBlockTranspose; // Transpose of Jacobian matrix

class RigidBody;

class Contact
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    // Constructor with all parameters.
    Contact(RigidBody* body0, RigidBody* body1, const Eigen::Vector3f& p, const Eigen::Vector3f& n, float _phi);
    virtual ~Contact() = default;

public:
    void computeContactFrame();
    virtual void computeJacobian();

public:

    Eigen::Vector3f _point;
    Eigen::Vector3f _normal;
    Eigen::Vector3f _tangent1, _tangent2;
    float _penetration;


    unsigned int _index;

    RigidBody* _body0;
    RigidBody* _body1;

    JBlock _Jacobian0;
    JBlock _Jacobian1;

    JBlock _Jacobian0Minv;
    JBlock _Jacobian1Minv;

    Eigen::VectorXf _phi;
    Eigen::VectorXf _lambda;

    float _frictionCoefficient;
    float _contactStiffness;
    float _contactDamping;

protected:
    explicit Contact();
};

#endif //CONTACT_CONTACT_H
