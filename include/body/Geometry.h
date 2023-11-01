#ifndef CONTACT_GEOMETRY_H
#define CONTACT_GEOMETRY_H

#include <Eigen/Dense>
#include <utility>

enum eGeometryType {
    kSphere,
    kBox };

class Geometry
{
public:

    virtual ~Geometry() = default;

    virtual Eigen::Matrix3f computeInertia(float _mass) = 0;

    virtual float getDiagonal() = 0;

public:
    eGeometryType _type;
    Eigen::Matrix3f m_I;          // Inertia 3x3 matrix for this. Only used for local computations. (internal)
};

class Sphere : public Geometry
{
public:

    Sphere(float _radius) : radius(_radius), _type(kSphere) {} // Initialize _type in the constructor.
    ~Sphere() override = default;

    Eigen::Matrix3f computeInertia(float _mass) override
    {
        m_I.setZero();
        m_I(0,0) = m_I(1,1) = m_I(2,2) = (2.0f/5.0f) * _mass * radius * radius;
        return m_I;
    }
    float getDiagonal() override {
        return radius;
    }
public:
    float radius;           // Sphere radius.
    eGeometryType _type;    // Adding eGeometryType member.
};

class Box : public Geometry
{
public:
    Box(Eigen::Vector3f  _dim) : dim(std::move(_dim)), _type(kBox) {
        halfSize = dim / 2.0f;
    } // Initialize _type in the constructor.
    ~Box() override = default;

    Eigen::Matrix3f computeInertia(float _mass) override
    {
        m_I.setZero();
        m_I(0,0) = (1.0f/12.0f) * _mass * (dim.y() * dim.y() + dim.z() * dim.z());
        m_I(1,1) = (1.0f/12.0f) * _mass * (dim.x() * dim.x() + dim.z() * dim.z());
        m_I(2,2) = (1.0f/12.0f) * _mass * (dim.x() * dim.x() + dim.y() * dim.y());
        return m_I;
    }

    float getDiagonal() {
        return dim.norm();
    }
public:
    Eigen::Vector3f halfSize;
    Eigen::Vector3f dim;        // Box dimensions.
    eGeometryType _type;        // Adding eGeometryType member.
};

#endif //CONTACT_GEOMETRY_H
