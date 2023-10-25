
#include "RigidBody.h"

int RigidBody::counter = 0;

RigidBody::RigidBody(float _mass, Geometry *_geometry, const std::string &meshFilename) {
    RigidBodyData._bodyTypes = DYNAMIC;
    RigidBodyData._mass = _mass;

    RigidBodyData._position = Eigen::Vector3f(0, 0, 0);
    RigidBodyData._orientation = Eigen::Quaternionf(1, 0, 0, 0);

    RigidBodyData._linearVelocity = Eigen::Vector3f(0, 0, 0);
    RigidBodyData._angularVelocity = Eigen::Vector3f(0, 0, 0);

    RigidBodyData._linearForce = Eigen::Vector3f(0, 0, 0);
    RigidBodyData._torque = Eigen::Vector3f(0, 0, 0);

    RigidBodyData._linearConstraintForce = Eigen::Vector3f(0, 0, 0);
    RigidBodyData._angularConstraintForce = Eigen::Vector3f(0, 0, 0);

    RigidBodyData._geometry = std::unique_ptr<Geometry>(_geometry);
    RigidBodyData._contacts.clear();
    RigidBodyData._mesh = nullptr;

    RigidBodyData._worldI = Eigen::Matrix3f::Zero();
    RigidBodyData._worldIinv = Eigen::Matrix3f::Zero();

    RigidBodyData._localI = RigidBodyData._geometry->computeInertia(RigidBodyData._mass);
    RigidBodyData._localIinv = RigidBodyData._localI.inverse();

    if (!meshFilename.empty()) {
        auto *cachedMesh = MeshAssetRegistry::loadObj(meshFilename);
        if (cachedMesh != nullptr) {

            RigidBodyData._mesh = polyscope::registerSurfaceMesh(std::to_string(RigidBody::counter),
                                                                 cachedMesh->meshV, cachedMesh->meshF);
            RigidBodyData._mesh->setSmoothShade(true);
        }
    }
    RigidBodyData._contacts.clear();
    RigidBody::counter++;
}

void RigidBody::updateInertiaMatrix() {
    if (RigidBodyData._bodyTypes == STATIC) {
        RigidBodyData._worldIinv.setZero();
    } else {
        RigidBodyData._worldI = RigidBodyData._orientation * RigidBodyData._localI *
                                RigidBodyData._orientation.inverse();
        RigidBodyData._worldIinv = RigidBodyData._orientation * RigidBodyData._localIinv *
                                   RigidBodyData._orientation.inverse();
    }
}

void RigidBody::addForceAtPos(const Eigen::Vector3f &pos, const Eigen::Vector3f &force) {
    const Eigen::Vector3f r = pos - RigidBodyData._position;
    RigidBodyData._linearForce += force;
    RigidBodyData._torque += r.cross(force);
}

void RigidBody::getVelocityAtPos(const Eigen::Vector3f &pos, Eigen::Vector3f &vel) {
    const Eigen::Vector3f r = pos - RigidBodyData._position;
    vel = RigidBodyData._linearVelocity + RigidBodyData._angularVelocity.cross(r);
}

