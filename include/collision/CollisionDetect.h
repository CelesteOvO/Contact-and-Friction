#ifndef CONTACT_COLLISIONDETECT_H
#define CONTACT_COLLISIONDETECT_H

#include <Eigen/Dense>
#include <vector>
#include "engine/PhysicsWorld.h"
#include "body/RigidBody.h"
#include "contact/Contact.h"

class Contact;
class RigidBody;
class PhysicsWorld;

class CollisionDetect
{
public:
    explicit CollisionDetect(PhysicsWorld* physicsWorld);
    virtual ~CollisionDetect() = default;

    void detectCollisions();

    void clear();

    void computeContactJacobians();

public:
    void collisionDetectSphereSphere(RigidBody* body0, RigidBody* body1);
    void collisionDetectSphereBox(RigidBody* body0, RigidBody* body1);
    void collisionDetectBoxBox(RigidBody* body0, RigidBody* body1);

    /// further
    float penetrationOnAxis(RigidBody *pBody, RigidBody *pBody1, const Eigen::Vector3f& matrix);
    static float transformToAxis(RigidBody *pBody, const Eigen::Vector3f& axis);
    void collisionDetectFaceVertex(RigidBody* body0, RigidBody* body1, Eigen::Vector3f axis, float penetration);
    void collisionDetectEdgeEdge(RigidBody* body0, RigidBody* body1, Eigen::Vector3f axis, float penetration, int oneAxisIndex, int twoAxisIndex);


public:
    PhysicsWorld* _physicsWorld;
    std::vector<Contact*> _contacts;


};

#endif //CONTACT_COLLISIONDETECT_H
