#ifndef CONTACT_PHYSICSWORLD_H
#define CONTACT_PHYSICSWORLD_H

class PhysicsWorld
{
public:
    PhysicsWorld();
    virtual ~PhysicsWorld();

    void step(float _dt);
    void clear();

    void addRigidBody(RigidBody* _body);

    // -------------------- Friendship -------------------- //
    friend class RigidBody;
};
#endif //CONTACT_PHYSICSWORLD_H
