#ifndef CONTACT_SCENARIOS_H
#define CONTACT_SCENARIOS_H

#include "RigidBody.h"
#include "engine/PhysicsWorld.h"
#include <Eigen/Dense>

class Scenarios
{
    public:
        Scenarios() = default;
        ~Scenarios() = default;

        static void createMarbleBox();
        static void createSphereOnBox();
};

#endif //CONTACT_SCENARIOS_H
