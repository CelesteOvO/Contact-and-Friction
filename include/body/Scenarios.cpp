#include "Scenarios.h"


void Scenarios::createMarbleBox(PhysicsWorld& physicsWorld) {
    physicsWorld.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading marble box scenario" << std::endl;

    // Create two layers of "marbles", in a grid layout
    for (int i = 0; i < 9; ++i)
    {
        for (int j = 0; j < 9; ++j)
        {
            auto* body1 = new RigidBody(1.0f, new Sphere(0.5f), "D:\\project\\rigidBodyTutorial\\resources\\sphere.obj");
            body1->RigidBodyData._position.x() = -4.0f + (float)i * 1.0f;
            body1->RigidBodyData._position.z() = -4.0f + (float)j * 1.0f;
            body1->RigidBodyData._position.y() = 2.0f;
            body1->RigidBodyData._bodyTypes = DYNAMIC;
            physicsWorld.addRigidBody(body1);
            body1->RigidBodyData._mesh->setSurfaceColor({ 1.0f, 0.1f, 0.1f });
            body1->RigidBodyData._mesh->setTransparency(0.8f);
            auto* body2 = new RigidBody(1.0f, new Sphere(0.5f), "D:\\project\\rigidBodyTutorial\\resources\\sphere.obj");
            body2->RigidBodyData._position.x() = -4.0f + (float)i * 1.0f;
            body2->RigidBodyData._position.z() = -4.0f + (float)j * 1.0f;
            body2->RigidBodyData._position.y() = 3.0f;
            body2->RigidBodyData._bodyTypes = DYNAMIC;
            physicsWorld.addRigidBody(body2);
            body2->RigidBodyData._mesh->setSurfaceColor({ 1.0f, 0.1f, 0.1f });
            body2->RigidBodyData._mesh->setTransparency(0.8f);
        }
    }
    auto* body0 = new RigidBody(1.0f, new Box(Eigen::Vector3f(0.4f, 4.0f, 10.0f)), "D:\\project\\rigidBodyTutorial\\resources\\box_side.obj");
    auto* body1 = new RigidBody(1.0f, new Box(Eigen::Vector3f(0.4f, 4.0f, 10.0f)), "D:\\project\\rigidBodyTutorial\\resources\\box_side.obj");
    auto* body2 = new RigidBody(1.0f, new Box(Eigen::Vector3f(0.4f, 4.0f, 10.0f)), "D:\\project\\rigidBodyTutorial\\resources\\box_side.obj");
    auto* body3 = new RigidBody(1.0f, new Box(Eigen::Vector3f(0.4f, 4.0f, 10.4f)), "D:\\project\\rigidBodyTutorial\\resources\\box_side.obj");
    auto* body4 = new RigidBody(1.0f, new Box(Eigen::Vector3f(10.0f, 0.4f, 10.0f)), "D:\\project\\rigidBodyTutorial\\resources\\box_bot.obj");
    body0->RigidBodyData._bodyTypes = STATIC;
    body1->RigidBodyData._bodyTypes = STATIC;
    body2->RigidBodyData._bodyTypes = STATIC;
    body3->RigidBodyData._bodyTypes = STATIC;
    body4->RigidBodyData._bodyTypes = STATIC;
    body0->RigidBodyData._mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setSmoothShade(false)->setTransparency(0.4f);
    body1->RigidBodyData._mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setSmoothShade(false)->setTransparency(0.4f);
    body2->RigidBodyData._mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setSmoothShade(false)->setTransparency(0.4f);
    body3->RigidBodyData._mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setSmoothShade(false)->setTransparency(0.4f);
    body4->RigidBodyData._mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setSmoothShade(false)->setTransparency(0.4f);
    body0->RigidBodyData._position.x() = 4.75f;
    body1->RigidBodyData._position.x() = -4.75f;
    body2->RigidBodyData._position.z() = 4.75f;
    body2->RigidBodyData._orientation = Eigen::AngleAxisf(1.57, Eigen::Vector3f(0, 1, 0));
    body3->RigidBodyData._position.z() = -4.75f;
    body3->RigidBodyData._orientation = Eigen::AngleAxisf(1.57, Eigen::Vector3f(0, 1, 0));
    body4->RigidBodyData._position.y() = -2.0f;

    physicsWorld.addRigidBody(body0);
    physicsWorld.addRigidBody(body1);
    physicsWorld.addRigidBody(body2);
    physicsWorld.addRigidBody(body3);
    physicsWorld.addRigidBody(body4);
}

void Scenarios::createSphereOnBox(PhysicsWorld& physicsWorld) {
    physicsWorld.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading sphere on box scenario" << std::endl;

    // Create a sphere.
    auto* bodySphere = new RigidBody(1.0f, new Sphere(0.5f), "D:\\project\\rigidBodyTutorial\\resources\\sphere.obj");
    bodySphere->RigidBodyData._position.y() = 4.0f;
    bodySphere->RigidBodyData._angularVelocity = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
    bodySphere->RigidBodyData._mesh->setTransparency(0.8f);
    bodySphere->RigidBodyData._bodyTypes = DYNAMIC;

    // Create a box.
    auto* bodyBox = new RigidBody(1.0f, new Box(Eigen::Vector3f(10.0f, 0.4f, 10.0f)), "D:\\project\\rigidBodyTutorial\\resources\\box_bot.obj");
    bodyBox->RigidBodyData._bodyTypes = STATIC;

    physicsWorld.addRigidBody(bodySphere);
    physicsWorld.addRigidBody(bodyBox);

    bodySphere->RigidBodyData._mesh->setSurfaceColor({ 0.1f, 1.0f, 0.2f })->setEdgeWidth(1.0f);
    bodyBox->RigidBodyData._mesh->setSurfaceColor({ 0.2f, 0.2f, 0.2f })->setSmoothShade(false)->setTransparency(0.4f);
}
