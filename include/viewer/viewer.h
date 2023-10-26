#ifndef CONTACT_AND_FRICTION_VIEWER_H
#define CONTACT_AND_FRICTION_VIEWER_H
#pragma once

#include <Eigen/Dense>
#include <vector>

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/view.h"
#include "imgui.h"

#include <iostream>
#include <functional>

#include "contact/Contact.h"
#include "engine/PhysicsWorld.h"
#include "body/Scenarios.h"

namespace polyscope // polyscope is a visualization toolkit
{
    class SurfaceMesh; // SurfaceMesh is a class for storing and visualizing triangle meshes
    class PointCloud; // PointCloud is a class for storing and visualizing point clouds
}

class Contact; // storing contact information
class PhysicsWorld; // storing rigid body information
class RigidBody; // storing rigid body information

class SimViewer // visualizing a simulation
{
public:
    SimViewer();
    virtual ~SimViewer() = default;

    void start(); // start the simulation

private:
    /*
    void setTimestep(float); // set the time step
    void setSubsteps(int); // set the number of substeps
    void setMaxIterations(int); // set the maximum number of iterations
    /// Should it not be set here?
    void setFrictionCoefficient(float); // set the friction coefficient
    void setContactStiffness(float); // set the contact stiffness
    void setContactDamping(float); // set the contact damping
    void setPaused(bool); // set the simulation to be paused
    void stepOnce(); // step the simulation once
    */

    void createMarbleBox(); // create a marble box
    void createSphereOnBox(); // create a sphere on a box

    void draw(); // draw the simulation
    void drawGUI(); // draw the GUI

    void preStep(std::vector<RigidBody*>&); // pre-step

private:

    // Simulation parameters
    float _dt;                         //< Time step parameter.
    int _subSteps;
    bool _paused;                      //< Pause the simulation.
    bool _stepOnce;                    //< Advance the simulation by one frame and then stop.

    std::unique_ptr<PhysicsWorld> _physicsWorld;
};

#endif //CONTACT_AND_FRICTION_VIEWER_H
