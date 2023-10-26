#include <memory>

#include "viewer/viewer.h"

using namespace std;

/// 在刚体系统中更新网格模型的位置和接触点的可视化信息
namespace
{
    Eigen::Vector3f sImpulseDirs[8] = { // 8 directions
            Eigen::Vector3f(1, 0, 0),
            Eigen::Vector3f(0, 0, -1),
            Eigen::Vector3f(-1, 0, 0),
            Eigen::Vector3f(0, 0, 1),
            Eigen::Vector3f(0.71f, 0, 0.71f),
            Eigen::Vector3f(0.71f, 0, -0.71f),
            Eigen::Vector3f(-0.71f, 0, 0.71f),
            Eigen::Vector3f(-0.71f, 0, -0.71f)
    };

    void updateRigidBodyMeshes(PhysicsWorld& _physicsWorld) // update the position of the rigid body meshes
    {
        auto& bodies = _physicsWorld._bodies;
        for(auto & bodie : bodies)
        {
            Eigen::Isometry3f tm = Eigen::Isometry3f::Identity(); // 4x4 matrix
            tm.linear() = bodie->RigidBodyData._orientation.toRotationMatrix(); // rotation matrix
            tm.translation() = bodie->RigidBodyData._position; // translation vector (position)
            bodie->RigidBodyData._mesh->setTransform( glm::make_mat4x4(tm.data()) ); // set the transformation matrix
        }
    }

    void updateContactPoints(PhysicsWorld& _physicsWorld) // update the contact points
    {
        const auto& contacts = _physicsWorld._collisionDetect->_contacts; // get the contact points
        const unsigned int numContacts = contacts.size(); // get the number of contact points
        Eigen::MatrixXf contactP(numContacts, 3); // 3D position
        /// 行数为 numContacts,列数为 3 的矩阵,用于存储接触点的位置,每一行代表一个接触点的位置，每一列代表一个坐标轴（x、y、z）
        Eigen::MatrixXf contactN(numContacts, 3); // 3D normal
        /// 行数为 numContacts,列数为 3 的矩阵,用于存储接触点的法向量,每一行代表一个接触点的法向量，每一列代表一个坐标轴（x、y、z）

        for (unsigned int i = 0; i < numContacts; ++i) // for each contact point
        {
            contactP.row(i) = contacts[i]->_point.transpose(); // position
            /// 列向量转置为行向量
            contactN.row(i) = contacts[i]->_point.transpose(); // normal
        }

        auto pointCloud = polyscope::registerPointCloud("contacts", contactP); // register the point cloud
        pointCloud->setPointColor({ 1.0f, 0.0f, 0.0f }); // set the color of the point cloud
        pointCloud->setPointRadius(0.005); // set the radius of the point cloud
        pointCloud->addVectorQuantity("normal", contactN)->setVectorColor({ 1.0f, 1.0f, 0.0f })->setVectorLengthScale(0.05f)->setEnabled(true); // add the normal vector
    }
}

SimViewer::SimViewer() :
        _dt(0.0167f), _subSteps(1),
        _paused(true), _stepOnce(false),
        _physicsWorld()
{
}

void SimViewer::start()
{
    // Create the rigid body system and renderer
    _physicsWorld = std::make_unique<PhysicsWorld>();

    // Setup Polyscope
    polyscope::options::programName = "Rigid Body Tutorial"; // set the program name
    polyscope::options::verbosity = 0; // set the verbosity level
    polyscope::options::usePrefsFile = false; // use the preferences file
    polyscope::options::alwaysRedraw = true; // always redraw
    polyscope::options::openImGuiWindowForUserCallback = true; // open the ImGui window for user callback
    polyscope::options::buildGui = false;
    polyscope::options::maxFPS = 60;
    polyscope::options::groundPlaneEnabled = false;

    // initialize
    polyscope::init();

    // Specify the update callback
    polyscope::state::userCallback = std::bind(&SimViewer::draw, this);
    /// std::bind() 函数的作用是将一个函数绑定到一个对象上，从而生成一个可调用的对象
    /// 将 SimViewer 类的 draw 函数与 polyscope 库中的用户回调函数进行绑定，以便在适当的时候执行 draw 函数。用于在可视化程序中绘制或更新图形内容。

    // Add pre-step hook.
    _physicsWorld->setPreStepFunc(std::bind(&SimViewer::preStep, this, std::placeholders::_1));
    /// std::placeholders::_1 表示占位符，表示第一个参数，即 std::vector<RigidBody*>& _bodies
    /// 将 SimViewer 类的 preStep 函数与 m_rigidBodySystem 对象的预步进函数相关联。当 m_rigidBodySystem 执行预步进时，会调用 SimViewer 类的 preStep 函数，并可能传递一个参数（取决于 preStep 函数的定义）。
    /// 这样可以实现在模拟过程中在预步进时执行特定的操作或计算

    createSphereOnBox(); // create a sphere on a box

    // Show the window
    polyscope::show();

}

void SimViewer::drawGUI()
{
    ImGui::Text("Simulation:");
    ImGui::Checkbox("Pause", &_paused);
    if (ImGui::Button("Step once"))
    {
        _stepOnce = true;
    }
    ImGui::PushItemWidth(100);
    ImGui::SliderFloat("Time step", &_dt, 0.0f, 0.1f, "%.3f");
    ImGui::SliderInt("Num. sub-steps", &_subSteps, 1, 20, "%u");
    ImGui::SliderInt("Solver iters.", &(_physicsWorld->physicsWorldData._solverIter), 1, 100, "%u");
    ImGui::SliderFloat("Friction coeff.", &(_physicsWorld->physicsWorldData._frictionCoefficient), 0.0f, 2.0f, "%.2f");
    ImGui::PopItemWidth();

    if (ImGui::Button("Sphere on box")) {
        createSphereOnBox();
    }
    if (ImGui::Button("Marble box")) {
        createMarbleBox();
    }
}

void SimViewer::draw()
{
    drawGUI();

    if( !_paused || _stepOnce )
    {
        // Step the simulation.
        // The time step dt is divided by the number of sub-steps.

        const float dt = _dt / (float)_subSteps; // time step
        for(int i = 0; i < _subSteps; ++i) // for each sub-step
        {
            _physicsWorld->step(dt); // step the rigid body system
        }

        updateRigidBodyMeshes(*_physicsWorld);
        updateContactPoints(*_physicsWorld);


        // Clear step-once flag.
        _stepOnce = false;
        /// 用于在模拟过程中暂停模拟，或者在模拟过程中单步执行模拟
    }
}

void SimViewer::createMarbleBox()
{
    Scenarios::createMarbleBox(*_physicsWorld);
    updateRigidBodyMeshes(*_physicsWorld);
    polyscope::view::resetCameraToHomeView(); /// 将视图的相机重置为“主视图”
}

void SimViewer::createSphereOnBox()
{
    Scenarios::createSphereOnBox(*_physicsWorld);
    updateRigidBodyMeshes(*_physicsWorld);
    polyscope::view::resetCameraToHomeView();
}

/// 留出一个接口，用于在模拟过程中执行特定的操作或计算
void SimViewer::preStep(std::vector<RigidBody*>& _bodies)
{

}

