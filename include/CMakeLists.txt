cmake_minimum_required(VERSION 3.25)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#[[set(collision_HEADERS collision/CollisionDetect.h
        collision/Geometry.h )
set(collision_SOURCE collision/CollisionDetect.cpp )

set(rigidbody_HEADERS rigidbody/RigidBody.h
        rigidbody/RigidBodySystem.h
        rigidbody/Scenarios.h )
set(rigidbody_SOURCE rigidbody/RigidBody.cpp
        rigidbody/RigidBodySystem.cpp )

set(contact_HEADERS contact/Contact.h )
set(contact_SOURCE contact/Contact.cpp )

set(solver_HEADERS solvers/Solver.h
        solvers/SolverBoxPGS.h )
set(solver_SOURCE solvers/SolverBoxPGS.cpp )

set(util_HEADERS util/OBJLoader.h
        util/MeshAssets.h )
set(util_SOURCE util/OBJLoader.cpp
        util/MeshAssets.cpp )]]

set(viewer_HEADERS viewer/viewer.h)
set(viewer_SOURCE viewer/viewer.cpp)

add_library(Contact_include INTERFACE)
target_include_directories(Contact_include INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_sources(Contact_include INTERFACE ${viewer_HEADERS} ${viewer_SOURCE})

#add_executable (
##        ${contact_HEADERS} ${contact_SOURCE}
##        ${collision_HEADERS} ${collision_SOURCE}
##        ${rigidbody_HEADERS} ${rigidbody_SOURCE}
##        ${solver_HEADERS} ${solver_SOURCE}
##        ${util_HEADERS} ${util_SOURCE}
#        ${viewer_HEADERS} ${viewer_SOURCE} )


