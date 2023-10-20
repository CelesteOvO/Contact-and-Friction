cmake_minimum_required(VERSION 3.25)
project(Contact)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenGL REQUIRED)

set(collision_HEADERS include/collision/CollisionDetect.h
        include/collision/Geometry.h )
set(collision_SOURCE include/collision/CollisionDetect.cpp )

set(rigidbody_HEADERS include/rigidbody/RigidBody.h
        include/rigidbody/RigidBodySystem.h
        include/rigidbody/Scenarios.h )
set(rigidbody_SOURCE include/rigidbody/RigidBody.cpp
        include/rigidbody/RigidBodySystem.cpp )

set(contact_HEADERS include/contact/Contact.h )
set(contact_SOURCE include/contact/Contact.cpp )

set(solver_HEADERS include/solvers/Solver.h
        include/solvers/SolverBoxPGS.h )
set(solver_SOURCE include/solvers/SolverBoxPGS.cpp )

set(util_HEADERS include/util/OBJLoader.h
        include/util/MeshAssets.h )
set(util_SOURCE include/util/OBJLoader.cpp
        include/util/MeshAssets.cpp )

set(viewer_HEADERS include/viewer/viewer.h)
set(viewer_SOURCE include/viewer/viewer.cpp)

set(tutorials_SOURCE tutorials/000-basic-collision/main.cpp)

add_executable (Contact ${tutorials_SOURCE}
        ${contact_HEADERS} ${contact_SOURCE}
        ${collision_HEADERS} ${collision_SOURCE}
        ${rigidbody_HEADERS} ${rigidbody_SOURCE}
        ${solver_HEADERS} ${solver_SOURCE}
        ${util_HEADERS} ${util_SOURCE}
        ${viewer_HEADERS} ${viewer_SOURCE} )

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/polyscope)
target_link_libraries(Contact OpenGL::GL polyscope)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include 3rdParty/Eigen3/include/eigen3 ${COMMON_INCLUDES})

if(MSVC)
    set_property(TARGET Contact PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()

