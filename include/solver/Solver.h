#ifndef CONTACT_SOLVER_H
#define CONTACT_SOLVER_H

class PhysicsWorld;

class Solver
{
public:
    explicit Solver(PhysicsWorld* physicsWorld);
    virtual ~Solver() = default;

    virtual void solve(float dt) = 0;
public:
    PhysicsWorld* _physicsWorld;
    int _maxIter;
};

Solver::Solver(PhysicsWorld *physicsWorld) {
    _physicsWorld = physicsWorld;
    _maxIter = 20;
}

#endif //CONTACT_SOLVER_H
