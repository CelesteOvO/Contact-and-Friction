#ifndef CONTACT_SOLVERBOXPGS_H
#define CONTACT_SOLVERBOXPGS_H

#include "solver/Solver.h"
#include "engine/PhysicsWorld.h"
#include <vector>

class SolverBoxPGS : public Solver
{
public:
    explicit SolverBoxPGS(PhysicsWorld* physicsWorld);
    ~SolverBoxPGS() override = default;

    void solve(float dt) override;
};

#endif //CONTACT_SOLVERBOXPGS_H
