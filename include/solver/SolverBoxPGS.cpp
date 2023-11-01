#include "SolverBoxPGS.h"

SolverBoxPGS::SolverBoxPGS(PhysicsWorld *physicsWorld) : Solver(physicsWorld) {

}

void SolverBoxPGS::solve(float dt) {
    /// TODO: 4.实现PGS求解器
    std::vector<Contact*>& contacts = _physicsWorld->_collisionDetect->_contacts;
    const int numContacts = contacts.size();

    std::vector<Eigen::Matrix3f> Aii_contact; /// 接触点i的刚度矩阵

    //    A x + b = 0
    //   (U + D + L) x = - b
    //    D x = - b -U x - L x
    //    D x = - b -U x - L x - Dx + Dx
    //    D x = - b - A x  + Dx
    //      x = D^{-1}( - b - A x) + x

    /// 正则化参数
    // A' = A + gamma
    /// 所以
    // A' = L + D' + U where D' = D + gamma
    // x = D'^{-1}( - b - A' x) + x
    // x = (D+gamma)^{-1}( - b - A x - gamma x) + x

    // A = J * Minv * J^T
    // b = J * Minv * ( M * vel + h * force )
    // b = J * Minv *  M * vel + J * Minv * h * force
    // Minv *  M = I
    // b = J * vel + h * J * Minv * force
    // b = J * vel + h * JMinvJT * force

    // -gamma*phi/h 表示Baumgarte stabilization
    // -J*vel 表示速度相关的冲量
    // -dt*JMinvJT*force 表示力相关的冲量

    // b = -gamma*phi/h - J*vel - dt*JMinvJT*force
    float gamma = 0.1f;
    if(numContacts > 0 ) {
        for(int iter = 0; iter < _maxIter; ++iter)
        {
            for(int i = 0; i < numContacts; ++i)
            {
                /*Contact* c = contacts[i];
                Eigen::Vector3f x = c->_lambda;
                Eigen::Vector3f b = -gamma * c->_phi / dt - c->_Jacobian0 * c->_body0->RigidBodyData._linearVelocity - dt * c->_Jacobian0Minv * c->_body0->RigidBodyData._linearForce;
                for(auto other : c->_body0->RigidBodyData._contacts)
                {
                    if(other != c)
                    {
                        x -= other->_JMinvJT * other->_lambda;
                    }
                }
                for(auto other : c->_body1->RigidBodyData._contacts)
                {
                    if(other != c)
                    {
                        x -= other->_JMinvJT * other->_lambda;
                    }
                }
                Aii_contact.push_back(c->_JMinvJT);
                c->_lambda = Aii_contact[i].inverse() * x;*/
            }
        }
    }



}

