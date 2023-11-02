
#include "CollisionDetect.h"

CollisionDetect::CollisionDetect(PhysicsWorld *physicsWorld) : _physicsWorld(physicsWorld) {}

void CollisionDetect::detectCollisions() {
    /// 1. 清除所有的碰撞信息
    clear();

    /// 2. 遍历所有的刚体，检测碰撞
    auto bodies = _physicsWorld->_bodies;
    for(unsigned int i = 0; i < bodies.size(); ++i)
    {
        for(unsigned int j = i+1; j < bodies.size(); ++j)
        {
            RigidBody* body0 = bodies[i];
            RigidBody* body1 = bodies[j];

            /// 跳过两个静止刚体的碰撞检测
            if (body0->RigidBodyData._bodyTypes == STATIC && body1->RigidBodyData._bodyTypes == STATIC)
                continue;

            /// 测试球-球碰撞
            if( body0->RigidBodyData._geometry->_type == kSphere &&
                body1->RigidBodyData._geometry->_type == kSphere )
            {
                collisionDetectSphereSphere(body0, body1);
            }
            /// 测试球-盒碰撞
            else if( body0->RigidBodyData._geometry->_type == kSphere &&
                     body1->RigidBodyData._geometry->_type == kBox )
            {
                collisionDetectSphereBox(body0, body1);
            }
            /// 测试盒-球碰撞
            else if( body1->RigidBodyData._geometry->_type == kSphere &&
                     body0->RigidBodyData._geometry->_type == kBox )
            {
                collisionDetectSphereBox(body1, body0);
            }else if( body0->RigidBodyData._geometry->_type == kBox &&
                      body1->RigidBodyData._geometry->_type == kBox )
            {
                double dist = (body0->RigidBodyData._position - body1->RigidBodyData._position).norm();
                double diagonal = body0->RigidBodyData._geometry->getDiagonal() + body1->RigidBodyData._geometry->getDiagonal();
                if(dist < diagonal)
                    collisionDetectBoxBox(body0, body1);
            }
        }
    }
}

void CollisionDetect::computeContactJacobians() {
    for(auto c : _contacts)
    {
        c->computeContactFrame();
        c->computeJacobian();
    }
}

void CollisionDetect::clear() {
    /// 1. 清除所有的刚体碰撞信息
    auto bodies = _physicsWorld->_bodies;
    for (auto b : bodies) {
        b->RigidBodyData._contacts.clear();
    }
    /// 2. 清除所有的本地接触点
    for(auto c : _contacts)
    {
        delete c;
    }
    _contacts.clear();
}

void CollisionDetect::collisionDetectSphereSphere(RigidBody* body0, RigidBody* body1) {
    Sphere* sphere0 = dynamic_cast<Sphere*>(body0->RigidBodyData._geometry.get());
    Sphere* sphere1 = dynamic_cast<Sphere*>(body1->RigidBodyData._geometry.get());

    /// 1. 计算两个球心之间的距离
    Eigen::Vector3f vec = body0->RigidBodyData._position - body1->RigidBodyData._position;
    /// 2. 计算两个球的半径之和
    const float radiusSum = sphere0->radius + sphere1->radius;
    const float dist = vec.norm(); /// 两个球心之间的距离
    /// 3. 如果距离小于半径之和，说明两个球相交
    if(dist < radiusSum)
    {
        const Eigen::Vector3f n = vec / dist; /// 碰撞法线方向
        const Eigen::Vector3f p = 0.5f * ((body0->RigidBodyData._position - sphere0->radius * n) + (body1->RigidBodyData._position + sphere1->radius * n));
        const float phi = dist - radiusSum;

        _contacts.push_back( new Contact(body0, body1, p, n, phi));
    }
}

/// TODO 2. 实现球-盒碰撞检测
void CollisionDetect::collisionDetectSphereBox(RigidBody *body0, RigidBody *body1) {
    Sphere* sphere = dynamic_cast<Sphere*>(body0->RigidBodyData._geometry.get());
    Box* box = dynamic_cast<Box*>(body1->RigidBodyData._geometry.get());

    /// 1. 将球心坐标转换到盒子的局部坐标系下
    Eigen::Vector3f c_local = body1->RigidBodyData._orientation.inverse() * (body0->RigidBodyData._position - body1->RigidBodyData._position);
    /// 2. 计算球心到盒子的最近点
    Eigen::Vector3f g = c_local;
    for (int i = 0; i < 3; ++i) {
        if (c_local(i) < -box->halfSize(i)) /// 球心超出盒子的左边界
            g(i) = -box->halfSize(i); /// 最近点在盒子的左边界
        else if (c_local(i) > box->halfSize(i)) /// 球心超出盒子的右边界
            g(i) = box->halfSize(i); /// 最近点在盒子的右边界
    }
    /// 通过以上操作，确保了 q 是球心到AABB的最近点，而且不会超出AABB的边界。
    /// 3. 计算球心到盒子的最近距离
    Eigen::Vector3f vec = g - c_local;
    const float dist = vec.norm();
    float epsilon = 1e-6f; /// 用于判断深浅的阈值
    /// 4. 如果距离小于球的半径，说明相交
    if (dist < sphere->radius) {
        if (dist < sphere->radius - epsilon) /// 深度碰撞
        {
            /// 5. 计算碰撞深度
            const float penetration = -std::min(std::min(abs(box->halfSize(0) - c_local(0)), abs(box->halfSize(1) - c_local(1))),abs( box->halfSize(2) - c_local(2)));
            /// 6. 计算碰撞法线,分情况讨论
            Eigen::Vector3f n;
            if (penetration == abs(box->halfSize(0) - c_local(0))) {
                n = Eigen::Vector3f::UnitX() * (c_local(0) < 0 ? -1 : 1); /// X轴方向
            } else if (penetration == abs(box->halfSize(1) - c_local(1))) {
                n = Eigen::Vector3f::UnitY() * (c_local(1) < 0 ? -1 : 1); /// Y轴方向
            } else {
                n = Eigen::Vector3f::UnitZ() * (c_local(2) < 0 ? -1 : 1); /// Z轴方向
            }
            n = body1->RigidBodyData._orientation * n;
            /// 7. 计算碰撞点
            Eigen::Vector3f p = body1->RigidBodyData._orientation * g + body1->RigidBodyData._position;

            _contacts.push_back(new Contact(body0, body1, p, n, penetration));
        }else /// 浅度碰撞
        {
            /// 5. 计算碰撞深度
            const float penetration =  dist - sphere->radius;
            /// 6. 计算碰撞法线
            Eigen::Vector3f n = body1->RigidBodyData._orientation * vec / dist;
            /// 7. 计算碰撞点
            Eigen::Vector3f p = body1->RigidBodyData._orientation * g + body1->RigidBodyData._position;

            _contacts.push_back(new Contact(body0, body1, p, n, penetration));
        }
    }
}

void CollisionDetect::collisionDetectBoxBox(RigidBody *body0, RigidBody *body1) {
    std::vector<Eigen::Vector3f> axis;

    for (int i = 0; i < 3; i++) {
        axis.emplace_back(body0->RigidBodyData._orientation.matrix().col(i));
        axis.emplace_back(body1->RigidBodyData._orientation.matrix().col(i));
    }

    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            Eigen::Vector3f temp = axis[i].cross(axis[3+j]);
            if(temp.norm() > 1e-6f)
                axis.emplace_back(temp.normalized());
        }
    }

    float minOverlap = 1e10f;
    unsigned int bestAxis = 15;
    for (unsigned int i = 0; i < 15; i++)
    {
        Eigen::Vector3f temp = axis[i];
        if(temp.norm() < 1e-6f)
            continue;
        temp.normalize();
        float overlap = penetrationOnAxis(body0, body1, temp);
        if(overlap < 0)
            return;
        if(overlap < minOverlap)
        {
            minOverlap = overlap;
            bestAxis = i;
        }
    }

    if(bestAxis <= 2)
    {
        collisionDetectFaceVertex(body0, body1, axis[bestAxis], minOverlap);
    }else if(bestAxis <= 5)
    {
        collisionDetectFaceVertex(body1, body0, axis[bestAxis], minOverlap);
    }else{
        int oneAxisIndex = (bestAxis - 6) / 3;
        int twoAxisIndex = bestAxis % 3;
        collisionDetectEdgeEdge(body0, body1, axis[bestAxis], minOverlap, oneAxisIndex, twoAxisIndex);
    }
}

float CollisionDetect::penetrationOnAxis(RigidBody *pBody, RigidBody *pBody1, const Eigen::Vector3f& axis) {
    float one = transformToAxis(pBody, axis);
    float two = transformToAxis(pBody1, axis);
    float center = abs((pBody->RigidBodyData._position - pBody1->RigidBodyData._position).dot(axis));
    return one + two - center;
}

float CollisionDetect::transformToAxis(RigidBody *pBody, const Eigen::Vector3f &axis) {
    Box* box0 = dynamic_cast<Box*>(pBody->RigidBodyData._geometry.get());
    float projectionMax = -1e10f;
    float projectionMin = 1e10f;
    for (float cx = -box0->halfSize(0);cx < box0->dim(0);cx += box0->dim(0))
    {
        for (float cy = -box0->halfSize(1);cy < box0->dim(1);cy += box0->dim(1))
        {
            for (float cz = -box0->halfSize(2);cz < box0->dim(2);cz += box0->dim(2))
            {
                Eigen::Vector3f vertex = Eigen::Vector3f(cx,cy,cz);
                float projection = vertex.dot(axis);
                projectionMax = std::max(projectionMax,projection);
                projectionMin = std::min(projectionMin,projection);
            }
        }
    }
    return (projectionMax - projectionMin) / 2.0f;
}

void CollisionDetect::collisionDetectFaceVertex(RigidBody *body0, RigidBody *body1, Eigen::Vector3f axis,
                                                float penetration) {
    Eigen::Vector3f toCenter = body1->RigidBodyData._position - body0->RigidBodyData._position;
    if(axis.dot(toCenter) > 0)
        axis *= -1;
    Box* box1 = dynamic_cast<Box*>(body1->RigidBodyData._geometry.get());
    Eigen::Vector3f vertex = body1->RigidBodyData._position + box1->dim(0) * body1->RigidBodyData._orientation.matrix().col(0) + box1->dim(1) * body1->RigidBodyData._orientation.matrix().col(1) + box1->dim(2) * body1->RigidBodyData._orientation.matrix().col(2);
    if(body1->RigidBodyData._orientation.matrix().col(0).dot(axis) < 0)
        vertex -= body1->RigidBodyData._orientation.matrix().col(0);
    if(body1->RigidBodyData._orientation.matrix().col(1).dot(axis) < 0)
        vertex -= body1->RigidBodyData._orientation.matrix().col(1);
    if(body1->RigidBodyData._orientation.matrix().col(2).dot(axis) < 0)
        vertex -= body1->RigidBodyData._orientation.matrix().col(2);

    auto* c = new Contact(body0, body1, vertex, axis, penetration);
    c->vf = true;
    _contacts.push_back(c);
}

void
CollisionDetect::collisionDetectEdgeEdge(RigidBody *body0, RigidBody *body1, Eigen::Vector3f axis, float penetration, int oneAxisIndex, int twoAxisIndex) {
    Eigen::Vector3f toCenter = body1->RigidBodyData._position - body0->RigidBodyData._position;
    if(axis.dot(toCenter) > 0)
        axis *= -1;
    Eigen::Vector3f ptOnEdgeOne = body0->RigidBodyData._position;
    Eigen::Vector3f ptOnEdgeTwo = body1->RigidBodyData._position;
    Box* box0 = dynamic_cast<Box*>(body0->RigidBodyData._geometry.get());
    Box* box1 = dynamic_cast<Box*>(body1->RigidBodyData._geometry.get());
    for(int i = 0; i < 3; i++)
    {
        if(i == oneAxisIndex){}
        else if(body0->RigidBodyData._orientation.matrix().col(i).dot(axis) > 0)
            ptOnEdgeOne -= box0->halfSize(i) * body0->RigidBodyData._orientation.matrix().col(i);
        else
            ptOnEdgeOne += box0->halfSize(i) * body0->RigidBodyData._orientation.matrix().col(i);
        if(i == twoAxisIndex){}
        else if(body1->RigidBodyData._orientation.matrix().col(i).dot(axis) < 0)
            ptOnEdgeTwo -= box1->halfSize(i) * body1->RigidBodyData._orientation.matrix().col(i);
        else
            ptOnEdgeTwo += box1->halfSize(i) * body1->RigidBodyData._orientation.matrix().col(i);
    }
    Eigen::Vector3f axisOne = body0->RigidBodyData._orientation.matrix().col(oneAxisIndex);
    Eigen::Vector3f axisTwo = body1->RigidBodyData._orientation.matrix().col(twoAxisIndex);
    Eigen::Vector3f toSt = ptOnEdgeOne - ptOnEdgeTwo;

    float dpStaOne = axisOne.dot(toSt);
    float dpStaTwo = axisTwo.dot(toSt);

    float smOne = axisOne.squaredNorm();
    float smTwo = axisTwo.squaredNorm();

    double dotProductEdges = axisTwo.dot(axisOne);
    double denom = smOne * smTwo - dotProductEdges * dotProductEdges;
    double ta = (dotProductEdges * dpStaTwo - smTwo * dpStaOne) / denom;
    double tb = (smOne * dpStaTwo - dotProductEdges * dpStaOne) / denom;

    Eigen::Vector3f ptOne = ptOnEdgeOne + ta * axisOne;
    Eigen::Vector3f ptTwo = ptOnEdgeTwo + tb * axisTwo;

    auto* c = new Contact(body0, body1, 0.5f * (ptOne + ptTwo), axis, penetration);
    c->ee = true;
    _contacts.push_back(c);
}








