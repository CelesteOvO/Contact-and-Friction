#ifndef CONTACT_MESHASSETS_H
#define CONTACT_MESHASSETS_H
#pragma once

#include <Eigen/Dense>
#include <string>
#include <map>

/// 简单的多边形网格数据结构
struct Mesh
{
    Mesh() : name(""), meshV(), meshF() { }

    std::string name;
    Eigen::MatrixXf meshV; /// 表示顶点的矩阵
    Eigen::MatrixXi meshF; /// 表示面的矩阵
};

/// 用于缓存网格数据，每个文件名只加载一次
typedef std::map<unsigned int, Mesh> MeshCache;

/// 网格注册表是一个静态类，用于存储几何体和材质信息，用于绘制刚体
class MeshAssetRegistry
{
public:
    /// 从OBJ文件加载网格,并将其添加到缓存中,返回缓存中网格的指针
    static Mesh* loadObj(const std::string& _filename);

    /// 清除网格缓存
    static void clear();

public:
    /// 返回缓存网格的容器
    static MeshCache m_meshCache;
    /// 将 m_meshCache 声明为静态成员变量的原因是为了保证所有的 MeshAssetRegistry 对象共享同一个网格缓存。这样可以避免重复加载相同的网格数据，节省内存和提高效率。
};

#endif //CONTACT_MESHASSETS_H
