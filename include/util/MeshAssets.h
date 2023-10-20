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

    // Clears the mesh cache.
    static void clear();

    // Returns the container of cached meshes.
    static MeshCache& cachedMeshes();

private:

    // The one and only instance of the mesh cache.
    static MeshCache m_meshCache;
};

#endif //CONTACT_MESHASSETS_H
