#ifndef CONTACT_OBJLOADER_H
#define CONTACT_OBJLOADER_H
#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <cassert>

namespace {
    // 2D/3D point data structures
    struct Point3D {
        Point3D() : x(0), y(0), z(0) {}

        float x, y, z;
    };

    struct TriangleInds {
        TriangleInds() : i(0), j(0), k(0) {}

        int i, j, k;
    };

    /// 从字符串中提取路径
    std::string extractPath(const std::string &filepathname) {
        std::size_t pos = filepathname.find_last_of("/\\");

        if (pos == std::string::npos)
            return std::string(".");

        return filepathname.substr(0, pos);
    }
}

/// 用于OBJ网格的文件加载
class OBJLoader
{
public:

    /// 从OBJ文件路径@文件名加载网格，并返回网格顶点@网格V和面@网格F
    /// 注意：使用MeshAssets.h中的MeshAssetRegistry类来加载网格，而不是使用此函数，因为之前加载的网格文件将从缓存中加载，这要快得多。
    static bool load(const std::string& filename, Eigen::MatrixXf& meshV, Eigen::MatrixXi& meshF);
};

#endif //CONTACT_OBJLOADER_H
