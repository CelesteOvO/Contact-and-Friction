#include "OBJLoader.h"

bool OBJLoader::load(const std::string &filename, Eigen::MatrixXf &meshV, Eigen::MatrixXi &meshF) {
    /// 打开文件
    std::ifstream file(filename.c_str(), std::ifstream::in);
    if (!file.is_open())
    {
        std::cout << "Error: Failed to open file " << filename << " for reading!" << std::endl;
        return false;
    }

    /// 提取路径，稍后加载mtl文件时会用到
    std::string path = extractPath(filename);

    // Read file
    std::string line;
    while (std::getline(file, line))
    {
        if (line[0] == '#')
        {
            // Comments... just ignore the line
            continue;
        }
        else if (line[0] == 'v' && line[1] == ' ')
        {
            // Vertex! Add it to the list.
            Point3D v;
            std::stringstream ss(line.substr(2));
            ss >> v.x >> v.y >> v.z;
            verts.push_back(v);

        }
        else if (line[0] == 'f')
        {
            // Face! First, get its vertices data
            std::string vertexData;
            std::string dummy;
            std::stringstream ssLine(line.substr(2));
            while (std::getline(ssLine, vertexData, ' '))
            {
                int tmp;
                const unsigned int index = inds.size();
                inds.push_back(0);

                std::stringstream ss(vertexData);
                std::string stringVal;
                std::getline(ss, stringVal, '/');
                std::stringstream ss2(stringVal);
                ss2 >> inds[index];

                std::getline(ss, stringVal, '/');
                std::stringstream ss3(stringVal);
                ss3 >> tmp;

                std::getline(ss, stringVal, '/');
                std::stringstream ss4(stringVal);
                ss4 >> tmp;
            }

        }
    }

    const unsigned int numVerts = verts.size();
    meshV.resize(numVerts, 3);
    for (unsigned int i = 0; i < numVerts; ++i)
    {
        meshV(i, 0) = verts[i].x;
        meshV(i, 1) = verts[i].y;
        meshV(i, 2) = verts[i].z;
    }

    // Assemble the face matrix
    assert((inds.size() % 3) == 0);
    const unsigned int numFaces = inds.size() / 3;
    meshF.resize(numFaces, 3);
    for (unsigned int i = 0; i < numFaces; ++i)
    {
        meshF(i, 0) = inds[3 * i]-1;
        meshF(i, 1) = inds[3 * i + 1]-1;
        meshF(i, 2) = inds[3 * i + 2]-1;
    }


    // Close file
    file.close();
}
