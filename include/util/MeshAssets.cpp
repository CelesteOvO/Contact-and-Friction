
#include "MeshAssets.h"
#include "OBJLoader.h"

std::map<unsigned int, Mesh> MeshAssetRegistry::m_meshCache = std::map<unsigned int, Mesh>();

Mesh *MeshAssetRegistry::loadObj(const std::string &_filename) {
    unsigned int key = std::hash<std::string>{}(_filename);
    auto cacheItr = m_meshCache.find(key);

    if( cacheItr != m_meshCache.end() )
    {
        return &(cacheItr->second);
    }

    Mesh mesh;
    OBJLoader::load(_filename, mesh.meshV, mesh.meshF);

    m_meshCache[key] = mesh;
    return &(m_meshCache[key]);
}

void MeshAssetRegistry::clear() {
    m_meshCache.clear();
}

