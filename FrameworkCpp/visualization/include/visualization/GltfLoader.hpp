#pragma once

#include "visualization/PrimitiveMeshFactory.hpp"

#include <filesystem>
#include <vector>

namespace hexaarch::visualization {

// Loads all triangle mesh primitives from a glTF file, extracting POSITION, NORMAL, and TEXCOORD_0.
// scale:        applied to vertex positions (e.g. 1.80f/182.53f to normalize UE4 cm to 1.80 m).
// remap_y_to_z: applies Rx(+90°) so glTF Y-up maps to viewer Z-up: x'=x, y'=−z, z'=y.
std::vector<MeshData> loadGltfMeshes(
    const std::filesystem::path& gltf_path,
    float scale = 1.0f,
    bool remap_y_to_z = true);

}  // namespace hexaarch::visualization
