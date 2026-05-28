#include "visualization/GltfLoader.hpp"

#include <cstring>
#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace hexaarch::visualization {
namespace fs = std::filesystem;
using json = nlohmann::json;

std::vector<MeshData> loadGltfMeshes(const fs::path& gltf_path, const float scale, const bool remap_y_to_z) {
    std::ifstream f(gltf_path);
    if (!f) {
        throw std::runtime_error("GltfLoader: cannot open " + gltf_path.string());
    }
    const json doc = json::parse(f);

    const fs::path bin_path = gltf_path.parent_path() / doc["buffers"][0]["uri"].get<std::string>();
    std::ifstream bin_file(bin_path, std::ios::binary | std::ios::ate);
    if (!bin_file) {
        throw std::runtime_error("GltfLoader: cannot open binary buffer " + bin_path.string());
    }
    const auto bin_size = static_cast<std::size_t>(bin_file.tellg());
    bin_file.seekg(0);
    std::vector<std::uint8_t> bin(bin_size);
    bin_file.read(reinterpret_cast<char*>(bin.data()), static_cast<std::streamsize>(bin_size));

    const auto accessorPtr = [&](const int acc_idx) -> const std::uint8_t* {
        const auto& acc    = doc["accessors"][acc_idx];
        const int bv_idx   = acc["bufferView"].get<int>();
        const int acc_off  = acc.value("byteOffset", 0);
        const int bv_off   = doc["bufferViews"][bv_idx].value("byteOffset", 0);
        return bin.data() + bv_off + acc_off;
    };

    const auto accessorStride = [&](const int acc_idx, const int natural) -> int {
        const int bv_idx = doc["accessors"][acc_idx]["bufferView"].get<int>();
        return doc["bufferViews"][bv_idx].value("byteStride", natural);
    };

    std::vector<MeshData> result;

    for (const auto& mesh : doc["meshes"]) {
        for (const auto& prim : mesh["primitives"]) {
            if (prim.value("mode", 4) != 4) { continue; }
            const auto& attrs = prim["attributes"];
            if (!attrs.contains("POSITION") || !attrs.contains("NORMAL")) { continue; }
            if (!prim.contains("indices")) { continue; }

            const int pos_acc  = attrs["POSITION"].get<int>();
            const int norm_acc = attrs["NORMAL"].get<int>();
            const int idx_acc  = prim["indices"].get<int>();

            const int vtx_count  = doc["accessors"][pos_acc]["count"].get<int>();
            const int idx_count  = doc["accessors"][idx_acc]["count"].get<int>();
            const int idx_comp   = doc["accessors"][idx_acc]["componentType"].get<int>();

            const auto* pos_ptr  = accessorPtr(pos_acc);
            const auto* norm_ptr = accessorPtr(norm_acc);
            const auto* idx_ptr  = accessorPtr(idx_acc);
            const int pos_stride  = accessorStride(pos_acc,  12);
            const int norm_stride = accessorStride(norm_acc, 12);

            MeshData data;
            data.vertices.reserve(static_cast<std::size_t>(vtx_count));
            data.indices.reserve(static_cast<std::size_t>(idx_count));
            if (prim.contains("material")) {
                const int mat_idx = prim["material"].get<int>();
                if (doc.contains("materials") && mat_idx < static_cast<int>(doc["materials"].size())) {
                    data.material_name = doc["materials"][mat_idx].value("name", "");
                }
            }

            for (int i = 0; i < vtx_count; ++i) {
                float px, py, pz, nx, ny, nz;
                std::memcpy(&px, pos_ptr  + i * pos_stride,       4);
                std::memcpy(&py, pos_ptr  + i * pos_stride  + 4,  4);
                std::memcpy(&pz, pos_ptr  + i * pos_stride  + 8,  4);
                std::memcpy(&nx, norm_ptr + i * norm_stride,       4);
                std::memcpy(&ny, norm_ptr + i * norm_stride + 4,   4);
                std::memcpy(&nz, norm_ptr + i * norm_stride + 8,   4);

                MeshVertex v;
                if (remap_y_to_z) {
                    // Rx(+90°): glTF Y-up (right-hand) → viewer Z-up (right-hand)
                    // x' = x,  y' = -z,  z' = y
                    v.position = {px * scale, -pz * scale, py * scale};
                    v.normal   = {nx,          -nz,          ny};
                } else {
                    v.position = {px * scale, py * scale, pz * scale};
                    v.normal   = {nx, ny, nz};
                }
                data.vertices.push_back(v);
            }

            if (idx_comp == 5125) {
                for (int i = 0; i < idx_count; ++i) {
                    std::uint32_t val = 0;
                    std::memcpy(&val, idx_ptr + i * 4, 4);
                    data.indices.push_back(val);
                }
            } else {
                for (int i = 0; i < idx_count; ++i) {
                    std::uint16_t val = 0;
                    std::memcpy(&val, idx_ptr + i * 2, 2);
                    data.indices.push_back(static_cast<std::uint32_t>(val));
                }
            }

            result.push_back(std::move(data));
        }
    }

    return result;
}

}  // namespace hexaarch::visualization
