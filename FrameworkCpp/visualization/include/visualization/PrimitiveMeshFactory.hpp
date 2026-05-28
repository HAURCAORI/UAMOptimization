#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace hexaarch::visualization {

struct MeshVertex {
    std::array<float, 3> position{0.0f, 0.0f, 0.0f};
    std::array<float, 3> normal{0.0f, 0.0f, 1.0f};
};

struct MeshData {
    std::vector<MeshVertex> vertices;
    std::vector<std::uint32_t> indices;
    std::string material_name;
};

class PrimitiveMeshFactory {
public:
    [[nodiscard]] static MeshData makeUnitBox();
    [[nodiscard]] static MeshData makeUnitSphere(int latitude_segments = 16, int longitude_segments = 32);
    [[nodiscard]] static MeshData makeUnitCylinder(int radial_segments = 32);
    [[nodiscard]] static MeshData makeUnitDisk(int radial_segments = 32);
    [[nodiscard]] static MeshData makeUnitSegmentProxy();
};

}  // namespace hexaarch::visualization
