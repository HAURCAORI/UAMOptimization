#include "visualization/PrimitiveMeshFactory.hpp"

#include <algorithm>
#include <cmath>

namespace hexaarch::visualization {
namespace {

constexpr float kPi = 3.14159265358979323846f;

MeshVertex makeVertex(
    const float px,
    const float py,
    const float pz,
    const float nx,
    const float ny,
    const float nz) {
    return {{{px, py, pz}}, {{nx, ny, nz}}};
}

void appendTriangle(
    MeshData& mesh,
    const MeshVertex& a,
    const MeshVertex& b,
    const MeshVertex& c) {
    const std::uint32_t base = static_cast<std::uint32_t>(mesh.vertices.size());
    mesh.vertices.push_back(a);
    mesh.vertices.push_back(b);
    mesh.vertices.push_back(c);
    mesh.indices.push_back(base + 0U);
    mesh.indices.push_back(base + 1U);
    mesh.indices.push_back(base + 2U);
}

void appendQuad(
    MeshData& mesh,
    const MeshVertex& a,
    const MeshVertex& b,
    const MeshVertex& c,
    const MeshVertex& d) {
    appendTriangle(mesh, a, b, c);
    appendTriangle(mesh, a, c, d);
}

}  // namespace

MeshData PrimitiveMeshFactory::makeUnitBox() {
    MeshData mesh;
    appendQuad(mesh,
        makeVertex(-0.5f, -0.5f,  0.5f, 0.0f, 0.0f, 1.0f),
        makeVertex( 0.5f, -0.5f,  0.5f, 0.0f, 0.0f, 1.0f),
        makeVertex( 0.5f,  0.5f,  0.5f, 0.0f, 0.0f, 1.0f),
        makeVertex(-0.5f,  0.5f,  0.5f, 0.0f, 0.0f, 1.0f));
    appendQuad(mesh,
        makeVertex( 0.5f, -0.5f, -0.5f, 0.0f, 0.0f, -1.0f),
        makeVertex(-0.5f, -0.5f, -0.5f, 0.0f, 0.0f, -1.0f),
        makeVertex(-0.5f,  0.5f, -0.5f, 0.0f, 0.0f, -1.0f),
        makeVertex( 0.5f,  0.5f, -0.5f, 0.0f, 0.0f, -1.0f));
    appendQuad(mesh,
        makeVertex(-0.5f,  0.5f,  0.5f, 0.0f, 1.0f, 0.0f),
        makeVertex( 0.5f,  0.5f,  0.5f, 0.0f, 1.0f, 0.0f),
        makeVertex( 0.5f,  0.5f, -0.5f, 0.0f, 1.0f, 0.0f),
        makeVertex(-0.5f,  0.5f, -0.5f, 0.0f, 1.0f, 0.0f));
    appendQuad(mesh,
        makeVertex(-0.5f, -0.5f, -0.5f, 0.0f, -1.0f, 0.0f),
        makeVertex( 0.5f, -0.5f, -0.5f, 0.0f, -1.0f, 0.0f),
        makeVertex( 0.5f, -0.5f,  0.5f, 0.0f, -1.0f, 0.0f),
        makeVertex(-0.5f, -0.5f,  0.5f, 0.0f, -1.0f, 0.0f));
    appendQuad(mesh,
        makeVertex( 0.5f, -0.5f,  0.5f, 1.0f, 0.0f, 0.0f),
        makeVertex( 0.5f, -0.5f, -0.5f, 1.0f, 0.0f, 0.0f),
        makeVertex( 0.5f,  0.5f, -0.5f, 1.0f, 0.0f, 0.0f),
        makeVertex( 0.5f,  0.5f,  0.5f, 1.0f, 0.0f, 0.0f));
    appendQuad(mesh,
        makeVertex(-0.5f, -0.5f, -0.5f, -1.0f, 0.0f, 0.0f),
        makeVertex(-0.5f, -0.5f,  0.5f, -1.0f, 0.0f, 0.0f),
        makeVertex(-0.5f,  0.5f,  0.5f, -1.0f, 0.0f, 0.0f),
        makeVertex(-0.5f,  0.5f, -0.5f, -1.0f, 0.0f, 0.0f));
    return mesh;
}

MeshData PrimitiveMeshFactory::makeUnitSphere(const int latitude_segments, const int longitude_segments) {
    MeshData mesh;
    const int lat_count = std::max(3, latitude_segments);
    const int lon_count = std::max(6, longitude_segments);
    for (int lat = 0; lat <= lat_count; ++lat) {
        const float v = static_cast<float>(lat) / static_cast<float>(lat_count);
        const float theta = v * kPi;
        const float sin_theta = std::sin(theta);
        const float cos_theta = std::cos(theta);
        for (int lon = 0; lon <= lon_count; ++lon) {
            const float u = static_cast<float>(lon) / static_cast<float>(lon_count);
            const float phi = u * 2.0f * kPi;
            const float sin_phi = std::sin(phi);
            const float cos_phi = std::cos(phi);
            const float x = 0.5f * sin_theta * cos_phi;
            const float y = 0.5f * cos_theta;
            const float z = 0.5f * sin_theta * sin_phi;
            const float nx = sin_theta * cos_phi;
            const float ny = cos_theta;
            const float nz = sin_theta * sin_phi;
            mesh.vertices.push_back(makeVertex(x, y, z, nx, ny, nz));
        }
    }

    const int stride = lon_count + 1;
    for (int lat = 0; lat < lat_count; ++lat) {
        for (int lon = 0; lon < lon_count; ++lon) {
            const std::uint32_t i0 = static_cast<std::uint32_t>(lat * stride + lon);
            const std::uint32_t i1 = static_cast<std::uint32_t>((lat + 1) * stride + lon);
            const std::uint32_t i2 = i0 + 1U;
            const std::uint32_t i3 = i1 + 1U;
            mesh.indices.push_back(i0);
            mesh.indices.push_back(i1);
            mesh.indices.push_back(i2);
            mesh.indices.push_back(i2);
            mesh.indices.push_back(i1);
            mesh.indices.push_back(i3);
        }
    }
    return mesh;
}

MeshData PrimitiveMeshFactory::makeUnitCylinder(const int radial_segments) {
    MeshData mesh;
    const int segments = std::max(8, radial_segments);
    for (int i = 0; i < segments; ++i) {
        const float a0 = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(segments);
        const float a1 = 2.0f * kPi * static_cast<float>(i + 1) / static_cast<float>(segments);
        const float x0 = 0.5f * std::cos(a0);
        const float z0 = 0.5f * std::sin(a0);
        const float x1 = 0.5f * std::cos(a1);
        const float z1 = 0.5f * std::sin(a1);

        appendQuad(mesh,
            makeVertex(x0, -0.5f, z0, std::cos(a0), 0.0f, std::sin(a0)),
            makeVertex(x0,  0.5f, z0, std::cos(a0), 0.0f, std::sin(a0)),
            makeVertex(x1,  0.5f, z1, std::cos(a1), 0.0f, std::sin(a1)),
            makeVertex(x1, -0.5f, z1, std::cos(a1), 0.0f, std::sin(a1)));

        appendTriangle(mesh,
            makeVertex(0.0f, 0.5f, 0.0f, 0.0f, 1.0f, 0.0f),
            makeVertex(x1, 0.5f, z1, 0.0f, 1.0f, 0.0f),
            makeVertex(x0, 0.5f, z0, 0.0f, 1.0f, 0.0f));

        appendTriangle(mesh,
            makeVertex(0.0f, -0.5f, 0.0f, 0.0f, -1.0f, 0.0f),
            makeVertex(x0, -0.5f, z0, 0.0f, -1.0f, 0.0f),
            makeVertex(x1, -0.5f, z1, 0.0f, -1.0f, 0.0f));
    }
    return mesh;
}

MeshData PrimitiveMeshFactory::makeUnitDisk(const int radial_segments) {
    MeshData mesh;
    const int segments = std::max(8, radial_segments);
    for (int i = 0; i < segments; ++i) {
        const float a0 = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(segments);
        const float a1 = 2.0f * kPi * static_cast<float>(i + 1) / static_cast<float>(segments);
        appendTriangle(mesh,
            makeVertex(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f),
            makeVertex(0.5f * std::cos(a0), 0.5f * std::sin(a0), 0.0f, 0.0f, 0.0f, 1.0f),
            makeVertex(0.5f * std::cos(a1), 0.5f * std::sin(a1), 0.0f, 0.0f, 0.0f, 1.0f));
        appendTriangle(mesh,
            makeVertex(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f),
            makeVertex(0.5f * std::cos(a1), 0.5f * std::sin(a1), 0.0f, 0.0f, 0.0f, -1.0f),
            makeVertex(0.5f * std::cos(a0), 0.5f * std::sin(a0), 0.0f, 0.0f, 0.0f, -1.0f));
    }
    return mesh;
}

MeshData PrimitiveMeshFactory::makeUnitSegmentProxy() {
    return makeUnitCylinder();
}

}  // namespace hexaarch::visualization
