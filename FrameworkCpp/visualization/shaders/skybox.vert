#version 450

layout(location = 0) out vec3 outDir;

// Only view and projection are read — the struct must match the first 128 bytes
// of GlobalUniforms so binding the full UBO descriptor set here is valid.
layout(set = 0, binding = 0) uniform GlobalUniforms {
    mat4 view;
    mat4 projection;
} ubo;

// Unit cube vertices (36 = 6 faces x 2 triangles x 3 verts), wound CCW from outside.
// The camera is inside the cube, so back-face culling must be disabled.
const vec3 kCubeVerts[36] = vec3[](
    // +X face
    vec3( 1,-1, 1), vec3( 1,-1,-1), vec3( 1, 1,-1),
    vec3( 1, 1,-1), vec3( 1, 1, 1), vec3( 1,-1, 1),
    // -X face
    vec3(-1,-1,-1), vec3(-1,-1, 1), vec3(-1, 1, 1),
    vec3(-1, 1, 1), vec3(-1, 1,-1), vec3(-1,-1,-1),
    // +Y face
    vec3(-1, 1, 1), vec3( 1, 1, 1), vec3( 1, 1,-1),
    vec3( 1, 1,-1), vec3(-1, 1,-1), vec3(-1, 1, 1),
    // -Y face
    vec3(-1,-1,-1), vec3( 1,-1,-1), vec3( 1,-1, 1),
    vec3( 1,-1, 1), vec3(-1,-1, 1), vec3(-1,-1,-1),
    // +Z face
    vec3(-1,-1, 1), vec3( 1,-1, 1), vec3( 1, 1, 1),
    vec3( 1, 1, 1), vec3(-1, 1, 1), vec3(-1,-1, 1),
    // -Z face
    vec3( 1,-1,-1), vec3(-1,-1,-1), vec3(-1, 1,-1),
    vec3(-1, 1,-1), vec3( 1, 1,-1), vec3( 1,-1,-1)
);

void main() {
    vec3 pos = kCubeVerts[gl_VertexIndex];
    outDir   = pos;

    // Strip translation so the sky box stays at infinity regardless of camera movement.
    mat4 viewNoTrans = ubo.view;
    viewNoTrans[3]   = vec4(0.0, 0.0, 0.0, 1.0);

    vec4 clipPos = ubo.projection * viewNoTrans * vec4(pos, 1.0);
    // Force depth to far plane (z/w = 1.0) so all geometry renders in front.
    gl_Position = clipPos.xyww;
}
