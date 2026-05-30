#version 450

// Depth-only shadow pass — writes light-space depth for each fragment.
// The model matrix and the light-space VP matrix come from push constants
// (128 bytes total, within the guaranteed Vulkan minimum).

layout(location = 0) in vec3 inPosition;
// Attributes at locations 1-2 (normal, texcoord) are bound but not declared;
// Vulkan silently ignores unused input attributes.

layout(push_constant) uniform ShadowPassPC {
    mat4 lightSpaceVP;  // pre-multiplied light view-projection  (64 bytes)
    mat4 model;         // per-draw model matrix                  (64 bytes)
} push;                 // Total: 128 bytes (guaranteed Vulkan minimum)

void main() {
    gl_Position = push.lightSpaceVP * push.model * vec4(inPosition, 1.0);
}
