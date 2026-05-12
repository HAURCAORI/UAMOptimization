#version 450

layout(binding = 0) uniform GlobalUniforms {
    mat4 view;
    mat4 projection;
    vec4 lightDirection;
} ubo;

layout(push_constant) uniform PushConstants {
    mat4 model;
    vec4 color;
} pushData;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;

layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec4 fragColor;

void main() {
    vec4 worldPosition = pushData.model * vec4(inPosition, 1.0);
    mat3 normalMatrix = transpose(inverse(mat3(pushData.model)));
    fragNormal = normalize(normalMatrix * inNormal);
    fragColor = pushData.color;
    gl_Position = ubo.projection * ubo.view * worldPosition;
}
