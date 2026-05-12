#version 450

layout(binding = 0) uniform GlobalUniforms {
    mat4 view;
    mat4 projection;
    vec4 lightDirection;
} ubo;

layout(location = 0) in vec3 fragNormal;
layout(location = 1) in vec4 fragColor;

layout(location = 0) out vec4 outColor;

void main() {
    vec3 normal = normalize(fragNormal);
    vec3 lightDir = normalize(-ubo.lightDirection.xyz);
    float diffuse = max(dot(normal, lightDir), 0.0);
    float lighting = 0.25 + 0.75 * diffuse;
    outColor = vec4(fragColor.rgb * lighting, fragColor.a);
}
