#version 450

layout(binding = 0) uniform GlobalUniforms {
    mat4 view;
    mat4 projection;
    vec4 lightDirection;
} ubo;

layout(binding = 1) uniform sampler2D baseColorTexture;

layout(location = 0) in vec3 fragNormal;
layout(location = 1) in vec4 fragColor;
layout(location = 2) in vec2 fragTexCoord;

layout(location = 0) out vec4 outColor;

void main() {
    vec3 normal = normalize(fragNormal);
    vec3 lightDir = normalize(-ubo.lightDirection.xyz);
    float diffuse = max(dot(normal, lightDir), 0.0);
    vec3 viewDir = normalize(vec3(0.0, 0.0, 1.0));
    vec3 halfDir = normalize(lightDir + viewDir);
    float specular = pow(max(dot(normal, halfDir), 0.0), 48.0);
    float lighting = 0.28 + 0.72 * diffuse;
    vec4 texel = texture(baseColorTexture, fragTexCoord);
    vec3 textureDetail = mix(vec3(1.0), texel.rgb, 0.25);
    vec3 color = fragColor.rgb * textureDetail * lighting + vec3(0.18) * specular;
    outColor = vec4(color, fragColor.a);
}
