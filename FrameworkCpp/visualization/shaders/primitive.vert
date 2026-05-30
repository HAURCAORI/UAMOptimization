#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inTexCoord;

layout(location = 0) out vec3 fragWorldPos;
layout(location = 1) out vec3 fragNormal;
layout(location = 2) out vec2 fragTexCoord;
layout(location = 3) out vec4 fragColor;

struct GPULight {
    vec4 position;    // xyz=pos or dir, w=type (0=dir, 1=point, 2=spot)
    vec4 color;       // rgb=color, a=intensity
    vec4 direction;   // xyz=dir, w=range
    vec4 spotAngles;  // x=cos(inner), y=cos(outer), zw=unused
};

// Full block — must match primitive.frag and the C++ GlobalUniformData exactly.
layout(set = 0, binding = 0) uniform GlobalUniforms {
    mat4     view;
    mat4     projection;
    vec4     ambient_color;   // rgb=ambient, a=intensity
    vec4     camera_pos;      // xyz=world camera position
    GPULight lights[4];
    int      light_count;
    float    _pad0;
    float    _pad1;
    float    _pad2;
    mat4     light_space_matrix;
    int      shadow_enabled;
    float    shadow_bias;
    float    _pad3;
    float    _pad4;
} ubo;

layout(push_constant) uniform PushConstants {
    mat4  model;
    vec4  color;
    float roughness;
    float metallic;
    int   use_pbr;
    int   has_normal_map;
} push;

void main() {
    vec4 worldPos = push.model * vec4(inPosition, 1.0);
    fragWorldPos  = worldPos.xyz;
    // Normal matrix corrects normals under non-uniform scale.
    mat3 normalMatrix = transpose(inverse(mat3(push.model)));
    fragNormal    = normalize(normalMatrix * inNormal);
    fragTexCoord  = inTexCoord;
    fragColor     = push.color;
    gl_Position   = ubo.projection * ubo.view * worldPos;
}
