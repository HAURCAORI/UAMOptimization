#version 450

layout(location = 0) in  vec3 inDir;
layout(location = 0) out vec4 outColor;

layout(push_constant) uniform SkyPush {
    vec4 sky_color;  // top / zenith color used in gradient and procedural modes
    int  sky_mode;   // 1=gradient, 2=procedural, 3=cubemap, 4=HDR equirect (0=Off is skipped CPU-side)
    int  _pad0;
    int  _pad1;
    int  _pad2;
} push;

// Skybox textures live in the shared descriptor set (set 0) alongside the UBO.
layout(set = 0, binding = 5) uniform samplerCube skyCubemap;   // 4x3 cross -> cubemap
layout(set = 0, binding = 6) uniform sampler2D   skyEquirect;  // equirectangular HDR (decoded to LDR)

const float PI = 3.14159265358979324;

void main() {
    vec3 dir = normalize(inDir);

    if (push.sky_mode == 3) {
        // Cubemap. Viewer is Z-up; remap so +Z maps to the cubemap's +Y face.
        vec3 sampleDir = vec3(dir.x, dir.z, dir.y);
        outColor = texture(skyCubemap, sampleDir);
        return;
    }
    if (push.sky_mode == 4) {
        // Equirectangular. Z is the pole (viewer is Z-up).
        float u = atan(dir.y, dir.x) / (2.0 * PI) + 0.5;
        float v = acos(clamp(dir.z, -1.0, 1.0)) / PI;
        outColor = texture(skyEquirect, vec2(u, v));
        return;
    }

    float t = clamp(dir.z * 0.5 + 0.5, 0.0, 1.0);  // 0 = ground, 1 = zenith (Z-up)

    if (push.sky_mode == 2) {
        // Procedural sky: analytical gradient with a sun-glow disc.
        float sunGlow = pow(max(dot(dir, normalize(vec3(0.25, 0.92, 0.45))), 0.0), 80.0);
        vec3 zenith     = push.sky_color.rgb;
        vec3 horizonCol = mix(vec3(0.20, 0.28, 0.40), zenith, 0.55);
        vec3 groundCol  = vec3(0.06, 0.05, 0.04);
        vec3 color = mix(groundCol, horizonCol, smoothstep(0.0,  0.42, t));
        color      = mix(color,     zenith,     smoothstep(0.35, 1.00, t));
        color     += vec3(1.00, 0.85, 0.55) * sunGlow * 0.40;
        outColor = vec4(color, 1.0);
    } else {
        // Gradient (mode 1): ground -> horizon -> zenith blend.
        vec3 groundCol  = vec3(0.10, 0.09, 0.07);
        vec3 horizonCol = vec3(0.52, 0.62, 0.74);
        vec3 zenith     = push.sky_color.rgb;
        vec3 color = mix(mix(groundCol, horizonCol, t), zenith, t * t);
        outColor = vec4(color, 1.0);
    }
}
