#version 450

layout(location = 0) in vec3 fragWorldPos;
layout(location = 1) in vec3 fragNormal;
layout(location = 2) in vec2 fragTexCoord;
layout(location = 3) in vec4 fragColor;

layout(location = 0) out vec4 outColor;

struct GPULight {
    vec4 position;    // xyz=pos or dir, w=type (0=dir, 1=point, 2=spot)
    vec4 color;       // rgb=color, a=intensity
    vec4 direction;   // xyz=dir, w=range
    vec4 spotAngles;  // x=cos(inner), y=cos(outer), zw=unused
};

layout(set = 0, binding = 0) uniform GlobalUniforms {
    mat4     view;
    mat4     projection;
    vec4     ambient_color;          // rgb=ambient, a=intensity
    vec4     camera_pos;             // xyz=world camera position
    GPULight lights[4];
    int      light_count;
    // Individual scalar pads: std140 rounds each array element up to 16 bytes,
    // so a float[3] here would shift light_space_matrix by 48 bytes. Keep scalars.
    float    _pad0;
    float    _pad1;
    float    _pad2;
    mat4     light_space_matrix;     // orthographic light VP for shadow mapping
    int      shadow_enabled;         // 0 = off, 1 = on
    float    shadow_bias;            // depth offset to prevent self-shadowing
    float    _pad3;
    float    _pad4;
    vec4     ambient_ground;         // rgb = environment ground tint (hemisphere lower half)
} ubo;

layout(set = 0, binding = 1) uniform sampler2D    albedoTex;
layout(set = 0, binding = 2) uniform sampler2D    mrTex;      // glTF ORM: r=AO, g=roughness, b=metallic
layout(set = 0, binding = 3) uniform sampler2D    normalTex;  // tangent-space normal map
layout(set = 0, binding = 4) uniform sampler2DShadow shadowMap; // depth comparison sampler

layout(push_constant) uniform PushConstants {
    mat4  model;
    vec4  color;
    float roughness;
    float metallic;
    int   use_pbr;
    int   has_normal_map;
} push;

// ---- Light direction + attenuation helper ------------------------------------
vec4 lightDirAtten(GPULight light, vec3 fragPos) {
    int   ltype = clamp(int(light.position.w + 0.5), 0, 2);
    vec3  ld    = vec3(0.0);
    float att   = 1.0;

    if (ltype == 0) {
        ld = normalize(-light.direction.xyz);
    } else {
        vec3  toLight = light.position.xyz - fragPos;
        float dist    = max(length(toLight), 1e-4);
        ld            = toLight / dist;
        float range   = max(light.direction.w, 1e-4);
        float ratio   = clamp(dist / range, 0.0, 1.0);
        att           = clamp(1.0 - ratio * ratio, 0.0, 1.0);
        att          *= att;
        if (ltype == 2) {
            float cosA  = dot(-ld, normalize(light.direction.xyz));
            float inner = light.spotAngles.x;
            float outer = light.spotAngles.y;
            att *= clamp((cosA - outer) / max(inner - outer, 0.001), 0.0, 1.0);
        }
    }
    return vec4(ld, att);
}

// ---- Blinn-Phong shading ----------------------------------------------------
vec3 blinnPhong(GPULight light, vec3 fragPos, vec3 normal, vec3 viewDir, vec3 albedo) {
    vec4  lda     = lightDirAtten(light, fragPos);
    vec3  ld      = lda.xyz;
    float att     = lda.w;
    float NdotL   = max(dot(normal, ld), 0.0);
    vec3  halfVec = normalize(ld + viewDir + vec3(1e-6));
    float NdotH   = max(dot(normal, halfVec), 0.0);
    float spec    = pow(NdotH, 64.0) * 0.4;
    return (NdotL * albedo + spec) * light.color.rgb * light.color.a * att;
}

// ---- Cook-Torrance PBR BRDF -------------------------------------------------
float D_GGX(float NdotH, float roughness) {
    float a  = roughness * roughness;
    float a2 = a * a;
    float d  = (NdotH * NdotH) * (a2 - 1.0) + 1.0;
    return a2 / (3.14159265 * d * d);
}

float G_SmithGGX(float NdotV, float NdotL, float roughness) {
    float r  = roughness + 1.0;
    float k  = (r * r) / 8.0;
    float g1 = NdotV / (NdotV * (1.0 - k) + k);
    float g2 = NdotL / (NdotL * (1.0 - k) + k);
    return g1 * g2;
}

vec3 F_Schlick(float VdotH, vec3 F0) {
    return F0 + (1.0 - F0) * pow(clamp(1.0 - VdotH, 0.0, 1.0), 5.0);
}

vec3 cookTorrance(GPULight light, vec3 fragPos, vec3 normal, vec3 viewDir,
                  vec3 albedo, float roughness, float metallic) {
    vec4  lda   = lightDirAtten(light, fragPos);
    vec3  ld    = lda.xyz;
    float att   = lda.w;
    float NdotL = max(dot(normal, ld), 0.0);
    if (NdotL <= 0.0) return vec3(0.0);

    vec3  halfVec = normalize(ld + viewDir);
    float NdotV   = max(dot(normal, viewDir), 1e-5);
    float NdotH   = max(dot(normal, halfVec), 0.0);
    float VdotH   = max(dot(viewDir, halfVec), 0.0);

    vec3  F0   = mix(vec3(0.04), albedo, metallic);
    float D    = D_GGX(NdotH, max(roughness, 0.05));
    float G    = G_SmithGGX(NdotV, NdotL, roughness);
    vec3  F    = F_Schlick(VdotH, F0);
    vec3  spec = D * G * F / max(4.0 * NdotV * NdotL, 0.0001);
    vec3  kd   = (1.0 - F) * (1.0 - metallic);
    return (kd * albedo / 3.14159265 + spec) * light.color.rgb * light.color.a * att * NdotL;
}

// ---- Screen-space TBN normal map perturbation --------------------------------
vec3 perturbNormal(vec3 worldNormal, vec3 worldPos, vec2 uv) {
    vec3 dPdx = dFdx(worldPos);
    vec3 dPdy = dFdy(worldPos);
    vec2 dUdx = dFdx(uv);
    vec2 dUdy = dFdy(uv);

    float det    = dUdx.x * dUdy.y - dUdy.x * dUdx.y;
    float rcpDet = 1.0 / (abs(det) + 1e-8);
    vec3 T = rcpDet * (dUdy.y * dPdx - dUdx.y * dPdy);
    vec3 B = rcpDet * (dUdx.x * dPdy - dUdy.x * dPdx);

    vec3 N = normalize(worldNormal);
    T = normalize(T - dot(T, N) * N);
    B = normalize(B - dot(B, N) * N);

    vec3 tn = texture(normalTex, uv).rgb * 2.0 - 1.0;
    return normalize(T * tn.x + B * tn.y + N * tn.z);
}

// ---- 3×3 PCF shadow test -----------------------------------------------------
// Returns 1.0 (fully lit) to 0.0 (fully in shadow).
// sampler2DShadow with LESS_OR_EQUAL compare returns 1.0 when fragment is lit.
float shadowTest(vec3 worldPos) {
    if (ubo.shadow_enabled == 0) return 1.0;

    vec4 lightSpacePos = ubo.light_space_matrix * vec4(worldPos, 1.0);
    vec3 proj     = lightSpacePos.xyz / lightSpacePos.w;
    vec2 shadowUV = proj.xy * 0.5 + 0.5;

    // Outside the shadow frustum: treat as fully lit.
    if (any(lessThan(shadowUV, vec2(0.001))) || any(greaterThan(shadowUV, vec2(0.999)))) {
        return 1.0;
    }

    float currentDepth = proj.z;
    if (currentDepth < 0.0 || currentDepth > 1.0) return 1.0;

    float biasedDepth = currentDepth - ubo.shadow_bias;

    // 3×3 PCF (hardware bilinear comparison per sample for a softer result).
    float shadow   = 0.0;
    vec2  texelSz  = 1.0 / vec2(2048.0);
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            shadow += texture(shadowMap,
                vec3(shadowUV + vec2(float(x), float(y)) * texelSz, biasedDepth));
        }
    }
    return shadow / 9.0;
}

void main() {
    vec3 normal  = normalize(fragNormal);
    if (push.has_normal_map != 0) {
        normal = perturbNormal(normal, fragWorldPos, fragTexCoord);
    }
    vec3 viewDir = normalize(ubo.camera_pos.xyz - fragWorldPos);

    vec4 texSample = texture(albedoTex, fragTexCoord);
    vec3 albedo;
    float alpha = fragColor.a;
    if (push.use_pbr != 0) {
        albedo = texSample.rgb * fragColor.rgb;
    } else {
        albedo = fragColor.rgb * mix(vec3(1.0), texSample.rgb, 0.25);
    }

    // Metallic-roughness: glTF ORM (r=AO, g=roughness, b=metallic).
    vec3  orm       = texture(mrTex, fragTexCoord).rgb;
    float roughness = (push.use_pbr != 0) ? clamp(orm.g, 0.04, 1.0) : push.roughness;
    float metallic  = (push.use_pbr != 0) ? clamp(orm.b, 0.0,  1.0) : push.metallic;

    // Hemispheric environment ambient (unshadowed): sky tint on up-facing surfaces,
    // ground tint on down-facing ones. ambient_color.rgb = sky env, .a = intensity;
    // ambient_ground.rgb = ground env. Both are CPU-derived from the active skybox.
    float hemi        = clamp(normal.z * 0.5 + 0.5, 0.0, 1.0);
    vec3  envAmbient  = mix(ubo.ambient_ground.rgb, ubo.ambient_color.rgb, hemi);
    vec3  result      = envAmbient * ubo.ambient_color.a * albedo;

    // Shadow visibility for direct light contributions.
    float shadowVis = shadowTest(fragWorldPos);

    // Accumulate direct lights.
    int nLights = min(ubo.light_count, 4);
    for (int i = 0; i < nLights; ++i) {
        // Shadow only attenuates the directional light (type 0).
        float vis = (int(ubo.lights[i].position.w + 0.5) == 0) ? shadowVis : 1.0;
        if (push.use_pbr != 0) {
            result += cookTorrance(ubo.lights[i], fragWorldPos, normal, viewDir,
                                   albedo, roughness, metallic) * vis;
        } else {
            result += blinnPhong(ubo.lights[i], fragWorldPos, normal, viewDir, albedo) * vis;
        }
    }

    // Gamma correction: convert linear PBR output to approximate sRGB for the
    // UNORM swapchain.  Applied to both paths so the scene looks consistent.
    result = pow(max(result, vec3(0.0)), vec3(1.0 / 2.2));

    outColor = vec4(result, alpha);
}
