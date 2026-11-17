#include <metal_stdlib>
using namespace metal;

struct WarpParams {
    float2 outSize;
    float2 inSize;
    float3x3 homography;
};

inline float sinc_f(float x) {
    if (fabs(x) < 1e-5f) {
        return 1.0f;
    }
    const float pix = 3.14159265358979323846f * x;
    return sin(pix) / pix;
}

inline float lanczos4_weight(float x) {
    const float ax = fabs(x);
    if (ax >= 4.0f) {
        return 0.0f;
    }
    return sinc_f(x) * sinc_f(x * 0.25f);
}

kernel void lanczos4_warp_luma(
    texture2d<float, access::sample> inTex [[texture(0)]],
    texture2d<float, access::write> outTex [[texture(1)]],
    constant WarpParams& params [[buffer(0)]],
    uint2 gid [[thread_position_in_grid]])
{
    if (gid.x >= outTex.get_width() || gid.y >= outTex.get_height()) {
        return;
    }

    constexpr sampler bilinearSampler(coord::normalized, address::clamp_to_edge, filter::linear);
    const float2 uv = (float2(gid) + 0.5f) / params.outSize;
    const float3 warped = params.homography * float3(uv, 1.0f);
    const float2 srcUV = warped.xy / max(warped.z, 1e-6f);

    const float4 sampleVal = inTex.sample(bilinearSampler, srcUV);
    outTex.write(sampleVal, gid);
}

kernel void lanczos4_warp_chroma(
    texture2d<float, access::sample> inTex [[texture(0)]],
    texture2d<float, access::write> outTex [[texture(1)]],
    constant WarpParams& params [[buffer(0)]],
    uint2 gid [[thread_position_in_grid]])
{
    if (gid.x >= outTex.get_width() || gid.y >= outTex.get_height()) {
        return;
    }

    constexpr sampler bilinearSampler(coord::normalized, address::clamp_to_edge, filter::linear);
    const float2 uv = (float2(gid) + 0.5f) / params.outSize;
    const float3 warped = params.homography * float3(uv, 1.0f);
    const float2 srcUV = warped.xy / max(warped.z, 1e-6f);

    const float4 sampleVal = inTex.sample(bilinearSampler, srcUV);
    outTex.write(sampleVal, gid);
}
