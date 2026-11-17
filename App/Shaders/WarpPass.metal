#include <metal_stdlib>
using namespace metal;

struct WarpUniforms {
    float4 hRow0;
    float4 hRow1;
    float4 hRow2;
    uint2 outSize;
    uint2 inSize;
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

inline float lanczos2_weight(float x) {
    const float ax = fabs(x);
    if (ax >= 2.0f) {
        return 0.0f;
    }
    return sinc_f(x) * sinc_f(x * 0.5f);
}

kernel void lanczos4_warp_luma(
    texture2d<float, access::read> inTex [[texture(0)]],
    texture2d<float, access::write> outTex [[texture(1)]],
    constant WarpUniforms& u [[buffer(0)]],
    uint2 gid [[thread_position_in_grid]])
{
    if (gid.x >= u.outSize.x || gid.y >= u.outSize.y) {
        return;
    }

    const float3 dstP = float3(float(gid.x), float(gid.y), 1.0f);
    const float w = dot(u.hRow2.xyz, dstP);
    const float invW = 1.0f / max(w, 1e-5f);
    const float sx = dot(u.hRow0.xyz, dstP) * invW;
    const float sy = dot(u.hRow1.xyz, dstP) * invW;

    const int ix = int(floor(sx));
    const int iy = int(floor(sy));
    const float fx = sx - float(ix);
    const float fy = sy - float(iy);

    float wx[8];
    for (int k = 0; k < 8; ++k) {
        wx[k] = lanczos4_weight(fx - float(k - 3));
    }

    const int maxX = int(u.inSize.x) - 1;
    const int maxY = int(u.inSize.y) - 1;

    float sum = 0.0f;
    float wSum = 0.0f;
    for (int dy = -3; dy <= 4; ++dy) {
        const float wy = lanczos4_weight(fy - float(dy));
        const int py = clamp(iy + dy, 0, maxY);
        for (int dx = -3; dx <= 4; ++dx) {
            const float weight = wx[dx + 3] * wy;
            const int px = clamp(ix + dx, 0, maxX);
            sum += inTex.read(uint2(px, py)).r * weight;
            wSum += weight;
        }
    }

    const float outY = clamp(sum / max(wSum, 1e-5f), 0.0f, 1.0f);
    outTex.write(float4(outY, 0.0f, 0.0f, 1.0f), gid);
}

kernel void lanczos4_warp_chroma(
    texture2d<float, access::read> inTex [[texture(0)]],
    texture2d<float, access::write> outTex [[texture(1)]],
    constant WarpUniforms& u [[buffer(0)]],
    uint2 gid [[thread_position_in_grid]])
{
    if (gid.x >= u.outSize.x || gid.y >= u.outSize.y) {
        return;
    }

    const float2 lumaDst = float2(gid) * 2.0f + 0.5f;
    const float3 dstP = float3(lumaDst.x, lumaDst.y, 1.0f);
    const float w = dot(u.hRow2.xyz, dstP);
    const float invW = 1.0f / max(w, 1e-5f);
    const float sxLuma = dot(u.hRow0.xyz, dstP) * invW;
    const float syLuma = dot(u.hRow1.xyz, dstP) * invW;

    const float sx = (sxLuma - 0.5f) * 0.5f;
    const float sy = (syLuma - 0.5f) * 0.5f;

    const int ix = int(floor(sx));
    const int iy = int(floor(sy));
    const float fx = sx - float(ix);
    const float fy = sy - float(iy);

    float wx[4];
    for (int k = 0; k < 4; ++k) {
        wx[k] = lanczos2_weight(fx - float(k - 1));
    }

    const int maxX = int(u.inSize.x) - 1;
    const int maxY = int(u.inSize.y) - 1;

    float2 sum = float2(0.0f, 0.0f);
    float wSum = 0.0f;
    for (int dy = -1; dy <= 2; ++dy) {
        const float wy = lanczos2_weight(fy - float(dy));
        const int py = clamp(iy + dy, 0, maxY);
        for (int dx = -1; dx <= 2; ++dx) {
            const float weight = wx[dx + 1] * wy;
            const int px = clamp(ix + dx, 0, maxX);
            sum += inTex.read(uint2(px, py)).rg * weight;
            wSum += weight;
        }
    }

    const float2 outUV = clamp(sum / max(wSum, 1e-5f), 0.0f, 1.0f);
    outTex.write(float4(outUV.x, outUV.y, 0.0f, 1.0f), gid);
}
