#include <metal_stdlib>
using namespace metal;

struct RowSlice {
    float4 hRow0;
    float4 hRow1;
    float4 hRow2;
};

struct WarpFrameUniforms {
    RowSlice slices[32];
    uint2 outSize;
    uint2 inSize;
    uint2 lumaOutSize;
    uint2 lumaInSize;
    float4 intrinsics; // (fx, fy, cx, cy)
    float4 distortion; // (k1, k2, p1, p2)
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

inline float2 solveRollingShutterAndDistort(
    float dstLumaX,
    float dstLumaY,
    constant WarpFrameUniforms& u
) {
    const float outHMinus1 = max(float(u.lumaOutSize.y) - 1.0f, 1.0f);
    const float inHMinus1 = max(float(u.lumaInSize.y) - 1.0f, 1.0f);
    float rowNorm = clamp(dstLumaY / outHMinus1, 0.0f, 1.0f);

    const float3 dstP = float3(dstLumaX, dstLumaY, 1.0f);
    float sx = dstLumaX;
    float sy = dstLumaY;

    // 3-iteration fixed-point solver for per-row rolling shutter convergence (< 0.01 px)
    for (int iter = 0; iter < 3; ++iter) {
        const float fIdx = clamp(rowNorm * 31.0f, 0.0f, 31.0f);
        const int i0 = min(int(floor(fIdx)), 30);
        const float t = fIdx - float(i0);

        const float3 r0 = mix(u.slices[i0].hRow0.xyz, u.slices[i0 + 1].hRow0.xyz, t);
        const float3 r1 = mix(u.slices[i0].hRow1.xyz, u.slices[i0 + 1].hRow1.xyz, t);
        const float3 r2 = mix(u.slices[i0].hRow2.xyz, u.slices[i0 + 1].hRow2.xyz, t);

        const float w = dot(r2, dstP);
        const float invW = 1.0f / max(w, 1e-5f);
        sx = dot(r0, dstP) * invW;
        sy = dot(r1, dstP) * invW;
        rowNorm = clamp(sy / inHMinus1, 0.0f, 1.0f);
    }

    // Apply radial Brown-Conrady lens distortion model if non-zero
    const float k1 = u.distortion.x;
    const float k2 = u.distortion.y;
    if (fabs(k1) > 1e-6f || fabs(k2) > 1e-6f) {
        const float xn = (sx - u.intrinsics.z) / u.intrinsics.x;
        const float yn = (sy - u.intrinsics.w) / u.intrinsics.y;
        const float r2 = xn * xn + yn * yn;
        const float radial = 1.0f + k1 * r2 + k2 * r2 * r2;
        sx = xn * radial * u.intrinsics.x + u.intrinsics.z;
        sy = yn * radial * u.intrinsics.y + u.intrinsics.w;
    }

    return float2(sx, sy);
}

kernel void lanczos4_warp_luma(
    texture2d<float, access::read> inTex [[texture(0)]],
    texture2d<float, access::write> outTex [[texture(1)]],
    constant WarpFrameUniforms& u [[buffer(0)]],
    uint2 gid [[thread_position_in_grid]])
{
    if (gid.x >= u.outSize.x || gid.y >= u.outSize.y) {
        return;
    }

    const float2 src = solveRollingShutterAndDistort(float(gid.x) + 0.5f, float(gid.y) + 0.5f, u) - 0.5f;
    const int ix = int(floor(src.x));
    const int iy = int(floor(src.y));
    const float fx = src.x - float(ix);
    const float fy = src.y - float(iy);

    float wx[8];
    float wy[8];
    for (int k = 0; k < 8; ++k) {
        const float offset = float(k - 3);
        wx[k] = lanczos4_weight(fx - offset);
        wy[k] = lanczos4_weight(fy - offset);
    }

    const int maxX = int(u.inSize.x) - 1;
    const int maxY = int(u.inSize.y) - 1;

    float sum = 0.0f;
    float wSum = 0.0f;
    for (int dy = -3; dy <= 4; ++dy) {
        const float wRow = wy[dy + 3];
        const int py = clamp(iy + dy, 0, maxY);
        for (int dx = -3; dx <= 4; ++dx) {
            const float weight = wx[dx + 3] * wRow;
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
    constant WarpFrameUniforms& u [[buffer(0)]],
    uint2 gid [[thread_position_in_grid]])
{
    if (gid.x >= u.outSize.x || gid.y >= u.outSize.y) {
        return;
    }

    const float2 lumaDst = float2(gid) * 2.0f + 1.0f;
    const float2 srcLuma = solveRollingShutterAndDistort(lumaDst.x, lumaDst.y, u);
    const float sx = (srcLuma.x - 1.0f) * 0.5f;
    const float sy = (srcLuma.y - 1.0f) * 0.5f;

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
