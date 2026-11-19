#include <metal_stdlib>
using namespace metal;

struct RowSlice {
    float4 hRow0;
    float4 hRow1;
    float4 hRow2;
};

struct WarpFrameUniforms {
    RowSlice slices[64];
    RowSlice histH[3];
    float4 histGains;    // (gain0, gain1, gain2, readoutAxis: 0=Y, 1=X)
    float4 polishParams; // (edgeFillMode, sharpenStrength, currentZoom, histCount)
    uint2 outSize;
    uint2 inSize;
    uint2 lumaOutSize;
    uint2 lumaInSize;
    float4 intrinsics;   // (fx, fy, cx, cy)
    float4 distortion;   // (k1, k2, p1, p2)
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

inline float3 applyBrownConradyDistort(float sx, float sy, constant WarpFrameUniforms& u) {
    const float k1 = u.distortion.x;
    const float k2 = u.distortion.y;
    if (fabs(k1) > 1e-6f || fabs(k2) > 1e-6f) {
        const float xn = (sx - u.intrinsics.z) / u.intrinsics.x;
        const float yn = (sy - u.intrinsics.w) / u.intrinsics.y;
        const float r2 = xn * xn + yn * yn;
        if (r2 > 2.5f) {
            return float3(0.0f, 0.0f, 0.0f);
        }
        const float radial = 1.0f + k1 * r2 + k2 * r2 * r2;
        sx = xn * radial * u.intrinsics.x + u.intrinsics.z;
        sy = yn * radial * u.intrinsics.y + u.intrinsics.w;
    }
    return float3(sx, sy, 1.0f);
}

inline float3 solveRollingShutterAndDistort(
    float dstLumaX,
    float dstLumaY,
    constant WarpFrameUniforms& u
) {
    const float inWMinus1 = max(float(u.lumaInSize.x) - 1.0f, 1.0f);
    const float inHMinus1 = max(float(u.lumaInSize.y) - 1.0f, 1.0f);
    const bool scanAlongX = (u.histGains.w > 0.5f);
    // Start at sensor center (0.5) so 90-degree rotated frames converge accurately in 4 iterations
    float rowNorm = 0.5f;

    const float3 dstP = float3(dstLumaX, dstLumaY, 1.0f);
    float sx = dstLumaX;
    float sy = dstLumaY;

    // 4-iteration fixed-point solver for per-row rolling shutter convergence (< 0.005 px even at 90 deg roll)
    for (int iter = 0; iter < 4; ++iter) {
        const float fIdx = clamp(rowNorm * 63.0f, 0.0f, 63.0f);
        const int i0 = min(int(floor(fIdx)), 62);
        const float t = fIdx - float(i0);

        const float3 r0 = mix(u.slices[i0].hRow0.xyz, u.slices[i0 + 1].hRow0.xyz, t);
        const float3 r1 = mix(u.slices[i0].hRow1.xyz, u.slices[i0 + 1].hRow1.xyz, t);
        const float3 r2 = mix(u.slices[i0].hRow2.xyz, u.slices[i0 + 1].hRow2.xyz, t);

        const float w = dot(r2, dstP);
        if (w <= 0.10f) {
            return float3(0.0f, 0.0f, 0.0f);
        }
        const float invW = 1.0f / w;
        sx = dot(r0, dstP) * invW;
        sy = dot(r1, dstP) * invW;
        rowNorm = scanAlongX ? clamp(sx / inWMinus1, 0.0f, 1.0f) : clamp(sy / inHMinus1, 0.0f, 1.0f);
    }

    return applyBrownConradyDistort(sx, sy, u);
}

inline float3 projectHistoryHomography(
    float dstLumaX,
    float dstLumaY,
    int histIdx,
    constant WarpFrameUniforms& u
) {
    const float3 dstP = float3(dstLumaX, dstLumaY, 1.0f);
    const float w = dot(u.histH[histIdx].hRow2.xyz, dstP);
    if (w <= 0.15f) {
        return float3(0.0f, 0.0f, 0.0f);
    }
    const float invW = 1.0f / w;
    const float sx = dot(u.histH[histIdx].hRow0.xyz, dstP) * invW;
    const float sy = dot(u.histH[histIdx].hRow1.xyz, dstP) * invW;
    return applyBrownConradyDistort(sx, sy, u);
}

inline float mirrorCoord(float v, float maxV) {
    if (maxV <= 1.0f) {
        return 0.0f;
    }
    float m = fmod(fabs(v), 2.0f * maxV);
    if (m > maxV) {
        m = 2.0f * maxV - m;
    }
    return clamp(m, 0.0f, maxV);
}

inline float sampleBilinearLuma(texture2d<float, access::read> tex, float sx, float sy, int maxX, int maxY) {
    const int x0 = clamp(int(floor(sx)), 0, maxX);
    const int y0 = clamp(int(floor(sy)), 0, maxY);
    const int x1 = min(x0 + 1, maxX);
    const int y1 = min(y0 + 1, maxY);
    const float fx = clamp(sx - float(x0), 0.0f, 1.0f);
    const float fy = clamp(sy - float(y0), 0.0f, 1.0f);
    const float v00 = tex.read(uint2(x0, y0)).r;
    const float v10 = tex.read(uint2(x1, y0)).r;
    const float v01 = tex.read(uint2(x0, y1)).r;
    const float v11 = tex.read(uint2(x1, y1)).r;
    return mix(mix(v00, v10, fx), mix(v01, v11, fx), fy);
}

inline float2 sampleBilinearChroma(texture2d<float, access::read> tex, float sx, float sy, int maxX, int maxY) {
    const int x0 = clamp(int(floor(sx)), 0, maxX);
    const int y0 = clamp(int(floor(sy)), 0, maxY);
    const int x1 = min(x0 + 1, maxX);
    const int y1 = min(y0 + 1, maxY);
    const float fx = clamp(sx - float(x0), 0.0f, 1.0f);
    const float fy = clamp(sy - float(y0), 0.0f, 1.0f);
    const float2 v00 = tex.read(uint2(x0, y0)).rg;
    const float2 v10 = tex.read(uint2(x1, y0)).rg;
    const float2 v01 = tex.read(uint2(x0, y1)).rg;
    const float2 v11 = tex.read(uint2(x1, y1)).rg;
    return mix(mix(v00, v10, fx), mix(v01, v11, fx), fy);
}

inline float sampleMirrorBlurLuma(texture2d<float, access::read> tex, float sx, float sy, int maxX, int maxY) {
    const float mx = mirrorCoord(sx, float(maxX));
    const float my = mirrorCoord(sy, float(maxY));
    const float c = sampleBilinearLuma(tex, mx, my, maxX, maxY) * 0.4f;
    const float l = sampleBilinearLuma(tex, mirrorCoord(mx - 2.0f, float(maxX)), my, maxX, maxY) * 0.15f;
    const float r = sampleBilinearLuma(tex, mirrorCoord(mx + 2.0f, float(maxX)), my, maxX, maxY) * 0.15f;
    const float u = sampleBilinearLuma(tex, mx, mirrorCoord(my - 2.0f, float(maxY)), maxX, maxY) * 0.15f;
    const float d = sampleBilinearLuma(tex, mx, mirrorCoord(my + 2.0f, float(maxY)), maxX, maxY) * 0.15f;
    return c + l + r + u + d;
}

kernel void lanczos4_warp_luma(
    texture2d<float, access::read> inTex [[texture(0)]],
    texture2d<float, access::write> outTex [[texture(1)]],
    texture2d<float, access::read> histTex0 [[texture(2)]],
    texture2d<float, access::read> histTex1 [[texture(3)]],
    texture2d<float, access::read> histTex2 [[texture(4)]],
    constant WarpFrameUniforms& u [[buffer(0)]],
    uint2 gid [[thread_position_in_grid]])
{
    if (gid.x >= u.outSize.x || gid.y >= u.outSize.y) {
        return;
    }

    const int edgeFillMode = int(u.polishParams.x + 0.5f);
    const float sharpenStrength = u.polishParams.y;
    const float currentZoom = u.polishParams.z;
    const int histCount = int(u.polishParams.w + 0.5f);

    const float dstX = float(gid.x) + 0.5f;
    const float dstY = float(gid.y) + 0.5f;
    const float3 solved = solveRollingShutterAndDistort(dstX, dstY, u);

    const int maxX = int(u.inSize.x) - 1;
    const int maxY = int(u.inSize.y) - 1;
    const float2 src = solved.xy - 0.5f;

    float edgeDist = -100.0f;
    if (solved.z > 0.5f) {
        edgeDist = min(min(src.x, float(maxX) - src.x), min(src.y, float(maxY) - src.y));
    }

    float curVal = 0.0f;
    const bool inValid = (edgeDist >= 0.0f);

    if (inValid) {
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

        // 2x2 neighborhood bounds for Anti-Ringing Clamped Lanczos4 + zero-extra-read CAS
        const float c00 = inTex.read(uint2(clamp(ix, 0, maxX), clamp(iy, 0, maxY))).r;
        const float c10 = inTex.read(uint2(clamp(ix + 1, 0, maxX), clamp(iy, 0, maxY))).r;
        const float c01 = inTex.read(uint2(clamp(ix, 0, maxX), clamp(iy + 1, 0, maxY))).r;
        const float c11 = inTex.read(uint2(clamp(ix + 1, 0, maxX), clamp(iy + 1, 0, maxY))).r;
        const float min2x2 = min(min(c00, c10), min(c01, c11));
        const float max2x2 = max(max(c00, c10), max(c01, c11));

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

        const float rawY = sum / max(wSum, 1e-5f);
        const float clampedY = clamp(rawY, min2x2 - 0.025f, max2x2 + 0.025f);
        const float relaxedY = mix(rawY, clampedY, 0.75f);

        // Zoom-Proportional Contrast-Adaptive Sharpening (zero extra texture reads)
        float sharpenedY = relaxedY;
        if (sharpenStrength > 1e-4f) {
            const float localContrast = max2x2 - min2x2;
            const float flatNoiseGate = smoothstep(0.015f, 0.05f, localContrast);
            const float haloGuard = clamp(1.0f - max2x2, 0.0f, 1.0f);
            const float zoomBoost = clamp(1.0f + 1.15f * (currentZoom - 1.0f), 1.0f, 2.05f);
            const float avg2x2 = 0.25f * (c00 + c10 + c01 + c11);
            const float detail = relaxedY - avg2x2;
            const float casWeight = sharpenStrength * zoomBoost * flatNoiseGate * (0.35f + 0.65f * haloGuard);
            sharpenedY = clamp(relaxedY + casWeight * detail, 0.0f, 1.0f);
        }
        curVal = sharpenedY;

        if (edgeFillMode != 1 || edgeDist >= 16.0f || histCount <= 0) {
            outTex.write(float4(curVal, 0.0f, 0.0f, 1.0f), gid);
            return;
        }
    }

    // Temporal Full-Frame Edge Fill (or Mirror Fallback)
    if (edgeFillMode == 1 && histCount > 0) {
        for (int h = 0; h < min(histCount, 3); ++h) {
            const float3 hSolved = projectHistoryHomography(dstX, dstY, h, u);
            if (hSolved.z > 0.5f &&
                hSolved.x >= 4.0f && hSolved.x <= float(u.lumaInSize.x - 5) &&
                hSolved.y >= 4.0f && hSolved.y <= float(u.lumaInSize.y - 5)) {
                const float hx = hSolved.x - 0.5f;
                const float hy = hSolved.y - 0.5f;
                float hVal = 0.0f;
                if (h == 0) {
                    hVal = sampleBilinearLuma(histTex0, hx, hy, maxX, maxY) * u.histGains.x;
                } else if (h == 1) {
                    hVal = sampleBilinearLuma(histTex1, hx, hy, maxX, maxY) * u.histGains.y;
                } else {
                    hVal = sampleBilinearLuma(histTex2, hx, hy, maxX, maxY) * u.histGains.z;
                }
                hVal = clamp(hVal, 0.0f, 1.0f);
                if (inValid) {
                    const float feather = smoothstep(0.0f, 16.0f, edgeDist);
                    outTex.write(float4(mix(hVal, curVal, feather), 0.0f, 0.0f, 1.0f), gid);
                } else {
                    outTex.write(float4(hVal, 0.0f, 0.0f, 1.0f), gid);
                }
                return;
            }
        }
    }

    if (inValid) {
        outTex.write(float4(curVal, 0.0f, 0.0f, 1.0f), gid);
        return;
    }

    if (edgeFillMode != 0 && solved.z > 0.5f) {
        const float mirrorVal = sampleMirrorBlurLuma(inTex, src.x, src.y, maxX, maxY);
        outTex.write(float4(clamp(mirrorVal, 0.0f, 1.0f), 0.0f, 0.0f, 1.0f), gid);
        return;
    }

    outTex.write(float4(0.0f, 0.0f, 0.0f, 1.0f), gid);
}

kernel void lanczos4_warp_chroma(
    texture2d<float, access::read> inTex [[texture(0)]],
    texture2d<float, access::write> outTex [[texture(1)]],
    texture2d<float, access::read> histTex0 [[texture(2)]],
    texture2d<float, access::read> histTex1 [[texture(3)]],
    texture2d<float, access::read> histTex2 [[texture(4)]],
    constant WarpFrameUniforms& u [[buffer(0)]],
    uint2 gid [[thread_position_in_grid]])
{
    if (gid.x >= u.outSize.x || gid.y >= u.outSize.y) {
        return;
    }

    const int edgeFillMode = int(u.polishParams.x + 0.5f);
    const int histCount = int(u.polishParams.w + 0.5f);

    const float2 lumaDst = float2(gid) * 2.0f + 1.0f;
    const float3 solved = solveRollingShutterAndDistort(lumaDst.x, lumaDst.y, u);

    const int maxX = int(u.inSize.x) - 1;
    const int maxY = int(u.inSize.y) - 1;
    const float sx = (solved.x - 1.0f) * 0.5f;
    const float sy = (solved.y - 1.0f) * 0.5f;

    float edgeDist = -100.0f;
    if (solved.z > 0.5f) {
        edgeDist = min(min(sx, float(maxX) - sx), min(sy, float(maxY) - sy));
    }

    float2 curUV = float2(0.5f, 0.5f);
    const bool inValid = (edgeDist >= 0.0f);

    if (inValid) {
        const int ix = int(floor(sx));
        const int iy = int(floor(sy));
        const float fx = sx - float(ix);
        const float fy = sy - float(iy);

        float wx[4];
        float wy[4];
        for (int k = 0; k < 4; ++k) {
            const float offset = float(k - 1);
            wx[k] = lanczos2_weight(fx - offset);
            wy[k] = lanczos2_weight(fy - offset);
        }

        float2 sum = float2(0.0f, 0.0f);
        float wSum = 0.0f;
        for (int dy = -1; dy <= 2; ++dy) {
            const float wRow = wy[dy + 1];
            const int py = clamp(iy + dy, 0, maxY);
            for (int dx = -1; dx <= 2; ++dx) {
                const float weight = wx[dx + 1] * wRow;
                const int px = clamp(ix + dx, 0, maxX);
                sum += inTex.read(uint2(px, py)).rg * weight;
                wSum += weight;
            }
        }
        curUV = clamp(sum / max(wSum, 1e-5f), 0.0f, 1.0f);

        if (edgeFillMode != 1 || edgeDist >= 8.0f || histCount <= 0) {
            outTex.write(float4(curUV.x, curUV.y, 0.0f, 1.0f), gid);
            return;
        }
    }

    if (edgeFillMode == 1 && histCount > 0) {
        for (int h = 0; h < min(histCount, 3); ++h) {
            const float3 hSolved = projectHistoryHomography(lumaDst.x, lumaDst.y, h, u);
            if (hSolved.z > 0.5f &&
                hSolved.x >= 4.0f && hSolved.x <= float(u.lumaInSize.x - 5) &&
                hSolved.y >= 4.0f && hSolved.y <= float(u.lumaInSize.y - 5)) {
                const float hsx = (hSolved.x - 1.0f) * 0.5f;
                const float hsy = (hSolved.y - 1.0f) * 0.5f;
                float2 hUV = float2(0.5f, 0.5f);
                if (h == 0) {
                    hUV = sampleBilinearChroma(histTex0, hsx, hsy, maxX, maxY);
                } else if (h == 1) {
                    hUV = sampleBilinearChroma(histTex1, hsx, hsy, maxX, maxY);
                } else {
                    hUV = sampleBilinearChroma(histTex2, hsx, hsy, maxX, maxY);
                }
                if (inValid) {
                    const float feather = smoothstep(0.0f, 8.0f, edgeDist);
                    const float2 blended = mix(hUV, curUV, feather);
                    outTex.write(float4(blended.x, blended.y, 0.0f, 1.0f), gid);
                } else {
                    outTex.write(float4(hUV.x, hUV.y, 0.0f, 1.0f), gid);
                }
                return;
            }
        }
    }

    if (inValid) {
        outTex.write(float4(curUV.x, curUV.y, 0.0f, 1.0f), gid);
        return;
    }

    if (edgeFillMode != 0 && solved.z > 0.5f) {
        const float2 mUV = sampleBilinearChroma(
            inTex, mirrorCoord(sx, float(maxX)), mirrorCoord(sy, float(maxY)), maxX, maxY
        );
        outTex.write(float4(mUV.x, mUV.y, 0.0f, 1.0f), gid);
        return;
    }

    outTex.write(float4(0.5f, 0.5f, 0.0f, 1.0f), gid);
}
