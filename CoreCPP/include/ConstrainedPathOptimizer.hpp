#pragma once

#include "Quaternion.hpp"
#include "TrajectoryEngine.hpp"

#include <cstdint>
#include <vector>

namespace gf {

struct ConstrainedPathInput {
    const TrajectoryEngine* engine = nullptr;
    const std::vector<Quat>* rawFrameQuats = nullptr;
    const std::vector<Vec3>* frameGravityW = nullptr;
    const std::vector<double>* frameTimestampsSec = nullptr;
    double nominalFps = 30.0;
    uint32_t width = 1920;
    uint32_t height = 1080;
    double fx = 1815.0;
    double fy = 1815.0;
    double cx = 960.0;
    double cy = 540.0;
    double readoutTimeSec = 0.0152;
};

struct ConstrainedPathResult {
    std::vector<Quat> smoothFrameQuats;
    int detectedGravityQuadrantDeg = 0;
    double avgPanRatio = 0.0;
    double maxSwingCapDeg = 24.45;
    int outerIterationsUsed = 0;
    bool valid = false;
};

/**
 * Offline L1 Optimal Camera Path Solver (Grundmann 2011 / Karpenko 2011 inspired).
 * Jointly minimizes L1 angular velocity, acceleration, and jerk under anisotropic
 * 16:9 crop constraints in camera tangent space via O(N) banded LDL^T IRLS + PDAS,
 * followed by an outer RS-slice verification loop (up to 3 passes).
 */
class ConstrainedPathOptimizer {
public:
    static ConstrainedPathResult solve(
        const ConstrainedPathInput& input,
        const StabilizerConfig& config
    );

    /// Solves SPD 7-diagonal (half-bandwidth 3) linear system H * x = rhs in O(N).
    static bool solveBandedLDLT(
        const std::vector<double>& diag0,
        const std::vector<double>& diag1,
        const std::vector<double>& diag2,
        const std::vector<double>& diag3,
        const std::vector<double>& rhs,
        std::vector<double>& outX
    );
};

} // namespace gf
