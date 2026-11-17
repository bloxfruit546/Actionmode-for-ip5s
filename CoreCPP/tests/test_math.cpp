#include "Quaternion.hpp"
#include "SidecarFormat.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

namespace {

constexpr double kPi = 3.14159265358979323846;

bool approxEqual(double a, double b, double tol = 1e-12) {
    return std::fabs(a - b) <= tol;
}

bool approxQuat(const gf::Quat& a, const gf::Quat& b, double tol = 1e-12) {
    const bool sameSign =
        approxEqual(a.w, b.w, tol) &&
        approxEqual(a.x, b.x, tol) &&
        approxEqual(a.y, b.y, tol) &&
        approxEqual(a.z, b.z, tol);
    const bool flippedSign =
        approxEqual(a.w, -b.w, tol) &&
        approxEqual(a.x, -b.x, tol) &&
        approxEqual(a.y, -b.y, tol) &&
        approxEqual(a.z, -b.z, tol);
    return sameSign || flippedSign;
}

void testMultiplicationAndInverse() {
    const gf::Quat q1 = gf::Quat::fromAxisAngle({1.0, 2.0, -0.5}, 0.73);
    const gf::Quat q1Inv = q1.conjugate();
    const gf::Quat prod = q1 * q1Inv;
    assert(approxQuat(prod, gf::Quat::identity(), 1e-12));

    const gf::Quat q2 = gf::Quat::fromAxisAngle({-0.3, 0.8, 1.2}, -1.15);
    const gf::Quat q12 = q1 * q2;
    const gf::Vec3 v(0.4, -1.3, 2.1);
    const gf::Vec3 vSeq = q1.rotateVector(q2.rotateVector(v));
    const gf::Vec3 vComb = q12.rotateVector(v);
    assert(approxEqual(vSeq.x, vComb.x, 1e-12));
    assert(approxEqual(vSeq.y, vComb.y, 1e-12));
    assert(approxEqual(vSeq.z, vComb.z, 1e-12));
}

void testGyroIntegration90DegZ() {
    gf::Quat q = gf::Quat::identity();
    const gf::Vec3 omega(0.0, 0.0, kPi * 0.5);
    const double dt = 0.01; // 100 Hz for 1.0 second
    for (int i = 0; i < 100; ++i) {
        q = gf::Quat::integrateGyroStep(q, omega, dt);
    }

    const gf::Quat expected = gf::Quat::fromAxisAngle({0.0, 0.0, 1.0}, kPi * 0.5);
    assert(approxQuat(q, expected, 1e-9));

    const gf::Vec3 rotated = q.rotateVector({1.0, 0.0, 0.0});
    assert(approxEqual(rotated.x, 0.0, 1e-9));
    assert(approxEqual(rotated.y, 1.0, 1e-9));
    assert(approxEqual(rotated.z, 0.0, 1e-9));
}

void testSwingTwistDecomposition() {
    const gf::Vec3 gravity = gf::Vec3(0.25, -0.90, 0.35).normalized();
    const gf::Quat tilt = gf::Quat::fromAxisAngle({1.0, 0.3, -0.4}, 0.61);
    const gf::Quat yaw = gf::Quat::fromAxisAngle(gravity, -1.23);
    const gf::Quat q = (tilt * yaw).normalized();

    gf::Quat swing;
    gf::Quat twist;
    gf::Quat::decomposeSwingTwist(q, gravity, swing, twist);

    const gf::Quat reconstructed = (swing * twist).normalized();
    assert(approxQuat(reconstructed, q, 1e-12));

    // Twist must leave the gravity axis invariant.
    const gf::Vec3 rotatedGravity = twist.rotateVector(gravity);
    assert(approxEqual(rotatedGravity.x, gravity.x, 1e-12));
    assert(approxEqual(rotatedGravity.y, gravity.y, 1e-12));
    assert(approxEqual(rotatedGravity.z, gravity.z, 1e-12));

    // Verify slerp midpoint sanity.
    const gf::Quat mid = gf::Quat::slerp(gf::Quat::identity(), yaw, 0.5);
    const gf::Quat expectedMid = gf::Quat::fromAxisAngle(gravity, -1.23 * 0.5);
    assert(approxQuat(mid, expectedMid, 1e-12));
}

void testSidecarStructSizes() {
    assert(sizeof(gf::GFHeader) == 69);
    assert(sizeof(gf::GFImuSample) == 44);
    assert(sizeof(gf::GFFrameSample) == 20);
}

} // namespace

int main() {
    testMultiplicationAndInverse();
    testGyroIntegration90DegZ();
    testSwingTwistDecomposition();
    testSidecarStructSizes();
    std::printf("All CoreCPP math and sidecar tests passed.\n");
    return 0;
}
