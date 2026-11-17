#include "Quaternion.hpp"

#include <algorithm>
#include <cmath>

namespace gf {

namespace {

constexpr double kEps = 1e-12;

Quat quatDerivative(const Quat& q, const Vec3& omega) {
    // dq/dt = 0.5 * q * (0, omega.x, omega.y, omega.z)
    const Quat pureOmega(0.0, omega.x, omega.y, omega.z);
    const Quat p = q * pureOmega;
    return Quat(0.5 * p.w, 0.5 * p.x, 0.5 * p.y, 0.5 * p.z);
}

Quat addScaled(const Quat& q, const Quat& dq, double scale) {
    return Quat(
        q.w + dq.w * scale,
        q.x + dq.x * scale,
        q.y + dq.y * scale,
        q.z + dq.z * scale
    );
}

} // namespace

Vec3 Vec3::operator+(const Vec3& rhs) const {
    return Vec3(x + rhs.x, y + rhs.y, z + rhs.z);
}

Vec3 Vec3::operator-(const Vec3& rhs) const {
    return Vec3(x - rhs.x, y - rhs.y, z - rhs.z);
}

Vec3 Vec3::operator*(double s) const {
    return Vec3(x * s, y * s, z * s);
}

double Vec3::dot(const Vec3& rhs) const {
    return x * rhs.x + y * rhs.y + z * rhs.z;
}

Vec3 Vec3::cross(const Vec3& rhs) const {
    return Vec3(
        y * rhs.z - z * rhs.y,
        z * rhs.x - x * rhs.z,
        x * rhs.y - y * rhs.x
    );
}

double Vec3::normSquared() const {
    return x * x + y * y + z * z;
}

double Vec3::norm() const {
    return std::sqrt(normSquared());
}

Vec3 Vec3::normalized() const {
    const double n = norm();
    if (n <= kEps) {
        return Vec3(0.0, 0.0, 0.0);
    }
    const double inv = 1.0 / n;
    return Vec3(x * inv, y * inv, z * inv);
}

Quat Quat::identity() {
    return Quat(1.0, 0.0, 0.0, 0.0);
}

Quat Quat::fromAxisAngle(const Vec3& axis, double angleRad) {
    const Vec3 unitAxis = axis.normalized();
    if (unitAxis.normSquared() <= kEps) {
        return identity();
    }
    const double half = 0.5 * angleRad;
    const double s = std::sin(half);
    return Quat(std::cos(half), unitAxis.x * s, unitAxis.y * s, unitAxis.z * s).normalized();
}

Quat Quat::operator*(const Quat& rhs) const {
    return Quat(
        w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
        w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
        w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
        w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w
    );
}

Quat Quat::conjugate() const {
    return Quat(w, -x, -y, -z);
}

double Quat::normSquared() const {
    return w * w + x * x + y * y + z * z;
}

double Quat::norm() const {
    return std::sqrt(normSquared());
}

Quat Quat::normalized() const {
    const double n = norm();
    if (n <= kEps) {
        return identity();
    }
    const double inv = 1.0 / n;
    return Quat(w * inv, x * inv, y * inv, z * inv);
}

Vec3 Quat::rotateVector(const Vec3& v) const {
    const Quat q = normalized();
    const Vec3 u(q.x, q.y, q.z);
    const Vec3 t = u.cross(v) * 2.0;
    return v + (t * q.w) + u.cross(t);
}

Quat Quat::slerp(const Quat& a, const Quat& b, double t) {
    Quat qa = a.normalized();
    Quat qb = b.normalized();

    double dot = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z;
    if (dot < 0.0) {
        qb = Quat(-qb.w, -qb.x, -qb.y, -qb.z);
        dot = -dot;
    }

    dot = std::min(1.0, std::max(-1.0, dot));
    if (dot > 0.9995) {
        return Quat(
            qa.w + t * (qb.w - qa.w),
            qa.x + t * (qb.x - qa.x),
            qa.y + t * (qb.y - qa.y),
            qa.z + t * (qb.z - qa.z)
        ).normalized();
    }

    const double theta0 = std::acos(dot);
    const double theta = theta0 * t;
    const double sinTheta = std::sin(theta);
    const double sinTheta0 = std::sin(theta0);

    const double s0 = std::cos(theta) - dot * sinTheta / sinTheta0;
    const double s1 = sinTheta / sinTheta0;

    return Quat(
        s0 * qa.w + s1 * qb.w,
        s0 * qa.x + s1 * qb.x,
        s0 * qa.y + s1 * qb.y,
        s0 * qa.z + s1 * qb.z
    ).normalized();
}

Quat Quat::integrateGyroStep(const Quat& q, const Vec3& omegaRadPerSec, double dt) {
    const Quat k1 = quatDerivative(q, omegaRadPerSec);
    const Quat k2 = quatDerivative(addScaled(q, k1, 0.5 * dt), omegaRadPerSec);
    const Quat k3 = quatDerivative(addScaled(q, k2, 0.5 * dt), omegaRadPerSec);
    const Quat k4 = quatDerivative(addScaled(q, k3, dt), omegaRadPerSec);

    const Quat next(
        q.w + (dt / 6.0) * (k1.w + 2.0 * k2.w + 2.0 * k3.w + k4.w),
        q.x + (dt / 6.0) * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x),
        q.y + (dt / 6.0) * (k1.y + 2.0 * k2.y + 2.0 * k3.y + k4.y),
        q.z + (dt / 6.0) * (k1.z + 2.0 * k2.z + 2.0 * k3.z + k4.z)
    );
    return next.normalized();
}

void Quat::decomposeSwingTwist(const Quat& q, const Vec3& gravityAxis, Quat& outSwing, Quat& outTwist) {
    const Quat qNorm = q.normalized();
    const Vec3 axis = gravityAxis.normalized();
    if (axis.normSquared() <= kEps) {
        outSwing = qNorm;
        outTwist = identity();
        return;
    }

    const Vec3 imag(qNorm.x, qNorm.y, qNorm.z);
    const Vec3 proj = axis * imag.dot(axis);
    const Quat rawTwist(qNorm.w, proj.x, proj.y, proj.z);

    if (rawTwist.normSquared() <= kEps * kEps) {
        // Singularity: 180-degree rotation around an axis orthogonal to gravityAxis.
        outTwist = identity();
        outSwing = qNorm;
        return;
    }

    outTwist = rawTwist.normalized();
    outSwing = (qNorm * outTwist.conjugate()).normalized();
}

} // namespace gf
