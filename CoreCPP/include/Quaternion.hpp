#pragma once

namespace gf {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& rhs) const;
    Vec3 operator-(const Vec3& rhs) const;
    Vec3 operator*(double s) const;

    double dot(const Vec3& rhs) const;
    Vec3 cross(const Vec3& rhs) const;
    double normSquared() const;
    double norm() const;
    Vec3 normalized() const;
};

struct Quat {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Quat() = default;
    Quat(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}

    static Quat identity();
    static Quat fromAxisAngle(const Vec3& axis, double angleRad);

    Quat operator*(const Quat& rhs) const;
    Quat conjugate() const;
    double normSquared() const;
    double norm() const;
    Quat normalized() const;

    Vec3 rotateVector(const Vec3& v) const;

    static Quat slerp(const Quat& a, const Quat& b, double t);

    /// Integrates angular velocity (rad/s) over dt seconds using 4th-order Runge-Kutta (RK4).
    static Quat integrateGyroStep(const Quat& q, const Vec3& omegaRadPerSec, double dt);

    /// Decomposes q into q = outSwing * outTwist where outTwist is a pure rotation around gravityAxis.
    static void decomposeSwingTwist(const Quat& q, const Vec3& gravityAxis, Quat& outSwing, Quat& outTwist);
};

} // namespace gf
