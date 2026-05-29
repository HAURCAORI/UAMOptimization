#include "visualization/ViewerCamera.hpp"

#include <algorithm>
#include <cmath>

namespace hexaarch::visualization {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = 0.5 * kPi;

Eigen::Matrix4f castMatrix(const Eigen::Matrix4d& matrix) {
    return matrix.cast<float>();
}

}  // namespace

void ViewerCamera::setViewport(const float width, const float height) {
    viewport_width_ = std::max(width, 1.0f);
    viewport_height_ = std::max(height, 1.0f);
}

void ViewerCamera::setPerspective(const float vertical_fov_radians, const float near_plane, const float far_plane) {
    vertical_fov_radians_ = vertical_fov_radians;
    near_plane_ = near_plane;
    far_plane_ = far_plane;
}

void ViewerCamera::reset(const Eigen::Vector3d& target, const double distance) {
    target_ = target;
    distance_ = std::max(distance, 0.1);
    yaw_radians_ = 0.75;
    pitch_radians_ = -0.45;
}

void ViewerCamera::snapTo(const double yaw_radians, const double pitch_radians) {
    yaw_radians_   = yaw_radians;
    pitch_radians_ = std::clamp(pitch_radians, -kHalfPi + 1e-3, kHalfPi - 1e-3);
}

void ViewerCamera::orbit(const double delta_yaw_radians, const double delta_pitch_radians) {
    yaw_radians_ += delta_yaw_radians;
    pitch_radians_ = std::clamp(pitch_radians_ + delta_pitch_radians, -kHalfPi + 1e-3, kHalfPi - 1e-3);
}

void ViewerCamera::pan(const Eigen::Vector3d& delta_world) {
    target_ += delta_world;
}

void ViewerCamera::zoom(const double delta_distance) {
    distance_ = std::max(0.1, distance_ + delta_distance);
}

Eigen::Matrix4f ViewerCamera::viewMatrix() const {
    const Eigen::Vector3d eye = position();
    const Eigen::Vector3d up_vector = up();

    const Eigen::Vector3d f = (target_ - eye).normalized();
    const Eigen::Vector3d s = f.cross(up_vector).normalized();
    const Eigen::Vector3d u = s.cross(f);

    Eigen::Matrix4d view = Eigen::Matrix4d::Identity();
    view.block<1, 3>(0, 0) = s.transpose();
    view.block<1, 3>(1, 0) = u.transpose();
    view.block<1, 3>(2, 0) = (-f).transpose();
    view(0, 3) = -s.dot(eye);
    view(1, 3) = -u.dot(eye);
    view(2, 3) = f.dot(eye);
    return castMatrix(view);
}

Eigen::Matrix4f ViewerCamera::projectionMatrix() const {
    const float aspect = viewport_width_ / viewport_height_;
    const float tan_half_fov = std::tan(0.5f * vertical_fov_radians_);

    Eigen::Matrix4f projection = Eigen::Matrix4f::Zero();
    projection(0, 0) = 1.0f / (aspect * tan_half_fov);
    projection(1, 1) = -1.0f / tan_half_fov;
    projection(2, 2) = far_plane_ / (near_plane_ - far_plane_);
    projection(2, 3) = (far_plane_ * near_plane_) / (near_plane_ - far_plane_);
    projection(3, 2) = -1.0f;
    return projection;
}

Eigen::Vector3d ViewerCamera::position() const {
    return target_ - forward() * distance_;
}

Eigen::Vector3d ViewerCamera::target() const {
    return target_;
}

double ViewerCamera::distance() const {
    return distance_;
}

Eigen::Vector3d ViewerCamera::forward() const {
    const double cos_pitch = std::cos(pitch_radians_);
    return Eigen::Vector3d(
        std::cos(yaw_radians_) * cos_pitch,
        std::sin(yaw_radians_) * cos_pitch,
        std::sin(pitch_radians_)).normalized();
}

Eigen::Vector3d ViewerCamera::right() const {
    return forward().cross(Eigen::Vector3d::UnitZ()).normalized();
}

Eigen::Vector3d ViewerCamera::up() const {
    return right().cross(forward()).normalized();
}

}  // namespace hexaarch::visualization
