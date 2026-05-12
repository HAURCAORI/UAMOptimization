#pragma once

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

namespace hexaarch::visualization {

class ViewerCamera {
public:
    ViewerCamera() = default;

    void setViewport(float width, float height);
    void setPerspective(float vertical_fov_radians, float near_plane, float far_plane);
    void reset(const Eigen::Vector3d& target, double distance);
    void orbit(double delta_yaw_radians, double delta_pitch_radians);
    void pan(const Eigen::Vector3d& delta_world);
    void zoom(double delta_distance);

    [[nodiscard]] Eigen::Matrix4f viewMatrix() const;
    [[nodiscard]] Eigen::Matrix4f projectionMatrix() const;
    [[nodiscard]] Eigen::Vector3d position() const;
    [[nodiscard]] Eigen::Vector3d target() const;
    [[nodiscard]] double distance() const;

private:
    [[nodiscard]] Eigen::Vector3d forward() const;
    [[nodiscard]] Eigen::Vector3d right() const;
    [[nodiscard]] Eigen::Vector3d up() const;

    Eigen::Vector3d target_ = Eigen::Vector3d::Zero();
    double distance_ = 10.0;
    double yaw_radians_ = 0.75;
    double pitch_radians_ = -0.45;
    float viewport_width_ = 1280.0f;
    float viewport_height_ = 720.0f;
    float vertical_fov_radians_ = 60.0f * 3.14159265358979323846f / 180.0f;
    float near_plane_ = 0.1f;
    float far_plane_ = 1000.0f;
};

}  // namespace hexaarch::visualization
