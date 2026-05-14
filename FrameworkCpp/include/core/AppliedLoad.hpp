#pragma once

#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"

namespace hexaarch::core {

struct AppliedLoad {
    Eigen::Vector3d force_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d torque_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d point_world = Eigen::Vector3d::Zero();
    std::string label;
};

class ILoadReceiver {
public:
    virtual ~ILoadReceiver() = default;
    virtual void clearLoads() = 0;
    virtual void addLoad(const AppliedLoad& load) = 0;
    [[nodiscard]] virtual const std::vector<AppliedLoad>& loads() const = 0;
};

}  // namespace hexaarch::core
