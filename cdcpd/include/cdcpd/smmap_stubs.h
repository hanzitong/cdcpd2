#pragma once

#include <Eigen/Dense>
#include <vector>

// Minimal stubs for smmap types when smmap packages are not available
namespace smmap {

struct GripperPose {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

struct AllGrippersSinglePose {
    std::vector<GripperPose> poses;
    
    size_t size() const { return poses.size(); }
    const GripperPose& operator[](size_t idx) const { return poses[idx]; }
    GripperPose& operator[](size_t idx) { return poses[idx]; }
    void push_back(const GripperPose& pose) { poses.push_back(pose); }
};

struct GripperDelta {
    Eigen::Vector3d linear;
    Eigen::Vector3d angular;
};

struct AllGrippersSinglePoseDelta {
    std::vector<GripperDelta> deltas;
    
    size_t size() const { return deltas.size(); }
    const GripperDelta& operator[](size_t idx) const { return deltas[idx]; }
    GripperDelta& operator[](size_t idx) { return deltas[idx]; }
    void push_back(const GripperDelta& delta) { deltas.push_back(delta); }
};

// Stub for GripperData
struct GripperData {
    int node_idx_;
    std::vector<Eigen::Vector3d> link_points_;
};

} // namespace smmap
