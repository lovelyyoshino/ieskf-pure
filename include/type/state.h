#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace IESKFSlam
{
    struct State18
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Quaterniond rotation;
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d bg;
        Eigen::Vector3d ba;
        Eigen::Vector3d gravity;
        State18(){
            rotation = Eigen::Quaterniond::Identity();
            position = Eigen::Vector3d::Zero();
            velocity = Eigen::Vector3d::Zero();
            bg = Eigen::Vector3d::Zero();
            ba = Eigen::Vector3d::Zero();
            gravity = Eigen::Vector3d::Zero();
        }
    };
} // namespace IESKFSlam
