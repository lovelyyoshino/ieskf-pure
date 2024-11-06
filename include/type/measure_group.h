#pragma once 
#include "type/imu.h"
#include "type/pointcloud.h"
#include <deque>
namespace IESKFSlam
{
    struct MeasureGroup{
        double lidar_begin_time;
        std::deque<IMU> imus;
        PointCloud cloud;
        double lidar_end_time;
    };
} // namespace IESKFSlam
