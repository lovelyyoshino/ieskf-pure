#pragma once 
#include "module_base.h"
#include "type/pointcloud.h"
#include "type/base_type.h"
#include "pcl/common/transforms.h"
#include "math/math.h"
namespace IESKFSlam
{
    class RectMapManager :private ModuleBase
    {
    private:
        PCLPointCloudPtr local_map_ptr;
        KDTreePtr kdtree_ptr;
    public:
        RectMapManager(const std::string &config_file_path,const std::string & prefix );
        ~RectMapManager();
        void reset();
        void addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond &att_q,const Eigen::Vector3d &pos_t);
        PCLPointCloudConstPtr getLocalMap(); 
        KDTreeConstPtr readKDtree();
    };
    

} // namespace IESKFSlam
