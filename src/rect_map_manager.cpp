#include "rect_map_manager.h"
namespace IESKFSlam
{
    RectMapManager::RectMapManager(const std::string &config_file_path,const std::string & prefix ):ModuleBase(config_file_path,prefix,"RectMapManager")
    {
        local_map_ptr = pcl::make_shared<PCLPointCloud>();
        kdtree_ptr = pcl::make_shared<KDTree>();
    }
    
    RectMapManager::~RectMapManager()
    {
    }
    void RectMapManager::addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond &att_q,const Eigen::Vector3d &pos_t){
        PCLPointCloud scan;
        pcl::transformPointCloud(*curr_scan,scan,compositeTransform(att_q,pos_t).cast<float>());
        VoxelFilter filter;
        filter.setLeafSize(0.3,0.3,0.3);

        *local_map_ptr+=scan;
        filter.setInputCloud(local_map_ptr);
        filter.filter(*local_map_ptr);
        kdtree_ptr->setInputCloud(local_map_ptr);
        
    }
    void RectMapManager::reset(){
        local_map_ptr->clear();
    }
    PCLPointCloudConstPtr RectMapManager::getLocalMap(){
        return local_map_ptr;
    }
    KDTreeConstPtr RectMapManager::readKDtree(){
        return kdtree_ptr;
    }
} // namespace IESKFSlam
