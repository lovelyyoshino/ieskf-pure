#include "type/imu.h"
#include "type/base_type.h"
#include "type/measure_group.h"
#include "ieskf.h"
#include "rect_map_manager.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>	//	pcl::transformPointCloud 用到这个头文件
#include "chrono"

std::shared_ptr<IESKFSlam::IESKF> ieskf_ptr;
std::shared_ptr<IESKFSlam::RectMapManager> map_ptr;
IESKFSlam::LIOZHModel::Ptr lio_zh_model_ptr;
IESKFSlam::PCLPointCloudPtr filter_point_cloud_ptr;
IESKFSlam::PCLPointCloudPtr match_point_cloud_ptr;
pcl::VoxelGrid<IESKFSlam::Point> voxel_filter;

namespace IESKFSlam {
    void syncMeasureGroup(MeasureGroup&mg){
        mg.imus.clear();
        mg.cloud.cloud_ptr->clear();
        mg.lidar_begin_time = 0;
        mg.lidar_end_time = 0;
        // . 1. 生成点云
        mg.cloud.cloud_ptr = match_point_cloud_ptr;
        auto x = ieskf_ptr->getX();
        x.bg = Eigen::Vector3d::Zero();
        ieskf_ptr->setX(x); //ieskf设置状态量，这里是为了保证ieskf的状态量和IMU的状态量一致，第二步
    }

    bool track() {
        MeasureGroup mg;
        syncMeasureGroup(mg);
//        ieskf_ptr->propagate(mg,ieskf_ptr);//ieskf的预测，第三步,这一步需要imu数据，但是现在不需要
        voxel_filter.setInputCloud(mg.cloud.cloud_ptr);
        voxel_filter.filter(*match_point_cloud_ptr);
        ieskf_ptr->update();//ieskf的更新，第四步
        auto x = ieskf_ptr->getX();//ieskf的状态量.这对应的是ieskf的状态量，第五步
        std::cout<<x.position.transpose()<<std::endl;
//        std::cout<<x.rotation.coeffs().transpose()<<std::endl;
        //转为rpy
        std::cout<<x.rotation.toRotationMatrix().eulerAngles(2,1,0).transpose()<<std::endl;
        map_ptr->addScan(match_point_cloud_ptr,x.rotation,x.position);//添加点云到地图，第六步
        //pcl翻转
        Eigen::Affine3f transform_ieskf = Eigen::Affine3f::Identity();
        transform_ieskf.rotate(x.rotation.cast<float>());
        transform_ieskf.translation() << x.position.cast<float>();
        pcl::transformPointCloud(*match_point_cloud_ptr,*match_point_cloud_ptr,transform_ieskf);
        pcl::io::savePCDFileASCII ("after_cloud.pcd", *match_point_cloud_ptr);
        std::cerr << "Saved " << match_point_cloud_ptr->points.size () << " data points to cloud.pcd." << std::endl;
        return true;
    }

}

int main(int argc, char *argv[]) {
    // Load the point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr match_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string path = "/home/xxx/ieskf/src/1123.817035_1160.497190.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *map_cloud) == -1 ||
        pcl::io::loadPCDFile<pcl::PointXYZ>(path, *match_cloud) == -1) {
        PCL_ERROR("Couldn't read file.\n");
        return -1;
    }
    std::string config_file_path = "/home/xxx/ieskf/src/avia.yaml";
    ieskf_ptr = std::make_shared<IESKFSlam::IESKF>(config_file_path,"ieskf");
    map_ptr  = std::make_shared<IESKFSlam::RectMapManager>(config_file_path,"map");
    map_ptr->reset();
    lio_zh_model_ptr = std::make_shared<IESKFSlam::LIOZHModel>();
    ieskf_ptr->calc_zh_ptr =lio_zh_model_ptr;
    filter_point_cloud_ptr = pcl::make_shared<IESKFSlam::PCLPointCloud>();
    float leaf_size = 0.3;
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    for (auto &point:map_cloud->points) {
        IESKFSlam::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        p.offset_time = 0;
        p.ring = 0;
        filter_point_cloud_ptr->push_back(p);
    }

    match_point_cloud_ptr = pcl::make_shared<IESKFSlam::PCLPointCloud>();
    for (auto &point:match_cloud->points) {
        IESKFSlam::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        p.offset_time = 0;
        p.ring = 0;
        match_point_cloud_ptr->push_back(p);
    }
    Eigen::Affine3f transform_test = Eigen::Affine3f::Identity();
    transform_test.rotate(Eigen::AngleAxisf(0.05,Eigen::Vector3f::UnitZ()));
    transform_test.translate(Eigen::Vector3f(-0.3,0.5,0.0));
    pcl::transformPointCloud(*filter_point_cloud_ptr,*filter_point_cloud_ptr,transform_test);

    map_ptr->addScan(filter_point_cloud_ptr,Eigen::Quaterniond::Identity(),Eigen::Vector3d::Zero());//地图加载进去

    pcl::io::savePCDFileASCII ("cloud.pcd", *filter_point_cloud_ptr);
    std::cerr << "Saved " << filter_point_cloud_ptr->points.size () << " data points to cloud.pcd." << std::endl;

    lio_zh_model_ptr->prepare(map_ptr->readKDtree(),match_point_cloud_ptr,map_ptr->getLocalMap());//加载地图，设置kd树，第一步
    //当前时间
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    IESKFSlam::track();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()/1000000.0 << " [s]" << std::endl;
    return 0;
}