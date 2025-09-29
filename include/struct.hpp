#pragma once
#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// 相机参数结构体
struct CameraIntrinsics {
    std::string cam_model;
    int cam_width;
    int cam_height;
    double cam_fx;
    double cam_fy;
    double cam_cx;
    double cam_cy;
    double cam_d0;
    double cam_d1;
    double cam_d2;
    double cam_d3;
};

struct InputDataInfo {
    std::string base_path;
    std::string bag_file;
    std::string img_file;
    std::string pcd_dir;
    std::vector<std::string> pcd_txt_files;
};

struct Parameters {
    CameraIntrinsics camera_intrinsics;
    double marker_size = 0.2;
    double delta_width_qr_center = 0.55;
    double delta_height_qr_center = 0.35;
    double delta_width_circles = 0.5;
    double delta_height_circles = 0.4;
    uint32_t min_detected_markers = 3;
    double circle_radius = 0.12;
    std::vector<double> crop_min_xyz = {1.5, -1.5, -0.8};
    std::vector<double> crop_max_xyz = {3.0, 2.0, 2.0};
    double voxel_downsample_size = 0.005;
    double plane_dist_threshold = 0.01;
    // 标定板平面点云法向量估计搜索半径
    double target_normal_radius = 0.03;
    // 标定板平面点云边界检测搜索半径
    double target_boundary_radius = 0.03;
    // 标定板平面点云边界检测角度阈值
    double target_boundary_angle_thres = M_PI / 4;
};

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudPtr;

// define macro to new a PointCloudPtr
#define MAKE_POINTCLOUD() std::make_shared<PointCloud>()
