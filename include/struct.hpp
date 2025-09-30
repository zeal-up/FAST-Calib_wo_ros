#pragma once
#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>


#define DEBUG 1
#define GEOMETRY_TOLERANCE 0.06

// 相机参数结构体
struct CameraIntrinsics {
    std::string cam_model;
    int cam_width;
    int cam_height;
    double fx;
    double fy;
    double cx;
    double cy;
    double k1;
    double k2;
    double p1;
    double p2;

    cv::Mat getCameraMatrix() {
        return (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    }

    cv::Mat getDistCoeffs() {
        return (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, 0);
    }
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
    // Aruco中心距离标靶中心的width距离
    double delta_width_qr_center = 0.55;
    // Aruco中心距离标靶中心的height距离
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
