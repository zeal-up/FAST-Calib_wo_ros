#pragma once
#include <vector>
#include <utility>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "struct.hpp"



// 列出指定目录下的所有 .bag 和 .png 文件对
std::vector<InputDataInfo> listDataPairs(const std::string& data_dir);

CameraIntrinsics readCameraIntrinsics(const std::string& filepath);

// 每一行的前四个浮点数是 x, y, z, intensity
PointCloudPtr loadPointCloudFromTXT(const std::string& filepath);

/**
 * @brief 
 * 
 * @param transformation 
 * @param dirPath 
 * @return std::string save file path
 */
std::string saveCalibrationResults(const Eigen::Matrix4f& transformation, const std::string& dirPath);