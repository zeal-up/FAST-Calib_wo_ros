#pragma once

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "struct.hpp"

/**
 * @brief Function to project a point cloud to an image, given a camera matrix and distortion coefficients
 * colors the points with the corresponding pixel color in the image
 * 
 * @param cloud 
 * @param transformation 
 * @param cameraMatrix 
 * @param distCoeffs 
 * @param image 
 * @param colored_cloud 
 */
void colorPointCloudWithImage(
    const PointCloudPtr& cloud, const Eigen::Matrix4f& transformation,
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
    const cv::Mat& image,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud);

/**
 * @brief 计算点到点的均方根误差
 * 
 * @param cloud1 
 * @param cloud2 
 * @return double 
 */
double computeRMSE(const PointCloudPtr& cloud1, 
                   const PointCloudPtr& cloud2);