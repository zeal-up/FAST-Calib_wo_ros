#pragma once

#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>

#include "struct.hpp"
#include "io_utils.hpp"

class DataPreprocess
{
public:
    std::vector<PointCloudPtr> cloud_inputs;

    std::vector<cv::Mat> img_inputs;

    DataPreprocess(std::vector<InputDataInfo> all_input_data) {
        for (const auto &input_data : all_input_data) {
            cv::Mat img_input;
            PointCloudPtr cloud_input(new PointCloud);
            
            std::string image_path = input_data.img_file;
            img_input = cv::imread(image_path, cv::IMREAD_UNCHANGED);
            if (img_input.empty()) {
                std::cerr << "Error: Could not load image: " << image_path << std::endl;
                continue; // 跳过这个数据对，继续处理下一个
            }

            std::vector<std::string> pcd_txt_files = input_data.pcd_txt_files;

            for (const auto &pcd_txt_file : pcd_txt_files) {
                PointCloudPtr temp_cloud = loadPointCloudFromTXT(pcd_txt_file);
                if (temp_cloud->empty()) {
                    std::cerr << "Warning: Loaded empty point cloud from: " << pcd_txt_file << std::endl;
                    continue; // 跳过这个点云文件，继续处理下一个
                }
                *cloud_input += *temp_cloud; // 合并点云
            }
            
            if (cloud_input->empty()) {
                std::cerr << "Error: No valid point cloud data loaded for image: " << image_path << std::endl;
                continue; // 跳过这个数据对，继续处理下一个
            }

            cloud_inputs.push_back(cloud_input);
            img_inputs.push_back(img_input);
            std::cout << "Loaded image: " << image_path << " with size " 
                      << img_input.cols << "x" << img_input.rows 
                      << " and point cloud with " << cloud_input->size() << " points." << std::endl;
        }
    }

    /**
     * @brief Get the Point Cloud object. if index is out of range, return nullptr
     * if voxel_downsample_size>0, downsample the point cloud
     * 
     * @param index 
     * @param voxel_downsample_size 
     * @return PointCloudPtr 
     */
    PointCloudPtr getPointCloud(int index, double voxel_downsample_size) {
        if (index >= 0 && index < cloud_inputs.size()) {
            PointCloudPtr cloud = cloud_inputs[index];
            if (voxel_downsample_size > 0) {
                pcl::VoxelGrid<PointT> voxel_grid;
                voxel_grid.setLeafSize(voxel_downsample_size, voxel_downsample_size, voxel_downsample_size);
                voxel_grid.setInputCloud(cloud);
                PointCloudPtr downsample_cloud = MAKE_POINTCLOUD();
                voxel_grid.filter(*downsample_cloud);
                return downsample_cloud;
            } else {
                return cloud;
            }
        }
        return nullptr;
    }
};

typedef std::shared_ptr<DataPreprocess> DataPreprocessPtr;