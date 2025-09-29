#include <gflags/gflags.h>
#include <glog/logging.h>
#include <iostream>
#include <string>
#include <filesystem>

#include "io_utils.hpp"
#include "struct.hpp"
#include "print_utils.hpp"
#include "data_preprocess.hpp"
#include "lidar_detect.hpp"


// 定义命令行参数
DEFINE_string(cam_intrinsic_file, "", "Path to camera intrinsic parameters file");
DEFINE_string(data_dir, "", "Path to input data directory");
DEFINE_string(output_dir, "", "Path to output directory");

// 可选：添加参数验证函数
static bool ValidateDataDir(const char* flagname, const std::string& value) {
    if (!value.empty()) {
        return true;
    }
    std::cout << "Invalid value for --" << flagname << ": cannot be empty" << std::endl;
    return false;
}
DEFINE_validator(data_dir, &ValidateDataDir);

int main(int argc, char** argv) {
    // 初始化 gflags
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    // set glog level to INFO
    FLAGS_minloglevel = google::GLOG_INFO;
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_logbufsecs = 0;
    LOG(INFO) << "Begin program";
    
    // 打印欢迎信息
    std::cout << "=== Program Configuration ===" << std::endl;
    std::cout << "Camera intrinsic file: " << FLAGS_cam_intrinsic_file << std::endl;
    std::cout << "Data directory: " << FLAGS_data_dir << std::endl;
    std::cout << "Output directory: " << FLAGS_output_dir << std::endl;
    std::cout << "=============================" << std::endl;

    std::string lidar_detect_debug_dir = FLAGS_output_dir + "/lidar_detect_debug";
    std::filesystem::create_directories(lidar_detect_debug_dir);
    
    // 检查必要参数
    if (FLAGS_cam_intrinsic_file.empty()) {
        std::cout << "Warning: Camera intrinsic file not specified" << std::endl;
    }
    
    if (FLAGS_output_dir.empty()) {
        std::cout << "Warning: Output directory not specified" << std::endl;
    }
    
    CameraIntrinsics cam_params;
    cam_params = readCameraIntrinsics(FLAGS_cam_intrinsic_file);
    printCameraIntrinsics(cam_params);

    Parameters params;

    std::vector<InputDataInfo> all_input_data = listDataPairs(FLAGS_data_dir);
    std::cout << "Found " << all_input_data.size() << " .bag and .png file pairs in " << FLAGS_data_dir << std::endl;

    // 将所有帧的点云拼接到一起
    DataPreprocessPtr dataPreprocessPtr;
    dataPreprocessPtr = std::make_shared<DataPreprocess>(all_input_data);

    // 点云的圆心检测
    LidarDetectPtr lidarDetectPtr;
    lidarDetectPtr = std::make_shared<LidarDetect>(params);

    for (int data_idx = 0; data_idx < 1; data_idx++) {
        LOG(INFO) << "Processing data pair " << data_idx << " / " << all_input_data.size();
        PointCloudPtr center_cloud = MAKE_POINTCLOUD();
        center_cloud->reserve(4);
        try
        {
            lidarDetectPtr->detect_lidar(dataPreprocessPtr->cloud_inputs[data_idx], center_cloud);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        

        std::string edge_cloud_file = lidar_detect_debug_dir + "/edge_cloud_" + std::to_string(data_idx) + ".pcd";
        std::string center_cloud_file = lidar_detect_debug_dir + "/center_cloud_" + std::to_string(data_idx) + ".pcd";
        std::string plane_cloud_file = lidar_detect_debug_dir + "/plane_cloud_" + std::to_string(data_idx) + ".pcd";
        pcl::io::savePCDFileASCII(edge_cloud_file, *lidarDetectPtr->getEdgeCloud());
        pcl::io::savePCDFileASCII(center_cloud_file, *center_cloud);
        pcl::io::savePCDFileASCII(plane_cloud_file, *lidarDetectPtr->getPlaneCloud());

        std::vector<PointCloudPtr> cluster_clouds = lidarDetectPtr->getClusterClouds();
        for (size_t i = 0; i < cluster_clouds.size(); i++) {
            std::string cluster_cloud_file = lidar_detect_debug_dir + "/cluster_cloud_" + std::to_string(data_idx) + "_" + std::to_string(i) + ".pcd";
            pcl::io::savePCDFileASCII(cluster_cloud_file, *cluster_clouds[i]);
        }

        LOG(INFO) << "Processed data pair " << data_idx << " / " << all_input_data.size() << " successfully" << std::endl;
    }
    
    std::cout << "Processing completed!" << std::endl;
    
    return 0;
}