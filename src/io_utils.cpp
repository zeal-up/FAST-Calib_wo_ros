#include "io_utils.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>

// 列出指定目录下的所有 .bag 和 .png 文件对
std::vector<InputDataInfo> listDataPairs(const std::string& data_dir) {
    std::vector<InputDataInfo> all_input_datas; // 存储所有找到的文件对
    try {
        namespace fs = std::filesystem;
        
        // 检查目录是否存在
        if (!fs::exists(data_dir) || !fs::is_directory(data_dir)) {
            std::cerr << "Error: Data directory does not exist or is not a directory: " << data_dir << std::endl;
            return all_input_datas;
        }
        
        // 存储所有找到的文件
        std::vector<std::string> bag_files;
        std::vector<std::string> png_files;
        
        // 遍历目录
        for (const auto& entry : fs::directory_iterator(data_dir)) {
            if (entry.is_regular_file()) {
                std::string filename = entry.path().filename().string();
                std::string extension = entry.path().extension().string();
                
                if (extension == ".bag") {
                    bag_files.push_back(entry.path().string());
                } else if (extension == ".png") {
                    png_files.push_back(entry.path().string());
                }
            }
        }
        
        // 按文件名排序以便匹配
        std::sort(bag_files.begin(), bag_files.end());
        std::sort(png_files.begin(), png_files.end());
        
        // 也可以反向检查，确保没有孤立的PNG文件
        for (const auto& img_file : png_files) {
            fs::path png_path(img_file);
            std::string base_name = png_path.stem().string();
            
            std::string expected_bag = png_path.parent_path() / (base_name + ".bag");
            
            if (!fs::exists(expected_bag)) {
                std::cout << "Warning: No matching BAG file found for " << img_file << std::endl;
            }

            std::string pcd_dir = png_path.parent_path() / (base_name + "_pcd");
            if (!fs::exists(pcd_dir)) {
                std::cerr << "Warning: No matching PCD directory found for " << img_file << std::endl;
                throw std::runtime_error("Missing PCD directory for " + img_file);
            }
            std::vector<std::string> pcd_txt_files;
            for (const auto& pcd_entry : fs::directory_iterator(pcd_dir)) {
                if (pcd_entry.is_regular_file() && pcd_entry.path().extension() == ".txt") {
                    pcd_txt_files.push_back(pcd_entry.path().string());
                }
            }
            if (pcd_txt_files.empty()) {
                std::cerr << "Warning: No .txt files found in PCD directory: "
                            << pcd_dir << std::endl;
                throw std::runtime_error("No .txt files in " + pcd_dir);
            }
            std::sort(pcd_txt_files.begin(), pcd_txt_files.end());
            InputDataInfo data_info;
            data_info.base_path = png_path.parent_path().string();
            data_info.bag_file = expected_bag;
            data_info.img_file = img_file;
            data_info.pcd_dir = pcd_dir;
            data_info.pcd_txt_files = pcd_txt_files;
            all_input_datas.push_back(data_info);
        }
        
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Filesystem error: " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    
    return all_input_datas;
}


// 读取相机内参的函数
CameraIntrinsics readCameraIntrinsics(const std::string& filepath) {
    CameraIntrinsics params;
    
    try {
        // 加载YAML文件:cite[2]:cite[4]
        YAML::Node config = YAML::LoadFile(filepath);
        
        // 读取各个参数:cite[6]
        params.cam_model = config["cam_model"].as<std::string>();
        params.cam_width = config["cam_width"].as<int>();
        params.cam_height = config["cam_height"].as<int>();
        params.cam_fx = config["cam_fx"].as<double>();
        params.cam_fy = config["cam_fy"].as<double>();
        params.cam_cx = config["cam_cx"].as<double>();
        params.cam_cy = config["cam_cy"].as<double>();
        params.cam_d0 = config["cam_d0"].as<double>();
        params.cam_d1 = config["cam_d1"].as<double>();
        params.cam_d2 = config["cam_d2"].as<double>();
        params.cam_d3 = config["cam_d3"].as<double>();
        
    } catch (const YAML::Exception& e) {
        // 异常处理:cite[4]
        std::cerr << "Error reading YAML file: " << e.what() << std::endl;
        throw; // 重新抛出异常
    }
    
    return params;
}

// 每一行的前四个浮点数是 x, y, z, intensity
PointCloudPtr loadPointCloudFromTXT(const std::string& filepath) {
    PointCloudPtr cloud(new PointCloud);
    std::fstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filepath << std::endl;
        return cloud; // 返回空点云
    }
    std::string line;

    // std::getline(file, line);
    // std::cout << "First line for testing: " << line << std::endl;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double x, y, z, intensity;
        if (!(iss >> x >> y >> z >> intensity)) {
            break;
        }
        cloud->push_back(pcl::PointXYZI(x, y, z, intensity));
    }
    file.close();
    return cloud;
}