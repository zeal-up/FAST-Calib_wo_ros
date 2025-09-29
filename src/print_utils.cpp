#include "print_utils.hpp"


// 打印相机参数的辅助函数
void printCameraIntrinsics(const CameraIntrinsics& params) {
    std::cout << "Camera Model: " << params.cam_model << std::endl;
    std::cout << "Image Width: " << params.cam_width << std::endl;
    std::cout << "Image Height: " << params.cam_height << std::endl;
    std::cout << "Focal Length X: " << params.cam_fx << std::endl;
    std::cout << "Focal Length Y: " << params.cam_fy << std::endl;
    std::cout << "Principal Point X: " << params.cam_cx << std::endl;
    std::cout << "Principal Point Y: " << params.cam_cy << std::endl;
    std::cout << "Distortion Coefficient d0: " << params.cam_d0 << std::endl;
    std::cout << "Distortion Coefficient d1: " << params.cam_d1 << std::endl;
    std::cout << "Distortion Coefficient d2: " << params.cam_d2 << std::endl;
    std::cout << "Distortion Coefficient d3: " << params.cam_d3 << std::endl;
}