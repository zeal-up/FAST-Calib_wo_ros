#include "print_utils.hpp"


// 打印相机参数的辅助函数
void printCameraIntrinsics(const CameraIntrinsics& params) {
    std::cout << "Camera Model: " << params.cam_model << std::endl;
    std::cout << "Image Width: " << params.cam_width << std::endl;
    std::cout << "Image Height: " << params.cam_height << std::endl;
    std::cout << "Focal Length X: " << params.fx << std::endl;
    std::cout << "Focal Length Y: " << params.fy << std::endl;
    std::cout << "Principal Point X: " << params.cx << std::endl;
    std::cout << "Principal Point Y: " << params.cy << std::endl;
    std::cout << "Distortion Coefficient k1: " << params.k1 << std::endl;
    std::cout << "Distortion Coefficient k2: " << params.k2 << std::endl;
    std::cout << "Distortion Coefficient p1: " << params.p1 << std::endl;
    std::cout << "Distortion Coefficient p2: " << params.p2 << std::endl;
}