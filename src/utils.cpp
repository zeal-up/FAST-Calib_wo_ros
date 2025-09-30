#include "utils.hpp"

#include <glog/logging.h>

void colorPointCloudWithImage(
    const PointCloudPtr& cloud, const Eigen::Matrix4f& transformation,
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
    const cv::Mat& image,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud) {
  colored_cloud->clear();
  colored_cloud->reserve(cloud->size());

  // Undistort the entire image (preprocess outside if possible)
  cv::Mat undistortedImage;
  cv::undistort(image, undistortedImage, cameraMatrix, distCoeffs);

  // Precompute rotation and translation vectors (zero for this case)
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32F);
  cv::Mat zeroDistCoeffs = cv::Mat::zeros(5, 1, CV_32F);

  // Preallocate memory for projection
  std::vector<cv::Point3f> objectPoints(1);
  std::vector<cv::Point2f> imagePoints(1);

  for (const auto& point : *cloud) {
    // Transform the point
    Eigen::Vector4f homogeneous_point(point.x, point.y, point.z, 1.0f);
    Eigen::Vector4f transformed_point = transformation * homogeneous_point;

    // Skip points behind the camera
    if (transformed_point(2) < 0) continue;

    // Project the point to the image plane
    objectPoints[0] = cv::Point3f(transformed_point(0), transformed_point(1),
                                  transformed_point(2));
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, zeroDistCoeffs,
                      imagePoints);

    int u = static_cast<int>(imagePoints[0].x);
    int v = static_cast<int>(imagePoints[0].y);

    // Check if the point is within the image bounds
    if (u >= 0 && u < undistortedImage.cols && v >= 0 &&
        v < undistortedImage.rows) {
      // Get the color from the undistorted image
      cv::Vec3b color = undistortedImage.at<cv::Vec3b>(v, u);

      // Create a colored point and add it to the cloud
      pcl::PointXYZRGB colored_point;
      colored_point.x = transformed_point(0);
      colored_point.y = transformed_point(1);
      colored_point.z = transformed_point(2);
      colored_point.r = color[2];
      colored_point.g = color[1];
      colored_point.b = color[0];
      colored_cloud->push_back(colored_point);
    }
  }
}

double computeRMSE(const PointCloudPtr& cloud1, const PointCloudPtr& cloud2) {
  if (cloud1->size() != cloud2->size()) {
    LOG(ERROR) << "PointCloud sizes do not match, cannot compute RMSE.";
    return -1.0;
  }

  double sum = 0.0;
  for (size_t i = 0; i < cloud1->size(); ++i) {
    double dx = cloud1->points[i].x - cloud2->points[i].x;
    double dy = cloud1->points[i].y - cloud2->points[i].y;
    double dz = cloud1->points[i].z - cloud2->points[i].z;

    sum += dx * dx + dy * dy + dz * dz;
  }

  double mse = sum / cloud1->size();
  return std::sqrt(mse);
}