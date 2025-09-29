/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIDAR_DETECT_HPP
#define LIDAR_DETECT_HPP
#include <glog/logging.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_base.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "struct.hpp"
// #include "common_lib.h"

class LidarDetect {
 private:
  double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
  double circle_radius_;
  double voxel_down_size_;
  double plane_dist_threshold_;

  double target_normal_radius_;
  double target_boundary_radius_;
  double target_boundary_angle_thres_;

  // 存储中间结果的点云
  PointCloudPtr filtered_cloud_;
  PointCloudPtr plane_cloud_;
  PointCloudPtr aligned_cloud_;
  PointCloudPtr edge_cloud_;
  PointCloudPtr center_z0_cloud_;
  std::vector<pcl::PointIndices> cluster_indices_;
  std::vector<PointCloudPtr> cluster_clouds_;

 public:
  LidarDetect(Parameters& params)
      : filtered_cloud_(MAKE_POINTCLOUD()),
        plane_cloud_(MAKE_POINTCLOUD()),
        aligned_cloud_(MAKE_POINTCLOUD()),
        edge_cloud_(MAKE_POINTCLOUD()),
        center_z0_cloud_(MAKE_POINTCLOUD()) {
    x_min_ = params.crop_min_xyz[0];
    x_max_ = params.crop_max_xyz[0];
    y_min_ = params.crop_min_xyz[1];
    y_max_ = params.crop_max_xyz[1];
    z_min_ = params.crop_min_xyz[2];
    z_max_ = params.crop_max_xyz[2];

    voxel_down_size_ = params.voxel_downsample_size;

    circle_radius_ = params.circle_radius;

    LOG(INFO) << "LidarDetect initialized, x_min: " << x_min_
              << ", x_max: " << x_max_ << ", y_min: " << y_min_
              << ", y_max: " << y_max_ << ", z_min: " << z_min_
              << ", z_max: " << z_max_ << ", circle_radius: " << circle_radius_
              << ", voxel_down_size: " << voxel_down_size_;

    plane_dist_threshold_ = params.plane_dist_threshold;
    LOG(INFO) << "Plane fitting params, plane_dist_threshold: "
              << plane_dist_threshold_;

    target_normal_radius_ = params.target_normal_radius;
    target_boundary_radius_ = params.target_boundary_radius;
    target_boundary_angle_thres_ = params.target_boundary_angle_thres;
    LOG(INFO) << "Target params, target_normal_radius: "
              << target_normal_radius_
              << ", target_boundary_radius: " << target_boundary_radius_
              << ", target_boundary_angle_thres: "
              << target_boundary_angle_thres_;
  }

  void detect_lidar(PointCloudPtr cloud, PointCloudPtr center_cloud) {
    // 1. X、Y、Z方向滤波
    filtered_cloud_->reserve(cloud->size());

    pcl::PassThrough<PointT> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_min_, x_max_);  // 设置X轴范围
    pass_x.filter(*filtered_cloud_);

    pcl::PassThrough<PointT> pass_y;
    pass_y.setInputCloud(filtered_cloud_);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_min_, y_max_);  // 设置Y轴范围
    pass_y.filter(*filtered_cloud_);

    pcl::PassThrough<PointT> pass_z;
    pass_z.setInputCloud(filtered_cloud_);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_min_, z_max_);  // 设置Z轴范围
    pass_z.filter(*filtered_cloud_);

    LOG(INFO) << "PassThrough filtered cloud size: " << filtered_cloud_->size();

    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(filtered_cloud_);
    voxel_filter.setLeafSize(voxel_down_size_, voxel_down_size_,
                             voxel_down_size_);
    voxel_filter.filter(*filtered_cloud_);
    LOG(INFO) << "VoxelGrid filtered cloud size: " << filtered_cloud_->size();

    // 2. 平面分割
    plane_cloud_->reserve(filtered_cloud_->size());

    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> plane_segmentation;
    plane_segmentation.setModelType(pcl::SACMODEL_PLANE);
    plane_segmentation.setMethodType(pcl::SAC_RANSAC);
    plane_segmentation.setDistanceThreshold(
        plane_dist_threshold_);  // 平面分割阈值
    plane_segmentation.setInputCloud(filtered_cloud_);
    plane_segmentation.segment(*plane_inliers, *plane_coefficients);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(filtered_cloud_);
    extract.setIndices(plane_inliers);
    extract.filter(*plane_cloud_);
    LOG(INFO) << "Plane cloud size: " << plane_cloud_->size();

    // 3. 平面点云对齐
    aligned_cloud_->reserve(plane_cloud_->size());

    Eigen::Vector3d normal(plane_coefficients->values[0],
                           plane_coefficients->values[1],
                           plane_coefficients->values[2]);
    normal.normalize();
    Eigen::Vector3d z_axis(0, 0, 1);

    Eigen::Vector3d axis = normal.cross(z_axis);
    double angle = acos(normal.dot(z_axis));

    Eigen::AngleAxisd rotation(angle, axis);
    Eigen::Matrix3d R = rotation.toRotationMatrix();
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = R;

    // 应用旋转矩阵，将平面对齐到 Z=0 平面
    float average_z = 0.0;
    pcl::transformPointCloud(*plane_cloud_, *aligned_cloud_, transform);
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*aligned_cloud_, centroid);
    average_z = centroid[2];

    Eigen::Vector3d eulers_rad = R.eulerAngles(2, 1, 0);
    Eigen::Vector3d eulers_deg = eulers_rad * 180.0 / M_PI;
    LOG(INFO) << "Plane fitting euler angles(degree): "
              << eulers_deg.transpose() << ", average z: " << average_z;

    // set aligned_cloud_ z coordinate to 0
    for (auto& point : *aligned_cloud_) {
      point.z = 0;
    }

    // 4. 提取边缘点
    edge_cloud_->reserve(aligned_cloud_->size());

    pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normal_estimator.setInputCloud(aligned_cloud_);
    normal_estimator.setRadiusSearch(0.03);  // 设置法线估计的搜索半径
    normal_estimator.compute(*normals);

    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<PointT, pcl::Normal, pcl::Boundary>
        boundary_estimator;
    boundary_estimator.setInputCloud(aligned_cloud_);
    boundary_estimator.setInputNormals(normals);
    boundary_estimator.setRadiusSearch(0.03);        // 设置边界检测的搜索半径
    boundary_estimator.setAngleThreshold(M_PI / 4);  // 设置角度阈值
    boundary_estimator.compute(boundaries);

    for (size_t i = 0; i < aligned_cloud_->size(); ++i) {
      if (boundaries.points[i].boundary_point > 0) {
        edge_cloud_->push_back(aligned_cloud_->points[i]);
      }
    }
    LOG(INFO) << "Edge cloud size: " << edge_cloud_->size();

    // 5. 对边缘点进行聚类
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(edge_cloud_);

    // std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.02);  // 设置聚类距离阈值
    // NOTE: 这里实际上是通过最大点数的限制来去掉最外面的包围框的边缘点
    // 这里应该会受标靶尺寸的影响（因为已经Voxel降采样，所以应该不会受到边缘密度的影响）
    ec.setMinClusterSize(50);      // 最小点数
    ec.setMaxClusterSize(1000);    // 最大点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(edge_cloud_);
    ec.extract(cluster_indices_);
    LOG(INFO) << "Number of edge clusters: " << cluster_indices_.size();

    // 6. 对每个聚类进行圆拟合
    center_z0_cloud_->reserve(4);
    Eigen::Matrix3d R_inv = R.inverse();

    // 对每个聚类进行圆拟合
    for (size_t i = 0; i < cluster_indices_.size(); ++i) {
      PointCloudPtr cluster(new pcl::PointCloud<PointT>);
      for (const auto& idx : cluster_indices_[i].indices) {
        cluster->push_back(edge_cloud_->points[idx]);
      }
      cluster_clouds_.push_back(cluster);

      // 圆拟合
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      pcl::SACSegmentation<PointT> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_CIRCLE2D);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.01);  // 设置距离阈值
      seg.setMaxIterations(1000);      // 设置最大迭代次数
      seg.setInputCloud(cluster);
      seg.segment(*inliers, *coefficients);

      if (inliers->indices.size() > 0) {
        // 计算拟合误差
        double error = 0.0;
        for (const auto& idx : inliers->indices) {
          double dx = cluster->points[idx].x - coefficients->values[0];
          double dy = cluster->points[idx].y - coefficients->values[1];
          double distance =
              sqrt(dx * dx + dy * dy) - circle_radius_;  // 距离误差
          error += abs(distance);
        }
        error /= inliers->indices.size();

        // 如果拟合误差较小，则认为是一个圆洞
        if (error < 0.025) {
          // 将恢复后的圆心坐标添加到点云中
          PointT center_point;
          center_point.x = coefficients->values[0];
          center_point.y = coefficients->values[1];
          center_point.z = 0.0;
          center_z0_cloud_->push_back(center_point);

          // 将圆心坐标逆变换回原始坐标系
          Eigen::Vector3d aligned_point(center_point.x, center_point.y,
                                        center_point.z + average_z);
          Eigen::Vector3d original_point = R_inv * aligned_point;

          PointT center_point_origin;
          center_point_origin.x = original_point.x();
          center_point_origin.y = original_point.y();
          center_point_origin.z = original_point.z();
          center_cloud->points.push_back(center_point_origin);
        }
      }
    }

    center_cloud->width = 1;
    center_cloud->height = center_cloud->points.size();
  }

  // 获取中间结果的点云
  PointCloudPtr getFilteredCloud() const { return filtered_cloud_; }
  PointCloudPtr getPlaneCloud() const { return plane_cloud_; }
  PointCloudPtr getAlignedCloud() const { return aligned_cloud_; }
  PointCloudPtr getEdgeCloud() const { return edge_cloud_; }
  PointCloudPtr getCenterZ0Cloud() const { return center_z0_cloud_; }
  std::vector<PointCloudPtr> getClusterClouds() const { return cluster_clouds_; }
};

typedef std::shared_ptr<LidarDetect> LidarDetectPtr;

#endif