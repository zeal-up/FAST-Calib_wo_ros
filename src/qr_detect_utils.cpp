#include "qr_detect_utils.hpp"

#include <pcl/common/centroid.h>

void sortPatternCenters(PointCloudPtr in_pc, PointCloudPtr out_v,
                        const std::string& axis_mode) {
  if (in_pc->size() != 4) {
    std::cerr << "[sortPatternCenters] Number of " << axis_mode
              << " center points to be sorted is not 4." << std::endl;
    return;
  }

  PointCloudPtr work_pc = MAKE_POINTCLOUD();

  // Coordinate transformation (LiDAR -> Camera)
  if (axis_mode == "lidar") {
    for (const auto& p : *in_pc) {
      PointT pt;
      pt.x = -p.y;  // LiDAR Y -> Cam -X
      pt.y = -p.z;  // LiDAR Z -> Cam -Y
      pt.z = p.x;   // LiDAR X -> Cam Z
      work_pc->push_back(pt);
    }
  } else {
    *work_pc = *in_pc;
  }

  // --- Sorting based on the local coordinate system of the pattern ---
  // 1. Calculate the centroid of the points
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*work_pc, centroid);
  PointT ref_origin(centroid[0], centroid[1], centroid[2]);

  // 2. Project points to the XY plane relative to the centroid and calculate
  // angles
  std::vector<std::pair<float, int>> proj_points;
  for (size_t i = 0; i < work_pc->size(); ++i) {
    const auto& p = work_pc->points[i];
    Eigen::Vector3f rel_vec(p.x - ref_origin.x, p.y - ref_origin.y,
                            p.z - ref_origin.z);
    proj_points.emplace_back(atan2(rel_vec.y(), rel_vec.x()), i);
  }

  // 3. Sort points based on the calculated angle
  std::sort(proj_points.begin(), proj_points.end());

  // 4. Output the sorted points into the result vector 'out_v'
  out_v->resize(4);
  for (int i = 0; i < 4; ++i) {
    (*out_v)[i] = work_pc->points[proj_points[i].second];
  }

  // 5. Verify the order (ensure it's counter-clockwise) and fix if necessary
  const auto& p0 = out_v->points[0];
  const auto& p1 = out_v->points[1];
  const auto& p2 = out_v->points[2];
  Eigen::Vector3f v01(p1.x - p0.x, p1.y - p0.y, 0);
  Eigen::Vector3f v12(p2.x - p1.x, p2.y - p1.y, 0);
  if (v01.cross(v12).z() > 0) {
    std::swap((*out_v)[1], (*out_v)[3]);
  }

  // 6. If the original input was in the lidar frame, transform the sorted
  // points back
  if (axis_mode == "lidar") {
    for (auto& point : out_v->points) {
      float x_new = point.z;   // Cam Z -> LiDAR X
      float y_new = -point.x;  // Cam -X -> LiDAR Y
      float z_new = -point.y;  // Cam -Y -> LiDAR Z
      point.x = x_new;
      point.y = y_new;
      point.z = z_new;
    }
  }
}

/**
 * @brief Square类用来从可能检测出大于4个ArUco Markers的图片中筛选出符合
 * 标靶形状的4个markers
 *
 */
Square::Square(std::vector<PointT> candidates, float width, float height) {
  _candidates = candidates;
  _target_width = width;
  _target_height = height;
  _target_diagonal = sqrt(pow(width, 2) + pow(height, 2));

  // Compute candidates centroid
  _center.x = _center.y = _center.z = 0;
  for (size_t i = 0; i < candidates.size(); ++i) {
    _center.x += candidates[i].x;
    _center.y += candidates[i].y;
    _center.z += candidates[i].z;
  }

  _center.x /= candidates.size();
  _center.y /= candidates.size();
  _center.z /= candidates.size();
}

float Square::distance(PointT pt1, PointT pt2) {
  return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) +
              pow(pt1.z - pt2.z, 2));
}

PointT Square::at(int i) {
  assert(0 <= i && i < 4);
  return _candidates[i];
}

bool Square::is_valid() {
  if (_candidates.size() != 4) return false;

  pcl::PointCloud<PointT>::Ptr candidates_cloud(new pcl::PointCloud<PointT>());
  for (const auto& p : _candidates) candidates_cloud->push_back(p);

  // Check if candidates are at a reasonable distance from their centroid
  for (size_t i = 0; i < _candidates.size(); ++i) {
    float d = distance(_center, _candidates[i]);
    // Check if distance from center to corner is close to half the diagonal
    // length
    if (fabs(d - _target_diagonal / 2.) / (_target_diagonal / 2.) >
        GEOMETRY_TOLERANCE * 2.0) {  // Loosened tolerance slightly
      return false;
    }
  }

  // Sort the corners counter-clockwise
  pcl::PointCloud<PointT>::Ptr sorted_centers(new pcl::PointCloud<PointT>());
  sortPatternCenters(candidates_cloud, sorted_centers, "camera");

  // Get the four side lengths from the sorted points
  float s01 = distance(sorted_centers->points[0], sorted_centers->points[1]);
  float s12 = distance(sorted_centers->points[1], sorted_centers->points[2]);
  float s23 = distance(sorted_centers->points[2], sorted_centers->points[3]);
  float s30 = distance(sorted_centers->points[3], sorted_centers->points[0]);

  // Check for pattern 1: width, height, width, height
  bool pattern1_ok =
      (fabs(s01 - _target_width) / _target_width < GEOMETRY_TOLERANCE) &&
      (fabs(s12 - _target_height) / _target_height < GEOMETRY_TOLERANCE) &&
      (fabs(s23 - _target_width) / _target_width < GEOMETRY_TOLERANCE) &&
      (fabs(s30 - _target_height) / _target_height < GEOMETRY_TOLERANCE);

  // Check for pattern 2: height, width, height, width
  bool pattern2_ok =
      (fabs(s01 - _target_height) / _target_height < GEOMETRY_TOLERANCE) &&
      (fabs(s12 - _target_width) / _target_width < GEOMETRY_TOLERANCE) &&
      (fabs(s23 - _target_height) / _target_height < GEOMETRY_TOLERANCE) &&
      (fabs(s30 - _target_width) / _target_width < GEOMETRY_TOLERANCE);

  if (!pattern1_ok && !pattern2_ok) {
    return false;
  }

  // Final check on perimeter
  float perimeter = s01 + s12 + s23 + s30;
  float ideal_perimeter = 2 * (_target_width + _target_height);
  if (fabs(perimeter - ideal_perimeter) / ideal_perimeter >
      GEOMETRY_TOLERANCE) {
    return false;
  }

  return true;
}