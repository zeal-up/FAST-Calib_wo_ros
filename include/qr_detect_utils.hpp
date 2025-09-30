#pragma once
#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>


#include "struct.hpp"

void sortPatternCenters(PointCloudPtr pc,
                        PointCloudPtr v,
                        const std::string& axis_mode = "camera");
class Square 
{
private:
    PointT _center;
    std::vector<PointT> _candidates;
    float _target_width, _target_height, _target_diagonal;

public:
    Square(std::vector<PointT> candidates, float width, float height);
    
    float distance(PointT pt1, PointT pt2);
    PointT at(int i);
    bool is_valid();
};