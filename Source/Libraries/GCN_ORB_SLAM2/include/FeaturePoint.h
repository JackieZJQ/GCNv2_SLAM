#ifndef FEATUREPOINT_H
#define FEATUREPOINT_H

#include <vector>

#include "MapPoint.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2 {

class MapPoint;

class FeaturePoint
{
public:
  
  //Feature type
  int type;

  // Number of Keypoints
  int N;

  std::vector<cv::KeyPoint> Keys;
  std::vector<cv::KeyPoint> KeysRight;
  std::vector<cv::KeyPoint> KeysUn;

  std::vector<float> uRight;
  std::vector<float> Depth;
  
  cv::Mat Descriptors; 
  cv::Mat DescriptorsRight;

  std::vector<MapPoint *> MapPoints;
  std::vector<bool> Outlier;

  std::vector<std::vector<std::vector<std::size_t>>> Grid;
};
    
}

#endif