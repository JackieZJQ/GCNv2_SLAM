#include <vector>

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2 {

class FeaturePoint
{
public:
  FeaturePoint();

public:
  
  // Number of Keypoints
  int N;

  //
  std::vector<cv::KeyPoint> Keys, KeysRight;
  std::vector<cv::KeyPoint> KeysUn;

  //
  std::vector<float> uRight;
  std::vector<float> Depth;
  
  //
  cv::Mat Descriptors, DescriptorsRight;

  //
  std::vector<std::vector<std::vector<std::size_t>>> Grid;


};

    
    
}