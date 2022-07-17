#ifndef ASSOCIATER_H
#define ASSOCIATER_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"

namespace ORB_SLAM2 {

class Associater {
public:
  Associater(float nnratio = 0.6, bool checkOri = true);

  // Computes the Hamming distance between two ORB descriptors
  static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

  // Project MapPoints tracked in last frame into the current frame and search
  // matches. Used to track from previous frame (Tracking)
  int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono, const int Ftype);

  // Search matches between Frame keypoints and projected MapPoints. Returns
  // number of matches Used to track the local map (Tracking)
  int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th);

  int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint *> &vpMapPointMatches, const int Ftype);

  int SearchByNN(Frame &CurrentFrame, const Frame &LastFrame, const int Ftype);
  
  int SearchByNN(KeyFrame *pKF, Frame &F, std::vector<MapPoint *> &vpMapPointMatches, const int Ftype);

public:
  static const int TH_LOW;
  static const int TH_HIGH;
  static const int HISTO_LENGTH;

protected:
  float mfNNratio;
  bool mbCheckOrientation;

  void ComputeThreeMaxima(std::vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3);

  float RadiusByViewingCos(const float &viewCos);

};

}

#endif