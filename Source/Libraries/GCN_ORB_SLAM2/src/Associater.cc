#include "Associater.h"

#include <limits.h>

#include "DBoW2/FeatureVector.h"

#include <stdint.h>

using namespace ::std;

namespace ORB_SLAM2 {

const int Associater::TH_HIGH = 100;
const int Associater::TH_LOW = 50;
const int Associater::HISTO_LENGTH = 30;

Associater::Associater(float nnratio, bool checkOri)
    : mfNNratio(nnratio), mbCheckOrientation(checkOri) {}

// Rewrite, it should be used in the trackmotionmodel
int Associater::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame,
                         const float th, const bool bMono, const int Ftype) {
  
  int nmatches = 0;
  
  // step 1 build rotation histogram (to check rotation consistency)
  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++)
    rotHist[i].reserve(500);
  const float factor = 1.0f / HISTO_LENGTH;

  // step 2 calcualte translation of current frame and last frame 
  // pose of current frame
  const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
  const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0, 3).col(3);

  // translation from current frame to world coordinate
  const cv::Mat twc = -Rcw.t() * tcw;

  // pose of last frame 
  const cv::Mat Rlw = LastFrame.mTcw.rowRange(0, 3).colRange(0, 3);
  const cv::Mat tlw = LastFrame.mTcw.rowRange(0, 3).col(3);

  // translation from current frame to last frame
  const cv::Mat tlc = Rlw * twc + tlw;

  // judge forward or backward
  const bool bForward = tlc.at<float>(2) > CurrentFrame.mb && !bMono;
  const bool bBackward = -tlc.at<float>(2) > CurrentFrame.mb && !bMono;

  // step 3 project the mappoints created by last frame to current frame, calcualte its  u and v in pixel
  for (int i = 0; i < LastFrame.mFeatData[Ftype].N; i++) {
    MapPoint *pMP = LastFrame.mFeatData[Ftype].mvpMapPoints[i];
  
    if (pMP) {
      if (!LastFrame.mFeatData[Ftype].mvbOutlier[i]) {
        // Project
        cv::Mat x3Dw = pMP->GetWorldPos();
        cv::Mat x3Dc = Rcw * x3Dw + tcw;

        const float xc = x3Dc.at<float>(0);
        const float yc = x3Dc.at<float>(1);
        const float invzc = 1.0 / x3Dc.at<float>(2);

        if (invzc < 0)
          continue;
        
        float u = CurrentFrame.fx * xc * invzc + CurrentFrame.cx;
        float v = CurrentFrame.fy * yc * invzc + CurrentFrame.cy;

        if (u < CurrentFrame.mnMinX || u > CurrentFrame.mnMaxX)
          continue;
        if (v < CurrentFrame.mnMinY || v > CurrentFrame.mnMaxY)
          continue;

        int nLastOctave = LastFrame.mFeatData[Ftype].mvKeys[i].octave;

        // Search in a window. Size depends on scale
        float radius = th * CurrentFrame.mvScaleFactors[nLastOctave];

        vector<size_t> vIndices2;

        if (bForward)
          vIndices2 = CurrentFrame.GetFeaturesInArea(Ftype, u, v, radius, nLastOctave);
        else if (bBackward)
          vIndices2 =
              CurrentFrame.GetFeaturesInArea(Ftype ,u, v, radius, 0, nLastOctave);
        else
          vIndices2 = CurrentFrame.GetFeaturesInArea(
              Ftype, u, v, radius, nLastOctave - 1, nLastOctave + 1);

        if (vIndices2.empty())
          continue;

        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = 256;
        int bestIdx2 = -1;

        for (vector<size_t>::const_iterator vit = vIndices2.begin(),
                                            vend = vIndices2.end();
             vit != vend; vit++) {
          const size_t i2 = *vit;
          if (CurrentFrame.mFeatData[Ftype].mvpMapPoints[i2])
            if (CurrentFrame.mFeatData[Ftype].mvpMapPoints[i2]->Observations() > 0)
              continue;

          if (CurrentFrame.mFeatData[Ftype].mvuRight[i2] > 0) {
            const float ur = u - CurrentFrame.mbf * invzc;
            const float er = fabs(ur - CurrentFrame.mFeatData[Ftype].mvuRight[i2]);
            if (er > radius)
              continue;
          }

          const cv::Mat &d = CurrentFrame.mFeatData[Ftype].mDescriptors.row(i2);

          const int dist = DescriptorDistance(dMP, d);

          if (dist < bestDist) {
            bestDist = dist;
            bestIdx2 = i2;
          }
        }

        if (bestDist <= TH_HIGH) {
          CurrentFrame.mFeatData[Ftype].mvpMapPoints[bestIdx2] = pMP;
          nmatches++;

          if (mbCheckOrientation) {
            float rot = LastFrame.mFeatData[Ftype].mvKeysUn[i].angle -
                        CurrentFrame.mFeatData[Ftype].mvKeysUn[bestIdx2].angle;
            if (rot < 0.0)
              rot += 360.0f;
            int bin = round(rot * factor);
            if (bin == HISTO_LENGTH)
              bin = 0;
            assert(bin >= 0 && bin < HISTO_LENGTH);
            rotHist[bin].push_back(bestIdx2);
          }
        }
      }
    }
  }

  // Apply rotation consistency
  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i != ind1 && i != ind2 && i != ind3) {
        for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
          CurrentFrame.mFeatData[Ftype].mvpMapPoints[rotHist[i][j]] =
              static_cast<MapPoint *>(NULL);
          nmatches--;
        }
      }
    }
  }
  
  return nmatches;
}

int Associater::SearchByBoW(KeyFrame *pKF, Frame &F,
                            vector<MapPoint *> &vpMapPointMatches, const int Ftype) {
  const vector<MapPoint *> vpMapPointsKF = pKF->GetMapPointMatches(Ftype);

  vpMapPointMatches = vector<MapPoint *>(F.mFeatData[Ftype].N, static_cast<MapPoint *>(NULL));

  // suppose the featvec equals the featvec in frame.featdata, mybe it was not updated
  const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatData[Ftype].mFeatVec;

  int nmatches = 0;

  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++)
    rotHist[i].reserve(500);
  const float factor = 1.0f / HISTO_LENGTH;

  DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
  DBoW2::FeatureVector::const_iterator Fit = F.mFeatData[Ftype].mFeatVec.begin();
  DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
  DBoW2::FeatureVector::const_iterator Fend = F.mFeatData[Ftype].mFeatVec.end();

  while (KFit != KFend && Fit != Fend) {
    if (KFit->first == Fit->first) {
      const vector<unsigned int> vIndicesKF = KFit->second;
      const vector<unsigned int> vIndicesF = Fit->second;

      for (size_t iKF = 0; iKF < vIndicesKF.size(); iKF++) {
        const unsigned int realIdxKF = vIndicesKF[iKF];

        MapPoint *pMP = vpMapPointsKF[realIdxKF];

        if (!pMP)
          continue;

        if (pMP->isBad())
          continue;

        const cv::Mat &dKF = pKF->mFeatData[Ftype].mDescriptors.row(realIdxKF);

        int bestDist1 = 256;
        int bestIdxF = -1;
        int bestDist2 = 256;

        for (size_t iF = 0; iF < vIndicesF.size(); iF++) {
          const unsigned int realIdxF = vIndicesF[iF];

          if (vpMapPointMatches[realIdxF])
            continue;

          const cv::Mat &dF = F.mFeatData[Ftype].mDescriptors.row(realIdxF);

          const int dist = DescriptorDistance(dKF, dF);

          if (dist < bestDist1) {
            bestDist2 = bestDist1;
            bestDist1 = dist;
            bestIdxF = realIdxF;
          } else if (dist < bestDist2) {
            bestDist2 = dist;
          }
        }

        if (bestDist1 <= TH_LOW) {
          if (static_cast<float>(bestDist1) <
              mfNNratio * static_cast<float>(bestDist2)) {
            vpMapPointMatches[bestIdxF] = pMP;

            const cv::KeyPoint &kp = pKF->mFeatData[Ftype].mvKeysUn[realIdxKF];

            if (mbCheckOrientation) {
              float rot = kp.angle - F.mFeatData[Ftype].mvKeys[bestIdxF].angle;
              if (rot < 0.0)
                rot += 360.0f;
              int bin = round(rot * factor);
              if (bin == HISTO_LENGTH)
                bin = 0;
              assert(bin >= 0 && bin < HISTO_LENGTH);
              rotHist[bin].push_back(bestIdxF);
            }
            nmatches++;
          }
        }
      }

      KFit++;
      Fit++;
    } else if (KFit->first < Fit->first) {
      KFit = vFeatVecKF.lower_bound(Fit->first);
    } else {
      Fit = F.mFeatData[Ftype].mFeatVec.lower_bound(KFit->first);
    }
  }

  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i == ind1 || i == ind2 || i == ind3)
        continue;
      for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
        vpMapPointMatches[rotHist[i][j]] = static_cast<MapPoint *>(NULL);
        nmatches--;
      }
    }
  }

  return nmatches;
}

// search by nearest nerghbour
int Associater::SearchByNN(Frame &CurrentFrame, const Frame &LastFrame, const int Ftype) {
  
  std::vector<cv::DMatch> matches;
  cv::BFMatcher desc_matcher(cv::NORM_HAMMING, true);
  desc_matcher.match(LastFrame.mFeatData[Ftype].mDescriptors, CurrentFrame.mFeatData[Ftype].mDescriptors, matches,
                     cv::Mat());

  int nmatches = 0;
  for (int i = 0; i < static_cast<int>(matches.size()); ++i) {
    int realIdxKF = matches[i].queryIdx;
    int bestIdxF = matches[i].trainIdx;

    if (matches[i].distance > TH_LOW)
      continue;
    
    MapPoint *pMP = LastFrame.mFeatData[Ftype].mvpMapPoints[realIdxKF];
    if (!pMP)
      continue;

    if (pMP->isBad())
      continue;

    if (!LastFrame.mFeatData[Ftype].mvbOutlier[realIdxKF])
      CurrentFrame.mFeatData[Ftype].mvpMapPoints[bestIdxF] = pMP;

    nmatches++;

  }
  
  return nmatches;
}

// compute three maxima
void Associater::ComputeThreeMaxima(vector<int> *histo, const int L, int &ind1,
                                    int &ind2, int &ind3) {
  int max1 = 0;
  int max2 = 0;
  int max3 = 0;

  for (int i = 0; i < L; i++) {
    const int s = histo[i].size();
    if (s > max1) {
      max3 = max2;
      max2 = max1;
      max1 = s;
      ind3 = ind2;
      ind2 = ind1;
      ind1 = i;
    } else if (s > max2) {
      max3 = max2;
      max2 = s;
      ind3 = ind2;
      ind2 = i;
    } else if (s > max3) {
      max3 = s;
      ind3 = i;
    }
  }

  if (max2 < 0.1f * (float)max1) {
    ind2 = -1;
    ind3 = -1;
  } else if (max3 < 0.1f * (float)max1) {
    ind3 = -1;
  }
}

// Bit set count operation from http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int Associater::DescriptorDistance(const cv::Mat &a, const cv::Mat &b) {
  const int *pa = a.ptr<int32_t>();
  const int *pb = b.ptr<int32_t>();

  int dist = 0;

  for (int i = 0; i < 8; i++, pa++, pb++) {
    unsigned int v = *pa ^ *pb;
    v = v - ((v >> 1) & 0x55555555);
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }

  return dist;
}

}