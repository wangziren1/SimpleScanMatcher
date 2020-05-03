#ifndef MAP_H
#define MAP_H

#include <vector>
#include "scanmatch/pose_and_point.h"
using std::vector;

class GaussianDistribution {
 public:
  GaussianDistribution();
  const vector<vector<float>>& ProbabilityDistribution() {
      return probability_distribution_;}
  int HalfKernel() {return half_kernel_;}
 private:
  int gaussian_kernel_size_;
  int half_kernel_;
  float sigma2_;
  vector<vector<float>> probability_distribution_;
};

class Map {
 public:
  Map(const vector<Point>& point_cloud);
  vector<Point>& PointCloud() {return point_cloud_;}
  float Resolution() {return resolution_;}
  float GetScore(float x, float y);

 private:
  float resolution_;
  vector<Point> point_cloud_;
  vector<vector<float>> grid_;
  float offset_x_;
  float offset_y_;
  static GaussianDistribution gaussian_distribution_;
};

#endif