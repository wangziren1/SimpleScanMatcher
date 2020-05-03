#include "scanmatch/map.h"
#include <cmath>
#include <iostream>
using namespace std;

GaussianDistribution::GaussianDistribution():gaussian_kernel_size_(3),
    sigma2_(0.3), half_kernel_(gaussian_kernel_size_ / 2){
  cout << "kernel:" << endl;
  probability_distribution_.resize(gaussian_kernel_size_);
  for (int j = -half_kernel_; j <= half_kernel_; ++j) {
    probability_distribution_[j + half_kernel_].resize(gaussian_kernel_size_);
    for (int i = -half_kernel_; i <= half_kernel_; ++i) {
      float dist = sqrt(i * i + j * j);
      probability_distribution_[j + half_kernel_][i + half_kernel_] = 
          1 / sqrt(2 * M_PI * sigma2_) * exp(- (dist * dist) / (2 * sigma2_));
      cout << probability_distribution_[j + half_kernel_][i + half_kernel_] << " ";
    }
    cout << endl;
  }
}

Map::Map(const vector<Point>& point_cloud) : resolution_(0.05),
    point_cloud_(point_cloud) {
  // find top right corner and bottom left corner
  float min_x = point_cloud[0].x_;
  float min_y = point_cloud[0].y_;
  float max_x = point_cloud[0].x_;
  float max_y = point_cloud[0].y_;
  for (auto& point : point_cloud) {
    if (point.x_ < min_x) min_x = point.x_;
    if (point.y_ < min_y) min_y = point.y_;
    if (point.x_ > max_x) max_x = point.x_;
    if (point.y_ > max_y) max_y = point.y_;
  }
  // expand box width and height two times and translate bottom left corner 
  // of box to (0,0)
  float width = max_x - min_x;
  float height = max_y - min_y;
  float new_bottom_left_corner_x = (min_x - width / 2) >= 0 ? 
      ceil(min_x - width / 2) : floor(min_x - width / 2);
  float new_bottom_left_corner_y  = (min_y - height / 2) >= 0 ?
      ceil(min_y - height / 2) : floor(min_y - height / 2);
  Point new_bottom_left_corner(new_bottom_left_corner_x, 
      new_bottom_left_corner_y);
  offset_x_ = -new_bottom_left_corner.x_;
  offset_y_ = -new_bottom_left_corner.y_;
  float new_width = 2 * width;
  float new_height = 2 * height;
  grid_.resize(ceil(new_height / resolution_));
  for (auto& row : grid_) {
    row = vector<float>(ceil(new_width / resolution_), 0);
  }
  // fill map
  for (auto& point : point_cloud) {
    int x = floor((point.x_ + offset_x_) / resolution_);
    int y = floor((point.y_ + offset_y_) / resolution_);
    int half_kernel = gaussian_distribution_.HalfKernel();
    for (int j = -half_kernel; j <= half_kernel; ++j) {
      for (int i = -half_kernel; i <= half_kernel; ++i) {
        grid_[y+j][x+i] = max(grid_[y+j][x+i],
            gaussian_distribution_.ProbabilityDistribution()
            [j+half_kernel][i+half_kernel]);
      }
    }
  }
}

float Map::GetScore(float x, float y) {
  int grid_x = floor((x + offset_x_) / resolution_);
  int grid_y = floor((y + offset_y_) / resolution_);
  return grid_[grid_y][grid_x];
}

GaussianDistribution Map::gaussian_distribution_;
