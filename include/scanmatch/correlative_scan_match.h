#ifndef CORRELATIVE_SCAN_MATCH_H
#define CORRELATIVE_SCAN_MATCH_H
#include <vector>

#include "scanmatch/common.h"
#include "scanmatch/pose_and_point.h"
#include "scanmatch/map.h"

using std::vector;

struct RotatedPointCloud {
  Pose rotated_pose_;
  vector<Point> point_cloud_;
};

struct Candidate {
  Candidate()=default;
  Candidate(int index, float delta_x, float delta_y):
      rotated_point_cloud_index_(index),delta_x_(delta_x),delta_y_(delta_y){}
 
  int rotated_point_cloud_index_;
  float delta_x_;
  float delta_y_;
  float score_;
};

class CorrelativeScanMatcher {
 public:
  CorrelativeScanMatcher(const YAML::Node& config);
  Pose ComputePose(const Pose& initial_pose, const vector<Point>& point_cloud, 
      const Map& map);
  Pose ComputePoseAnother(const Pose& initial_pose, vector<Point>& point_cloud, 
      const Map& map);
 private:
  float max_angle_;
  float angle_step_; // degree
  int search_window_width_;
  int search_window_height_;
};

#endif 