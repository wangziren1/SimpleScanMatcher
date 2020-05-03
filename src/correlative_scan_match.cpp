#include <cmath>
#include <chrono>
#include <iostream>
using namespace std;
using namespace std::chrono;

#include "scanmatch/correlative_scan_match.h"


Point TransformPoint(const Point& point, const Pose& pose) {
  Point new_point;
  new_point.x_ = cos(pose.theta_) * point.x_ - sin(pose.theta_) * point.y_ + 
      pose.x_;
  new_point.y_ = sin(pose.theta_) * point.x_ + cos(pose.theta_) * point.y_ + 
      pose.y_;
  return new_point;
}

Pose CorrelativeScanMatcher::ComputePose(const Pose& initial_pose,
    vector<Point>& point_cloud, std::shared_ptr<Map> map) {
  // rotate point clouds rotated by delta theta
  steady_clock::time_point t1 = steady_clock::now();
  vector<RotatedPointCloud> rotated_point_clouds;
  for (float delta_theta = -max_angle_; delta_theta <= max_angle_; delta_theta
      +=angle_step_) {
    Pose new_pose(initial_pose.x_, initial_pose.y_, initial_pose.theta_ + 
        delta_theta * M_PI / 180.0);
    rotated_point_clouds.emplace_back();
    RotatedPointCloud& rotated_point_cloud = rotated_point_clouds.back();
    rotated_point_cloud.rotated_pose_ = new_pose;
    for (auto& point : point_cloud) {
      rotated_point_cloud.point_cloud_.push_back(TransformPoint(point, new_pose));
    }
  }
  cout << "Rotating point clouds takes: " << duration_cast<milliseconds>(
        steady_clock::now() - t1).count() << " ms" << endl;
  // translate point clouds by delta_x and delta_y
  steady_clock::time_point t2 = steady_clock::now();
  vector<Candidate> candidates;
  candidates.reserve(rotated_point_clouds.size() * search_window_width_ *
      search_window_height_);
  int max_delta_x = floor(search_window_width_ / 2);
  int max_delta_y = floor(search_window_height_ / 2);
  for (int index = 0; index < rotated_point_clouds.size(); ++index) {
    for (int delta_x = -max_delta_x; delta_x <= max_delta_x; ++delta_x) {
      for (int delta_y = -max_delta_y; delta_y <= max_delta_y; ++delta_y) {
        Candidate candidate(index, delta_x * map->Resolution(), delta_y * 
            map->Resolution());
        candidates.push_back(candidate);
      }
    }
  }
  cout << "Generating candidates takes: " << duration_cast<milliseconds>(
        steady_clock::now() - t2).count() << " ms" << endl;
  // compute score and find the maximium one
  steady_clock::time_point t3 = steady_clock::now();
  float max_score = 0;
  Candidate best_candidate;
  for (auto& candidate : candidates) {
    float score = 0;
    for (auto& point : rotated_point_clouds[candidate.
        rotated_point_cloud_index_].point_cloud_) {
      score += map->GetScore(point.x_ + candidate.delta_x_, point.y_ + 
          candidate.delta_y_);
    }
    score /= rotated_point_clouds[candidate.rotated_point_cloud_index_].
        point_cloud_.size();
    candidate.score_ = score;
    if (score > max_score) {
      max_score = score;
      best_candidate = candidate;
    }
  }
  cout << "Finding maximium score takes: " << duration_cast<milliseconds>(
        steady_clock::now() - t3).count() << " ms" << endl;
  cout << "Total time takes: " << duration_cast<milliseconds>(
        steady_clock::now() - t1).count() << " ms" << endl;
  Pose best_pose;
  best_pose.theta_ = rotated_point_clouds[best_candidate.
      rotated_point_cloud_index_].rotated_pose_.theta_;
  best_pose.x_ = rotated_point_clouds[best_candidate.
      rotated_point_cloud_index_].rotated_pose_.x_ + best_candidate.delta_x_;
  best_pose.y_ = rotated_point_clouds[best_candidate.
      rotated_point_cloud_index_].rotated_pose_.y_ + best_candidate.delta_y_;
  cout << "pose: (" << best_pose.x_ << "," << best_pose.y_ << "," 
      << best_pose.theta_ << ") max_score: " << max_score << endl;
  return best_pose;
}

// Pose CorrelativeScanMatcher::ComputePose(const Pose& initial_pose,
//     vector<Point>& point_cloud, std::shared_ptr<Map> map) {
//   // rotate point clouds rotated by delta theta
//   steady_clock::time_point t1 = steady_clock::now();
//   float max_score = 0;
//   Pose best_pose;
//   for (float delta_theta = -max_angle_; delta_theta <= max_angle_; delta_theta
//       +=angle_step_) {
//     RotatedPointCloud rotated_point_cloud;
//     Pose new_pose(initial_pose.x_, initial_pose.y_, initial_pose.theta_ + 
//         delta_theta * M_PI / 180.0);
//     rotated_point_cloud.rotated_pose_ = new_pose;
//     for (auto& point : point_cloud) {
//       rotated_point_cloud.point_cloud_.push_back(TransformPoint(point, new_pose));
//     }

//     int max_delta_x = floor(search_window_width_ / 2);
//     int max_delta_y = floor(search_window_height_ / 2);
//     for (int delta_x = -max_delta_x; delta_x <= max_delta_x; ++delta_x) {
//       for (int delta_y = -max_delta_y; delta_y <= max_delta_y; ++delta_y) {
//         float score = 0;
//         for (const auto& point : rotated_point_cloud.point_cloud_) {
//           float x = point.x_ + delta_x * map->Resolution();
//           float y = point.y_ + delta_y * map->Resolution();
//           score += map->GetScore(x, y);
//         }
//         score /= rotated_point_cloud.point_cloud_.size();
//         if (score > max_score) {
//           max_score = score;
//           float best_x = rotated_point_cloud.rotated_pose_.x_ + delta_x * 
//               map->Resolution();
//           float best_y = rotated_point_cloud.rotated_pose_.y_ + delta_y * 
//               map->Resolution();
//           best_pose = Pose(best_x, best_y, rotated_point_cloud.rotated_pose_.
//               theta_);
//         }
//       }
//     }
//   }
//   cout << "Total time takes: " << duration_cast<milliseconds>(
//         steady_clock::now() - t1).count() << " ms" << endl;
//   cout << "pose: (" << best_pose.x_ << "," << best_pose.y_ << "," 
//       << best_pose.theta_ << ") max_score: " << max_score << endl;
//   return best_pose;
// }