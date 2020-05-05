#include "scanmatch/map.h"
#include <iostream>
#include <chrono>
using namespace std;
using namespace std::chrono;

float ProbToLogOdds(float p) {
  return log(p / (1 - p));
}

float LogOddsToProb(float l) {
  return 1 - 1 / (1 + exp(l));
}

Map::Map():resolution_(0.05),prior_log_odds_(ProbToLogOdds(0.5)),
    hit_log_odds_(ProbToLogOdds(0.55)), miss_log_odds_(ProbToLogOdds(0.49)), 
    offset_x_(500 * resolution_),offset_y_(500 * resolution_),
    bottom_left_corner_(1000, 1000),top_right_corner_(-1000, -1000),
    grid_(1000, vector<float>(1000, prior_log_odds_)) {
  // cout << grid_.size() << "," << grid_[0].size() << endl;
}

float Map::GetLogOdds(const Point& p) const {
  GridPoint grid_point = GetGridCoordinate(p);
  return grid_[grid_point.y_][grid_point.x_];
}

void Map::Update(const Pose& pose, const vector<Point>& point_cloud) {
  // hit
  for (const auto& point : point_cloud) {
    GridPoint hit_point = GetGridCoordinate(point);
    grid_[hit_point.y_][hit_point.x_] = grid_[hit_point.y_][hit_point.x_] + 
        hit_log_odds_ - prior_log_odds_;
  }
  // miss
  GridPoint grid_pose = GetGridCoordinate(Point(pose.x_, pose.y_));
  for (const auto& end : point_cloud) {
    GridPoint grid_end = GetGridCoordinate(end);
    vector<GridPoint> miss_points = Bresenham(grid_pose, grid_end);
    for (const auto& miss_point : miss_points) {
      grid_[miss_point.y_][miss_point.x_] = grid_[miss_point.y_][miss_point.x_] + 
          miss_log_odds_ - prior_log_odds_;
    }
  }
}

vector<GridPoint> Bresenham(GridPoint start, GridPoint end) {
  bool steep = abs(end.y_ -start.y_) > abs(end.x_ -start.x_);
  if (steep) {
    start.SwapXY();
    end.SwapXY();
  }
  bool swap = false;
  if (start.x_ > end.x_) {
    swap = true;
    GridPoint temp(start);
    start = end;
    end = temp;
  }
  int deltax = end.x_ -start.x_;
  int deltay = abs(end.y_ -start.y_);
  int error = 0;

  int y_step = 0;
  if (start.y_ < end.y_) y_step = 1;
  else y_step = -1;
  int x_n =start.x_;
  int y_n =start.y_;

  vector<GridPoint> result;
  result.reserve(deltax + 1);
  if (steep) {
    for (int n = 0; n <= deltax; ++n) {
      result.push_back(GridPoint(y_n, x_n));
      x_n++;
      error += deltay;
      if (2 * error >= deltax) {
        y_n += y_step;
        error -= deltax;
      }
    }
  } else {
    for (int n = 0; n <= deltax; ++n) {
      result.push_back(GridPoint(x_n, y_n));
      x_n++;
      error += deltay;
      if (2 * error >= deltax) {
        y_n += y_step;
        error -= deltax;
      }
    }
  }
  if (swap) {
    for (int i = 0, j = result.size()-1; i < j; ++i, --j) {
      GridPoint temp(result[i]);
      result[i] = result[j];
      result[j] = temp;
    }
  }
  result.pop_back();
  return result;
}