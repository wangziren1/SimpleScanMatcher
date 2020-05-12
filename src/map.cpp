#include "scanmatch/map.h"
#include <iostream>
#include <chrono>
using namespace std;
using namespace std::chrono;

inline float ProbToCost(float prob) {
  return 1 - prob;
}

inline float CostToProb(float cost) {
  return 1 - cost;
}

inline float ProbToOdds(float prob) {
  return prob / (1 - prob);
}

inline float OddsToProb(float odd) {
  return odd / (1 + odd);
}

inline int ProbToValue(float prob) {
  return int(prob * 100);
}

inline float Clamp(float x) {
  if (x < MinProb) return MinProb;
  else if (x > MaxProb) return MaxProb;
  else return x;
}

Map::Map():resolution_(0.05),width_(300),height_(300),
    prior_prob_(0.5),hit_prob_(0.55),miss_prob_(0.49),
    prior_odds_(ProbToOdds(prior_prob_)),
    hit_odds_(ProbToOdds(hit_prob_)),
    miss_odds_(ProbToOdds(miss_prob_)),
    offset_x_(width_ * resolution_ / 2),
    offset_y_(height_ * resolution_ / 2),
    unknown_cost_(-1),
    cost_(height_, vector<float>(width_, unknown_cost_)),
    discrete_prob_(height_, vector<int>(width_, -1)){
}

void Map::Update(const Pose& pose, const vector<Point>& point_cloud) {
  // hit
  for (const auto& point : point_cloud) {
    GridPoint hit_point = GetGridCoordinate(point);
    float& c = cost_[hit_point.y_][hit_point.x_];
    int& discrete_prob = discrete_prob_[hit_point.y_][hit_point.x_];
    if (c == unknown_cost_) {
      c = ProbToCost(Clamp(OddsToProb(prior_odds_ * hit_odds_)));
    } else {
      c = ProbToCost(Clamp(OddsToProb(ProbToOdds(CostToProb(c)) * hit_odds_)));
    }
    discrete_prob = ProbToValue(CostToProb(c));
  }
  // miss
  GridPoint grid_pose = GetGridCoordinate(Point(pose.x_, pose.y_));
  for (const auto& end : point_cloud) {
    GridPoint grid_end = GetGridCoordinate(end);
    vector<GridPoint> miss_points = Bresenham(grid_pose, grid_end);
    for (const auto& miss_point : miss_points) {
      float& c = cost_[miss_point.y_][miss_point.x_];
      int& discrete_prob = discrete_prob_[miss_point.y_][miss_point.x_];
      if (c == unknown_cost_) {
        c = ProbToCost(Clamp(OddsToProb(prior_odds_ * miss_odds_)));
      } else {
        c = ProbToCost(Clamp(OddsToProb(ProbToOdds(CostToProb(c)) * miss_odds_)));
      }
      discrete_prob = ProbToValue(CostToProb(c));
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