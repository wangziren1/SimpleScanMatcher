#ifndef MAP_H
#define MAP_H

#include <vector>
#include "scanmatch/pose_and_point.h"
#include <cmath>
using std::vector;

const float MinProb = 0.1;
const float MaxProb = 1 - MinProb;
const float MinCost = MinProb;
const float MaxCost = MaxProb;

class Map {
 public:
  Map();

  float OffSetX() const {return offset_x_;}

  float OffSetY() const {return offset_y_;}

  float Resolution() const {return resolution_;}

  int Width() const {return width_;}

  int Height() const {return height_;}

  float GetProb(const Point& p) const {
    GridPoint grid_point = GetGridCoordinate(p);
    if (cost_[grid_point.y_][grid_point.x_] == unknown_cost_) {
      return MinProb;
    } else {
      return 1 - cost_[grid_point.y_][grid_point.x_];
    }
  }

  float GetCost(const GridPoint& p) const {
    if (cost_[p.y_][p.x_] == unknown_cost_) {
      return MaxCost;
    } else {
      return cost_[p.y_][p.x_];
    }
  }

  void Update(const Pose& pose, const vector<Point>& point_cloud);

  Point Origin() {return Point(offset_x_, offset_y_);}

  const vector<vector<int>>& DiscreteProbabilityMap() {return discrete_prob_;}

  const vector<vector<float>>& CostMap() {return cost_;}

  Point GetFullGridCoordinate(const Point& p) const {
    float grid_x = (p.x_ + offset_x_) / resolution_;
    float grid_y = (p.y_ + offset_y_) / resolution_;
    return Point(grid_x, grid_y);
  }

 private:
  GridPoint GetGridCoordinate(const Point& p) const {
    int grid_x = floor((p.x_ + offset_x_) / resolution_);
    int grid_y = floor((p.y_ + offset_y_) / resolution_);
    return GridPoint(grid_x, grid_y);
  }

  float resolution_;
  int width_;
  int height_;
  float prior_prob_;
  float hit_prob_;
  float miss_prob_;
  float prior_odds_;
  float hit_odds_;
  float miss_odds_;
  float offset_x_;
  float offset_y_;
  float unknown_cost_;
  vector<vector<float>> cost_;
  vector<vector<int>> discrete_prob_;
};

vector<GridPoint> Bresenham(GridPoint start, GridPoint end);

#endif