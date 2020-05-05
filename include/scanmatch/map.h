#ifndef MAP_H
#define MAP_H

#include <vector>
#include "scanmatch/pose_and_point.h"
#include <cmath>
using std::vector;

vector<GridPoint> Bresenham(GridPoint start, GridPoint end);

class Map {
 public:
  Map();
  float Resolution() const {return resolution_;}
  float GetLogOdds(const Point& p) const;
  void Update(const Pose& pose, const vector<Point>& point_cloud);

 private:
  GridPoint GetGridCoordinate(const Point& p) const {
    int grid_x = floor((p.x_ + offset_x_) / resolution_);
    int grid_y = floor((p.y_ + offset_y_) / resolution_);
    // if (grid_x >= 1000 || grid_y >= 1000) 
    //   cout << "out of range:(" << grid_x << "," << grid_y << ") " <<
    //   "(" << p.x_ << "," << p.y_ << ")" << endl;
    return GridPoint(grid_x, grid_y);
  }
  float resolution_;
  float prior_log_odds_;
  float hit_log_odds_;
  float miss_log_odds_;
  float offset_x_;
  float offset_y_;
  Point bottom_left_corner_;
  Point top_right_corner_;
  vector<vector<float>> grid_;
};

#endif