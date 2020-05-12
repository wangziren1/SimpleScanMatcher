#include "scanmatch/pose_and_point.h"
#include <cmath>
#include <iomanip>

Point TransformPoint(const Point& point, const Pose& pose) {
  Point new_point;
  new_point.x_ = cos(pose.theta_) * point.x_ - sin(pose.theta_) * point.y_ + 
      pose.x_;
  new_point.y_ = sin(pose.theta_) * point.x_ + cos(pose.theta_) * point.y_ + 
      pose.y_;
  return new_point;
}

std::ostream& operator<<(std::ostream& os, const Pose& pose) {
  os << std::fixed << std::setprecision(3) << "(" << pose.x_ << "," << pose.y_
      << "," << pose.theta_ << ")";
	return os;
}

std::ostream& operator<<(std::ostream& os, const GridPoint& p) {
	os << "(" << p.y_ << "," << p.x_ << ")";
	return os;
}

std::ostream& operator<<(std::ostream& os, const Point& p) {
	os << std::fixed << std::setprecision(3) << "(" << p.y_ << "," << p.x_ << ")";
	return os;
}