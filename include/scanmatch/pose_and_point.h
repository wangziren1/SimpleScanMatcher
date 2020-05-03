#ifndef POSE_AND_POINT_H
#define POSE_AND_POINT_H

struct Pose {
	Pose()=default;
	Pose(float x, float y, float theta):x_(x),y_(y),theta_(theta) {}
	Pose(const Pose& other):x_(other.x_),y_(other.y_),theta_(other.theta_) {}
	float x_;
	float y_;
	float theta_;
};

struct Point {
	Point()=default;
	Point(float x, float y):x_(x),y_(y){}
	float x_;
	float y_;
};

#endif