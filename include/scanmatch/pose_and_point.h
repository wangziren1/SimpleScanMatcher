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

struct GridPoint {
	GridPoint(int x, int y):x_(x),y_(y) {}
	GridPoint(const GridPoint& other):x_(other.x_),y_(other.y_){}
	GridPoint& operator=(GridPoint& other) {
		x_ = other.x_;
		y_ = other.y_;
		return *this;
	}
	void SwapXY() {
		int temp = x_;
		x_ = y_;
		y_ = temp;
	}
	int x_;
	int y_;
};

#endif