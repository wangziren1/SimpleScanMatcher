#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include <visualization_msgs/Marker.h>

#include <cmath>
#include <vector>
#include <memory>
#include <iostream>
#include "scanmatch/correlative_scan_match.h"
#include "scanmatch/map.h"

using namespace std;

class Node {
 public:
	Node();
	
 private:
  void CallBack(const sensor_msgs::LaserScan::ConstPtr& scan);
  vector<Point> GetPointCloud(const sensor_msgs::LaserScan::ConstPtr& scan);
	vector<Point> TransformPointCloudFromLaserToWorld(const Pose& pose, const vector<Point>& 
			point_cloud);
	void Visualize(const vector<Point>& point_cloud);

  ros::NodeHandle n_;
	ros::Subscriber sub_;
	ros::Publisher trajectory_pub_;
	ros::Publisher map_pub_;
	ros::Publisher point_cloud_pub_;
	vector<Pose> poses_;
	std::shared_ptr<Map> map_;
	CorrelativeScanMatcher correlative_scan_matcher_;
	bool first_scan_;
};

Node::Node() {
	sub_ = n_.subscribe("scan", 100, &Node::CallBack, this);
	trajectory_pub_ = n_.advertise<visualization_msgs::Marker>("trajectory", 50);
	map_pub_ = n_.advertise<visualization_msgs::Marker>("map", 50);
	point_cloud_pub_ = n_.advertise<visualization_msgs::Marker>("point_cloud", 50);
	first_scan_ = true;
	// initial pose
	poses_.push_back(Pose(0, 0, 0));
}

void Node::CallBack(const sensor_msgs::LaserScan::ConstPtr& scan) {
	if (first_scan_) {
		vector<Point> point_cloud = GetPointCloud(scan);
		map_ = std::make_shared<Map>(point_cloud);
		first_scan_ = false;
	} else {
		Pose initial_pose(poses_.back());
		vector<Point> point_cloud = GetPointCloud(scan);
		Pose pose = correlative_scan_matcher_.ComputePose(initial_pose,
				point_cloud, map_);
		poses_.push_back(pose);
		vector<Point> world_point_cloud = TransformPointCloudFromLaserToWorld(pose,
				point_cloud);
		Visualize(world_point_cloud);
		map_ = std::make_shared<Map>(world_point_cloud);
	}
}

vector<Point> Node::GetPointCloud(const sensor_msgs::LaserScan::ConstPtr& 
		scan) {
	vector<Point> point_cloud;
	for (float angle = scan->angle_min, i = 0; angle <= scan->angle_max;
			angle+=scan->angle_increment, i++) {
		if (scan->ranges[i] > scan->range_min && 
				scan->ranges[i] < scan->range_max) {
			point_cloud.push_back(Point{scan->ranges[i] * cos(angle),
					scan->ranges[i] * sin(angle)});
		}
	}
	return point_cloud;
}

vector<Point> Node::TransformPointCloudFromLaserToWorld(const Pose& pose, 
		const vector<Point>& point_cloud) {
	vector<Point> trans_point_cloud;
	for (auto& point : point_cloud) {
		float x = cos(pose.theta_) * point.x_ - sin(pose.theta_) * point.y_ + 
				pose.x_;
		float y = sin(pose.theta_) * point.x_ + cos(pose.theta_) * point.y_ + 
				pose.y_;
		trans_point_cloud.push_back(Point(x, y));
	}
	return trans_point_cloud;
}

void Node::Visualize(const vector<Point>& point_cloud) {
	visualization_msgs::Marker trajectory_line, map_points, current_points;
	trajectory_line.header.frame_id = map_points.header.frame_id = 
			current_points.header.frame_id = "/my_frame";
	trajectory_line.header.stamp = map_points.header.stamp = 
			current_points.header.stamp = ros::Time::now();
	trajectory_line.ns = map_points.ns = current_points.ns = "my_namespace";
	trajectory_line.action = map_points.action = current_points.action = 
			visualization_msgs::Marker::ADD;
	trajectory_line.pose.orientation.w = map_points.pose.orientation.w = 
			current_points.pose.orientation.w = 1.0;
	
	trajectory_line.id = 0;
	map_points.id = 1;
	current_points.id = 2;

	trajectory_line.type = visualization_msgs::Marker::LINE_STRIP;
	map_points.type = visualization_msgs::Marker::POINTS;
	current_points.type = visualization_msgs::Marker::POINTS;

	trajectory_line.scale.x = 0.05;
	map_points.scale.x = 0.02;
	map_points.scale.y = 0.02;
	current_points.scale.x = 0.02;
	current_points.scale.y = 0.02;

	trajectory_line.color.r = 0.54;
  trajectory_line.color.g = 0.41;
	trajectory_line.color.b = 0.8;
	trajectory_line.color.a = 0.8;

	map_points.color.r = 1;
	map_points.color.g = 1;
	map_points.color.b = 1;
	map_points.color.a = 1;

	current_points.color.r = 1;
	current_points.color.a = 1;

	for (auto& pose : poses_) {
		geometry_msgs::Point p;
		p.x = pose.x_;
		p.y = pose.y_;
		p.z = 0;
		trajectory_line.points.push_back(p);
	}

	for (auto& point : map_->PointCloud()) {
		geometry_msgs::Point p;
		p.x = point.x_;
		p.y = point.y_;
		p.z = 0;
		map_points.points.push_back(p);
	}

	for (auto& point : point_cloud) {
		geometry_msgs::Point p;
		p.x = point.x_;
		p.y = point.y_;
		p.z = 0;
		current_points.points.push_back(p);
	}

	trajectory_pub_.publish(trajectory_line);
	map_pub_.publish(map_points);
	point_cloud_pub_.publish(current_points);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "scan_matcher");
	Node node;
	ros::spin();

	return 0;
}