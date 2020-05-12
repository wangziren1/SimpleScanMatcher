#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/OccupancyGrid.h"

#include <cmath>
#include <vector>
#include <memory>
#include <iostream>
#include "scanmatch/correlative_scan_match.h"
#include "scanmatch/optimization_scan_match.h"
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
	void MapCallBack(const ros::WallTimerEvent&);
  ros::NodeHandle n_;
	ros::Subscriber sub_;
	ros::Publisher trajectory_pub_;
	ros::Publisher map_pub_;
	ros::WallTimer map_timer_;
	ros::Publisher point_cloud_pub_;
	vector<Pose> poses_;
	Map map_;
	CorrelativeScanMatcher correlative_scan_matcher_;
	OptimizationScanMatch optimization_Scan_Match_;
	bool first_scan_;
};

Node::Node() {
	sub_ = n_.subscribe("scan", 100, &Node::CallBack, this);
	trajectory_pub_ = n_.advertise<visualization_msgs::Marker>("trajectory", 50);
	point_cloud_pub_ = n_.advertise<visualization_msgs::Marker>("point_cloud", 50);
	map_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("grid_map", 50);
	map_timer_ = n_.createWallTimer(ros::WallDuration(1.0), &Node::MapCallBack, this);
	first_scan_ = true;
	// initial pose
	poses_.push_back(Pose(0, 0, 0));
}

void Node::CallBack(const sensor_msgs::LaserScan::ConstPtr& scan) {
	if (first_scan_) {
		vector<Point> point_cloud = GetPointCloud(scan);
		map_.Update(poses_.back(), point_cloud);
		first_scan_ = false;
	} else {
		Pose initial_pose(poses_.back());
		vector<Point> point_cloud = GetPointCloud(scan);
		Pose pose = correlative_scan_matcher_.ComputePose(initial_pose,
				point_cloud, map_);
		// Pose ceres_pose = optimization_Scan_Match_.Match(initial_pose, point_cloud, map_);
		poses_.push_back(pose);
		vector<Point> world_point_cloud = TransformPointCloudFromLaserToWorld(
				pose, point_cloud);
		// optimization_Scan_Match_.Test(world_point_cloud, map_);
		map_.Update(pose, world_point_cloud);
		Visualize(world_point_cloud);
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
	visualization_msgs::Marker trajectory_line, current_points;
	trajectory_line.header.frame_id = 
			current_points.header.frame_id = "/my_frame";
	trajectory_line.header.stamp = 
			current_points.header.stamp = ros::Time::now();
	trajectory_line.ns = current_points.ns = "my_namespace";
	trajectory_line.action = current_points.action = 
			visualization_msgs::Marker::ADD;
	trajectory_line.pose.orientation.w = 
			current_points.pose.orientation.w = 1.0;
	
	trajectory_line.id = 0;
	current_points.id = 2;

	trajectory_line.type = visualization_msgs::Marker::LINE_STRIP;
	current_points.type = visualization_msgs::Marker::POINTS;

	trajectory_line.scale.x = 0.05;
	current_points.scale.x = 0.02;
	current_points.scale.y = 0.02;

	trajectory_line.color.r = 0.54;
  trajectory_line.color.g = 0.41;
	trajectory_line.color.b = 0.8;
	trajectory_line.color.a = 0.8;

	current_points.color.r = 1;
	current_points.color.a = 1;

	for (auto& pose : poses_) {
		geometry_msgs::Point p;
		p.x = pose.x_;
		p.y = pose.y_;
		p.z = 0;
		trajectory_line.points.push_back(p);
	}

	for (auto& point : point_cloud) {
		geometry_msgs::Point p;
		p.x = point.x_;
		p.y = point.y_;
		p.z = 0;
		current_points.points.push_back(p);
	}

	trajectory_pub_.publish(trajectory_line);
	point_cloud_pub_.publish(current_points);
}

void Node::MapCallBack(const ros::WallTimerEvent&) {
	const auto& discrete_map = map_.DiscreteProbabilityMap();
	nav_msgs::OccupancyGrid map_msg;
	map_msg.header.frame_id = "/my_frame";
	map_msg.header.stamp = ros::Time::now();
	map_msg.info.resolution = map_.Resolution();
	map_msg.info.width = discrete_map[0].size();
	map_msg.info.height = discrete_map.size();
	map_msg.info.origin.orientation.w = 1.0;
	map_msg.info.origin.position.x = -map_.Origin().x_;
	map_msg.info.origin.position.y = -map_.Origin().y_;
	map_msg.info.origin.position.z = 0;
	map_msg.data.reserve(discrete_map[0].size() * discrete_map.size());
	for (const auto& row : discrete_map) {
		for (const auto& p : row) {
			map_msg.data.push_back(p);
		}
	}
	map_pub_.publish(map_msg);
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "scan_matcher");
	Node node;
	ros::spin();

	return 0;
}