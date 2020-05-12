#include "scanmatch/optimization_scan_match.h"
#include <iostream>
#include <iomanip>
using namespace std;

OptimizationScanMatch::OptimizationScanMatch() {
  ceres_solver_options_.use_nonmonotonic_steps = false;
  ceres_solver_options_.max_num_iterations = 20;
  ceres_solver_options_.num_threads = 1;
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
}

Pose OptimizationScanMatch::Match(const Pose& initial_pose, 
    const vector<Point>& point_cloud, const Map& map) {
  double ceres_pose_estimate[3] = {initial_pose.x_,
                                   initial_pose.y_,
                                   initial_pose.theta_};
  ceres::Problem problem;
  problem.AddResidualBlock(
      OccupiedSpaceCostFunction2D::CreateAutoDiffCostFunction(
          1 / std::sqrt(static_cast<double>(point_cloud.size())),
          point_cloud, map),
      nullptr /* loss function */, ceres_pose_estimate);
  problem.AddResidualBlock(
      TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          10, Eigen::Vector2d(ceres_pose_estimate[0], ceres_pose_estimate[1])),
      nullptr /* loss function */, ceres_pose_estimate);
  problem.AddResidualBlock(
      RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          40, ceres_pose_estimate[2]),
      nullptr /* loss function */, ceres_pose_estimate);

  ceres::Solver::Summary summary;
  ceres::Solve(ceres_solver_options_, &problem, &summary);
  Pose optimized_pose(ceres_pose_estimate[0], ceres_pose_estimate[1], 
      ceres_pose_estimate[2]);
  cout << "ceres pose: " << optimized_pose << endl;
  cout << summary.BriefReport() << "\n";

  return optimized_pose;
}

void OptimizationScanMatch::Test(const vector<Point>& world_point_cloud, 
    const Map& map) {
  const GridArrayAdapter adapter(map);
  ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
  for (const auto& point : world_point_cloud) {
    double value;
    Point grid_point = map.GetFullGridCoordinate(point);
    interpolator.Evaluate(double(grid_point.y_), double(grid_point.x_),
        &value);
    GridPoint p(int(grid_point.x_), int(grid_point.y_));
    cout << "interpolator:" << endl;
    for (int j = -1; j < 3; ++j) {
      for (int i = -1; i < 3; ++i) {
        GridPoint grid_p(p.x_ + i, p.y_ + j);
        cout << grid_p << " ";
      }
      for (int i = -1; i < 3; ++i) {
        GridPoint grid_p(p.x_ + i, p.y_ + j);
        cout << map.GetCost(grid_p) << " ";
      }
      cout << "\n";
    }
    cout << grid_point << " " << value << endl;
  }
}