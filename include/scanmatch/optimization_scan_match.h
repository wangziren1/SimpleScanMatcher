#include "scanmatch/pose_and_point.h"
#include "scanmatch/map.h"
#include "ceres/cubic_interpolation.h"
#include "ceres/ceres.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

class GridArrayAdapter {
 public:
  enum { DATA_DIMENSION = 1 };

  explicit GridArrayAdapter(const Map& map) : map_(map) {}

  void GetValue(const int row, const int column, double* const value) const {
      *value = static_cast<double>(map_.GetCost(GridPoint(column, row)));
  }

  int NumRows() const {return map_.Height();}

  int NumCols() const {return map_.Width();}

 private:
  const Map& map_;
};

class OccupiedSpaceCostFunction2D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const vector<Point>& point_cloud,
      const Map& map) {
    return new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunction2D,
                                           ceres::DYNAMIC /* residuals */,
                                           3 /* pose variables */>(
        new OccupiedSpaceCostFunction2D(scaling_factor, point_cloud, map),
        point_cloud.size());
  }

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);
    
    const GridArrayAdapter adapter(map_);
    ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
    for (size_t i = 0; i < point_cloud_.size(); ++i) {
      const Eigen::Matrix<T, 3, 1> point(T(point_cloud_[i].x_),
                                         T(point_cloud_[i].y_), T(1.));
      const Eigen::Matrix<T, 3, 1> world_point = transform * point;
      
      interpolator.Evaluate((world_point[1] + (double)map_.OffSetY()) / 
            (double)map_.Resolution(),
          (world_point[0] + (double)map_.OffSetX()) / (double)map_.
            Resolution(), 
          &residual[i]);
      residual[i] = scaling_factor_ * residual[i];
    }
    return true;
  }

 private:
  OccupiedSpaceCostFunction2D(const double scaling_factor,
                              const vector<Point>& point_cloud,
                              const Map& map)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        map_(map) {}

  OccupiedSpaceCostFunction2D(const OccupiedSpaceCostFunction2D&) = delete;
  OccupiedSpaceCostFunction2D& operator=(const OccupiedSpaceCostFunction2D&) =
      delete;

  const double scaling_factor_;
  const vector<Point>& point_cloud_;
  const Map& map_;
};

// Computes the cost of translating 'pose' to 'target_translation'.
// Cost increases with the solution's distance from 'target_translation'.
class TranslationDeltaCostFunctor2D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const Eigen::Vector2d& target_translation) {
    return new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor2D,
                                           2 /* residuals */,
                                           3 /* pose variables */>(
        new TranslationDeltaCostFunctor2D(scaling_factor, target_translation));
  }

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = scaling_factor_ * (pose[0] - x_);
    residual[1] = scaling_factor_ * (pose[1] - y_);
    return true;
  }

 private:
  // Constructs a new TranslationDeltaCostFunctor2D from the given
  // 'target_translation' (x, y).
  explicit TranslationDeltaCostFunctor2D(
      const double scaling_factor, const Eigen::Vector2d& target_translation)
      : scaling_factor_(scaling_factor),
        x_(target_translation.x()),
        y_(target_translation.y()) {}

  TranslationDeltaCostFunctor2D(const TranslationDeltaCostFunctor2D&) = delete;
  TranslationDeltaCostFunctor2D& operator=(
      const TranslationDeltaCostFunctor2D&) = delete;

  const double scaling_factor_;
  const double x_;
  const double y_;
};

// Computes the cost of rotating 'pose' to 'target_angle'. Cost increases with
// the solution's distance from 'target_angle'.
class RotationDeltaCostFunctor2D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const double target_angle) {
    return new ceres::AutoDiffCostFunction<
        RotationDeltaCostFunctor2D, 1 /* residuals */, 3 /* pose variables */>(
        new RotationDeltaCostFunctor2D(scaling_factor, target_angle));
  }

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = scaling_factor_ * (pose[2] - angle_);
    return true;
  }

 private:
  explicit RotationDeltaCostFunctor2D(const double scaling_factor,
                                      const double target_angle)
      : scaling_factor_(scaling_factor), angle_(target_angle) {}

  RotationDeltaCostFunctor2D(const RotationDeltaCostFunctor2D&) = delete;
  RotationDeltaCostFunctor2D& operator=(const RotationDeltaCostFunctor2D&) =
      delete;

  const double scaling_factor_;
  const double angle_;
};

class OptimizationScanMatch {
 public:
  OptimizationScanMatch();
  Pose Match(const Pose& initial_pose, const vector<Point>& point_cloud, 
      const Map& map);
  void Test(const vector<Point>& world_point_cloud, 
      const Map& map);
 private:
  ceres::Solver::Options ceres_solver_options_;
};
