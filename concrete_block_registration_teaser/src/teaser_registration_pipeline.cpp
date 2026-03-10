#include "concrete_block_registration_teaser/teaser_registration_pipeline.hpp"

#include <algorithm>
#include <numeric>
#include <random>
#include <unordered_map>

#include <teaser/registration.h>

using namespace open3d;
using namespace pcd_block;

namespace concrete_block_perception
{

TeaserRegistrationPipeline::TeaserRegistrationPipeline(
  const Eigen::Matrix4d & T_P_C,
  const Eigen::Matrix3d & K,
  const std::vector<pcd_block::TemplateData> & templates,
  const PreprocessingParams & pre,
  const LocalRegistrationParams & loc,
  const TeaserRegistrationParams & teaser,
  const rclcpp::Logger & logger,
  bool verbose_logs)
: T_P_C_(T_P_C),
  K_(K),
  templates_(templates),
  pre_(pre),
  loc_(loc),
  teaser_(teaser),
  logger_(logger),
  verbose_logs_(verbose_logs)
{
}

RegistrationOutput TeaserRegistrationPipeline::run(const RegistrationInput & in)
{
  RegistrationOutput out;

  geometry::PointCloud cutout;
  if (!computeCutout(in.scene, in.mask, cutout)) {
    out.failure_stage = "cutout";
    out.failure_reason = "no points selected from mask";
    return out;
  }

  preprocess(cutout, in.T_world_cloud);
  out.debug_scene = cutout;

  if (cutout.points_.empty()) {
    out.failure_stage = "preprocess";
    out.failure_reason = "all points removed during preprocess";
    return out;
  }

  if (!cutout.HasNormals()) {
    cutout.EstimateNormals(geometry::KDTreeSearchParamHybrid(0.02, 30));
  }

  double best_fitness = -1.0;
  double best_rmse = std::numeric_limits<double>::infinity();
  int best_template_idx = -1;
  Eigen::Matrix4d best_T = Eigen::Matrix4d::Identity();

  for (size_t ti = 0; ti < templates_.size(); ++ti) {
    const auto & tpl = templates_[ti];
    if (!tpl.pcd || tpl.pcd->points_.empty()) {
      continue;
    }

    geometry::PointCloud tpl_cloud = *tpl.pcd;
    if (!tpl_cloud.HasNormals()) {
      tpl_cloud.EstimateNormals(geometry::KDTreeSearchParamHybrid(0.02, 30));
    }

    if (tpl_cloud.points_.size() > teaser_.max_template_points) {
      std::vector<size_t> idx(tpl_cloud.points_.size());
      std::iota(idx.begin(), idx.end(), 0);
      std::shuffle(idx.begin(), idx.end(), std::mt19937{42});
      idx.resize(teaser_.max_template_points);
      tpl_cloud = *tpl_cloud.SelectByIndex(idx);
      tpl_cloud.EstimateNormals(geometry::KDTreeSearchParamHybrid(0.02, 30));
    }

    Eigen::Matrix<double, 3, Eigen::Dynamic> src_corr;
    Eigen::Matrix<double, 3, Eigen::Dynamic> dst_corr;
    if (!buildNearestNeighborCorrespondences(tpl_cloud, cutout, src_corr, dst_corr)) {
      if (verbose_logs_) {
        RCLCPP_INFO(
          logger_,
          "TEASER skip template %zu: insufficient correspondences",
          ti);
      }
      continue;
    }

    teaser::RobustRegistrationSolver::Params params;
    params.noise_bound = teaser_.noise_bound;
    params.cbar2 = teaser_.cbar2;
    params.estimate_scaling = teaser_.estimate_scaling;
    params.rotation_estimation_algorithm = teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.rotation_gnc_factor = teaser_.rotation_gnc_factor;
    params.rotation_max_iterations = teaser_.rotation_max_iterations;
    params.rotation_cost_threshold = teaser_.rotation_cost_threshold;
    params.inlier_selection_mode = teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_EXACT;
    params.max_clique_time_limit = teaser_.max_clique_time_limit_s;

    teaser::RobustRegistrationSolver solver(params);
    solver.solve(src_corr, dst_corr);
    const auto sol = solver.getSolution();

    if (!sol.valid) {
      if (verbose_logs_) {
        RCLCPP_INFO(logger_, "TEASER invalid solution for template %zu", ti);
      }
      continue;
    }

    Eigen::Matrix4d T_candidate = makeTransform(sol.rotation, sol.translation);

    if (teaser_.enable_icp_refinement) {
      const auto icp_result = pipelines::registration::RegistrationICP(
        tpl_cloud,
        cutout,
        teaser_.icp_refine_dist,
        T_candidate,
        pipelines::registration::TransformationEstimationPointToPlane(),
        pipelines::registration::ICPConvergenceCriteria());
      T_candidate = icp_result.transformation_;
    }

    const auto eval = pipelines::registration::EvaluateRegistration(
      tpl_cloud,
      cutout,
      teaser_.eval_corr_dist,
      T_candidate);

    const bool better =
      (eval.fitness_ > best_fitness) ||
      ((std::abs(eval.fitness_ - best_fitness) < 1e-9) && eval.inlier_rmse_ < best_rmse);

    if (better) {
      best_fitness = eval.fitness_;
      best_rmse = eval.inlier_rmse_;
      best_template_idx = static_cast<int>(ti);
      best_T = T_candidate;
    }
  }

  if (best_template_idx < 0) {
    out.failure_stage = "teaser_registration";
    out.failure_reason = "no valid template pose from TEASER";
    return out;
  }

  out.success = true;
  out.T_world_block = best_T;
  out.fitness = best_fitness;
  out.rmse = best_rmse;
  out.template_index = best_template_idx;
  out.debug_scene = cutout;
  return out;
}

bool TeaserRegistrationPipeline::computeCutout(
  const geometry::PointCloud & scene,
  const cv::Mat & mask,
  geometry::PointCloud & cutout)
{
  auto pts = select_points_by_mask(scene.points_, mask, K_, T_P_C_);
  if (pts.empty()) {
    return false;
  }

  cutout.points_ = pts;
  cutout.EstimateNormals();
  return true;
}

void TeaserRegistrationPipeline::preprocess(
  geometry::PointCloud & cutout,
  const Eigen::Matrix4d & T_world_cloud)
{
  std::shared_ptr<geometry::PointCloud> pcd;
  std::vector<size_t> ind;

  std::tie(pcd, ind) = cutout.RemoveStatisticalOutliers(pre_.nb_neighbors, pre_.std_dev);
  cutout = *pcd;

  if (pre_.enable_cluster_filter) {
    keepDominantCluster(cutout);
  }

  if (cutout.points_.size() > pre_.max_pts) {
    std::vector<size_t> idx(cutout.points_.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::shuffle(idx.begin(), idx.end(), std::mt19937{42});
    idx.resize(pre_.max_pts);
    cutout = *cutout.SelectByIndex(idx);
  }

  cutout.Transform(T_world_cloud);
  cutout.EstimateNormals(geometry::KDTreeSearchParamHybrid(0.02, 30));
}

bool TeaserRegistrationPipeline::keepDominantCluster(geometry::PointCloud & cutout)
{
  if (cutout.points_.empty()) {
    return false;
  }

  const auto labels = cutout.ClusterDBSCAN(pre_.cluster_eps, pre_.cluster_min_points, false);
  if (labels.empty()) {
    return false;
  }

  std::unordered_map<int, size_t> counts;
  for (const int label : labels) {
    if (label >= 0) {
      ++counts[label];
    }
  }

  if (counts.empty()) {
    return false;
  }

  int best_label = -1;
  size_t best_count = 0;
  for (const auto & [label, count] : counts) {
    if (count > best_count) {
      best_label = label;
      best_count = count;
    }
  }

  if (best_label < 0 || best_count < static_cast<size_t>(pre_.cluster_min_size)) {
    return false;
  }

  std::vector<size_t> keep_indices;
  keep_indices.reserve(best_count);
  for (size_t i = 0; i < labels.size(); ++i) {
    if (labels[i] == best_label) {
      keep_indices.push_back(i);
    }
  }

  cutout = *cutout.SelectByIndex(keep_indices);
  return true;
}

bool TeaserRegistrationPipeline::buildNearestNeighborCorrespondences(
  const geometry::PointCloud & source_tpl,
  const geometry::PointCloud & target_scene,
  Eigen::Matrix<double, 3, Eigen::Dynamic> & src_corr,
  Eigen::Matrix<double, 3, Eigen::Dynamic> & dst_corr) const
{
  if (source_tpl.points_.empty() || target_scene.points_.empty()) {
    return false;
  }

  geometry::KDTreeFlann tree(target_scene);
  std::vector<Eigen::Vector3d> src_points;
  std::vector<Eigen::Vector3d> dst_points;
  src_points.reserve(source_tpl.points_.size());
  dst_points.reserve(source_tpl.points_.size());

  std::vector<int> indices(1);
  std::vector<double> dists2(1);
  const double max_dist2 = teaser_.nn_corr_max_dist * teaser_.nn_corr_max_dist;

  for (const auto & p_src : source_tpl.points_) {
    if (tree.SearchKNN(p_src, 1, indices, dists2) > 0) {
      if (dists2[0] <= max_dist2) {
        src_points.push_back(p_src);
        dst_points.push_back(target_scene.points_.at(static_cast<size_t>(indices[0])));
      }
    }
  }

  if (src_points.size() < teaser_.min_correspondences) {
    return false;
  }

  src_corr.resize(3, static_cast<int>(src_points.size()));
  dst_corr.resize(3, static_cast<int>(dst_points.size()));

  for (size_t i = 0; i < src_points.size(); ++i) {
    src_corr.col(static_cast<int>(i)) = src_points[i];
    dst_corr.col(static_cast<int>(i)) = dst_points[i];
  }

  return true;
}

Eigen::Matrix4d TeaserRegistrationPipeline::makeTransform(
  const Eigen::Matrix3d & R,
  const Eigen::Vector3d & t) const
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;
  return T;
}

}  // namespace concrete_block_perception
