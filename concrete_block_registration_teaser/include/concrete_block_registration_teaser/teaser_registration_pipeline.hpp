#pragma once

#include <open3d/Open3D.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include "pcd_block_estimation/mask_projection.hpp"
#include "pcd_block_estimation/template_utils.hpp"
#include "concrete_block_perception/registration/block_registration_pipeline.hpp"

namespace concrete_block_perception
{

class TeaserRegistrationPipeline
{
public:
  TeaserRegistrationPipeline(
    const Eigen::Matrix4d & T_P_C,
    const Eigen::Matrix3d & K,
    const std::vector<pcd_block::TemplateData> & templates,
    const PreprocessingParams & pre,
    const LocalRegistrationParams & loc,
    const TeaserRegistrationParams & teaser,
    const rclcpp::Logger & logger,
    bool verbose_logs);

  RegistrationOutput run(const RegistrationInput & in);

private:
  bool computeCutout(
    const open3d::geometry::PointCloud & scene,
    const cv::Mat & mask,
    open3d::geometry::PointCloud & cutout);

  void preprocess(
    open3d::geometry::PointCloud & cutout,
    const Eigen::Matrix4d & T_world_cloud);

  bool keepDominantCluster(open3d::geometry::PointCloud & cutout);

  bool buildNearestNeighborCorrespondences(
    const open3d::geometry::PointCloud & source_tpl,
    const open3d::geometry::PointCloud & target_scene,
    Eigen::Matrix<double, 3, Eigen::Dynamic> & src_corr,
    Eigen::Matrix<double, 3, Eigen::Dynamic> & dst_corr) const;

  Eigen::Matrix4d makeTransform(
    const Eigen::Matrix3d & R,
    const Eigen::Vector3d & t) const;

  Eigen::Matrix4d T_P_C_;
  Eigen::Matrix3d K_;
  std::vector<pcd_block::TemplateData> templates_;

  PreprocessingParams pre_;
  LocalRegistrationParams loc_;
  TeaserRegistrationParams teaser_;
  rclcpp::Logger logger_;
  bool verbose_logs_{false};
};

}  // namespace concrete_block_perception
