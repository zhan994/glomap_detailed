#include "glomap/estimators/view_graph_calibration.h"

#include "glomap/estimators/cost_function.h"
#include "glomap/math/two_view_geometry.h"

#include <colmap/scene/two_view_geometry.h>

#include <thread>

namespace glomap {

bool ViewGraphCalibrator::Solve(ViewGraph& view_graph,
                                std::unordered_map<camera_t, Camera>& cameras,
                                std::unordered_map<image_t, Image>& images) {
  // Reset the problem
  LOG(INFO) << "Start ViewGraphCalibrator";

  // step: 1 重置相机相关参数
  Reset(cameras);

  // Set the solver options.
  // step: 2 设置求解方式
  if (cameras.size() < 50)
    options_.solver_options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  else
    options_.solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  // Add the image pairs into the problem
  // step: 3 添加图像对
  AddImagePairsToProblem(view_graph, cameras, images);

  // Set the cameras to be constant if they have prior intrinsics
  // step: 4 设置为常量，如果有先验内参
  const size_t num_cameras = ParameterizeCameras(cameras);

  if (num_cameras == 0) {
    LOG(INFO) << "No cameras to optimize";
    return true;
  }

  // Solve the problem
  // step: 5 求解
  ceres::Solver::Summary summary;
  options_.solver_options.minimizer_progress_to_stdout = VLOG_IS_ON(2);
  ceres::Solve(options_.solver_options, problem_.get(), &summary);

  VLOG(2) << summary.FullReport();

  // Convert the results back to the camera
  // step: 6 拷贝参数并过滤图像对
  CopyBackResults(cameras);
  FilterImagePairs(view_graph);

  return summary.IsSolutionUsable();
}

void ViewGraphCalibrator::Reset(
    const std::unordered_map<camera_t, Camera>& cameras) {
  std::cout << "vgcalib cameras size: " << cameras.size() << std::endl;
  // Initialize the problem
  // step: 1 初始化参数
  focals_.clear();
  focals_.reserve(cameras.size());
  for (const auto& [camera_id, camera] : cameras) {
    std::cout << "vgcalib camera.Focal: " << camera.Focal() << std::endl;
    focals_[camera_id] = camera.Focal();
  }

  // Set up the problem
  // step: 2 设置优化问题ceres::Problem
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_ = std::make_unique<ceres::Problem>(problem_options);
  options_.loss_function =
      std::make_shared<ceres::CauchyLoss>(options_.thres_loss_function);
}

void ViewGraphCalibrator::AddImagePairsToProblem(
    const ViewGraph& view_graph,
    const std::unordered_map<camera_t, Camera>& cameras,
    const std::unordered_map<image_t, Image>& images) {
  for (auto& [image_pair_id, image_pair] : view_graph.image_pairs) {
    // note: 只添加 CALIBRATED 和 UNCALIBRATED 以及 is_valid=true
    if (image_pair.config != colmap::TwoViewGeometry::CALIBRATED &&
        image_pair.config != colmap::TwoViewGeometry::UNCALIBRATED)
      continue;
    if (image_pair.is_valid == false) continue;

    AddImagePair(image_pair, cameras, images);
  }
}

void ViewGraphCalibrator::AddImagePair(
    const ImagePair& image_pair,
    const std::unordered_map<camera_t, Camera>& cameras,
    const std::unordered_map<image_t, Image>& images) {
  // step: 1 获取相机id
  const camera_t camera_id1 = images.at(image_pair.image_id1).camera_id;
  const camera_t camera_id2 = images.at(image_pair.image_id2).camera_id;

  // step: 2 同一 / 不同 相机的残差模块
  if (camera_id1 == camera_id2) {
    // note: 一般为 single_camera

    problem_->AddResidualBlock(
        FetzerFocalLengthSameCameraCost::Create(
            image_pair.F, cameras.at(camera_id1).PrincipalPoint()),
        options_.loss_function.get(),
        &(focals_[camera_id1]));
  } else {
    problem_->AddResidualBlock(
        FetzerFocalLengthCost::Create(image_pair.F,
                                      cameras.at(camera_id1).PrincipalPoint(),
                                      cameras.at(camera_id2).PrincipalPoint()),
        options_.loss_function.get(),
        &(focals_[camera_id1]),
        &(focals_[camera_id2]));
  }
}

size_t ViewGraphCalibrator::ParameterizeCameras(
    const std::unordered_map<camera_t, Camera>& cameras) {
  size_t num_cameras = 0;

  for (auto& [camera_id, camera] : cameras) {
    // step: 1 判断是否存在
    if (!problem_->HasParameterBlock(&(focals_[camera_id]))) continue;

    // step: 2 设置lower_bound，相机+1
    num_cameras++;
    problem_->SetParameterLowerBound(&(focals_[camera_id]), 0, 1e-3);

    // step: 3 有先验focal_len，参数块设为常量，并相机-1
    if (camera.has_prior_focal_length) {
      problem_->SetParameterBlockConstant(&(focals_[camera_id]));
      num_cameras--;
    }
  }

  return num_cameras;
}

void ViewGraphCalibrator::CopyBackResults(
    std::unordered_map<camera_t, Camera>& cameras) {
  size_t counter = 0;
  for (auto& [camera_id, camera] : cameras) {
    // step: 1 是否有该参数块
    if (!problem_->HasParameterBlock(&(focals_[camera_id]))) continue;

    // if the estimated parameter is too crazy, reject it
    // step: 2 判断该参数是否合理
    if (focals_[camera_id] / camera.Focal() > options_.thres_higher_ratio ||
        focals_[camera_id] / camera.Focal() < options_.thres_lower_ratio) {
      VLOG(2) << "Ignoring degenerate camera camera " << camera_id
              << " focal: " << focals_[camera_id]
              << " original focal: " << camera.Focal();
      counter++;

      continue;
    }

    // Marke that the camera has refined intrinsics
    // step: 3 标记参数优化并更新参数
    camera.has_refined_focal_length = true;
    // Update the focal length
    for (const size_t idx : camera.FocalLengthIdxs()) {
      camera.params[idx] = focals_[camera_id];
    }
  }
  LOG(INFO) << counter << " cameras are rejected in view graph calibration";
}

size_t ViewGraphCalibrator::FilterImagePairs(ViewGraph& view_graph) const {
  // step: 1 建立eval评估残差
  ceres::Problem::EvaluateOptions eval_options;
  eval_options.num_threads = options_.solver_options.num_threads;
  eval_options.apply_loss_function = false;
  std::vector<double> residuals;
  problem_->Evaluate(eval_options, nullptr, &residuals, nullptr, nullptr);

  // Dump the residuals into the original data structure
  // step: 2 统计有效图像对
  size_t counter = 0;
  size_t invalid_counter = 0;
  const double thres_two_view_error_sq =
      options_.thres_two_view_error * options_.thres_two_view_error;
  for (auto& [image_pair_id, image_pair] : view_graph.image_pairs) {
    // step: 2.1 图像对是否有效
    if (image_pair.config != colmap::TwoViewGeometry::CALIBRATED &&
        image_pair.config != colmap::TwoViewGeometry::UNCALIBRATED)
      continue;
    if (image_pair.is_valid == false) continue;

    // step: 2.2 误差过滤
    const Eigen::Vector2d error(residuals[counter], residuals[counter + 1]);
    // Set the two view geometry to be invalid if the error is too high
    if (error.squaredNorm() > thres_two_view_error_sq) {
      invalid_counter++;
      image_pair.is_valid = false;
    }

    counter += 2;
  }

  LOG(INFO) << "invalid / total number of two view geometry: "
            << invalid_counter << " / " << counter;

  return invalid_counter;
}

}  // namespace glomap
