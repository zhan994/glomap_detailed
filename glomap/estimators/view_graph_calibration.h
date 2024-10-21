#pragma once

#include "glomap/estimators/optimization_base.h"
#include "glomap/scene/types_sfm.h"

#include <memory>

namespace glomap {

struct ViewGraphCalibratorOptions : public OptimizationBaseOptions {
  // The minimal ratio of the estimated focal length against the prior focal
  // length
  double thres_lower_ratio = 0.1;
  // The maximal ratio of the estimated focal length against the prior focal
  // length
  double thres_higher_ratio = 10;

  // The threshold for the corresponding error in the problem for an image pair
  double thres_two_view_error = 2.;

  ViewGraphCalibratorOptions() : OptimizationBaseOptions() {
    thres_loss_function = 1e-2;
  }
};

// api: view_graph 标定类
class ViewGraphCalibrator {
 public:
  ViewGraphCalibrator(const ViewGraphCalibratorOptions& options)
      : options_(options) {}

  // Entry point for the calibration
  // api: 求解自标定
  bool Solve(ViewGraph& view_graph,
             std::unordered_map<camera_t, Camera>& cameras,
             std::unordered_map<image_t, Image>& images);

 private:
  // Reset the problem
  // api: 重置
  void Reset(const std::unordered_map<camera_t, Camera>& cameras);

  // Add the image pairs to the problem
  // api: 添加图像对观测数据
  void AddImagePairsToProblem(
      const ViewGraph& view_graph,
      const std::unordered_map<camera_t, Camera>& cameras,
      const std::unordered_map<image_t, Image>& images);

  // Add a single image pair to the problem
  // api: 添加一个图像对观测数据
  void AddImagePair(const ImagePair& image_pair,
                    const std::unordered_map<camera_t, Camera>& cameras,
                    const std::unordered_map<image_t, Image>& images);

  // Set the cameras to be constant if they have prior intrinsics
  // api: 设置参数块，若有先验内参，则为常量参数块
  size_t ParameterizeCameras(
      const std::unordered_map<camera_t, Camera>& cameras);

  // Convert the results back to the camera
  // api: 将结果数据拷贝进相机参数中
  void CopyBackResults(std::unordered_map<camera_t, Camera>& cameras);

  // Filter the image pairs based on the calibration results
  // api: 根据标定结果过滤图像对
  size_t FilterImagePairs(ViewGraph& view_graph) const;

  ViewGraphCalibratorOptions options_;
  std::unique_ptr<ceres::Problem> problem_;
  std::unordered_map<camera_t, double> focals_;
};

}  // namespace glomap
