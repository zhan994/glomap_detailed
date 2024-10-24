#pragma once

#include "glomap/controllers/track_establishment.h"
#include "glomap/controllers/track_retriangulation.h"
#include "glomap/estimators/bundle_adjustment.h"
#include "glomap/estimators/global_positioning.h"
#include "glomap/estimators/global_rotation_averaging.h"
#include "glomap/estimators/relpose_estimation.h"
#include "glomap/estimators/view_graph_calibration.h"
#include "glomap/types.h"

#include <colmap/scene/database.h>

namespace glomap {

// api: 全局建图参数类
struct GlobalMapperOptions {
  // Options for each component
  ViewGraphCalibratorOptions opt_vgcalib; // vg_calib
  RelativePoseEstimationOptions opt_relpose; // relpose_esti
  RotationEstimatorOptions opt_ra; // rotation_esti
  TrackEstablishmentOptions opt_track; // track_establish
  GlobalPositionerOptions opt_gp; // global_position
  BundleAdjusterOptions opt_ba; // ba
  TriangulatorOptions opt_triangulator; // retriangle

  // Inlier thresholds for each component
  InlierThresholdOptions inlier_thresholds;

  // Control the number of iterations for each component
  int num_iteration_bundle_adjustment = 3; // ba次数
  int num_iteration_retriangulation = 1; // retrian次数

  // Control the flow of the global sfm
  bool skip_preprocessing = false;
  bool skip_view_graph_calibration = false;
  bool skip_relative_pose_estimation = false;
  bool skip_rotation_averaging = false;
  bool skip_track_establishment = false;
  bool skip_global_positioning = false;
  bool skip_bundle_adjustment = false;
  bool skip_retriangulation = false;
  bool skip_pruning = true;
};

class GlobalMapper {
 public:
  GlobalMapper(const GlobalMapperOptions& options) : options_(options) {}

  // api: 主求解函数
  bool Solve(const colmap::Database& database,
             ViewGraph& view_graph,
             std::unordered_map<camera_t, Camera>& cameras,
             std::unordered_map<image_t, Image>& images,
             std::unordered_map<track_t, Track>& tracks);

 private:
  const GlobalMapperOptions options_;
};

}  // namespace glomap
