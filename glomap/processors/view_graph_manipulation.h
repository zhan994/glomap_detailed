#pragma once

#include "glomap/scene/types_sfm.h"

namespace glomap {

struct ViewGraphManipulater {
  enum StrongClusterCriteria {
    INLIER_NUM,
    WEIGHT,
  };

  static image_pair_t SparsifyGraph(ViewGraph& view_graph,
                                    std::unordered_map<image_t, Image>& images,
                                    int expected_degree = 50);

  static image_t EstablishStrongClusters(
      ViewGraph& view_graph,
      std::unordered_map<image_t, Image>& images,
      StrongClusterCriteria criteria = INLIER_NUM,
      double min_thres = 100,  // require strong edges
      int min_num_images = 2);

  // api: 通过帧间位姿来更新图像对E/F矩阵配置
  static void UpdateImagePairsConfig(
      ViewGraph& view_graph,
      const std::unordered_map<camera_t, Camera>& cameras,
      const std::unordered_map<image_t, Image>& images);

  // Decompose the relative camera postion from the camera config
  // api: 通过E/F矩阵分解得到相对位姿
  static void DecomposeRelPose(ViewGraph& view_graph,
                               std::unordered_map<camera_t, Camera>& cameras,
                               std::unordered_map<image_t, Image>& images);
};

}  // namespace glomap
