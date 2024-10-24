#pragma once

#include "glomap/math/rigid3d.h"
#include "glomap/scene/types_sfm.h"
#include "glomap/types.h"

namespace glomap {
// api: 图像对inliers类
class ImagePairInliers {
 public:
  ImagePairInliers(
      ImagePair& image_pair,
      const std::unordered_map<image_t, Image>& images,
      const InlierThresholdOptions& options,
      const std::unordered_map<camera_t, Camera>* cameras = nullptr)
      : image_pair(image_pair),
        images(images),
        cameras(cameras),
        options(options) {}

  // use the sampson error and put the inlier result into the image pair
  // api: 计算sampson误差
  double ScoreError();

 protected:
  // Error for the case of essential matrix
  // api: E矩阵sampson误差
  double ScoreErrorEssential();

  // Error for the case of fundamental matrix
  // api: F矩阵sampson误差
  double ScoreErrorFundamental();

  // Error for the case of homography matrix
  // api: H矩阵sampson误差
  double ScoreErrorHomography();

  ImagePair& image_pair;
  const std::unordered_map<image_t, Image>& images;
  const std::unordered_map<camera_t, Camera>* cameras;
  const InlierThresholdOptions& options;
};

// api: 图像对的inlier统计
void ImagePairsInlierCount(ViewGraph& view_graph,
                           const std::unordered_map<camera_t, Camera>& cameras,
                           const std::unordered_map<image_t, Image>& images,
                           const InlierThresholdOptions& options,
                           bool clean_inliers);

}  // namespace glomap
