#include "glomap/processors/image_undistorter.h"

#include <colmap/util/threading.h>

namespace glomap {

void UndistortImages(std::unordered_map<camera_t, Camera>& cameras,
                     std::unordered_map<image_t, Image>& images,
                     bool clean_points) {
  // step: 1 筛选需要去畸变的图像id
  std::vector<image_t> image_ids;
  for (auto& [image_id, image] : images) {
    const int num_points = image.features.size();
    std::cout << "image_id: " << image_id << ", feat_num: " << num_points
              << ", undist_num: " << image.features_undist.size() << std::endl;

    if (image.features_undist.size() == num_points && !clean_points)
      continue;  // already undistorted
    image_ids.push_back(image_id);
  }

  colmap::ThreadPool thread_pool(colmap::ThreadPool::kMaxNumThreads);
  // step: 2 线程池去畸变
  LOG(INFO) << "Undistorting images..";
  const int num_images = image_ids.size();
  for (int image_idx = 0; image_idx < num_images; image_idx++) {
    // step: 2.1 获取图像并判断是否需要去畸变
    Image& image = images[image_ids[image_idx]];
    const int num_points = image.features.size();
    if (image.features_undist.size() == num_points && !clean_points)
      continue;  // already undistorted

    // step: 2.2 获取相机参数
    const Camera& camera = cameras[image.camera_id];

    // step: 2.3 像素坐标转到归一化相机坐标系下的射线
    thread_pool.AddTask([&image, &camera, num_points]() {
      image.features_undist.clear();
      image.features_undist.reserve(num_points);
      for (int i = 0; i < num_points; i++) {
        image.features_undist.emplace_back(
            camera.CamFromImg(image.features[i]).homogeneous().normalized());
      }
    });
  }

  thread_pool.Wait();
  LOG(INFO) << "Image undistortion done";
}

}  // namespace glomap
