#pragma once

#include "glomap/types.h"

#include <colmap/scene/camera.h>
#include <colmap/sensor/models.h>

#include <PoseLib/misc/colmap_models.h>

namespace glomap {
// api: 相机类，继承colmap的scene/camera
struct Camera : public colmap::Camera {
  Camera() : colmap::Camera() {}
  Camera(const colmap::Camera& camera) : colmap::Camera(camera) {}

  Camera& operator=(const colmap::Camera& camera) {
    *this = Camera(camera);
    return *this;
  }

  bool has_refined_focal_length = false;

  // api: (fx+fy)/2
  inline double Focal() const;
  // api: (cx,cy)
  inline Eigen::Vector2d PrincipalPoint() const;
  // api: intrinsic_matrix
  inline Eigen::Matrix3d GetK() const;
};

double Camera::Focal() const { return (FocalLengthX() + FocalLengthY()) / 2.0; }

Eigen::Vector2d Camera::PrincipalPoint() const {
  return Eigen::Vector2d(PrincipalPointX(), PrincipalPointY());
}

Eigen::Matrix3d Camera::GetK() const {
  Eigen::Matrix3d K;
  K << FocalLengthX(), 0, PrincipalPointX(), 0, FocalLengthY(),
      PrincipalPointY(), 0, 0, 1;
  return K;
}

// api: glomap::Camera -> poselib::Camera
inline poselib::Camera ColmapCameraToPoseLibCamera(const Camera& camera) {
  poselib::Camera pose_lib_camera(
      camera.ModelName(), camera.params, camera.width, camera.height);
  return pose_lib_camera;
}

}  // namespace glomap
