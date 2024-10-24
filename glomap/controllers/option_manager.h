#pragma once

#include <colmap/util/logging.h>

#include <iostream>
#include <memory>

#include <boost/program_options.hpp>

namespace glomap {

struct GlobalMapperOptions;
struct ViewGraphCalibratorOptions;
struct RelativePoseEstimationOptions;
struct RotationEstimatorOptions;
struct TrackEstablishmentOptions;
struct GlobalPositionerOptions;
struct BundleAdjusterOptions;
struct TriangulatorOptions;
struct InlierThresholdOptions;

// api: 参数option管理类
class OptionManager {
 public:
  // api: 构造
  explicit OptionManager(bool add_project_options = true);
  
  // api: 添加所有的参数
  void AddAllOptions();
  
  // api: 添加各模块参数
  void AddDatabaseOptions();
  void AddImageOptions();
  void AddGlobalMapperOptions();
  void AddGlobalMapperFullOptions();
  void AddGlobalMapperResumeOptions();
  void AddGlobalMapperResumeFullOptions();
  void AddViewGraphCalibrationOptions();
  void AddRelativePoseEstimationOptions();
  void AddRotationEstimatorOptions();
  void AddTrackEstablishmentOptions();
  void AddGlobalPositionerOptions();
  void AddBundleAdjusterOptions();
  void AddTriangulatorOptions();
  void AddInlierThresholdOptions();

  // api: 新增必需的参数，option为最后存储参数值的指针
  template <typename T>
  void AddRequiredOption(const std::string& name,
                         T* option,
                         const std::string& help_text = "");

  // api: 新增带默认值的参数，option为最后存储参数值的指针
  template <typename T>
  void AddDefaultOption(const std::string& name,
                        T* option,
                        const std::string& help_text = "");

  // api: 重置
  void Reset();
  // api: 重置GlobalMapperOptions
  void ResetOptions(bool reset_paths);

  // api: 解析命令行
  void Parse(int argc, char** argv);

  std::shared_ptr<std::string> database_path;
  std::shared_ptr<std::string> image_path;

  std::shared_ptr<GlobalMapperOptions> mapper;

 private:
  // api: 新增并注册必需有值的参数
  template <typename T>
  void AddAndRegisterRequiredOption(const std::string& name,
                                    T* option,
                                    const std::string& help_text = "");

  // api: 新增并注册带默认值的参数
  template <typename T>
  void AddAndRegisterDefaultOption(const std::string& name,
                                   T* option,
                                   const std::string& help_text = "");

  // api: 注册参数名
  template <typename T>
  void RegisterOption(const std::string& name, const T* option);

  // option desc
  std::shared_ptr<boost::program_options::options_description> desc_;

  // options dict
  std::vector<std::pair<std::string, const bool*>> options_bool_;
  std::vector<std::pair<std::string, const int*>> options_int_;
  std::vector<std::pair<std::string, const double*>> options_double_;
  std::vector<std::pair<std::string, const std::string*>> options_string_;

  bool added_database_options_ = false;
  bool added_image_options_ = false;
  bool added_mapper_options_ = false;
  bool added_view_graph_calibration_options_ = false;
  bool added_relative_pose_options_ = false;
  bool added_rotation_averaging_options_ = false;
  bool added_track_establishment_options_ = false;
  bool added_global_positioning_options_ = false;
  bool added_bundle_adjustment_options_ = false;
  bool added_triangulation_options_ = false;
  bool added_inliers_options_ = false;
};

template <typename T>
void OptionManager::AddRequiredOption(const std::string& name,
                                      T* option,
                                      const std::string& help_text) {
  desc_->add_options()(name.c_str(),
                       boost::program_options::value<T>(option)->required(),
                       help_text.c_str());
}

template <typename T>
void OptionManager::AddDefaultOption(const std::string& name,
                                     T* option,
                                     const std::string& help_text) {
  desc_->add_options()(
      name.c_str(),
      boost::program_options::value<T>(option)->default_value(*option),
      help_text.c_str());
}

template <typename T>
void OptionManager::AddAndRegisterRequiredOption(const std::string& name,
                                                 T* option,
                                                 const std::string& help_text) {
  desc_->add_options()(name.c_str(),
                       boost::program_options::value<T>(option)->required(),
                       help_text.c_str());
  RegisterOption(name, option);
}

template <typename T>
void OptionManager::AddAndRegisterDefaultOption(const std::string& name,
                                                T* option,
                                                const std::string& help_text) {
  desc_->add_options()(
      name.c_str(),
      boost::program_options::value<T>(option)->default_value(*option),
      help_text.c_str());
  RegisterOption(name, option);
}

template <typename T>
void OptionManager::RegisterOption(const std::string& name, const T* option) {
  if (std::is_same<T, bool>::value) {
    options_bool_.emplace_back(name, reinterpret_cast<const bool*>(option));
  } else if (std::is_same<T, int>::value) {
    options_int_.emplace_back(name, reinterpret_cast<const int*>(option));
  } else if (std::is_same<T, double>::value) {
    options_double_.emplace_back(name, reinterpret_cast<const double*>(option));
  } else if (std::is_same<T, std::string>::value) {
    options_string_.emplace_back(name,
                                 reinterpret_cast<const std::string*>(option));
  } else {
    LOG(ERROR) << "Unsupported option type: " << name;
  }
}

}  // namespace glomap
