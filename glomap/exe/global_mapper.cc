#include "glomap/controllers/global_mapper.h"

#include "glomap/controllers/option_manager.h"
#include "glomap/io/colmap_io.h"
#include "glomap/types.h"

#include <colmap/util/misc.h>
#include <colmap/util/timer.h>

namespace glomap {
// -------------------------------------
// api: Mappers starting from COLMAP database
// -------------------------------------
int RunMapper(int argc, char** argv) {
  std::string database_path;
  std::string output_path;

  std::string image_path = "";
  std::string constraint_type = "ONLY_POINTS";
  std::string output_format = "bin";

  // step: 1 参数初始化并解析
  OptionManager options;
  // step: 1.1 添加 option_manager的基础参数
  options.AddRequiredOption("database_path", &database_path);
  options.AddRequiredOption("output_path", &output_path);
  options.AddDefaultOption("image_path", &image_path);
  options.AddDefaultOption("constraint_type",
                           &constraint_type,
                           "{ONLY_POINTS, ONLY_CAMERAS, "
                           "POINTS_AND_CAMERAS_BALANCED, POINTS_AND_CAMERAS}");
  options.AddDefaultOption("output_format", &output_format, "{bin, txt}");
  // step: 1.2 添加 全局建图 的所有参数
  options.AddGlobalMapperFullOptions();
  // step: 1.3 解析命令行
  options.Parse(argc, argv);

  // step: 2 检查database
  if (!colmap::ExistsFile(database_path)) {
    LOG(ERROR) << "`database_path` is not a file";
    return EXIT_FAILURE;
  }

  // step: 3 约束类型赋值
  if (constraint_type == "ONLY_POINTS") {
    options.mapper->opt_gp.constraint_type =
        GlobalPositionerOptions::ONLY_POINTS;
  } else if (constraint_type == "ONLY_CAMERAS") {
    options.mapper->opt_gp.constraint_type =
        GlobalPositionerOptions::ONLY_CAMERAS;
  } else if (constraint_type == "POINTS_AND_CAMERAS_BALANCED") {
    options.mapper->opt_gp.constraint_type =
        GlobalPositionerOptions::POINTS_AND_CAMERAS_BALANCED;
  } else if (constraint_type == "POINTS_AND_CAMERAS") {
    options.mapper->opt_gp.constraint_type =
        GlobalPositionerOptions::POINTS_AND_CAMERAS;
  } else {
    LOG(ERROR) << "Invalid constriant type";
    return EXIT_FAILURE;
  }

  // step: 4 检查输出格式
  if (output_format != "bin" && output_format != "txt") {
    LOG(ERROR) << "Invalid output format";
    return EXIT_FAILURE;
  }

  // step: 5 Load the database
  LOG(INFO) << "Loading Feature Database,,,";
  ViewGraph view_graph;
  std::unordered_map<camera_t, Camera> cameras;
  std::unordered_map<image_t, Image> images;
  std::unordered_map<track_t, Track> tracks;

  const colmap::Database database(database_path);
  ConvertDatabaseToGlomap(database, view_graph, cameras, images);

  if (view_graph.image_pairs.empty()) {
    LOG(ERROR) << "Can't continue without image pairs";
    return EXIT_FAILURE;
  }

  // step: 6 solver的主入口
  GlobalMapper global_mapper(*options.mapper);
  LOG(INFO) << "Loaded database";
  colmap::Timer run_timer;
  run_timer.Start();
  global_mapper.Solve(database, view_graph, cameras, images, tracks);
  run_timer.Pause();

  LOG(INFO) << "Reconstruction done in " << run_timer.ElapsedSeconds()
            << " seconds";

  // step: 7 输出重建结果
  WriteGlomapReconstruction(
      output_path, cameras, images, tracks, output_format, image_path);
  LOG(INFO) << "Export to COLMAP reconstruction done";

  return EXIT_SUCCESS;
}

// -------------------------------------
// Mappers starting from COLMAP reconstruction
// -------------------------------------
int RunMapperResume(int argc, char** argv) {
  std::string input_path;
  std::string output_path;
  std::string image_path = "";
  std::string output_format = "bin";

  OptionManager options;
  options.AddRequiredOption("input_path", &input_path);
  options.AddRequiredOption("output_path", &output_path);
  options.AddDefaultOption("image_path", &image_path);
  options.AddDefaultOption("output_format", &output_format, "{bin, txt}");
  options.AddGlobalMapperResumeFullOptions();

  options.Parse(argc, argv);

  if (!colmap::ExistsDir(input_path)) {
    LOG(ERROR) << "`input_path` is not a directory";
    return EXIT_FAILURE;
  }

  // Check whether output_format is valid
  if (output_format != "bin" && output_format != "txt") {
    LOG(ERROR) << "Invalid output format";
    return EXIT_FAILURE;
  }

  // Load the reconstruction
  ViewGraph view_graph;       // dummy variable
  colmap::Database database;  // dummy variable

  std::unordered_map<camera_t, Camera> cameras;
  std::unordered_map<image_t, Image> images;
  std::unordered_map<track_t, Track> tracks;
  colmap::Reconstruction reconstruction;
  reconstruction.Read(input_path);
  ConvertColmapToGlomap(reconstruction, cameras, images, tracks);

  GlobalMapper global_mapper(*options.mapper);

  // Main solver
  colmap::Timer run_timer;
  run_timer.Start();
  global_mapper.Solve(database, view_graph, cameras, images, tracks);
  run_timer.Pause();

  LOG(INFO) << "Reconstruction done in " << run_timer.ElapsedSeconds()
            << " seconds";

  WriteGlomapReconstruction(
      output_path, cameras, images, tracks, output_format, image_path);
  LOG(INFO) << "Export to COLMAP reconstruction done";

  return EXIT_SUCCESS;
}

}  // namespace glomap