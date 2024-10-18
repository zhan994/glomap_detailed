#!/bin/sh

log_time() {
    date "+%Y-%m-%d %H:%M:%S:%3N"
}

PROJECT="${PWD}/proj"

mkdir -p ${PROJECT}/sparse
echo "$(log_time) glomap mapper..."
./build/glomap/glomap mapper \
  --skip_preprocessing 0 \
  --skip_view_graph_calibration 0 \
  --skip_relative_pose_estimation 0 \
  --skip_rotation_averaging 0 \
  --skip_global_positioning 0 \
  --skip_bundle_adjustment 1 \
  --skip_retriangulation 1 \
  --skip_pruning 1 \
  --database_path ${PROJECT}/database.db \
  --image_path ${PROJECT}/images \
  --output_path ${PROJECT}/sparse
echo "$(log_time) glomap mapper done."
