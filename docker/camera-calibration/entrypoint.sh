#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
source "${WORKSPACE:-/catkin_ws}/devel/setup.bash"

export VIO_COMMON_PYTHON="${VIO_COMMON_PYTHON:-/opt/vio_common/python}"
export MPLBACKEND="${MPLBACKEND:-Agg}"
export PYTHONPATH="${WORKSPACE:-/catkin_ws}/src/kalibr/tools/camera_calibration:${PYTHONPATH:-}"

if [[ $# -eq 0 ]]; then
  exec python3 -m camera_calibration --help
fi

case "$1" in
  cam-cam|cam-imu|--help|-h)
    exec python3 -m camera_calibration "$@"
    ;;
  --*)
    exec python3 -m camera_calibration cam-cam "$@"
    ;;
  *)
    exec "$@"
    ;;
esac
