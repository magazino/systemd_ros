#!/usr/bin/env sh

_CATKIN_SETUP_DIR=$(cd "`dirname "$0"`" > /dev/null && pwd)
export ROS_PYTHON_LOG_CONFIG_FILE="$_CATKIN_SETUP_DIR"/etc/systemd_ros/systemd_logging_without_rosout.conf
export ROSCONSOLE_CONFIG_FILE="$_CATKIN_SETUP_DIR"/etc/systemd_ros/systemd_log4cxx_without_rosout.conf
. "$_CATKIN_SETUP_DIR/setup.sh"
exec "$@"
