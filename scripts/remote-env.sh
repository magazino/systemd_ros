#!/usr/bin/env sh
ROS_PYTHON_LOG_CONFIG_FILE="$(rospack find systemd_ros)/systemd_logging_stdout.conf"

export ROS_PYTHON_LOG_CONFIG_FILE
export ROSCONSOLE_FORMAT="\${message}"

exec "$@"
