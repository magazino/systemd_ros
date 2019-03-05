cmake_minimum_required(VERSION 2.8.3)
project(systemd_ros)

find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package()

install(
    FILES
     etc/systemd_log4cxx_no_output.conf
     etc/systemd_logging_stdout.conf
     etc/systemd_logging_with_rosout.conf
     etc/systemd_logging_without_rosout.conf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    PROGRAMS
     scripts/remote-env.sh
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
    PROGRAMS
     scripts/generate-services
     scripts/param-manager
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
    PROGRAMS
     scripts/systemd-roscore
    DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

