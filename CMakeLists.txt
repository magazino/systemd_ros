cmake_minimum_required(VERSION 2.8.3)
project(systemd_ros)

find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package()

install(
    FILES
     etc/systemd_logging_with_rosout.conf
     etc/systemd_logging_without_rosout.conf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    PROGRAMS scripts/systemd_env.sh
    DESTINATION ${CMAKE_INSTALL_PREFIX}
)

)
