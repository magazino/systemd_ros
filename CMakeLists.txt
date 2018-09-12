cmake_minimum_required(VERSION 2.8.3)
project(systemd_ros)

find_package(catkin REQUIRED COMPONENTS roscpp pluginlib)
find_package(PkgConfig REQUIRED)

pkg_check_modules(SYSTEMD "libsystemd" REQUIRED)
pkg_check_modules(LOG4CXX "liblog4cxx" REQUIRED)

catkin_python_setup()
catkin_package()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${SYSTEMD_INCLUDE_DIRS}
    ${LOG4CXX_INCLUDE_DIRS}
)

add_library(
    ${PROJECT_NAME}
    src/${PROJECT_NAME}/log_appender.cpp
)

target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${SYSTEMD_LIBRARIES}
    ${LOG4CXX_LIBRARIES}
)

install(
    FILES
    etc/systemd_log4cxx_with_rosout.conf
    etc/systemd_log4cxx_without_rosout.conf
    etc/systemd_logging_with_rosout.conf
    etc/systemd_logging_without_rosout.conf
    DESTINATION ${CATKIN_PACKAGE_ETC_DESTINATION}
)
install(
    FILES share/log_appenders.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    PROGRAMS scripts/systemd_env.sh
    DESTINATION ${CMAKE_INSTALL_PREFIX}
)

install(
    TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
