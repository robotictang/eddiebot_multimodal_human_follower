cmake_minimum_required (VERSION 2.4.6)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include ($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)


set (ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init ()
rosbuild_add_boost_directories ()
rosbuild_check_for_sse ()

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories (${CMAKE_CURRENT_BINARY_DIR})

pkg_check_modules (LIBUSB REQUIRED libusb-1.0)
include_directories (${LIBUSB_INCLUDE_DIRS})
link_directories (${LIBUSB_LIBRARY_DIRS})

#find_package(GLUT)
#find_package(OpenGL)


# Dynamic reconfigure
#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()

# ---[ OpenNI Driver
#rosbuild_add_library (openni_driver src/openni_driver.cpp)
#rosbuild_add_compile_flags (openni_driver ${SSE_FLAGS})

#rosbuild_add_library (openni_driver_nodelets src/openni_driver_nodelets.cpp)
#rosbuild_add_compile_flags (openni_driver_nodelets ${SSE_FLAGS})
#target_link_libraries (openni_driver_nodelets openni_driver)

#rosbuild_add_executable (tracker src/main.cpp src/SceneDrawer.cpp)
#target_link_libraries (tracker ${GLUT_LIBRARY} ${OPENGL_LIBRARY}  ${SSE_FLAGS})
#target_link_libraries (openni_node openni_driver)

#rosbuild_add_executable (calibration_synchronizer src/calibration_synchronizer.cpp)


########################
#include_directories(${Eigen_INCLUDE_DIRS})
#find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})
