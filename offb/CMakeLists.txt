cmake_minimum_required(VERSION 3.0.2)
project(offb)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  message_generation
  genmsg
  visualization_msgs

)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(gtk3 REQUIRED IMPORTED_TARGET gtk+-3.0)
pkg_search_module(gstreamer REQUIRED IMPORTED_TARGET gstreamer-1.0>=1.4)
pkg_search_module(gstreamer-sdp REQUIRED IMPORTED_TARGET gstreamer-sdp-1.0>=1.4)
pkg_search_module(gstreamer-app REQUIRED IMPORTED_TARGET gstreamer-app-1.0>=1.4)
pkg_search_module(gstreamer-video REQUIRED IMPORTED_TARGET gstreamer-video-1.0>=1.4)


# find_package(GStreamer REQUIRED COMPONENTS base)
# find_package(PkgConfig) 
# pkg_search_module(GLIB REQUIRED glib-2.0) 
# pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
# pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
# pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0)

# add_message_files(
#  FILES
#  obj.msg
# )


generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offb
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs message_generation
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(camera
    src/camera.cpp)
    # src/include/movement.cpp
    # src/include/run_yolo.cpp)
target_link_libraries(camera
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

add_executable(gstreamer 
src/gst_cpp.cpp)
target_link_libraries(gstreamer
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}

   PkgConfig::gtk3
   PkgConfig::gstreamer
   PkgConfig::gstreamer-sdp
   PkgConfig::gstreamer-app
   PkgConfig::gstreamer-video
   )

# target_include_directories(gstreamer ${GTK3_INCLUDE_DIRS} ${GST_INCLUDE_DIRS})
# target_link_libraries(gstreamer ${GST_APP_LIBRARIES} ${GST_VIDEO_LIBRARIES})





