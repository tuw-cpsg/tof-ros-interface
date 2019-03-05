cmake_minimum_required(VERSION 2.8.3)
project(ti_tof)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  image_transport
)

## System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED COMPONENTS common)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  DepthArrayStamped.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ti_tof
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

# build only when PCL lib is present (only on RaspberryPi)
if (PCL)
  add_definitions(${PCL_DEFINITIONS})
  add_definitions(-pthread -std=c++11 -fPIC -ffast-math -fpermissive)

  set(VOXEL_INCLUDE_DIRS . /usr/include/voxel /usr/include/voxel/pcl /usr/include/voxel/ti3dtof /usr/include/voxel/Filter )
  set(VOXEL_LIBRARIES /usr/lib/libti3dtof.so /usr/lib/libvoxel.so /usr/lib/libvoxelpcl.so)

  ## Specify additional locations of header files
  ## Your package locations should be listed before other locations
  include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
    ${VOXEL_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )

  ## Declare a C++ executable
  ## With catkin_make all packages are built within a single CMake context
  ## The recommended prefix ensures that target names across packages don't collide
  add_executable(${PROJECT_NAME}_node src/ti_tof_node.cpp)

  ## Add cmake target dependencies of the executable
  ## same as for the library above
  add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  ## Specify libraries to link a library or executable target against
  target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${VOXEL_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
  )
else (PCL)
  message(WARNING "PCL library not found. Only depth_to_image is built.")

  include_directories(
    ${catkin_INCLUDE_DIRS}
  )
endif (PCL)

add_executable(depth_to_image src/depth_to_image.cpp)
add_dependencies(depth_to_image ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(depth_to_image
  ${catkin_LIBRARIES}
)
