cmake_minimum_required(VERSION 2.8.3)
project(ibd_test_data)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  dynamic_reconfigure
)

# protected region additional user defined REQUIREMENTS begin #
# protected region additional user defined REQUIREMENTS end #

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

# This could be simplified with a test across nodes.
if (ibd_test_data_dyn_rec_files)
  list(APPEND ibd_test_data_dyn_rec_files cfg/wrench_from_csv.cfg)
  message("Append of cfg/wrench_from_csv.cfg")
else ()
  set(ibd_test_data_dyn_rec_files cfg/wrench_from_csv.cfg)
  message("Setting with cfg/wrench_from_csv.cfg")
endif ()

message("Test: ${ibd_test_data_dyn_rec_files}")
if (ibd_test_data_dyn_rec_files)
  generate_dynamic_reconfigure_options(${ibd_test_data_dyn_rec_files})
endif ()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
    rospy
    geometry_msgs
    dynamic_reconfigure
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})


#############
## Install ##
#############
## todo: To be completed

# protected region user Cmake macros begin #
# protected region user Cmake macros end #
