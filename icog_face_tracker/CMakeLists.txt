cmake_minimum_required(VERSION 2.8.3)
project(icog_face_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  usb_cam
)
find_package(OpenCV REQUIRED)
find_package(dlib REQUIRED)

add_compile_options(-std=c++11)
#set(CMAKE_INCLUDE_CURRENT_DIR ON)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
add_message_files(
  FILES
  myRect.msg
  myPoint.msg
  facegeometry.msg
  facegeos.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#INCLUDE_DIRS include
#  LIBRARIES icog_face_tracker
#  CATKIN_DEPENDS cv_bridge image_transport message_generation roscpp sensor_msgs std_msgs usb_cam
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(icog_face_tracker
#   src/${PROJECT_NAME}/icog_face_tracker.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(icog_face_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(PROJECTNAME "tracker_node")

set ( ${PROJECTNAME}_HDRS
utility/pietslib.h
)

set ( ${PROJECTNAME}_SRCS
src/tracker.cpp
utility/pietslib.cpp
)

## Declare a C++ executable
# add_executable(icog_face_tracker_node src/icog_face_tracker_node.cpp)
#add_executable(icog_face_tracker_node ${${PROJECTNAME}_SRCS} ${${PROJECTNAME}_HDRS})

add_executable(tracker_node ${${PROJECTNAME}_SRCS} ${${PROJECTNAME}_HDRS})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(icog_face_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(icog_face_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
# target_link_libraries(icog_face_tracker_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(tracker_node ${catkin_LIBRARIES})
target_link_libraries(tracker_node ${OpenCV_LIBS})
target_link_libraries(tracker_node ${dlib_LIBRARIES})

#Listener to visualize face tracking data
set(PROJECTLISTENER "listener_node")

set ( ${PROJECTLISTENER}_HDRS
utility/pietslib.h
)

set ( ${PROJECTLISTENER}_SRCS
src/listener.cpp
utility/pietslib.cpp
)

add_executable(listener_node ${${PROJECTLISTENER}_SRCS} ${${PROJECTLISTENER}_HDRS})

target_link_libraries(listener_node ${catkin_LIBRARIES})
target_link_libraries(listener_node ${OpenCV_LIBS})
target_link_libraries(listener_node ${dlib_LIBRARIES})


#Robot Controller
set(PROJECTJACO "jaco_node")

set ( ${PROJECTJACO}_HDRS
utility/pid.h
)

set ( ${PROJECTJACO}_SRCS
src/robot_controller.cpp
utility/pid.cpp
)

add_executable(jaco_node ${${PROJECTJACO}_SRCS} ${${PROJECTJACO}_HDRS})

target_link_libraries(jaco_node ${catkin_LIBRARIES})
target_link_libraries(jaco_node ${OpenCV_LIBS})

#3D Laser Reconstruction
set(PROJECTRECON "laserRecon_node")

set ( ${PROJECTRECON}_HDRS

)

set ( ${PROJECTRECON}_SRCS
src/laserRecon.cpp
)

add_executable(laserRecon_node ${${PROJECTRECON}_SRCS} ${${PROJECTRECON}_HDRS})

target_link_libraries(laserRecon_node ${catkin_LIBRARIES})
target_link_libraries(laserRecon_node ${OpenCV_LIBS})

#Robot Controller
set(PROJECTTRINK "trink_node")

set ( ${PROJECTTRINK}_HDRS
#utility/pietslib.h
utility/pid.h
)

set ( ${PROJECTTRINK}_SRCS
src/trinkvorgang.cpp
#utility/pietslib.cpp
utility/pid.cpp
)

add_executable(trink_node ${${PROJECTTRINK}_SRCS} ${${PROJECTTRINK}_HDRS})

target_link_libraries(trink_node ${catkin_LIBRARIES})
target_link_libraries(trink_node ${OpenCV_LIBS})
#target_link_libraries(listener_node ${dlib_LIBRARIES})

#Robot Controller
set(PROJECTRECORD "record_node")

set ( ${PROJECTRECORD}_HDRS
)

set ( ${PROJECTRECORD}_SRCS
src/excelRecorder.cpp
)

add_executable(record_node ${${PROJECTRECORD}_SRCS} ${${PROJECTRECORD}_HDRS})

target_link_libraries(record_node ${catkin_LIBRARIES})
target_link_libraries(record_node ${OpenCV_LIBS})
target_link_libraries(record_node ${dlib_LIBRARIES})
#target_link_libraries(jaco_node ${dlib_LIBRARIES})

#add_dependencies(listener icog_face_tracker_generate_messages_cpp)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS icog_face_tracker icog_face_tracker_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_icog_face_tracker.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
