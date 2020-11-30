########### PROJECT SETUP ###########

# CMake Version #
cmake_minimum_required( VERSION 2.8 )

### Project Name ### 
project( ur_fk_ik ) # This must match the project name in "package.xml" EXACTLY

### Compiler Config ###
add_definitions(-std=c++11)
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake )
SET( GCC_PTHREAD_FLAGS "-lpthread -pthread" ) # This is needed for LIBIGL when using 'catkin_make' , works with plain cmake/make
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_PTHREAD_FLAGS}" ) # Append the flags                 ^^^-- *shrug* ---^^^

### Library Search Paths ###
list( APPEND CMAKE_MODULE_PATH ${CMAKE_PREFIX_PATH} )
list( APPEND CMAKE_PREFIX_PATH "/opt/ros/melodic" ) # This saves a lot of `find_package`s


########### LIBRARIES ###########

### REQUIRED ROS Packages ###
find_package( catkin REQUIRED COMPONENTS
    roscpp
    trac_ik_lib
    kdl_parser
    urdf
    genmsg
    std_msgs
    message_generation
)

### Non-ROS REQUIRED Packages ###
# find_package( Eigen3 REQUIRED )
# include_directories( SYSTEM ${EIGEN3_INCLUDE_DIRS} )

# find_package(urdf REQUIRED)
# include_directories(SYSTEM ${URDF_INCLUDE_DIRS})

# find_package(orocos_kdl REQUIRED)
find_package( cmake_modules REQUIRED )
# find_package(Eigen3 REQUIRED)
# find_package(TinyXML REQUIRED)  # parse xml
# find_package(urdfdom REQUIRED)  # parse urdf 
# find_package(Boost REQUIRED COMPONENTS system signals thread)

### Package-Associated Includes ###
include_directories( 
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
    ${catkin_INCLUDE_DIRS} 
#   ${Boost_INCLUDE_DIRS} 
#   ${orocos_kdl_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIRS}
)

### Shared Lib .SO Paths ###

# Shortcut Vars #
set( ROS_LIB_DIR /opt/ros/melodic/lib )
set( ROS_INC_DIR /opt/ros/melodic/include )
# set( MOVEIT_SO_VRS 1.0.6 )

# ROS Shared Libs #
set( ROS_LIBS 
    # ${ROS_LIB_DIR}/libmoveit_kinematics_plugin_loader.so.${MOVEIT_SO_VRS}
    # ${ROS_LIB_DIR}/libmoveit_robot_model_loader.so.${MOVEIT_SO_VRS}
    # ${ROS_LIB_DIR}/libmoveit_robot_model.so.${MOVEIT_SO_VRS}
    # ${ROS_LIB_DIR}/libmoveit_robot_state.so.${MOVEIT_SO_VRS}
    # ${ROS_LIB_DIR}/libmoveit_profiler.so.${MOVEIT_SO_VRS}
    # ${ROS_LIB_DIR}/libmoveit_rdf_loader.so.${MOVEIT_SO_VRS}
    # ${ROS_LIB_DIR}/libmoveit_kinematics_base.so.${MOVEIT_SO_VRS}
    # ${ROS_LIB_DIR}/libclass_loader.so
    ${ROS_LIB_DIR}/libroscpp.so
    ${ROS_LIB_DIR}/librosconsole.so
    ${ROS_LIB_DIR}/librostime.so
    # ${ROS_LIB_DIR}/libtrac_ik_kinematics_plugin.so
)

# Non-ROS Shared Libs #
# set( OTHER_LIBS
#   /usr/lib/x86_64-linux-gnu/libjsoncpp.so
# )

### Local Project Includes ###
include_directories( ${ROS_INC_DIR} ) 
# include_directories( ../trac_ik/trac_ik_lib/include/ ) 
include_directories( src ) 
# include_directories( include )


########### CUSTOM MESSAGES ###########

# MSG #
# add_message_files( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/msg FILES 
#   FK_req.msg 
#   FK_rsp.msg 
#   IK_req.msg 
#   IK_rsp.msg 
# )
# SRV #
add_service_files( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/srv FILES 
  FK.srv 
  IK.srv
)
# Generate #
generate_messages( DEPENDENCIES std_msgs )


########### EXECUTABLES ###########

# Files and Paths #
set( HLPR_CPP src/helpers.cpp )

set( EXEC_NAME FK_IK )
add_executable( ${EXEC_NAME} src/FK_IK.cpp ${HLPR_CPP} )
add_dependencies( ${EXEC_NAME} 
    # ${catkin_EXPORTED_TARGETS} 
    ${PROJECT_NAME}_generate_messages_cpp 
)
target_link_libraries( ${EXEC_NAME} ${ROS_LIBS} ${catkin_LIBRARIES} )

# set( EXEC_NAME test2 )
# add_executable( ${EXEC_NAME} src/moveit_test_02.cpp )
# target_link_libraries( ${EXEC_NAME} urdf )

# set( EXEC_NAME test3 )
# add_executable( ${EXEC_NAME} src/moveit_test_03.cpp )
# target_link_libraries( ${EXEC_NAME} ${ROS_LIBS} )

# set( EXEC_NAME test4 )
# add_executable( ${EXEC_NAME} src/moveit_test_04.cpp )
# target_link_libraries( ${EXEC_NAME} ${ROS_LIBS} )

# set( EXEC_NAME test5 )
# add_executable( ${EXEC_NAME} src/moveit_test_05.cpp )
# target_link_libraries( ${EXEC_NAME} ${ROS_LIBS} ${catkin_LIBRARIES} )

# set( EXEC_NAME FK_IK )
# add_executable( ${EXEC_NAME} src/${EXEC_NAME}.cpp ${HLPR_CPP} )
# add_dependencies( ${EXEC_NAME} ${PROJECT_NAME}_generate_messages_cpp )
# target_link_libraries( ${EXEC_NAME} ${ROS_LIBS} ${OTHER_LIBS} ${catkin_LIBRARIES} )




#### SPARE PARTS ##################################################################################

# find_package( catkin REQUIRED PATHS /opt/ros/melodic/share/catkin )
# include_directories(SYSTEM ${CATKIN_INCLUDE_DIRS})

# find_package( genmsg REQUIRED PATHS /opt/ros/melodic/share/genmsg )
# include_directories(SYSTEM ${GENMSG_INCLUDE_DIRS})

# find_package( actionlib_msgs REQUIRED PATHS /opt/ros/melodic/share/actionlib_msgs )
# include_directories(SYSTEM ${ACTIONLIB_MSGS_INCLUDE_DIRS})

# find_package( moveit_msgs REQUIRED PATHS /opt/ros/melodic/share/moveit_msgs )
# include_directories(SYSTEM ${MOVEIT_MSGS_INCLUDE_DIRS})

# find_package( moveit_core REQUIRED )
# include_directories(SYSTEM ${MOVEIT_CORE_INCLUDE_DIRS})

# find_package( eigen_stl_containers REQUIRED PATHS /usr/share/eigen_stl_containers/ )
# include_directories(SYSTEM ${EIGEN_STL_CONTAINERS_INCLUDE_DIRS})

# find_package( geometric_shapes REQUIRED PATHS /usr/share/geometric_shapes/cmake )
# include_directories(SYSTEM ${GEOMETRIC_SHAPES_INCLUDE_DIRS})
