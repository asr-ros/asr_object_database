cmake_minimum_required(VERSION 2.8.3)
project(asr_object_database)

# C++11 is needed
add_definitions(-Wall -std=gnu++0x -ggdb)
set(CMAKE_BUILD_TYPE Debug)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  resource_retriever
  asr_ros_uri
# temporary workaround as assimp does not provide a .cmake file but just a .pc file
#  assimp
  asr_xsd2cpp
  message_generation
)
#message(WARNING "Object_database needs the packages 'assimp' and 'xerces' which do not provide .cmake files. Install them manually.")


find_package(cmake_modules)

#find Eigen (depends on ros distro):
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES}) 
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  regex
)

add_service_files(DIRECTORY srv FILES
	RecognizerList.srv
	ObjectTypeList.srv
	ObjectMetaData.srv
        RecognizerListMeshes.srv
	ObjectTypeGenerator.srv
)
generate_messages(DEPENDENCIES
	std_msgs
	geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy roslib geometry_msgs resource_retriever asr_ros_uri asr_xsd2cpp message_runtime std_msgs
  LIBRARIES ${PROJECT_NAME}_lib
  DEPENDS EIGEN3 Boost
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
# temporary workaround as assimp does not provide a .cmake file but just a .pc file
  /opt/ros/$ENV{ROS_DISTRO}/include/assimp_devel
  xsd_gen
)

include(${asr_xsd2cpp_SOURCE_DIR}/catkin_make.cmake)
xsd2cpp()

add_library(${PROJECT_NAME}_lib 
  xsd_gen/asr_object_database/ConfigurationXMLSchema.cpp  
  src/ObjectDatabase.cpp 
  src/ObjectDatabaseConfig.cpp 
  src/ObjectDatabaseRecognizer.cpp 
  src/ObjectDatabaseEntry.cpp
)
	
add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})

add_dependencies(${PROJECT_NAME}_lib
    ${PROJECT_NAME}_gencpp    
)

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${Boost_LIBRARIES}  
 # assimp
  /opt/ros/$ENV{ROS_DISTRO}/lib/libassimp_devel.so
  /usr/lib/x86_64-linux-gnu/libxerces-c-3.1.so
)
FILE(GLOB_RECURSE HEADERS "include/object_database/*.h")
add_executable(${PROJECT_NAME} 
  src/object_database.cpp
    ${HEADERS}
)
target_link_libraries(${PROJECT_NAME} 
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
    ${PROJECT_NAME}_gencpp    
)

catkin_install_python(PROGRAMS src/normal_generator_tool/normalsTool.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
