cmake_minimum_required(VERSION 2.8.3)
project(unknown_objects_segmentation)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  roscpp
  sensor_msgs
)

find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES unknown_objects_segmentation
#  CATKIN_DEPENDS pcl_conversions roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/svm.cpp
  src/SegmenterLight.cpp
  src/SVMPredictorSingle.cpp
  src/ContourDetector.cpp
  src/SurfaceModeling.cc
  src/ZAdaptiveNormals.cc
  src/StructuralRelationsLight.cpp
  src/ClusterNormalsToPlanes.cc
  src/Graph.cpp
  src/GraphCut.cpp
  src/ColorHistogram3D.cpp
  src/BoundaryRelations.cpp
  src/Texture.cpp
  src/disjoint-set.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


add_executable(segmenter_node src/segmenter_node.cpp)
target_link_libraries(segmenter_node
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

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
# install(TARGETS unknown_objects_segmentation unknown_objects_segmentation_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_unknown_objects_segmentation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
