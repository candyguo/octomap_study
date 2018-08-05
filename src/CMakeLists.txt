find_package(PCL REQUIRED COMPONENTS common io)

include_directories("usr/include/eigen3")
include_directories(Eigen)

ADD_DEFINITIONS(${PCL_DEFINATIONS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_LIBRARIES(${PCL_LIBRARY_DIRS})

FIND_PACKAGE(octomap REQUIRED)
INCLUDE_DIRECTORIES(${OCTOMAP_INCLUDE_DIRS})

FIND_PACKAGE(OpenCV 2.4.9 REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

ADD_EXECUTABLE(pcd2octomap pcd2octomap.cpp)
target_link_libraries(pcd2octomap ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

ADD_EXECUTABLE(pcd2ColorOctomap pcd2ColorOctomap.cpp)
target_link_libraries(pcd2ColorOctomap ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

ADD_EXECUTABLE(joinmap joinmap.cpp)
target_link_libraries(joinmap ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OpenCV_LIBRARIES})