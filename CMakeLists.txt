cmake_minimum_required(VERSION 3.15)
project(TriangulationFrom2Imgs)
find_package(OpenCV 3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system thread REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_CXX_STANDARD 14)
set(Boost_USE_STATIC_LIBS ON)

add_executable(TriangulationFrom2Imgs main.cpp MyCommon.cpp MyCommon.h MyFileUtils.cpp MyFileUtils.h FeatureUtils.cpp FeatureUtils.h MatchingUtils.cpp MatchingUtils.h StereoUtils.cpp StereoUtils.h MyTriDemo.cpp MyTriDemo.h)

target_link_libraries(TriangulationFrom2Imgs ${CERES_LIBRARIES})
target_link_libraries(TriangulationFrom2Imgs ${Boost_LIBRARIES})
target_link_libraries(TriangulationFrom2Imgs ${OpenCV_LIBS} )