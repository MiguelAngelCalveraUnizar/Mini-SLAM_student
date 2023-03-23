#Miinimum CMAKE version required
cmake_minimum_required(VERSION 3.15)

#Find required libraries
find_package(Eigen3 3.1.0 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Pangolin REQUIRED)

#Set the name of the Library
set(LIBRARY_NAME mini_slam)

#Current library
set(root ${CMAKE_CURRENT_SOURCE_DIR})

#Define header and source files of the library
set(HEADER_FILES
        ${root}/Calibration/CameraModel.h
        ${root}/Calibration/PinHole.h
        ${root}/DatasetLoader/EurocVisualLoader.h
        ${root}/DatasetLoader/TUMRGBDLoader.h
        ${root}/Features/Descriptor.h
        ${root}/Features/Feature.h
        ${root}/Features/FAST.h
        ${root}/Features/ORB.h
        ${root}/Map/KeyFrame.h
        ${root}/Map/Map.h
        ${root}/Map/MapPoint.h
        ${root}/Mapping/LocalMapping.h
        ${root}/Matching/DescriptorMatching.h
        ${root}/Optimization/g2oBundleAdjustment.h
        ${root}/Optimization/g2oTypes.h
        ${root}/System/Settings.h
        ${root}/System/MiniSLAM.h
        ${root}/Tracking/Frame.h
        ${root}/Tracking/MonocularMapInitializer.h
        ${root}/Tracking/Tracking.h
        ${root}/Utils/Geometry.h
        ${root}/Visualization/FrameVisualizer.h
        ${root}/Visualization/MapVisualizer.h)

set(SOURCE_FILES
        ${root}/Calibration/PinHole.cc
        ${root}/DatasetLoader/EurocVisualLoader.cc
        ${root}/DatasetLoader/TUMRGBDLoader.cc
        ${root}/Features/FAST.cc
        ${root}/Features/ORB.cc
        ${root}/Map/KeyFrame.cc
        ${root}/Map/Map.cc
        ${root}/Map/MapPoint.cc
        ${root}/Mapping/LocalMapping.cc
        ${root}/Matching/DescriptorMatching.cc
        ${root}/Optimization/g2oBundleAdjustment.cc
        ${root}/Optimization/g2oTypes.cc
        ${root}/System/Settings.cc
        ${root}/System/MiniSLAM.cc
        ${root}/Tracking/Frame.cc
        ${root}/Tracking/MonocularMapInitializer.cc
        ${root}/Tracking/Tracking.cc
        ${root}/Utils/Geometry.cc
        ${root}/Visualization/FrameVisualizer.cc
        ${root}/Visualization/MapVisualizer.cc)

set(MINI_SLAM_LIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

include_directories(${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/Thirdparty/Sophus
        ${CMAKE_SOURCE_DIR}/Thirdparty/g2o
        ${Pangolin_INCLUDE_DIRS})

#Add the library
add_library(${LIBRARY_NAME} SHARED
        ${SOURCE_FILES}
        ${HEADER_FILES})

#Define the link libraries used in the project
target_link_libraries(${LIBRARY_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${CMAKE_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_core.so
        ${CMAKE_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_stuff.so
        ${CMAKE_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sba.so)
