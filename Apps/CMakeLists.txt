#Miinimum CMAKE version required
cmake_minimum_required(VERSION 3.15)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/Thirdparty/Sophus
        ${MINI_SLAM_LIB_INCLUDE_DIR})

#Set executalble directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Apps)

#Set the name of the Program
set(PROGRAM_NAME mono_euroc)
add_executable(${PROGRAM_NAME} mono_euroc.cc)
target_link_libraries(${PROGRAM_NAME} ${OpenCV_LIBS} mini_slam)

set(PROGRAM_NAME mono_tumrgbd)
add_executable(${PROGRAM_NAME} mono_tumrgbd.cc)
target_link_libraries(${PROGRAM_NAME} ${OpenCV_LIBS} mini_slam)
