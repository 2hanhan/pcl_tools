cmake_minimum_required(VERSION 2.8)
project(pcl_tools)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE = Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_STANDARD 17)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(libLAS REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${PCL_INCLUDE_DIRS}
${LIBLAS_INCLUDE_DIR}
)




set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)


add_library(${PROJECT_NAME} SHARED
src/ReadGroundTruth.cc
src/BuildPriorMap.cc
include/ReadGroundTruth.h
include/BuildPriorMap.h
)

target_link_libraries(${PROJECT_NAME}
${EIGEN3_LIBS}
${PCL_LIBRARIES}
${libLAS_LIBRARIES}
)



# examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(kaisturban
        Examples/KaistUrban.cc)
target_link_libraries(kaisturban ${PROJECT_NAME})

add_executable(matrix2 Examples/matrix2.cc)
target_link_libraries(matrix2
${EIGEN3_LIBS})