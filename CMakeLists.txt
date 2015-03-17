cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(projection_mapping)

list(APPEND CMAKE_MODULE_PATH
  "${PROJECT_SOURCE_DIR}/cmake"
  "${PROJECT_SOURCE_DIR}/lib/pcl/share/pcl-1.8")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Debug)

option(DISABLE_SENSOR_PIPELINE "Disables the sensor pipeline" OFF)
option(DISABLE_PROJECTION_PIPELINE "Disables the projection pipeline" OFF)

if(DISABLE_SENSOR_PIPELINE)
  add_definitions(-DDISABLE_SENSOR_PIPELINE)
endif()
if(DISABLE_PROJECTION_PIPELINE)
  add_definitions(-DDISABLE_PROJECTION_PIPELINE)
endif()

include(FindPkgConfig)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(EXTERNAL_LIB_DIR "${PROJECT_SOURCE_DIR}/lib")

pkg_check_modules(USB libusb-1.0>=1.0.0)
include_directories(${USB_INCLUDE_DIRS})
link_directories(${USB_LIBRARIY_DIRS})

# include_directories("${EXTERNAL_LIB_DIR}/libusb/include/libusb-1.0")
# link_directories("${EXTERNAL_LIB_DIR}/libusb/lib")

find_package(libfreenect REQUIRED NO_MODULE)
include_directories(${FREENECT_INCLUDE_DIRS})
link_directories(${FREENECT_LIBRARY_DIRS})

include_directories("${EXTERNAL_LIB_DIR}/eigen/include/eigen3")
link_directories("${EXTERNAL_LIB_DIR}/eigen/lib")

include_directories("${EXTERNAL_LIB_DIR}/pcl/include/pcl-1.8")
link_directories("${EXTERNAL_LIB_DIR}/pcl/lib")
set(PCL_LIBRARIES 
  "pcl_common" 
  "pcl_search" 
  "pcl_io" 
  "pcl_segmentation" 
  "pcl_visualization" 
  "pcl_sample_consensus" 
  "pcl_features")

# Use VTK
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#  message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()

# Use OpenGL, GLU, and GLUT
find_package(OpenGL REQUIRED)
find_package(GLU REQUIRED)
find_package(GLUT REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS} ${GLU_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})
link_directories(${OPENGL_LIBRARY_DIRS} ${GLU_LIBRARY_DIRS} ${GLUT_LIBRARY_DIRS})
add_definitions(${OPENGL_DEFINITIONS} ${GLU_DEFINITIONS} ${GLUT_DEFINITIONS})

find_package(Boost COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})
message("${FREENECT_INCLUDE_DIRS}")

set(LIBRARIES
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${USB_LIBRARIES}
  ${FREENECT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLU_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${VTK_LIBRARIES}
  m
  pthread)

include_directories("include")
add_definitions(-std=c++11) 
add_executable(projection_mapping src/main.cpp)
target_link_libraries(projection_mapping ${LIBRARIES})

add_executable(cube_generator src/cube_generator.cpp)
target_link_libraries(cube_generator ${LIBRARIES})

add_executable(ransac src/ransac.cpp)
target_link_libraries(ransac ${LIBRARIES})

add_executable(ransac_cube src/ransac_cube.cpp)
target_link_libraries(ransac_cube ${LIBRARIES})

add_executable(reconstruct_cube src/reconstruct_cube.cpp)
target_link_libraries(reconstruct_cube ${LIBRARIES})
