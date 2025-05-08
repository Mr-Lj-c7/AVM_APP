cmake_minimum_required(VERSION 3.0)

project(camera)

if(x86)
  message("x86 start cmake")
endif(x86)
if(ARM)
  message("arm start cmake")
  set(CMAKE_CXX_STANDARD 11) 
endif(ARM)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# YAML-CPP
# find_package(yaml-cpp REQUIRED)
# include_directories(${YAML_CPP_INCLUDE_DIRS})

# include
# include_directories(${PROJECT_SOURCE_DIR}/include)
# add_subdirectory(common)

# add_executable(camera ./src/run_live_demo_optimize.cpp)
add_executable(camera ./camera.cpp)

target_link_libraries(camera 
                    # 2d_surround 
                    # ${YAML_CPP_LIBRARIES}
                    ${OpenCV_LIBS}
                    )
