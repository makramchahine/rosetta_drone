project(image_processing)

cmake_minimum_required(VERSION 3.0.0)

find_package(catkin_simple REQUIRED)

find_package(OpenCV REQUIRED)

option(BUILD_MP "Build Motion Planning" OFF)

catkin_simple()

# Setup Default Build Type as Release
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()

# Setup Architcture-specific Flags
if ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
    message(STATUS "Using ARMv7 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
elseif ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
    message(STATUS "Using ARM aarch64 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
else ()
    set(CMAKE_CXX_ARCH_FLAGS " -march=native")
endif ()

# Setup General C++ Flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_STACK_ALLOCATION_LIMIT=1048576")
# otherwise double free or corruption (out) error when running racing or motion_planning example
add_compile_options(-O3)

# Setup Release and Debug flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_ARCH_FLAGS} -Wall -DNDEBUG -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -g")


cs_add_library(paper_finder
    src/paper_finder.cpp
)

target_link_libraries(paper_finder
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  stdc++fs
)

cs_add_executable(paper_finder_node
   src/paper_finder_node.cpp
)

target_link_libraries(paper_finder_node
  paper_finder
  ${OpenCV_LIBRARIES}
  stdc++fs
  zmq
  zmqpp
)

cs_add_library(paper_commander
    src/paper_commander.cpp
)

target_link_libraries(paper_commander
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  stdc++fs
)

cs_add_executable(paper_commander_node
   src/paper_commander_node.cpp
)

target_link_libraries(paper_commander_node
  paper_commander
  ${OpenCV_LIBRARIES}
  stdc++fs
  zmq
  zmqpp
)

# Finish
cs_install()
cs_export()
