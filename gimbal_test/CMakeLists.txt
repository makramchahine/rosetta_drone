project(gimbal_test)

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

find_package(SDL2 REQUIRED)
if (SDL2_FOUND)
    message(STATUS "Found SDL2:")
    message(STATUS " - Includes: ${SDL2_INCLUDE_DIRS}")
    message(STATUS " - Libraries: ${SDL2_LIBRARIES}")
    add_definitions(-DSDL2_INSTALLED)
    set(SDL2_LIB_DEPEND_FLAG -lSDL2)
else ()
message(STATUS "--Recommendation : It is found that using \"cv::imshow\" on some platforms will cause more
                CPU resources and more processing time in camera_stream_node. Using SDL to display images can improve this
                situation. \n--Install SDL2 library in shell : \"sudo apt-get install libsdl2-dev\".")
endif()



cs_add_executable(gimbal_movement_test_node2
   src/gimbal_movement_test_node.cpp
)

target_link_libraries(gimbal_movement_test_node2
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${SDL2_LIB_DEPEND_FLAG}
  stdc++fs
  dji_osdk_ros
)

# Finish
cs_install()
cs_export()
