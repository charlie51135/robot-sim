cmake_minimum_required(VERSION 3.0.2)
project(romi_base)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  diff_drive_controller
  hardware_interface
  roscpp
  rosparam_shortcuts
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include src
)

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  /usr/local/lib
)

# Only link to WiringPi on Raspberry Pi
EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
if (${ARCHITECTURE} MATCHES "aarch64")
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)
    add_executable(romi_base
        src/romi_base.cpp
        src/romi_hw_interface.cpp
        src/a_star.cpp
    )
    target_link_libraries(romi_base PRIVATE Threads::Threads)
    target_link_libraries(romi_base PRIVATE ${catkin_LIBRARIES} wiringPi wiringPiDev)
else()
endif()