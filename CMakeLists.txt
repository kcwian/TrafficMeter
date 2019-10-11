cmake_minimum_required(VERSION 2.8)
project( TrafficMeter )

find_package( OpenCV REQUIRED )
add_executable( TrafficMeter functions.cpp main.cpp )
target_link_libraries( TrafficMeter ${OpenCV_LIBS} )

add_custom_command(TARGET TrafficMeter POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:TrafficMeter> ../TrafficMeter)
