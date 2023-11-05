cmake_minimum_required (VERSION 2.8)
project (motorlib)
add_library(motorlib src/group_handler.cpp src/group_bulk_read.cpp src/group_bulk_write.cpp src/group_sync_read.cpp src/group_sync_write.cpp src/packet_handler.cpp src/port_handler.cpp src/protocol1_packet_handler.cpp src/protocol2_packet_handler.cpp src/port_handler_linux.cpp src/Motor.cpp src/Motor_control.cpp src/group_fast_sync_read.cpp src/group_fast_bulk_read.cpp)
target_include_directories(motorlib PUBLIC include)