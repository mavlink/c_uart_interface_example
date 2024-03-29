cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(c_uart_interface_example CXX)

set(SOURCES
    autopilot_interface.cpp
    mavlink_control.cpp
    serial_port.cpp
    udp_port.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE mavlink/include/mavlink/v2.0)

target_link_libraries(${PROJECT_NAME} PRIVATE pthread)