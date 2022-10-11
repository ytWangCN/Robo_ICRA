project(roborts_referee)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    geometry_msgs
    actionlib
    roborts_msgs
    genmsg
    std_msgs
)
catkin_package()
include_directories(socket ${catkin_INCLUDE_DIRS})
#aux_source_directory(. DIR_SOCKETSRC)
#add_library (socket ${DIR_SOCKETSRC})
#target_include_directories(socket PUBLIC include)

add_executable(socket_node
    socket.cpp
)
target_link_libraries(socket_node
    ${socket_LIBRARIES}
    ${catkin_LIBRARIES}

)
add_dependencies(socket_node
    roborts_msgs_generate_messages
)

add_executable(sub_node
    socket/socket.h
    sub.cpp
)
target_link_libraries(sub_node
    ${socket_LIBRARIES}
    ${catkin_LIBRARIES}
)
add_dependencies(sub_node
    roborts_msgs_generate_messages
)
