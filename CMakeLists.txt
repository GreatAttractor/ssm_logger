project("ssm_logger")

cmake_minimum_required(VERSION 3.1)

add_executable(ssm_logger
    src/main.cpp
)

set_property(TARGET ssm_logger PROPERTY CXX_STANDARD 14)
set_property(TARGET ssm_logger PROPERTY CXX_STANDARD_REQUIRED ON)

find_package(Boost COMPONENTS system program_options REQUIRED)
target_include_directories(ssm_logger PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(ssm_logger ${Boost_LIBRARIES})

find_package(Curses REQUIRED)
target_include_directories(ssm_logger PUBLIC ${CURSES_INCLUDE_DIR})
target_link_libraries(ssm_logger ${CURSES_LIBRARIES})

if(WIN32)
    target_link_libraries(ssm_logger Ws2_32)
endif()

set(CMAKE_BUILD_TYPE Release)
