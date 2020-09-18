## Cmake file for building lensdriver makefile

project(grpc-cmake-example)
cmake_minimum_required(VERSION 3.5)

#Required cmake lines for image server
add_compile_options(-std=c++11)

# Cmake find modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

include_directories(/usr/include/)

# BOOST
add_definitions(-D_FILE_OFFSET_BITS=64 -DUSE_BOOST_FILESYSTEM_3 -DBL_TIMER_BACKEND=BL_TIMER_BACKEND_USEC)

set(BOOST_DEPS
    boost_program_options
    boost_thread
    boost_date_time
    boost_serialization
    boost_filesystem
    boost_system
    pthread)

set( CMAKE_EXE_LINKER_FLAGS ${BOOST_LIBRARY} )


add_library(motor STATIC
       ../motordriver_CFE.cpp ../motordriver_imx334_SFOX334T1.cpp ../motordriverbase.cpp
)

target_link_libraries(motor
        i2c
)


#required cmake lines for lensdriver
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

include_directories(..)

set(CMAKE_FIND_LIBRARY_SUFFIXES ".so;.a")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
set(CMAKE_EXPORT_COMPILE_COMMANDS "ON")

add_executable(lensDriver ../lensDriver.cpp)
target_link_libraries(lensDriver
    motor
    ${BOOST_DEPS}
    readline
	i2c
    )

#install(TARGETS lensDriver DESTINATION ~/)

