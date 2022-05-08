#########
# Setup #
#########

# Specify the minimum version for CMake
cmake_minimum_required (VERSION 3.5)

# Message
message(STATUS "CENG 4120 project")
message(STATUS ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS ${PROJECT_SOURCE_DIR})

# Find includes in the build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Project's name
project(cts)

set(CMAKE_CXX_STANDARD 14)

######################
#      SET PATH      #
######################
# Set the output folder where your program will be created
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(PATH_SRC ${PROJECT_SOURCE_DIR}/src/)

###################
# Warnings/Errors #
###################

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Werror=return-type")

######################
# Check Dependencies #
######################
if (ENABLE_STATIC_COMPILE)
  set(Boost_USE_STATIC_LIBS ON)
endif (ENABLE_STATIC_COMPILE)

find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
# find_package(Threads)
if(Boost_FOUND)
  message ("boost found")
  message( "Boost include dir ${Boost_INCLUDE_DIRS} ")
  message( "Boost Boost_LIBRARY_DIRS ${Boost_LIBRARY_DIRS} ")
else()
  message (FATAL_ERROR "Cannot find Boost")
endif()

###############
# Source Code #
###############
file(GLOB_RECURSE SRC_FILES
    ${PATH_SRC}/*.cpp
    ${PATH_SRC}/*.cc
    ${PATH_SRC}/*.c)

###########
# Targets #
###########

add_executable(${PROJECT_NAME} ${SRC_FILES})

#######################
# Include Directories #
#######################

include_directories(${PATH_SRC})

#################
# Link Libraries  #
#################
set(USED_LIBRARIES 
    ${Boost_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

set(INCLUDED_DIRECTORIES 
    ${Boost_INCLUDE_DIR}
)

#################
# Linker Flags  #
#################
# TODO: set static under release mode only
if (ENABLE_STATIC_COMPILE)
    message(STATUS "enable static compilation...")
    set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS 
            "-static -Wl,--whole-archive -rdynamic -lpthread -Wl,--no-whole-archive")
else (ENABLE_STATIC_COMPILE)
    set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS 
            "-Wl,--whole-archive -rdynamic -lpthread -Wl,--no-whole-archive")
endif (ENABLE_STATIC_COMPILE)

# Boost
target_include_directories(${PROJECT_NAME} PUBLIC ${INCLUDED_DIRECTORIES})
target_link_libraries(${PROJECT_NAME} ${USED_LIBRARIES})
