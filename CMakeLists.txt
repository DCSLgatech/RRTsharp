# Kamil Saigol
# CMakeLists.txt template for standalone planner development with system-installed OMPL
# Requirements: OMPL, Boost (1.48 or higher), OpenMP (optional)

cmake_minimum_required(VERSION 3.3)
project(RRTsharp)

### There is no official support for C++11 from OMPL at the moment ###
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp RRTsharp.h RRTsharp.cpp)
add_executable(RRTsharp ${SOURCE_FILES})

### Boost ###
find_package(Boost 1.48 COMPONENTS thread REQUIRED)
if(Boost_FOUND)
    message(STATUS "Boost Include Directory: ${Boost_INCLUDE_DIRS}")
    message(STATUS "Boost Library Directory: ${Boost_LIBRARY_DIRS}")
    message(STATUS "Boost Libraries: ${Boost_LIBRARIES}")

    # Include Boost headers and link required Boost compiled libraries
    target_include_directories(RRTsharp PUBLIC ${Boost_INCLUDE_DIRS})
    target_link_libraries(RRTsharp ${Boost_LIBRARIES})
else()
    message(FATAL_ERROR "Boost >= 1.48 Not Found or Boost.Thread Library Not Found")
endif()

### OMPL ###
find_package(OMPL REQUIRED)
if(OMPL_FOUND)
    message(STATUS "OMPL Include Directory: ${OMPL_INCLUDE_DIRS}")
    message(STATUS "OMPL Libraries: ${OMPL_LIBRARIES}")
    message(STATUS "OMPL App Libraries: ${OMPLAPP_LIBRARIES}")

    # Include OMPL headers and link OMPL compiled libraries
    target_include_directories(RRTsharp PUBLIC ${OMPL_INCLUDE_DIRS})
    target_link_libraries(RRTsharp ${OMPL_LIBRARIES})
    target_link_libraries(RRTsharp ${OMPLAPP_LIBRARIES})
else()
    message(FATAL_ERROR "OMPL Not Found")
endif()

### OpenMP (optional - for multithreading) ###
find_package(OpenMP)
if(OPENMP_FOUND)
    message(STATUS "OpenMP C++ Flags: ${OpenMP_CXX_FLAGS}")
    message(STATUS "OpenMP C Flags: ${OpenMP_C_FLAGS}")

    # Add OpenMP compile and link options
    target_compile_options(RRTsharp PUBLIC ${OpenMP_CXX_FLAGS})
    if(NOT MSVC)
        set_property(TARGET RRTsharp APPEND PROPERTY LINK_FLAGS ${OpenMP_CXX_FLAGS})
    endif()

    # Define the RRTSHARP_USE_OPENMP flag to enable CPU parallelization
    # NOTE: Multithreading not used in standard RRT#
    target_compile_definitions(RRTsharp PUBLIC RRTSHARP_USE_OPENMP)
else()
    message(WARNING "OpenMP Not Found - Compiling without CPU parallelization support")
endif()