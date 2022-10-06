cmake_minimum_required(VERSION 3.5.1)

set(CMAKE_CXX_STANDARD 20)

project(habitat_cv
        VERSION 2019.1.0
        DESCRIPTION "A very opinionated open cv implementation for research"
        LANGUAGES CXX)

string(APPEND CMAKE_CXX_FLAGS " -DC_GNU64=400 -DOS_LINUX -no-pie -Wall -Wextra -pthread ")
string(APPEND CMAKE_CXX_FLAGS_RELEASE " -O3 ")

####
#### DEPENDENCIES
####

find_package (Dependencies QUIET)

if (NOT ${Dependencies_FOUND})
    if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/DependenciesConfig.cmake)
        file(DOWNLOAD https://raw.githubusercontent.com/germanespinosa/dependencies/main/DependenciesConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/DependenciesConfig.cmake)
    endif()
    set(Dependencies_DIR "${CMAKE_CURRENT_BINARY_DIR}")
    find_package (Dependencies REQUIRED)
endif()

####
#### END DEPENDENCIES
####

include_directories (include)

find_package (OpenCV REQUIRED)
find_package(CUDA REQUIRED)

include_directories (${OpenCV_INCLUDE_DIRS})
include_directories (/usr/local/xclib/inc/)

###
### MAIN LIBRARY SETUP
###

set( habitat_cv_files
        src/detection.cpp
        src/composite.cpp
        src/camera_configuration.cpp
        src/image.cpp
        src/video.cpp
        src/layout.cpp
        src/camera.cpp
        src/camera_array.cpp
        src/frame_rate.cpp
        src/layouts.cpp
        src/background.cpp
        src/cv_service.cpp
        src/camera.cpp)

add_library(habitat_cv ${habitat_cv_files})

install_git_dependency ( Controller
        https://github.com/germanespinosa/cellworld_controller
        CMAKE_PROJECT
        IMPORT_TARGETS
        controller
        CONNECTED)

install_git_dependency( Perf-analysis
        https://github.com/germanespinosa/perf-analysis
        CMAKE_PROJECT
        IMPORT_TARGETS perf-analysis
        INCLUDE_DIRECTORIES include)

install_git_dependency ( Robot_lib
        https://github.com/germanespinosa/robot_library
        CMAKE_PROJECT
        IMPORT_TARGETS
        gamepad_lib
        cellworld
        requests
        json-cpp
        params-cpp
        easy-tcp
        tcp_messages
        agent_tracking
        experiment_service
        robot_lib
        CONNECTED)

target_link_libraries(habitat_cv
        PRIVATE
        cellworld
        ${OpenCV_LIBS}
        /usr/local/xclib/lib/pxipl_x86_64.a
        /usr/local/xclib/lib/xclib_x86_64.a m -pthread
        agent_tracking
        robot_lib
        perf-analysis)

add_executable(agent_tracker
        src/tools/agent_tracker.cpp )

add_executable(camera_test
        src/tools/camera_test.cpp )


target_link_libraries(agent_tracker habitat_cv )

target_link_libraries(camera_test
        cellworld
        ${OpenCV_LIBS}
        /usr/local/xclib/lib/pxipl_x86_64.a
        /usr/local/xclib/lib/xclib_x86_64.a m -pthread)

###
### TESTS
###
#install_dependency ( https://github.com/germanespinosa/catch )
#test_library(habitat_cv catchtests/camera_array.cpp)

###
### LIBRARY INSTALLATION
###

export(TARGETS habitat_cv FILE HabitatCvConfig.cmake)

include(GNUInstallDirs)

install(TARGETS habitat_cv EXPORT Habitat_cvConfig
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT Habitat_cvConfig
        DESTINATION ${CMAKE_INSTALL_DATADIR}/cmake/Habitat_cv
        EXPORT_LINK_INTERFACE_LIBRARIES)

export(TARGETS habitat_cv FILE Habitat_cvConfig.cmake)
