cmake_minimum_required(VERSION 3.5.1)

set(CMAKE_CXX_STANDARD 20)

project(habitat_cv
        VERSION 2019.1.0
        DESCRIPTION "A very opinionated open cv implementation for research"
        LANGUAGES CXX)

string(APPEND CMAKE_CXX_FLAGS " -DC_GNU64=400 -DOS_LINUX -no-pie -Wall -Wextra -pthread ")
string(APPEND CMAKE_CXX_FLAGS_RELEASE " -O3 ")

find_package( Dependencies )

install_dependency( https://github.com/germanespinosa/agent_tracking Agent_tracking )

install_dependency ( https://github.com/germanespinosa/cellworld Cellworld )
dependency_include (include)
add_dependency_package (OpenCV)
dependency_include (${OpenCV_INCLUDE_DIRS})

dependency_include (/usr/local/xclib/inc/)

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
        src/camera_array.cpp
        src/frame_rate.cpp
        src/layouts.cpp
        src/background.cpp
        src/cv_service.cpp)

add_library(habitat_cv ${habitat_cv_files})

target_link_libraries(habitat_cv
        LINK_PUBLIC
        cellworld
        ${OpenCV_LIBS}
        /usr/local/xclib/lib/pxipl_x86_64.a
        /usr/local/xclib/lib/xclib_x86_64.a m -pthread
        agent_tracking )

add_executable(agent_tracker
        src/tools/agent_tracker.cpp)

target_link_libraries(agent_tracker
        cellworld
        ${OpenCV_LIBS}
        /usr/local/xclib/lib/pxipl_x86_64.a
        /usr/local/xclib/lib/xclib_x86_64.a m -pthread
        agent_tracking
        habitat_cv)


###
### TESTS
###
install_dependency ( https://github.com/germanespinosa/catch )
test_library(habitat_cv)

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
