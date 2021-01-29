cmake_minimum_required(VERSION 3.5.1)

set(CMAKE_CXX_STANDARD 17)

project(maze_cv_stitching
        VERSION 2019.1.0
        DESCRIPTION "Agent cell world simulation framework"
        LANGUAGES CXX)

find_package( Cellworld REQUIRED )
find_package( OpenCV 4.0.1 REQUIRED )

###
### MAIN LIBRARY SETUP
###

set( maze_cv_stitching_files
        src/stitcher.cpp
)

add_library(maze_cv_stitching ${maze_cv_stitching_files})

target_link_libraries(maze_cv_stitching ${OpenCV_LIBS})
target_link_libraries(maze_cv_stitching cellworld)

target_include_directories(maze_cv_stitching
        SYSTEM INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_include_directories(maze_cv_stitching
        PRIVATE
        include )

add_executable(test src/test.cpp)
target_link_libraries(test maze_cv_stitching)

###
### TESTS
###
find_package(CatchTests CONFIG QUIET)


###
### LIBRARY INSTALLATION
###

if (CMAKE_BUILD_TYPE MATCHES Release)
    export(TARGETS maze_cv_stitching FILE MazeCVStitchingConfig.cmake)

    include(GNUInstallDirs)

    install(TARGETS maze_cv_stitching EXPORT Maze_cv_stitchingConfig
            ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
            LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})

    install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

    install(EXPORT Maze_cv_stitchingConfig
            DESTINATION ${CMAKE_INSTALL_DATADIR}/cmake/Maze_cv_stitching
            EXPORT_LINK_INTERFACE_LIBRARIES)

    export(TARGETS maze_cv_stitching FILE Maze_cv_stitchingConfig.cmake)
endif()
