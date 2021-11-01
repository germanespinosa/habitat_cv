cmake_minimum_required(VERSION 3.5.1)

set(CMAKE_CXX_STANDARD 20)

project(habitat_cv
        VERSION 2019.1.0
        DESCRIPTION "Agent cell world simulation framework"
        LANGUAGES CXX)

find_package( Cellworld REQUIRED )
find_package( OpenCV 4.0.1 REQUIRED )

###
### MAIN LIBRARY SETUP
###

set( habitat_cv_files
        src/detection.cpp
        src/composite.cpp
        src/camera_configuration.cpp
        src/image.cpp catchtests/image.cpp)

add_library(habitat_cv ${habitat_cv_files})

target_link_libraries(habitat_cv ${OpenCV_LIBS})
target_link_libraries(habitat_cv ${CELLWORLD_LIBRARIES})

target_include_directories(habitat_cv
        SYSTEM INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_include_directories(habitat_cv
        PRIVATE
        include )


###
### TESTS
###
find_package(CatchTests CONFIG QUIET)


###
### LIBRARY INSTALLATION
###

#if (CMAKE_BUILD_TYPE MATCHES Release)
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
#endif()
