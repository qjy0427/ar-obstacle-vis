CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
PROJECT(ar-obstacle-vis)

ENABLE_TESTING()  # enable CTest environment of subprojects
set(CMAKE_POSITION_INDEPENDENT_CODE ON)  # enables -fPIC in applicable compilers (required to avoid link errors in some cases)

option(BUILD_OCTOVIS_SUBPROJECT "Build targets from subproject octovis" ON)
option(BUILD_DYNAMICETD3D_SUBPROJECT  "Build targets from subproject dynamicEDT3D" ON)
option(OCTOVIS_QT5 "Link Octovis against Qt5?" ON)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

ADD_SUBDIRECTORY( octomap )

if(BUILD_OCTOVIS_SUBPROJECT)
    ADD_SUBDIRECTORY( octovis )
endif()
