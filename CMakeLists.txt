CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
PROJECT( octomap-distribution )

ENABLE_TESTING()  # enable CTest environment of subprojects
set(CMAKE_POSITION_INDEPENDENT_CODE ON)  # enables -fPIC in applicable compilers (required to avoid link errors in some cases)

option(BUILD_OCTOVIS_SUBPROJECT "Build targets from subproject octovis" ON)
option(BUILD_DYNAMICETD3D_SUBPROJECT  "Build targets from subproject dynamicEDT3D" ON)
option(OCTOVIS_QT5 "Link Octovis against Qt5?" ON)

set(CMAKE_CXX_STANDARD 11)

ADD_SUBDIRECTORY( octomap )

if(BUILD_OCTOVIS_SUBPROJECT)
    ADD_SUBDIRECTORY( octovis )
endif()

#if(BUILD_DYNAMICETD3D_SUBPROJECT)
#    ADD_SUBDIRECTORY( dynamicEDT3D )
#endif()



#cmake_minimum_required(VERSION 3.25)
#project(ar-obstacle-vis)
#
#set(CMAKE_CXX_STANDARD 17)
#
#find_package(PCL 1.2 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#
#find_package(catkin REQUIRED COMPONENTS roscpp tf)
#include_directories(${catkin_INCLUDE_DIRS})
#
##find_package(Qt5 COMPONENTS Core Gui OpenGL Widgets Xml REQUIRED)
##include_directories(${Qt5Core_INCLUDE_DIRS})
#
#option(BUILD_OCTOVIS_SUBPROJECT "Build targets from subproject octovis" ON)
#option(BUILD_DYNAMICETD3D_SUBPROJECT  "Build targets from subproject dynamicEDT3D" ON)
#option(OCTOVIS_QT5 "Link Octovis against Qt5?" ON)
#
#add_subdirectory(octomap)
#add_subdirectory(octovis)
##ADD_SUBDIRECTORY(dynamicEDT3D)
##
##SET(viewer_UIS
##        ${CMAKE_BINARY_DIR}/include/octovis/ViewerGui.ui
##        ${CMAKE_BINARY_DIR}/include/octovis/ViewerSettings.ui
##        ${CMAKE_BINARY_DIR}/include/octovis/ViewerSettingsPanel.ui
##        ${CMAKE_BINARY_DIR}/include/octovis/ViewerSettingsPanelCamera.ui
##)
##QT5_WRAP_UI(viewer_UIS_H ${viewer_UIS})
##
##message(viewer_UIS_H: ${viewer_UIS_H})
#
##find_package(Qt5Core REQUIRED)
##find_package(Qt5Gui REQUIRED)
##find_package(Qt5OpenGL REQUIRED)
##find_package(Qt5Widgets REQUIRED)
##find_package(Qt5Xml REQUIRED)
##set(QT_LIBRARIES Qt5::Core Qt5::Gui Qt5::OpenGL Qt5::Widgets Qt5::Xml "${OPENGL_gl_LIBRARY}" "${OPENGL_glu_LIBRARY}")
##include_directories(
##        "${Qt5Core_INCLUDE_DIRS}"
##        "${Qt5Gui_INCLUDE_DIRS}"
##        "${Qt5OpenGL_INCLUDE_DIRS}"
##        "${Qt5Widgets_INCLUDE_DIRS}"
##        "${Qt5Xml_INCLUDE_DIRS}"
##)
##FIND_PACKAGE(QGLViewer)
##include_directories(${QGLViewer_INCLUDE_DIR})
#
#add_executable(ar-obstacle-vis-main main.cpp)
#target_link_libraries(ar-obstacle-vis-main octomap viewer_SRCS ${PCL_LIBRARIES} ${catkin_LIBRARIES})
#target_compile_options(ar-obstacle-vis-main PRIVATE -fPIC)
