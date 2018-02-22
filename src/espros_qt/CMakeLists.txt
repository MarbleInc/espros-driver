##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.0.0)
project(espros_qt)

add_compile_options(-std=c++11)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker

#################################################
# Find the Qt module libraries
#################################################
find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5OpenGL)
find_package(Qt5Xml)
find_package(Qt5Network REQUIRED)

if(${Qt5Core_FOUND})
  if(${Qt5Widgets_FOUND})
    MESSAGE(STATUS "Found Qt ${Qt5Widgets_VERSION_STRING}")
    # FIX: Qt was built with -reduce-relocations
    if (Qt5_POSITION_INDEPENDENT_CODE)
      SET(CMAKE_POSITION_INDEPENDENT_CODE ON)
      add_definitions(-fPIC)
    endif()
  endif(${Qt5Widgets_FOUND})

  set(QT_INCLUDE_DIRS "${Qt5Core_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS} ${Qt5Xml_INCLUDE_DIRS}")
  get_target_property(QT_QTCORE_LIB Qt5::Core LOCATION)
  get_target_property(QT_QTXML_LIB Qt5::Xml LOCATION)
  get_target_property(QT_QTWIDGET_LIB Qt5::Widgets LOCATION)
  get_target_property(QT_QTGUI_LIB Qt5::Gui LOCATION)
  set(QT_QTGUI_LIB ${QT_QTGUI_LIB} ${QT_QTWIDGET_LIB})
  get_target_property(QT_QTOPENGL_LIB Qt5::OpenGL LOCATION)
  get_target_property(QT_QTNETWORK_LIB Qt5::Network LOCATION)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs)

include_directories(${catkin_INCLUDE_DIRS} ${QT_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/)

# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything.
catkin_package()

##############################################################################
# Sections
##############################################################################

file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC_HPP RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/espros_qt/*.hpp)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/espros_qt/*.h)

# QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
# QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
# QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

qt5_add_resources(QT_RESOURCES_CPP ${QT_RESOURCES})
#qt5_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
qt5_wrap_cpp(QT_MOC ${QT_MOC_HPP} ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(espros_qt ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC})
target_link_libraries(espros_qt ${QT_QTCORE_LIB} ${QT_QTGUI_LIB} ${QT_QTNETWORK_LIB} ${catkin_LIBRARIES})
install(TARGETS espros_qt RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
