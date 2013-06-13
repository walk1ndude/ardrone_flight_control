cmake_minimum_required(VERSION 2.8.3)
project(ardrone_flight_control)

find_package(catkin REQUIRED COMPONENTS ardrone_autonomy cv_bridge
    message_generation image_transport)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)

include(${QT_USE_FILE})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

catkin_package()

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(droneui ${QT_SOURCES} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(droneui ${QT_LIBRARIES} ${catkin_LIBRARIES})
