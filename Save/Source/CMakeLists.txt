cmake_minimum_required(VERSION 2.6)
file(TO_CMAKE_PATH "$ENV{IMAGINEPP_ROOT}/CMake" p)
list(APPEND CMAKE_MODULE_PATH "${p}") #For old Imagine++
list(APPEND CMAKE_SYSTEM_FRAMEWORK_PATH /Library/Frameworks) #Mac, why not auto?


link_directories(${CMAKE_SOURCE_DIR}/lib)

project(Thunder)
add_executable(Thunder main.cpp SerialPort.cpp SerialPort.h animation.cpp animation.h analyse.cpp analyse.h Tools.cpp Tools.h)
target_link_libraries(Thunder libsfml-audio)
