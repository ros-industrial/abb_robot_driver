cmake_minimum_required(VERSION 3.9.1)

########################################################################################################################
# Metadata
########################################################################################################################
# Read the package manifest.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" package_xml_str)

# Extract project name.
if(NOT package_xml_str MATCHES "<name>([A-Za-z0-9_]+)</name>")
  message(FATAL_ERROR "Could not parse project name from package manifest (aborting)")
else()
  set(extracted_name ${CMAKE_MATCH_1})
endif()

# Extract project version.
if(NOT package_xml_str MATCHES "<version>([0-9]+.[0-9]+.[0-9]+)</version>")
  message(FATAL_ERROR "Could not parse project version from package manifest (aborting)")
else()
  set(extracted_version ${CMAKE_MATCH_1})
endif()

########################################################################################################################
# CMake project
########################################################################################################################
project(${extracted_name} VERSION ${extracted_version} LANGUAGES CXX)

#===========================================================
# Dependencies
#===========================================================
#-----------------------------
# ABB robot controller
# communication package
#-----------------------------
find_package(abb_egm_rws_managers REQUIRED)

# Extract include directories, because some subsequent
# catkin packages can't seem to build without this.
get_target_property(abb_libegm_INCLUDE_DIRS ${abb_libegm_LIBRARIES} INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(abb_librws_INCLUDE_DIRS ${abb_librws_LIBRARIES} INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(abb_egm_rws_managers_INCLUDE_DIRS ${abb_egm_rws_managers_LIBRARIES} INTERFACE_INCLUDE_DIRECTORIES)

#-----------------------------
# Catkin packages
#-----------------------------
find_package(catkin REQUIRED
  COMPONENTS
    abb_egm_msgs
    abb_rapid_msgs
    abb_rapid_sm_addin_msgs
    abb_robot_msgs
    roscpp
)

#===========================================================
# Catkin package specific configurations
#===========================================================
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    abb_egm_msgs
    abb_rapid_msgs
    abb_rapid_sm_addin_msgs
    abb_robot_msgs
    roscpp
  DEPENDS
    abb_egm_rws_managers
    abb_libegm
    abb_librws
    Boost
    Poco
    Protobuf
)

#===========================================================
# Settings
#===========================================================
if(NOT DEFINED BUILD_SHARED_LIBS)
  set(BUILD_SHARED_LIBS ON)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message(STATUS "${PROJECT_NAME}: Defaulting build type to RelWithDebInfo")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

#===========================================================
# Build targets
#===========================================================
set(target_name ${PROJECT_NAME})

#-----------------------------
# Library target
#-----------------------------
set(
  src_files
    src/initialization.cpp
    src/mapping.cpp
    src/parameters.cpp
    src/verification.cpp
)

add_library(${target_name} ${src_files})

target_include_directories(${target_name} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>"
  "${catkin_INCLUDE_DIRS}"
)

target_link_libraries(${target_name} PUBLIC
  abb_egm_rws_managers::abb_egm_rws_managers
  ${catkin_LIBRARIES}
)

target_compile_features(${target_name} PRIVATE cxx_std_14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  target_compile_options(${target_name} PRIVATE /W4)
endif()

#===========================================================
# Install
#===========================================================
install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
  LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
  RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
)
