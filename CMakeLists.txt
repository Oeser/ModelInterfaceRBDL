# 
#  Copyright (C) 2016 IIT-ADVR
#  Author: Arturo Laurenzi, Luca Muratore
#  email: arturo.laurenzi@iit.it, luca.muratore@iit.it
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>
#

cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)

include(ExternalProject)

project(ModelInterfaceRBDL)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )

# C++ 11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(XBotCoreModel REQUIRED)
find_package(XBotInterface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sharedlib++ REQUIRED)
find_package(orocos_kdl REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(RBDL REQUIRED)

message("*************************${kdl_orocos_INCLUDE_DIRS}")
include_directories(include ${XBotCoreModel_INCLUDE_DIRS} 
                            ${XBotInterface_INCLUDE_DIRS}
                            ${orocos_kdl_INCLUDE_DIRS}
                            ${EIGEN3_INCLUDE_DIR}
                            ${CMAKE_SOURCE_DIR}/IXBotInterface/include
                            ${sharedlib++_INCLUDE_DIRS}
                            ${RBDL_INCLUDE_DIR}
                            ${RBDL_URDFReader_INCLUDE_DIR}
                            )
                            
# for every file in INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE ModelInterfaceRBDL_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h*)

add_library(ModelInterfaceRBDL SHARED ${ModelInterfaceRBDL_INCLUDES} 
                                     src/ModelInterfaceRBDL.cpp)
                                 
target_link_libraries(ModelInterfaceRBDL ${XBotInterface_LIBRARIES}
                                        yaml-cpp 
                                        ${sharedlib++_LIBRARIES}
                                        ${RBDL_LIBRARY}
                                        ${RBDL_URDFReader_LIBRARY}
                                        )
                                       

                                       
########################################################################

set(VARS_PREFIX "ModelInterfaceRBDL")

set(ModelInterfaceRBDL_MAJOR_VERSION 0)
set(ModelInterfaceRBDL_MINOR_VERSION 1)
set(ModelInterfaceRBDL_PATCH_VERSION 0)
set(ModelInterfaceRBDL_VERSION ${ModelInterfaceRBDL_MAJOR_VERSION}.${ModelInterfaceRBDL_MINOR_VERSION}.${ModelInterfaceRBDL_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(ModelInterfaceRBDL PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(ModelInterfaceRBDL PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS ModelInterfaceRBDL
        EXPORT ModelInterfaceRBDL
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all ModelInterfaceRBDL dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ModelInterfaceRBDL)


include(InstallBasicPackageFiles)
install_basic_package_files(ModelInterfaceRBDL VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

include(AddUninstallTarget)