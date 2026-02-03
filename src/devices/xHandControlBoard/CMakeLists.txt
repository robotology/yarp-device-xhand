# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-2-Clause

set(plugin_name xHandControlBoard)

yarp_prepare_plugin(${plugin_name}
  CATEGORY device
  TYPE yarp::dev::${plugin_name}
  INCLUDE xHandControlBoard.h
  OPTION ENABLE_xHandControlBoard
  EXTRA_CONFIG
    WRAPPER=controlBoard_nws_yarp
  DEFAULT ON
)

if (SKIP_${plugin_name})
  message(STATUS "Skipping ${plugin_name} plugin")
  return()
endif()


yarp_add_plugin(${plugin_name})
generateDeviceParamsParser(${plugin_name} ${plugin_name})

add_library(${plugin_name}_lib
    xHandControlBoard.cpp
    xHandControlBoard.h
    xHandControlBoard_ParamsParser.h
    xHandControlBoard_ParamsParser.cpp
    ControlBoardTrajectory.h
    ControlBoardTrajectory.cpp
)

target_link_libraries(${plugin_name}_lib
  PUBLIC
  YARP::YARP_os
  YARP::YARP_sig
  YARP::YARP_dev
  xhand_control
)

target_include_directories(${plugin_name}_lib
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_compile_features(${plugin_name}_lib PUBLIC cxx_std_17)

target_link_libraries(${plugin_name}
  PUBLIC
  ${plugin_name}_lib
)

yarp_install(
  TARGETS ${plugin_name}
  EXPORT ${plugin_name}
  COMPONENT ${plugin_name}
  LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
  ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
  YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
)
