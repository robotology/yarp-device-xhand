set(EXE_TARGET_NAME xhand-read-test)

# YARP
find_package(YARP REQUIRED COMPONENTS
                           dev
                           os
                           sig
                           idl_tools
                           eigen
)

# Eigen3
find_package(Eigen3 REQUIRED)

# Rerun
list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "$ENV{CONDA_PREFIX}/lib")
find_package(rerun_sdk REQUIRED)

# Executable
include_directories(${EXE_TARGET_NAME}/include)

file(GLOB_RECURSE SOURCES "src/*.cpp")


add_executable(${EXE_TARGET_NAME} ${SOURCES})

target_include_directories(${EXE_TARGET_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${EXE_TARGET_NAME} PRIVATE  Eigen3::Eigen
                                                  rerun_sdk
                                                  YARP::YARP_dev
                                                  YARP::YARP_init
                                                  YARP::YARP_os
                                                  YARP::YARP_sig
                                                  YARP::YARP_eigen
                                                  xhand_control
)

# Install the executable
install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

# Install configuration file
file(GLOB config_files ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
yarp_install(FILES ${config_files} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/${EXE_TARGET_NAME})
