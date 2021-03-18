cmake_minimum_required(VERSION 3.1)
project(idocp-sim CXX)
set(CMAKE_CXX_STANDARD 11)

###########
## Build ##
###########
if(NOT CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "WARNING: CMAKE_BUILD_TYPE not set to Release, which can decrease performance significantly")
endif()
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)
# find Eigen
find_package(Eigen3 REQUIRED)
# find Boost
find_package(Boost COMPONENTS system filesystem REQUIRED)
# find pinocchio
find_package(pinocchio REQUIRED)
# find raisim
find_package(raisim CONFIG REQUIRED)
# find raisimOgre
find_package(raisimOgre CONFIG REQUIRED)

# build idocp-sim
add_library(
  ${PROJECT_NAME} 
  SHARED
  src/idocp-sim.cpp
  src/sim_data_logger.cpp
  src/raisim_wrapper.cpp
)
target_link_libraries(
  ${PROJECT_NAME} 
  PUBLIC 
  pthread
  ${Boost_LIBRARIES}
  ${PINOCCHIO_LIBRARIES}
  urdfdom_model
  raisim::raisim
  raisim::raisimOgre
)
target_include_directories(
  ${PROJECT_NAME} 
  PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${PINOCCHIO_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if (BUILD_EXAMPLE)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/examples)
endif() # end if (TESTING)

#############
## Install ##
#############
include(GNUInstallDirs)
# Install lib files
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-config
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
)
# Install header files
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)
# Install config file 
set(config_package_location "share/${PROJECT_NAME}/cmake")
install(
  EXPORT ${PROJECT_NAME}-config
  FILE ${PROJECT_NAME}-config.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${config_package_location}
)
# Install alias
add_library(
  ${PROJECT_NAME}::${PROJECT_NAME} 
  ALIAS ${PROJECT_NAME}
)

