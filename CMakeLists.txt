cmake_minimum_required(VERSION 4.0.0)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME ExplicitCompCtrl)
set(PROJECT_DESCRIPTION "ExplicitCompCtrl")
set(PROJECT_URL "")

project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION 1.0.0)
include(CTest)
enable_testing()

if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_subdirectory(src)

configure_file(etc/ExplicitCompCtrl.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/ExplicitCompCtrl.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/ExplicitCompCtrl.yaml"
        DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
