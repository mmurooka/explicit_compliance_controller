set(controller_SRC ExplicitCompCtrl.cpp)

set(controller_HDR ExplicitCompCtrl.h)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS
                                                 "-DExplicitCompCtrl_EXPORTS")
if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc ROS plugin is required to build this controller")
endif()
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm
                                             mc_rtc::mc_rtc_ros)
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)

add_controller(${PROJECT_NAME}_controller lib.cpp "")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

add_subdirectory(states)
