set(DUAL_ARM_PLANNER_LIB dual_arm_manipulation_planner_interface)
find_package(GTest REQUIRED)

add_library(${DUAL_ARM_PLANNER_LIB}
  src/hybrid_object_state_space.cpp
  src/hybrid_state_validity_checker.cpp
  src/hybrid_motion_validator.cpp
  src/hybrid_valid_state_sampler.cpp
  src/davinci_needle_handoff_execution_manager.cpp
  src/hybrid_object_handoff_planner.cpp
  src/threadsafe_state_storage.cpp
)
set_target_properties(${DUAL_ARM_PLANNER_LIB} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

#find_package(OpenMP)

target_link_libraries(${DUAL_ARM_PLANNER_LIB} davinci_needle_pose_publisher ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#set_target_properties(${DUAL_ARM_PLANNER_LIB} PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#set_target_properties(${DUAL_ARM_PLANNER_LIB} PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

add_library(hybrid_object_handoff_planner_tester test/hybrid_object_handoff_planner_tester.cpp)
target_link_libraries(hybrid_object_handoff_planner_tester ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(hybrid_motion_validator_tester test/hybrid_motion_validator_tester.cpp)
target_link_libraries(hybrid_motion_validator_tester ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(hybrid_object_state_space_tester test/hybrid_object_state_space_tester.cpp)
target_link_libraries(hybrid_object_state_space_tester ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(handoff_main src/handoff_main.cpp)
target_link_libraries(handoff_main ${DUAL_ARM_PLANNER_LIB} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(davinci_needle_handoff_performance_test test/davinci_needle_handoff_performance_test.cpp)
target_link_libraries(davinci_needle_handoff_performance_test ${DUAL_ARM_PLANNER_LIB} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(davinci_needle_handoff_execution_main src/davinci_needle_handoff_execution_main.cpp)
target_link_libraries(davinci_needle_handoff_execution_main ${DUAL_ARM_PLANNER_LIB} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(davinci_needle_handoff_perturbation_test test/davinci_needle_handoff_perturbation_test.cpp)
target_link_libraries(davinci_needle_handoff_perturbation_test ${DUAL_ARM_PLANNER_LIB} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(hybrid_planner_planning_data_visualization script/hybrid_planner_planning_data_visualization.cpp)
target_link_libraries(hybrid_planner_planning_data_visualization ${PYTHON_INCLUDE_DIRS} ${DUAL_ARM_PLANNER_LIB} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_include_directories(hybrid_planner_planning_data_visualization PRIVATE ${PYTHON_INCLUDE_DIRS})

#install(TARGETS ${DUAL_ARM_PLANNER_LIB} moveit_ompl_planner moveit_ompl_planner_plugin
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(DIRECTORY include/ DESTINATION include)
#
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(ompl REQUIRED)

  add_rostest_gtest(test_hybrid_object_state_space test/test_hybrid_object_state_space.test test/test_hybrid_object_state_space.cpp)
  target_link_libraries(test_hybrid_object_state_space hybrid_object_state_space_tester ${DUAL_ARM_PLANNER_LIB} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

  add_rostest_gtest(test_hybrid_motion_validator test/test_hybrid_motion_validator.test test/test_hybrid_motion_validator.cpp)
  target_link_libraries(test_hybrid_motion_validator hybrid_motion_validator_tester ${DUAL_ARM_PLANNER_LIB} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

  add_rostest_gtest(test_hybrid_object_handoff_planner test/test_hybrid_object_handoff_planner_hardware.test test/test_hybrid_object_handoff_planner.cpp)
  target_link_libraries(test_hybrid_object_handoff_planner hybrid_object_handoff_planner_tester ${DUAL_ARM_PLANNER_LIB} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif()
