set(MOVEIT_LIB_NAME dual_arm_manipulation_planner_interface)

add_library(${MOVEIT_LIB_NAME}
  src/hybrid_object_state_space.cpp
  src/hybrid_state_validity_checker.cpp
  src/hybrid_motion_validator.cpp
  src/hybrid_valid_state_sampler.cpp
  src/davinci_needle_handoff_execution.cpp
  src/threadsafe_state_storage.cpp
  #  src/planning_context_manager.cpp
  #  src/constraints_library.cpp
  #  src/model_based_planning_context.cpp
  #  src/parameterization/model_based_state_space.cpp
  #  src/parameterization/model_based_state_space_factory.cpp
  #  src/parameterization/joint_space/joint_model_state_space.cpp
  #  src/parameterization/joint_space/joint_model_state_space_factory.cpp
  #  src/parameterization/work_space/pose_model_state_space.cpp
  #  src/parameterization/work_space/pose_model_state_space_factory.cpp
  #  src/detail/threadsafe_state_storage.cpp
#  src/detail/projection_evaluators.cpp
#  src/detail/goal_union.cpp
#  src/detail/constrained_sampler.cpp
#  src/detail/constrained_valid_state_sampler.cpp
#  src/detail/constrained_goal_sampler.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

#find_package(OpenMP)

target_link_libraries(${MOVEIT_LIB_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

add_library(hybrid_motion_validator_tester test/hybrid_motion_validator_tester.cpp)
target_link_libraries(hybrid_motion_validator_tester ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(hybrid_object_state_space_tester test/hybrid_object_state_space_tester.cpp)
target_link_libraries(hybrid_object_state_space_tester ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(main src/main.cpp)
target_link_libraries(main ${MOVEIT_LIB_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(davinci_needle_handoff_performance_test test/davinci_needle_handoff_performance_test.cpp)
target_link_libraries(davinci_needle_handoff_performance_test ${MOVEIT_LIB_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(hybrid_planner_planning_data_visualization script/hybrid_planner_planning_data_visualization.cpp)
target_link_libraries(hybrid_planner_planning_data_visualization ${PYTHON_INCLUDE_DIRS} ${MOVEIT_LIB_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_include_directories(hybrid_planner_planning_data_visualization PRIVATE ${PYTHON_INCLUDE_DIRS})
#add_executable(moveit_ompl_planner src/ompl_planner.cpp)
#target_link_libraries(moveit_ompl_planner ${MOVEIT_LIB_NAME})
#set_target_properties(moveit_ompl_planner PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")
#
#add_executable(moveit_demo_construct_constraints_database src/demo_construct_state_database.cpp)
#target_link_libraries(moveit_demo_construct_constraints_database ${MOVEIT_LIB_NAME})
#set_target_properties(moveit_demo_construct_constraints_database PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")
#
#add_library(moveit_ompl_planner_plugin src/ompl_planner_manager.cpp)
#set_target_properties(moveit_ompl_planner_plugin PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
#target_link_libraries(moveit_ompl_planner_plugin ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#
#install(TARGETS ${MOVEIT_LIB_NAME} moveit_ompl_planner moveit_ompl_planner_plugin
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(DIRECTORY include/ DESTINATION include)
#
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(ompl REQUIRED)
#  find_package(moveit_resources REQUIRED)
#  include_directories(${moveit_resources_INCLUDE_DIRS})

  add_rostest_gtest(test_hybrid_object_state_space test/test_hybrid_object_state_space.test test/test_hybrid_object_state_space.cpp)
  target_link_libraries(test_hybrid_object_state_space hybrid_object_state_space_tester ${MOVEIT_LIB_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#  set_target_properties(test_state_space PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")
  add_rostest_gtest(test_hybrid_motion_validator test/test_hybrid_motion_validator.test test/test_hybrid_motion_validator.cpp)
  target_link_libraries(test_hybrid_motion_validator hybrid_motion_validator_tester ${MOVEIT_LIB_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif()
