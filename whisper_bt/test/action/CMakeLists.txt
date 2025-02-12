find_package(ament_index_cpp REQUIRED)

# Test for the stt action
ament_add_gtest(test_stt_action
  test_stt_action.cpp
)
target_link_libraries(test_stt_action
  ament_index_cpp::ament_index_cpp
  whisper_stt_action_bt_node
)
ament_target_dependencies(test_stt_action
  ${dependencies}
)
