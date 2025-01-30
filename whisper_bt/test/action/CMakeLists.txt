# Test for the generate stt action
ament_add_gtest(test_stt_action
  test_stt_action.cpp
)
target_link_libraries(test_stt_action
  whisper_stt_action_bt_node
)
ament_target_dependencies(test_stt_action
  ${dependencies}
)
