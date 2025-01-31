// Copyright (c) 2025 Alberto J. Tudela Roldán
// Copyright (c) 2025 Grupo Avispa, DTE, Universidad de Málaga
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>

#include "whisper_bt/action/stt_action.hpp"

namespace whisper_bt
{

STTAction::STTAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<whisper_msgs::action::STT>(xml_tag_name, action_name, conf)
{
}

void STTAction::on_tick()
{
  std::string prompt;
  getInput("prompt", prompt);
  whisper_msgs::msg::GrammarConfig grammar_config;
  getInput("grammar_config", grammar_config);

  goal_.prompt = prompt;
  goal_.grammar_config = grammar_config;
}

BT::NodeStatus STTAction::on_success()
{
  setOutput("transcription", result_.result->transcription.text);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace whisper_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<whisper_bt::STTAction>(name, "listen", config);
    };

  factory.registerBuilder<whisper_bt::STTAction>("STT", builder);
}
