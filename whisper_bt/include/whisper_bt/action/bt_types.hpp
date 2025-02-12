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

#ifndef WHISPER_BT__ACTION__BT_TYPES_HPP_
#define WHISPER_BT__ACTION__BT_TYPES_HPP_

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/json_export.h"
#include "whisper_msgs/msg/grammar_config.hpp"
#include "whisper_msgs/msg/transcription.hpp"

// Allow bi-directional convertion to JSON
BT_JSON_CONVERTER(whisper_msgs::msg::GrammarConfig, grammar)
{
  add_field("grammar", &grammar.grammar);
  add_field("start_rule", &grammar.start_rule);
  add_field("grammar_penalty", &grammar.grammar_penalty);
}

// Template specialization to converts a string to Goal.
namespace BT
{
template<>
[[nodiscard]] whisper_msgs::msg::GrammarConfig convertFromString(BT::StringView str)
{
  if (StartWith(str, "json:")) {
    str.remove_prefix(5);
    return convertFromJSON<whisper_msgs::msg::GrammarConfig>(str);
  }

  whisper_msgs::msg::GrammarConfig output;
  if (!str.empty()) {
    // We expect values separated by /
    auto parts = splitString(str, '/');
    if (parts.size() != 3) {
      throw RuntimeError("invalid input)");
    } else {
      output.grammar = convertFromString<std::string>(parts[0]);
      output.start_rule = convertFromString<std::string>(parts[1]);
      output.grammar_penalty = convertFromString<float>(parts[2]);
    }
  }
  return output;
}

template<>
[[nodiscard]] whisper_msgs::msg::Transcription convertFromString(BT::StringView str)
{
  if (StartWith(str, "json:")) {
    str.remove_prefix(5);
    return convertFromJSON<whisper_msgs::msg::Transcription>(str);
  }

  whisper_msgs::msg::Transcription output;
  if (!str.empty()) {
    // We expect real numbers separated by /
    auto parts = splitString(str, '/');
    if (parts.size() != 3) {
      throw RuntimeError("invalid input)");
    } else {
      output.text = convertFromString<std::string>(parts[0]);
      output.audio_time = convertFromString<float>(parts[1]);
      output.transcription_time = convertFromString<float>(parts[2]);
    }
  }
  return output;
}
}  // namespace BT

#endif  // WHISPER_BT__ACTION__BT_TYPES_HPP_
