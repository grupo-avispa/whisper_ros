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

#include <behaviortree_cpp_v3/behavior_tree.h>

#include <string>

#include "whisper_msgs/msg/grammar_config.hpp"
#include "whisper_msgs/msg/transcription.hpp"

// Template specialization to converts a string to Goal.
namespace BT
{
template<> inline whisper_msgs::msg::GrammarConfig convertFromString(BT::StringView str)
{
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

template<> inline whisper_msgs::msg::Transcription convertFromString(BT::StringView str)
{
  // We expect real numbers separated by /
  auto parts = splitString(str, '/');
  if (parts.size() != 3) {
    throw RuntimeError("invalid input)");
  } else {
    whisper_msgs::msg::Transcription output;
    output.text = convertFromString<std::string>(parts[0]);
    output.audio_time = convertFromString<float>(parts[1]);
    output.transcription_time = convertFromString<float>(parts[2]);
    return output;
  }
}
}  // namespace BT

#endif  // WHISPER_BT__ACTION__BT_TYPES_HPP_
