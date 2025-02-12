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

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"

#include "nav2_behavior_tree/utils/test_action_server.hpp"
#include "whisper_bt/action/stt_action.hpp"

class STTActionServer
  : public TestActionServer<whisper_msgs::action::STT>
{
public:
  STTActionServer()
  : TestActionServer("stt")
  {
  }

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<whisper_msgs::action::STT>> goal_handle)
  override
  {
    whisper_msgs::action::STT::Result::SharedPtr result =
      std::make_shared<whisper_msgs::action::STT::Result>();
    result->transcription.text = "This is a test transcription";
    bool return_success = getReturnSuccess();
    if (return_success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }
};

class STTActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("stt_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set("node", node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout", std::chrono::milliseconds(20));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration", std::chrono::milliseconds(10));
    config_->blackboard->set<std::chrono::milliseconds>(
      "wait_for_service_timeout", std::chrono::milliseconds(1000));

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<whisper_bt::STTAction>(name, "stt", config);
      };

    factory_->registerBuilder<whisper_bt::STTAction>("STT", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    server_.reset();
    factory_.reset();
  }

  void SetUp() override
  {
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<STTActionServer> server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr STTActionTestFixture::node_ = nullptr;
std::shared_ptr<STTActionServer> STTActionTestFixture::server_ = nullptr;
BT::NodeConfiguration * STTActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> STTActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> STTActionTestFixture::tree_ = nullptr;

TEST_F(STTActionTestFixture, test_ports)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <STT/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <STT prompt="" grammar_config="" transcription="{transcription}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_TRUE(tree_->rootNode()->getInput<std::string>("prompt").value().empty());

  auto grammar_config =
    tree_->rootNode()->getInput<whisper_msgs::msg::GrammarConfig>("grammar_config").value();
  EXPECT_TRUE(grammar_config.grammar.empty());
  EXPECT_TRUE(grammar_config.start_rule.empty());
  EXPECT_EQ(grammar_config.grammar_penalty, 100.0);

  xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <STT prompt="This is a test" grammar_config="This/test/50.0" transcription="{transcription}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->getInput<std::string>("prompt"), "This is a test");
  grammar_config =
    tree_->rootNode()->getInput<whisper_msgs::msg::GrammarConfig>("grammar_config").value();
  EXPECT_EQ(grammar_config.grammar, "This");
  EXPECT_EQ(grammar_config.start_rule, "test");
  EXPECT_EQ(grammar_config.grammar_penalty, 50.0);
}

TEST_F(STTActionTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <STT prompt="" grammar_config="" transcription="{transcription}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_TRUE(tree_->rootNode()->getInput<std::string>("prompt").value().empty());
  auto grammar_config =
    tree_->rootNode()->getInput<whisper_msgs::msg::GrammarConfig>("grammar_config").value();
  EXPECT_TRUE(grammar_config.grammar.empty());
  EXPECT_TRUE(grammar_config.start_rule.empty());
  EXPECT_EQ(grammar_config.grammar_penalty, 100.0);

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);

  // Check if the output is correct
  auto transcription = config_->blackboard->get<std::string>("transcription");
  EXPECT_EQ(transcription, "This is a test transcription");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize service and spin on new thread
  STTActionTestFixture::server_ = std::make_shared<STTActionServer>();
  std::thread server_thread([]() {
      rclcpp::spin(STTActionTestFixture::server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  std::cout << "All tests passed: " << all_successful << std::endl;

  return all_successful;
}
