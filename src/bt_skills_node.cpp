#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

#ifndef USE_SLEEP_PLUGIN
  #include <btcpp_skills/sleep_action.hpp>
#endif

//-------------------------------------------------------------
// Simple Action to print a number
//-------------------------------------------------------------
class PrintValue : public BT::SyncActionNode
{
public:
  PrintValue(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override {
    std::string msg;
    if( getInput("message", msg ) ){
      std::cout << "PrintValue: " << msg << std::endl;
      return NodeStatus::SUCCESS;
    }
    else{
      std::cout << "PrintValue FAILED "<< std::endl;
      return NodeStatus::FAILURE;
    }
  }

  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("message") };
  }
};

//-------------------------------------------------------------
// Main loop: create a tree and execute it (i.e., multiple clients to action servers)
//-------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("bt_skills_node");

  BT::BehaviorTreeFactory factory;

  BT::RosNodeParams params;
  params.nh = nh;
  params.default_port_value = "sleep_service";

#ifdef USE_SLEEP_PLUGIN
  RegisterRosNode(factory, "../lib/libsleep_action_plugin.so", params);
#else
  factory.registerNodeType<SleepAction>("SleepAction", params);
#endif

  auto tree = factory.createTreeFromFile("./main_tree.xml");

  while(rclcpp::ok())
  {
    tree.tickWhileRunning();
  }

  return 0;
}