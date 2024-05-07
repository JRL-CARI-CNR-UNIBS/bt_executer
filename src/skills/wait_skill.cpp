#include <bt_executer/skills/wait_skill.hpp>
#include <behaviortree_ros2/plugins.hpp>

WaitSkill::WaitSkill(const std::string& name,
            const NodeConfig& conf,
            const RosNodeParams& params)
    : RosActionNode<btcpp_ros2_interfaces::action::Sleep>(name, conf, params)
{
  auto param_ns = getInput<std::string>("param_ns");
  ns_ = "/bt_executer/" + param_ns.value();
}

bool WaitSkill::setGoal(RosActionNode::Goal &goal)
{
  int wait_ms;
  // Get required parameters
  std::string w;
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/wait_ms", wait_ms, w);

  if(wait_ms<0)
  {
    RCLCPP_WARN_STREAM(node_.lock()->get_logger(),"/wait_ms cannot be negative, set to 0");
    wait_ms = 0;
  }
  
  goal.msec_timeout = wait_ms;
  return true;
}

NodeStatus WaitSkill::onResultReceived(const RosActionNode::WrappedResult &wr)
{
  RCLCPP_INFO( node_.lock()->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
               wr.result->done ? "true" : "false" );

  return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus WaitSkill::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(node_.lock()->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void WaitSkill::onHalt()
{
  RCLCPP_INFO( node_.lock()->get_logger(), "%s: onHalt", name().c_str() );
}

// Plugin registration.
// The class WaitSkill will self register with name  "Sleep".
CreateRosNodePlugin(WaitSkill, "WaitSkill");
