#include <bt_executer/skills/set_bool_service_skill.hpp>
#include <chrono>

SetBoolServiceSkill::SetBoolServiceSkill(const std::string& name,
                                         const BT::NodeConfig& conf,
                                         const BT::RosNodeParams& params)
  : BT::RosServiceNode<std_srvs::srv::SetBool>(name, conf, params)
{
  auto param_ns = getInput<std::string>("param_ns");
  ns_ = "/bt_executor/" + param_ns.value();
}

bool SetBoolServiceSkill::setRequest(Request::SharedPtr& goal)
{
  double pause; //seconds
  bool value;

  // Get required parameters
  std::string w;
  bt_executer::utils::get_param(node_.get(), ns_, "/pause", pause, w);
  bt_executer::utils::get_param(node_.get(), ns_, "/value", value, w);

  rclcpp::sleep_for(std::chrono::milliseconds(int(pause*1000)));

  // goal = std::make_shared<std_srvs::srv::SetBool_Request>();
  goal->data = value;

  return true;
}

BT::NodeStatus SetBoolServiceSkill::onResponseReceived(const Response::SharedPtr& response)
{
  RCLCPP_INFO(node_->get_logger(), "%s: onResponseReceived. Done = %s", name().c_str(),
              response->success ? "true" : "false");
  if (response->success)
    return BT::NodeStatus::SUCCESS;
  else
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Error: " << response->message);
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus SetBoolServiceSkill::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return BT::NodeStatus::FAILURE;
}

// Plugin registration.
// The class SetBoolServiceSkill will self register with name  "SetBoolSkill".
CreateRosNodePlugin(SetBoolServiceSkill, "SetBoolSkill");
