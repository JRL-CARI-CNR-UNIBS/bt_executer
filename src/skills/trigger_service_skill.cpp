#include <bt_executer/skills/trigger_service_skill.hpp>
#include <chrono>

TriggerServiceSkill::TriggerServiceSkill(const std::string& name,
                                         const BT::NodeConfig& conf,
                                         const BT::RosNodeParams& params)
  : BT::RosServiceNode<std_srvs::srv::Trigger>(name, conf, params)
{
  auto param_ns = getInput<std::string>("param_ns");
  ns_ = "/bt_executor/" + param_ns.value();
}

bool TriggerServiceSkill::setRequest(Request::SharedPtr& goal)
{
  double pause; //seconds

  // Get required parameters
  std::string w;
  bt_executer::utils::get_param(node_.get(), ns_, "/pause", pause, w);

  rclcpp::sleep_for(std::chrono::milliseconds(int(pause*1000)));

  goal = std::make_shared<std_srvs::srv::Trigger_Request>();

  return true;
}

BT::NodeStatus TriggerServiceSkill::onResponseReceived(const Response::SharedPtr& response)
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

BT::NodeStatus TriggerServiceSkill::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return BT::NodeStatus::FAILURE;
}

// Plugin registration.
// The class TriggerServiceSkill will self register with name  "TriggerServiceSkill".
CreateRosNodePlugin(TriggerServiceSkill, "TriggerServiceSkill");
