#include <behaviortree_ros2/plugins.hpp>
#include <bt_executer/move_to_client.hpp>

bool MoveToClient::setGoal(RosActionNode::Goal &goal)
{
  auto group_name = getInput<std::string>("group_name");
  auto ik_service_name = getInput<std::string>("ik_service_name");
  auto pose = getInput<geometry_msgs::msg::PoseStamped>("pose");
  auto simulation = getInput<bool>("simulation");

  goal.group_name = group_name.value();
  goal.ik_service_name = ik_service_name.value(); 
  goal.pose = pose.value();
  goal.simulation = simulation.value();

  return true;
}

BT::NodeStatus MoveToClient::onResultReceived(const RosActionNode::WrappedResult &wr)
{
  RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(),
              wr.result->ok ? "true" : "false");
  if (wr.result->ok)
    return BT::NodeStatus::SUCCESS;
  else
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Error: " << wr.result->error);
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus MoveToClient::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return BT::NodeStatus::FAILURE;
}

void MoveToClient::onHalt()
{
  RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
}

// Plugin registration.
// The class MoveToClient will self register with name  "MoveTo".
CreateRosNodePlugin(MoveToClient, "MoveTo");
