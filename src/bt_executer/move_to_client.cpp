#include <behaviortree_ros2/plugins.hpp>
#include <bt_executer/move_to_client.hpp>
#include <cnr_param/cnr_param.h>
#include <utils.hpp>

bool MoveToClient::setGoal(RosActionNode::Goal &goal)
{
  auto param_ns = getInput<std::string>("param_ns");
  std::string param_ns_value = param_ns.value();

  std::string ns = "/moveTo/" + param_ns_value, w;

  // Get parameters
  bt_executer::utils::get_param(node_.get(), ns, "/group_name", group_name_, w);
  bt_executer::utils::get_param(node_.get(), ns, "/ik_service_name", ik_service_name_, w);
  bt_executer::utils::get_param(node_.get(), ns, "/pose_name", pose_name_, w);
  bt_executer::utils::get_param(node_.get(), ns, "/simulation", simulation_, w);

  // Set start and target frames
  ns = "/poses/" + pose_name_;
  bt_executer::utils::get_param(node_.get(), ns, "/start_frame", start_frame_, w);
  bt_executer::utils::get_param(node_.get(), ns, "/target_frame", target_frame_, w);

  // Get queried pose
  geometry_msgs::msg::TransformStamped pose;
  try {
    pose = tf_buffer_->lookupTransform(
      target_frame_, start_frame_,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(node_->get_logger(), "Could not transform %s to %s: %s",
                target_frame_.c_str(), start_frame_.c_str(), ex.what());
    return false;
  }


  // Set goal fields
  goal.group_name = group_name_;
  goal.ik_service_name = ik_service_name_;
  goal.pose = pose;
  goal.simulation = simulation_;

  return true;
}

BT::NodeStatus MoveToClient::onResultReceived(const RosActionNode::WrappedResult &wr)
{
  RCLCPP_INFO(node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(),
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
