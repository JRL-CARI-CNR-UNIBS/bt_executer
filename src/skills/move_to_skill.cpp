#include <bt_executer/skills/move_to_skill.hpp>

MoveToSkill::MoveToSkill(const std::string& name,
                         const BT::NodeConfig& conf,
                         const BT::RosNodeParams& params)
  : RosActionNode<trajectory_loader::action::MoveToAction>(name, conf, params)
{
  auto param_ns = getInput<std::string>("param_ns");
  ns_ = "/bt_executer/" + param_ns.value();

  std::string w;
  if(not bt_executer::utils::get_param(node_.get(), ns_, "/world_name", world_, w))
    world_ = "world";
}

bool MoveToSkill::setGoal(RosActionNode::Goal &goal)
{
  int scaling;
  bool simulation;
  std::string fjt_action_name, action_name, group_name, ik_service_name, location_name, speed_scaling_topic;

  // Get required parameters
  std::string w;
  bt_executer::utils::get_param(node_.get(), ns_, "/group_name", group_name, w);
  bt_executer::utils::get_param(node_.get(), ns_, "/ik_service_name", ik_service_name, w);
  bt_executer::utils::get_param(node_.get(), ns_, "/simulation", simulation, w);
  bt_executer::utils::get_param(node_.get(), ns_, "/location_name", location_name, w);  //pose to reach
  bt_executer::utils::get_param(node_.get(), ns_, "/fjt_action_name", fjt_action_name, w);
  bt_executer::utils::get_param(node_.get(), ns_, "/speed_scaling_topic", speed_scaling_topic, w);
  bt_executer::utils::get_param(node_.get(), ns_, "/scaling", scaling, w);

  // Get queried location
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
          location_name, world_,
          tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(node_->get_logger(), "Could not transform %s to %s: %s",
                location_name.c_str(), location_name.c_str(), ex.what());
    return false;
  }

  // Set goal fields
  goal.group_name = group_name;
  goal.ik_service_name = ik_service_name;
  goal.simulation = simulation;
  goal.fjt_action_name = fjt_action_name;
  goal.speed_scaling_topic = speed_scaling_topic;
  goal.scaling = scaling;

  goal.pose.header = transform.header;
  goal.pose.pose.orientation = transform.transform.rotation;
  goal.pose.pose.position.x = transform.transform.translation.x;
  goal.pose.pose.position.y = transform.transform.translation.y;
  goal.pose.pose.position.z = transform.transform.translation.z;

  return true;
}

BT::NodeStatus MoveToSkill::onResultReceived(const RosActionNode::WrappedResult &wr)
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

BT::NodeStatus MoveToSkill::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveToSkill::onFeedback(const std::shared_ptr<const trajectory_loader::action::MoveToAction::Feedback> feedback)
{
  (void) feedback;
  return BT::NodeStatus::RUNNING;
}

// Plugin registration.
// The class MoveToSkill will self register with name  "MoveToSkill".
CreateRosNodePlugin(MoveToSkill, "MoveToSkill");
