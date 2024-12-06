#include <bt_executer/skills/move_to_conf_skill.hpp>

MoveToConfSkill::MoveToConfSkill(const std::string& name,
                                 const BT::NodeConfig& conf,
                                 const BT::RosNodeParams& params)
  : RosActionNode<trajectory_loader::action::MoveToConfAction>(name, conf, params)
{
  auto param_ns = getInput<std::string>("param_ns");
  ns_ = "/bt_executer/" + param_ns.value();
}

bool MoveToConfSkill::setGoal(RosActionNode::Goal &goal)
{
  int scaling;
  bool simulation = false;
  double velocity_scaling_factor = 1.0;
  double acceleration_scaling_factor = 1.0;
  std::vector<std::string> joints_names;
  std::vector<double> target_joints_configurations;
  std::string fjt_action_name, group_name, speed_scaling_topic;
  std::string pipeline_id = "ompl";
  std::string planner_id = "RRTConnect";

  // Get required parameters
  std::string w;
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/group_name", group_name, w);
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/simulation", simulation, w);
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/fjt_action_name", fjt_action_name, w);
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/speed_scaling_topic", speed_scaling_topic, w);
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/scaling", scaling, w);
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/acceleration_scaling_factor", acceleration_scaling_factor, w);
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/velocity_scaling_factor", velocity_scaling_factor, w);
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/pipeline_id", pipeline_id, w);
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/planner_id", planner_id, w);
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/joints_names", joints_names, w);
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/target_joints_configurations", target_joints_configurations, w);

  goal.scaling = scaling;
  goal.group_name = group_name;
  goal.simulation = simulation;
  goal.planner_id = planner_id;
  goal.pipeline_id = pipeline_id;
  goal.joints_names = joints_names;
  goal.fjt_action_name = fjt_action_name;
  goal.speed_scaling_topic = speed_scaling_topic;
  goal.velocity_scaling_factor = velocity_scaling_factor;
  goal.acceleration_scaling_factor = acceleration_scaling_factor;
  goal.target_joints_configurations = target_joints_configurations;

  return true;
}

BT::NodeStatus MoveToConfSkill::onResultReceived(const RosActionNode::WrappedResult &wr)
{
  RCLCPP_INFO(node_.lock()->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(),
              wr.result->ok ? "true" : "false");
  if (wr.result->ok)
    return BT::NodeStatus::SUCCESS;
  else
  {
    RCLCPP_ERROR_STREAM(node_.lock()->get_logger(), "Error: " << wr.result->error);
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus MoveToConfSkill::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_.lock()->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveToConfSkill::onFeedback(const std::shared_ptr<const trajectory_loader::action::MoveToConfAction::Feedback> feedback)
{
  (void) feedback;
  return BT::NodeStatus::RUNNING;
}

// Plugin registration.
// The class MoveToConfSkill will self register with name  "MoveToConfSkill".
CreateRosNodePlugin(MoveToConfSkill, "MoveToConfSkill");
