#include <bt_executer/skills/trajectory_loader_skill.hpp>

TrajectoryLoaderSkill::TrajectoryLoaderSkill(const std::string& name,
                                             const BT::NodeConfig& conf,
                                             const BT::RosNodeParams& params)
  : RosActionNode<trajectory_loader::action::TrajectoryLoaderAction>(name, conf, params)
{
  auto param_ns = getInput<std::string>("param_ns");
  ns_ = "/bt_executer/" + param_ns.value();
}

bool TrajectoryLoaderSkill::setGoal(RosActionNode::Goal &goal)
{
  int repetitions;
  bool simulation, recompute_time_law;

  std::vector<std::string> trj;
  std::string fjt_action_name, action_name, group_name, ik_service_name, location_name;

  // Get required parameters
  std::string w;
  bt_executer::utils::get_param(node_.get(), ns_, "/trj_names", trj, w);
  bt_executer::utils::get_param(node_.get(), ns_, "/group_name", group_name, w);
  bt_executer::utils::get_param(node_.get(), ns_, "/simulation", simulation, w);
  bt_executer::utils::get_param(node_.get(), ns_, "/repetitions", repetitions, w);
  bt_executer::utils::get_param(node_.get(), ns_, "/fjt_action_name", fjt_action_name, w);
  bt_executer::utils::get_param(node_.get(), ns_, "/recompute_time_law", recompute_time_law, w);

  goal.trj_names = trj;
  goal.group_name = group_name;
  goal.repetitions = repetitions;
  goal.fjt_action_name = fjt_action_name;
  goal.recompute_time_law = recompute_time_law;

  return true;
}

BT::NodeStatus TrajectoryLoaderSkill::onResultReceived(const RosActionNode::WrappedResult &wr)
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

BT::NodeStatus TrajectoryLoaderSkill::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus TrajectoryLoaderSkill::onFeedback(const std::shared_ptr<const trajectory_loader::action::TrajectoryLoaderAction::Feedback> feedback)
{
  (void) feedback;
  return BT::NodeStatus::RUNNING;
}

// Plugin registration.
// The class TrajectoryLoaderSkill will self register with name  "TrajectoryLoaderSkill".
CreateRosNodePlugin(TrajectoryLoaderSkill, "TrajectoryLoaderSkill");
