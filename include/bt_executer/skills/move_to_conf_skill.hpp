#include <bt_executer/utils.hpp>
#include <trajectory_loader/action/move_to_conf_action.hpp>

class MoveToConfSkill: public BT::RosActionNode<trajectory_loader::action::MoveToConfAction>
{
public:
  MoveToConfSkill(const std::string& name,
                  const BT::NodeConfig& conf,
                  const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
          {
            BT::InputPort<std::string>("param_ns")
          }
          );
  }

  bool setGoal(Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
  BT::NodeStatus onFeedback(const std::shared_ptr<const trajectory_loader::action::MoveToConfAction::Feedback> feedback) override;

private:
  std::string ns_;
};
