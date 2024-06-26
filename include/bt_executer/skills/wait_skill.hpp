#include "behaviortree_ros2/bt_action_node.hpp"
#include "btcpp_ros2_interfaces/action/sleep.hpp"
#include <bt_executer/utils.hpp>

using namespace BT;

class WaitSkill: public RosActionNode<btcpp_ros2_interfaces::action::Sleep>
{
public:
  WaitSkill(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
        return providedBasicPorts(
          {
            BT::InputPort<std::string>("param_ns")
          }
          );
  }

  bool setGoal(Goal& goal) override;
  void onHalt() override;
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
  
private:
  std::string ns_;
};
