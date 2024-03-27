#include <behaviortree_ros2/bt_action_node.hpp>
#include <trajectory_loader/action/move_to_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class MoveToClient: public BT::RosActionNode<trajectory_loader::action::MoveToAction>
{
public:
  MoveToClient(const std::string& name,
               const BT::NodeConfig& conf,
               const BT::RosNodeParams& params)
    : RosActionNode<trajectory_loader::action::MoveToAction>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("group_name"),
        BT::InputPort<std::string>("ik_service_name"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("pose"),
        BT::InputPort<bool>("simulation"),
      }
    );
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};
