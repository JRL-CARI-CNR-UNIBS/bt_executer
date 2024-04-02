#include <behaviortree_ros2/bt_action_node.hpp>
#include <trajectory_loader/action/move_to_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

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
        BT::InputPort<std::string>("param_ns")
      }
    );
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

private:
  std::string group_name_, ik_service_name_, pose_name_;
  bool simulation_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string start_frame_;
  std::string target_frame_;
};
