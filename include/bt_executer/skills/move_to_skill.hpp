#include <bt_executer/utils.hpp>
#include <trajectory_loader/action/move_to_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

class MoveToSkill: public BT::RosActionNode<trajectory_loader::action::MoveToAction>
{
public:
  MoveToSkill(const std::string& name,
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
  BT::NodeStatus onFeedback(const std::shared_ptr<const trajectory_loader::action::MoveToAction::Feedback> feedback) override;

private:
  std::string ns_, world_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};
