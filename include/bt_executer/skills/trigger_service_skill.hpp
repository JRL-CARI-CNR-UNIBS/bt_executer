#include <bt_executer/utils.hpp>
#include <std_srvs/srv/trigger.hpp>

class TriggerServiceSkill: public BT::RosServiceNode<std_srvs::srv::Trigger>
{
public:
  TriggerServiceSkill(const std::string& name,
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

  bool setRequest(Request::SharedPtr& goal) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

private:
  std::string ns_;
};
