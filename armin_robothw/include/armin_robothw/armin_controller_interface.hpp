#pragma once
#include <controller_interface/controller.h>
#include "armin_hw_interface.hpp"

namespace armin_controller_ns {
class ArminControllerInterface : public controller_interface::Controller<ArminHWInterface>
{
  public:
    ArminControllerInterface();
    virtual ~ArminControllerInterface();
    bool init(ArminHWInterface* /*hw*/, ros::NodeHandle& /*controller_nh*/) override;
    bool init(ArminHWInterface* /*hw*/, ros::NodeHandle& /*root_nh*/, ros::NodeHandle& /*controller_nh*/) override;

    // from ControllerBase
    void starting(const ros::Time& /*time*/) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    void stopping(const ros::Time& /*time*/) override;
    bool initRequest(hardware_interface::RobotHW* robot_hw,
                           ros::NodeHandle&             root_nh,
                           ros::NodeHandle&             controller_nh,
                           ClaimedResources&            claimed_resources) override;
};
};
