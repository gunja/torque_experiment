#include <controller_interface/controller.h>
#include "armin_robothw/armin_controller_interface.hpp"
#include "armin_robothw/armin_hw_interface.hpp"
#include <pluginlib/class_list_macros.h>

namespace armin_controller_ns {
    ArminControllerInterface::ArminControllerInterface()
    {
        //TODO. output something while debugging
    }
    
    ArminControllerInterface::~ArminControllerInterface()
    {
        //TODO. clear internal things when it will be needed
    }
    
    bool ArminControllerInterface::init(ArminHWInterface* hw, ros::NodeHandle& controller_nh)
    {
        auto rv = controller_interface::Controller<ArminHWInterface>::init(hw, controller_nh);
        //TODO do other initialization
        return rv;
    }
    bool ArminControllerInterface::init(ArminHWInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
    {
        auto rv = controller_interface::Controller<ArminHWInterface>::init(hw, root_nh, controller_nh);
        //TODO do other initialization
        return rv;
    }

    // from ControllerBase
    void ArminControllerInterface::starting(const ros::Time& time)
    {
        //TODO implement properly
        controller_interface::Controller<ArminHWInterface>::starting(time);
    }
    void ArminControllerInterface::update(const ros::Time& time, const ros::Duration& period)
    {
        // TODO. base class is pure virtual
        return;
    }
    void ArminControllerInterface::stopping(const ros::Time& time)
    {
        //TODO define properly.
        controller_interface::Controller<ArminHWInterface>::stopping(time);
        return;
    }
    bool ArminControllerInterface::initRequest(hardware_interface::RobotHW* robot_hw,
                           ros::NodeHandle&             root_nh,
                           ros::NodeHandle&             controller_nh,
                           ClaimedResources&            claimed_resources)
    {
/** \brief Request that the controller be initialized
   *
   * \param robot_hw The robot hardware abstraction.
   *
   * \param root_nh A NodeHandle in the root of the controller manager namespace.
   * This is where the ROS interfaces are setup (publishers, subscribers, services).
   *
   * \param controller_nh A NodeHandle in the namespace of the controller.
   * This is where the controller-specific configuration resides.
   *
   * \param[out] claimed_resources The resources claimed by this controller.
   * They can belong to multiple hardware interfaces.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
        //TODO implement
    }
};

PLUGINLIB_EXPORT_CLASS(armin_controller_ns::ArminControllerInterface, controller_interface::Controller<ArminHWInterface>)
