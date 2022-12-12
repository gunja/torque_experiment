#include <iostream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <armin_robothw/armin_hw.hpp>
#include <controller_manager/controller_manager.h>
#include "armin_robothw/HowDoYouDo.h"

#define THIS_NODE_NAME "armin_control_node"

int main(int argc, char** argv){
    ros::init(argc, argv, THIS_NODE_NAME);
   
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    armin_hardware_interface::ArminHW hw;
    bool init_success = hw.init(nh,nh);
    if (not init_success) {
        ROS_FATAL("Unknown how to proceed with failure on initialization");
        exit(EXIT_FAILURE);
    }

    controller_manager::ControllerManager cm(&hw,nh);

    // EtherCAT network is served at rate 1 exchange / 5 ms ( 200 Hz)
    // TODO switch to Interpolate Positioning mode for Festo
    // as each update of target position takes at least 3 exchanges
    // reducing frequency from 1/200 to 1 / 100
    ros::Duration period(0.001 * SYNC_PERIOD_MS); // 100Hz update rate

    ros::Subscriber sub = nh.subscribe("commands", 1000,
        &armin_hardware_interface::ArminHW::commandsCallback, &hw);
    
    ros::Subscriber subWrist = nh.subscribe("wrist", 1000,
        &armin_hardware_interface::ArminHW::wristCallback, &hw);

    ros::Subscriber subPneumatic = nh.subscribe("pneumatic", 1000,
        &armin_hardware_interface::ArminHW::pneumaticCallback, &hw);

    ros::ServiceServer howDoYouDoService = nh.advertiseService("how_do_you_do",
        &armin_hardware_interface::ArminHW::HDYDCallback, &hw);

    ros::ServiceServer getErrorsListService = nh.advertiseService("report_errors",
        &armin_hardware_interface::ArminHW::GLoECallback, &hw);

    ros::ServiceServer executeCommandService = nh.advertiseService("execute_command",
        &armin_hardware_interface::ArminHW::ExeComCallback, &hw);
    ros::Subscriber subPneumaticAnalogCh1 = nh.subscribe("pneumatic_analog_ch1", 1000,
        &armin_hardware_interface::ArminHW::pneumaticAnalogPortCh1Callback, &hw);

    ros::Subscriber powerOffRequestSubscriber = nh.subscribe("/battery/commands", 1000,
            &armin_hardware_interface::ArminHW::batteryHandlerCallback, &hw);

    ros::Publisher batteryDependencyPub = nh.advertise<std_msgs::String>("/battery/dependecie", 10);

    std_msgs::String msg;
    msg.data = THIS_NODE_NAME;
    batteryDependencyPub.publish(msg);

    ROS_INFO("armin_control_node started");
    while(ros::ok() && hw.isActive()){
        hw.read(ros::Time::now(), period);
        cm.update(ros::Time::now(), period);
        hw.write(ros::Time::now(), period);
        period.sleep();
    }

    spinner.stop();
    msg.data = std::string("Done+") + THIS_NODE_NAME;
    batteryDependencyPub.publish(msg);
return 0;
}

