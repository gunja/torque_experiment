#include "armin_robothw/armin_hw.hpp"
#include "armin_robothw/HowDoYouDo.h"

#include <pluginlib/class_list_macros.hpp>


#include <iostream>
#include <ios>
#include <fstream>
#include <unistd.h>

#include <pthread.h>


namespace armin_hardware_interface {

ArminHW::ArminHW() :
    f_cnt(nullptr)
    , ek1828(nullptr)
    , el2819(nullptr)
    , el4102(nullptr)
    , festo_controllers_found(0)
    , enableReadWrite(false)
    , isRunning(true)
    , inited(false)
{
    Ax_motor_link_reduction[0] = A1_motor_link_reduction;
    Ax_motor_link_reduction[1] = A2_motor_link_reduction;
    Ax_motor_link_reduction[2] = A3_motor_link_reduction;
    Ax_motor_link_reduction[3] = A4_motor_link_reduction;
    Ax_motor_link_reduction[4] = A5_motor_link_reduction;
    Ax_motor_link_reduction[5] = A6_motor_link_reduction;
}

ArminHW::~ArminHW()
{
    enableReadWrite = false;
    if (keepUpdates && f_cnt != nullptr)
    {
        for(auto i=0; i < festo_controllers_found; ++i)
        {
            f_cnt[i].disableMotor();
        }
    }
    usleep( 400000);
    if (keepUpdates) {
        keepUpdates = false;
        pthread_join(pth_pid, nullptr);
    }
    if (f_cnt != nullptr) {
        storeHomePositions(f_cnt);
    }

    if (f_cnt != nullptr) {
        for (auto i=0; i < festo_controllers_found; ++i) {
            f_cnt[i].~FestoController();
        }
        std::free(f_cnt);
        f_cnt = nullptr;
    }
    if (ecn.isOperational()) {
        ecn.switchOP_SO();
    }
    std::cout<<"ArminHW destructor done"<<std::endl;
}

void * ArminHW::threadFunc(void *arg)
{
    ArminHW * self = reinterpret_cast<ArminHW *>(arg);
    self->updateThread();
    return nullptr;
}

void ArminHW::updateThread()
{
  while(keepUpdates) {
    std::unique_lock<std::mutex> dataLock { ecn.getReadyMutex()};
    //ecn.getReadyDataCond().wait(dataLock);

    for(auto i = 0; i < festo_controllers_found; ++i) {
        f_cnt[i].updateOutputs();
    }
    ek1828->reflectOutput();
    el2819->reflectOutput();
    el4102->reflectOutput();
    ecn.getReadyDataCond().wait(dataLock);

    if (inited)
        earlyReactOnNewData();
  }
}

bool ArminHW::clearErrorsOnInit()
{
    bool canComplete = false;
    int counter = 0;
    while (!canComplete)
    {
        for(int i=0; i < festo_controllers_found; ++i)
        {
            f_cnt[i].clearError();
        }
        usleep(10000);
        canComplete = true;
        for(auto i=0; i < festo_controllers_found; ++i)
        {
            if (f_cnt[i].getErrorRegister() != 0)
            {
                canComplete = false;
            }
        }
        counter++;
        if (counter > 100) {
            return false;
        }
    }
    for(auto i=0; i < festo_controllers_found; ++i)
    {
        f_cnt[i].resetClearingErrorState();
    }
    return true;
}

bool ArminHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
    ROS_DEBUG_STREAM("initing. Root NH namespace: "<<root_nh.getNamespace()
        <<"  and robot_hw_nh is "<<robot_hw_nh.getNamespace());
    robot_hw_nh.getParam("joints", joint_name);

    joint_name = std::vector<std::string> { "a1_joint", "a2_joint", "a3_joint",
        "a4_joint", "a5_joint", "a6_joint" };
    
    num_joints = joint_name.size();
    //resize vectors
    joint_position_state.resize(num_joints);
    joint_velocity_state.resize(num_joints);
    joint_effort_state.resize(num_joints);
    joint_position_command.resize(num_joints);

    //Register handles
    for(int i=0; i<num_joints; i++){
        //State
        hardware_interface::JointStateHandle jointStateHandle(joint_name[i], &joint_position_state[i],
            &joint_velocity_state[i], &joint_effort_state[i]);
        joint_state_interface.registerHandle(jointStateHandle);

        //Position
        hardware_interface::JointHandle jointPosHandle(jointStateHandle, &joint_position_command[i]);
        joint_pos_interface.registerHandle(jointPosHandle);
    }

    //Register interfaces
    registerInterface(&joint_state_interface);
    registerInterface(&joint_pos_interface);

    robot_hw_nh.param("iface", iface_name, std::string("enp59s0"));
    bool ecnStarted = ecn.startNetwork(iface_name.c_str());
    if (! ecnStarted) {
        ROS_ERROR("EtherCAT manager was not started. Are you root?");
        return false;
    } else {
        ROS_INFO("EtherCAT has started");
    }
    auto ecnInSafeOperational = ecn.SwitchSafeOperational();
    if (!ecnInSafeOperational) {
        ROS_ERROR("EtherCAT manager failed to switch to safe operational");
        return false;
    } else {
        ROS_INFO("EtherCAT is in Safe Op mode");
    }

    //TODO
    // iterate over ec_slave[i] and select only those with proper name
    festo_controllers_found = TOTAL_ROTATE_JOINTS_COUNT;
    f_cnt = reinterpret_cast<FestoController*>(std::calloc(
                        festo_controllers_found, sizeof(FestoController)));
    for(int i = 0; i < festo_controllers_found; ++i) {
        new (&f_cnt[i]) FestoController(
                    *(reinterpret_cast<Master2Slave*>(ecn.getOutputPointer(i+1))),
                    *(reinterpret_cast<Slave2Master*>(ecn.getInputPointer(i+1))),
                     ecn, i+1);

        f_cnt[i].m2s.modes_of_operation=1;
    }


    for(auto ec_i = 1; ec_i <= ecn.getDiscoveredSlavesCount(); ++ec_i)
    {
        //if (std::string("EK1828") == ecn.getSlaveName(ec_i))
        if( strcmp("EK1828", ecn.getSlaveName(ec_i)) == 0 )
            if(ek1828==nullptr)
                ek1828 = new EKXX(
          *(reinterpret_cast<Ekxx_Region_m2s*> (ecn.getOutputPointer(ec_i))),
          *(reinterpret_cast<Ekxx_Region_s2m*> (ecn.getInputPointer(ec_i))),
                            ecn, ec_i);

        if( strcmp("EL2819", ecn.getSlaveName(ec_i)) == 0 )
            if(el2819==nullptr)
                el2819 = new ELXX(
          *(reinterpret_cast<Elxx_Region_m2s*> (ecn.getOutputPointer(ec_i))),
          *(reinterpret_cast<Elxx_Region_s2m*> (ecn.getInputPointer(ec_i))),
                            ecn, ec_i);
        if( strcmp("EL4102", ecn.getSlaveName(ec_i)) == 0 )
            if(el4102==nullptr)
                el4102 = new EL41XX(
          *(reinterpret_cast<El41xx_Region_m2s*> (ecn.getOutputPointer(ec_i))),
          *(reinterpret_cast<El41xx_Region_s2m*> (ecn.getInputPointer(ec_i))),
                            ecn, ec_i);
    }
    if (el2819 == nullptr)
    {
        ROS_FATAL("EL2819 was not found online. Can not proceed");
        return false;
    }
    if (ek1828 == nullptr)
    {
        ROS_FATAL("EK1828 was not found online. Can not proceed");
        return false;
    }
    if (el4102==nullptr)
    {
        ROS_FATAL("EL4102 was not found online. Can not proceed");
          return false;
    }

    loadPositionsFromFile(f_cnt);

    keepUpdates = true;
    pthread_attr_t high_prio_attrs;
    pthread_attr_init(&high_prio_attrs);
    struct sched_param prio; prio.sched_priority = 10;
    pthread_attr_setschedparam(&high_prio_attrs, &prio);
    pthread_attr_setschedpolicy(&high_prio_attrs, SCHED_FIFO);
    if ( pthread_create(&pth_pid, &high_prio_attrs, threadFunc, this))
    {
        ROS_FATAL_STREAM("Failed to call create with high priority settings. Error is "<<(errno==EAGAIN?"no resources":"")
            <<(errno==EINVAL?"invalid settings in attr":"")<<(errno==EPERM?"no permission to set policy":""));
        ROS_INFO("Starting normal priority");
        pthread_create(&pth_pid, nullptr, threadFunc, this);
    }

    auto ecnOperational = ecn.SwitchOperational();
    if (!ecnOperational) {
        ROS_ERROR("EtherCAT manager failed to switch to operational");
        return false;
    } else {
        ROS_INFO("EtherCAT is in Operational mode");
    }
    // wait while everything comes online?
    ROS_INFO("wait while everything comes online");
    struct timeval tv_start, tv_now;
    gettimeofday( &tv_start, nullptr);
    bool canComplete = false;
    while(!canComplete) {
        canComplete = true;
        for(auto i=0; i < festo_controllers_found; ++i) {
            if (f_cnt[i].s2m.statusword == 0)
                canComplete = false;
        }
        gettimeofday( &tv_now, nullptr);
        // 10 is timeout for bring-up
        if (tv_now.tv_sec - tv_start.tv_sec > 10 ) {
            ROS_ERROR("Failed to wait while each slave starts transmitting data");
            return false;
        }
    }
    ROS_INFO("done");
    canComplete = false;
    auto counter = 0;
    ROS_INFO("starting clear errors procedure");
    if (not clearErrorsOnInit())
    {
        ROS_ERROR("Failed to reset Error flag on Festo drives in 10 attempts. BReaking");
        return false;
    }
    ROS_INFO("done");

    //TODO initialize dependencies of thw motors
    // from one another
    // refer to 
    //        reflectControlsToOneAnother();
    // of window_first.cpp

    ROS_INFO("wait while homing completes");
    for(int i=0; i < festo_controllers_found; ++i)
    {
        f_cnt[i].completeHoming();
    }

    canComplete = false;
    while(!canComplete && ros::ok() ) {
        canComplete = true;
        for(auto i=0; i < festo_controllers_found; ++i) {
            if (f_cnt[i].isHomingStarted() )
                canComplete = false;
        }
        gettimeofday( &tv_now, nullptr);
        // 10 is timeout for bring-up
        if (tv_now.tv_sec - tv_start.tv_sec > 10 ) {
            ROS_ERROR("Failed to wait while each slave starts transmitting data");
        }
    }
    ROS_INFO("Homing of each controller done");
    //TODO load zero states of joints
    enableReadWrite = true;
    inited = true;
    return true;
}

bool ArminHW::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                             const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    return hardware_interface::RobotHW::prepareSwitch(start_list, stop_list);
}

void ArminHW::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                        const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    //TODO
    return;
}

void ArminHW::read(const ros::Time& time, const ros::Duration& period)
{
    if (not enableReadWrite)
    {
        ROS_DEBUG(" ArminHW::read not enableReadWrite");
        return;
    }
    const double popugai_to_rads = 1. /1800. * M_PI;

    int32_t *knownPosition = new int32_t[festo_controllers_found];
    int32_t *knownVelocities = new int32_t[festo_controllers_found];

    ROS_DEBUG_STREAM("called READ with time="<<time<<"    and period="<<period);

    for(auto i = 0; i < festo_controllers_found; ++i) {
        knownPosition[i] = f_cnt[i].getPositionValue();
        knownVelocities[i] = f_cnt[i].getVelocityValue();
        joint_position_state[i] = popugai_to_rads * Ax_motor_link_reduction[i] * knownPosition[i];
        joint_velocity_state[i] = popugai_to_rads * Ax_motor_link_reduction[i] * knownVelocities[i];
        for(auto j =0; j < i; ++j) {
            joint_position_state[i] -= jointInfluence[i-1][j] * joint_position_state[j];
            joint_velocity_state[i] -= jointInfluence[i-1][j] * joint_velocity_state[j];
        }
        joint_effort_state[i] =  f_cnt[i].getCurrentValue();

        ROS_DEBUG_STREAM("reporting for joint #"<<i
            <<" position = "<<joint_position_state[i]
            <<" velocity = "<<joint_velocity_state[i]
            <<" current = "<<joint_effort_state[i]);
    }
    delete[] knownPosition;
    delete[] knownVelocities;
    return;
}

void ArminHW::write(const ros::Time& time, const ros::Duration& period)
{
    //TODO before actual write into devices of target positions + required velocities,
    // check if at least 1 motor will require velocity truncation
    // and recalculate velocities, reducing by maximal reduction coefficient

    if (not enableReadWrite)
    {
        ROS_DEBUG("  ArminHW::write   not enableReadWrite");
        return;
    }
    const int MAX_ALLOWED_VELOCITY = 180000; //4000 rpm
    const double popugai_to_rads = 1. /1800. * M_PI;
    // ATT sequence matters!!! each position of next controller is dependent on
    ROS_DEBUG_STREAM("called WRITE with time="<<time<<"    and period="<<period);
    // calculated previous position
    for(auto i=0; i < festo_controllers_found; ++i)
    {
        double motorPosition = joint_position_command[i];
        for(auto j=0; j < i; ++j) {
            motorPosition += jointInfluence[i-1][j] * joint_position_command[j];
        }
        motorPosition /= popugai_to_rads * Ax_motor_link_reduction[i];
        
        ROS_DEBUG_STREAM("setting for joint #"<<i
            <<"  new target position "<<motorPosition<<
            "   from request "<<joint_position_command[i]);
        //assuming that <current_position> to <desired_position> should be achieved
        // in same "period" as it was previously
        int lastKnownPosition = f_cnt[i].getPositionValue();
        f_cnt[i].setTargetPosition(static_cast<int>(motorPosition));

        int desiredVitess = std::abs((motorPosition - lastKnownPosition) / period.toSec());
        if (desiredVitess > MAX_ALLOWED_VELOCITY)
            desiredVitess = MAX_ALLOWED_VELOCITY;
        f_cnt[i].setPositioningVelocity(desiredVitess);
        f_cnt[i].startPositionChange();
    }
    return;
}

/**
    method allows to trigger each Festo's controller
    to Enabled state in case controller (and motor)
    went to disabled state (for example when Emergency button
    was pushed 
*/
bool ArminHW::setEnabled()
{
std::cout<<"entering setEnabled with counter = "<<festo_controllers_found<<std::endl;
    enableReadWrite = false;
    for( auto i=0; i < festo_controllers_found; ++i)
    {
        f_cnt[i].clearEIP();
        f_cnt[i].startEnabling();
    }
    enableReadWrite = true;
    return true;
}

void ArminHW::commandsCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO_STREAM("Got command callback with arg: "<<msg->data );
    if (msg->data == "enable")
    {
        setEnabled();
    }
}

void ArminHW::CallbackParser(const std_msgs::String::ConstPtr& msg, std::string& str0, int& key)
{
	if(msg->data == " ")
	{
		key = -1;
		str0 = " ";
		return;
	}
	std::size_t pos = msg->data.find(" ");
	if (pos == std::string::npos)
	{
		key = -1;
		str0 = msg->data;
		return;
	}
  try {
	key = std::stoi( msg->data.substr(pos) );
  } catch (...) {
    key = -1;
  }
	str0 = msg->data.substr(0, pos);
}

void ArminHW::wristCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_DEBUG_STREAM("Got wrist callback with arg: "<<msg->data );
    std::string str0;
    int key;
	CallbackParser(msg, str0, key);

    if (str0 == "open")
    {
        ek1828->powerOnChannelTimedOut(EK1828_OPEN_CHANNEL_NUMBER, EK1828_POWER_ON_DELAY_NS);
    }
    if (str0 == "close")
    {
        ek1828->powerOnChannelTimedOut(EK1828_CLOSE_CHANNEL_NUMBER, EK1828_POWER_ON_DELAY_NS);
    }

}

void ArminHW::pneumaticCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_DEBUG_STREAM("Got pneumatic callback with arg: "<<msg->data );
    std::string str0;
    int key;
	CallbackParser(msg, str0, key);

        if (key >= 0 && key < EL2819_NUMBER_OF_OUTPUTS_CHANNELS)
        {
            if (str0 == "open")
            {
                el2819->powerOnChannel(key);
            }
            if (str0 == "close")
            {
                el2819->powerOffChannel(key);
            }

        }
}

void ArminHW::pneumaticAnalogPortCh1Callback(const std_msgs::Int16::ConstPtr &msg)
{
     el4102->powerChannel_1(msg->data);
}

void ArminHW::loadPositionsFromFile(FestoController *base)
{
    std::list <int> zeroValues { 0, 0, 0, 0, 0, 522720 };
    std::ifstream zeroFile(ZERO_STREAM_FILENAME);
    if (!zeroFile.is_open())
    {
        int idx = 0;
        for( auto const & i: zeroValues) {
            f_cnt[idx++].setHomeOffset(i);
        }
        return;
    }
    int32_t val;
    for(auto i=0; i < festo_controllers_found; ++i) {
        zeroFile>>val;
        f_cnt[i].setHomeOffset(val);
    }
    zeroFile.close();
}

void ArminHW::storeHomePositions(FestoController *festoContr)
{
    std::ofstream zeroFile(ZERO_STREAM_FILENAME);
    if (!zeroFile.is_open()) {
        // TODO notify on the problem with file opening
        return;
    }
    for( int idx =0; idx < festo_controllers_found; ++idx)
    {
        zeroFile<<festoContr[idx].getPositionValue()<<std::endl;
    }
    zeroFile.close();
}

/**
    at the moment, separated place to analyze error flag

    Loops over all festo-controllers and checks if error_register is zero.
    Otherwise, force stop of analyzes input/output commands
    and that's all for now
*/
void ArminHW::earlyReactOnNewData()
{
    //ROS_INFO("                                          :earlyReactOnNewData()");
    bool errorPresents = false;
    for(auto i = 0; i < festo_controllers_found; ++i)
    {
        if (f_cnt[i].s2m.error_register != 0) {
            ROS_INFO_STREAM("Got error_reg = "<<
                static_cast<int>(f_cnt[i].s2m.error_register)
                <<" for controller "<<i<<"  and status=="<<std::hex<<f_cnt[i].s2m.statusword<<std::dec);
        }
        errorPresents |= (f_cnt[i].s2m.error_register != 0);
    }
    if (errorPresents)
    {
        ROS_INFO_STREAM(" after all errorPresents is "<<
            errorPresents<<" setting enableRW to false");
        enableReadWrite = false;
        for(auto i = 0; i < festo_controllers_found; ++i)
        {
            f_cnt[i].disableMotor();
            //f_cnt[i].m2s.controlword = 0;
        }
    }
}

bool ArminHW::HDYDCallback(armin_robothw::HowDoYouDo::Request &req, armin_robothw::HowDoYouDo::Response &resp)
{
    // request is empty
    // so, let's boogy-woogy
    for(auto i=0; i < festo_controllers_found; ++i)
    {
        armin_robothw::FestoStatus val;
        val.statusword = f_cnt[i].s2m.statusword;
        val.error_register = f_cnt[i].s2m.error_register;
        resp.statuses.push_back(val);
    }
    return true;
}

bool ArminHW::GLoECallback(armin_robothw::GetErrorsList::Request &req, armin_robothw::GetErrorsList::Response &resp)
{
    ROS_DEBUG_STREAM("ArminHW::GLoECallback called with "<<req);
    auto festoDevNum = req.device_number;
    if (festoDevNum < 1 || festoDevNum > festo_controllers_found)
    {
        return false;
    }
    resp.resp.count = ecn.getSlotDataUINT8(festoDevNum,
            PRE_DEFINED_ERROR_FIELD_SLOTNUM, 0);
    for( auto i=0; i < errorFieldsCount; ++i)
    {
        resp.resp.field_.push_back(
            ecn.getSlotDataUINT32( festoDevNum,
                PRE_DEFINED_ERROR_FIELD_SLOTNUM, i));
    }
    return true;
}

bool ArminHW::ExeComCallback(armin_robothw::ExecuteCommand::Request &req, armin_robothw::ExecuteCommand::Response &resp)
{
    ROS_DEBUG_STREAM("ArminHW::ExeComCallback called with req.cmd="<<static_cast<int>(req.cmd_type)<<"  and dev num = "<<req.dev_num );
    // for now ignoring req.cmd_type
    // and assume that it's always DELETE_ERROR
    if (req.dev_num < 1 || req.dev_num > festo_controllers_found)
    {
        return false;
    }
    ecn.setSlotDataUINT8(req.dev_num,
        PRE_DEFINED_ERROR_FIELD_SLOTNUM, 0, 0);
    f_cnt[req.dev_num - 1].clearError();
    resp.result = true;
    return true;
}

void ArminHW::batteryHandlerCallback(const std_msgs::String::ConstPtr& data)
{
    if (data->data == "prepareShutdown")
    {
        // TODO prepare shutdown
        // now, for simplicity, all the actions will put into destructor responcibility
        isRunning = false;
    }
}

const uint16_t ArminHW::PRE_DEFINED_ERROR_FIELD_SLOTNUM = 0x1003;
const int ArminHW::errorFieldsCount  = 4;

}; //namespace armin_hardware_interface

PLUGINLIB_EXPORT_CLASS(armin_hardware_interface::ArminHW, hardware_interface::RobotHW);
