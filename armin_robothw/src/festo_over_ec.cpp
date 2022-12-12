#include "festo_over_ec.h"
#include <iostream>
#include "ecn.h"

#include <ros/ros.h>

FestoController::FestoController(Master2Slave &_m2s, Slave2Master &_s2m, EtherCATNetwork & _ecn, int _idx) :
    clearingError(false)
    , disablingMotor(false)
    , homingStarted(false)
    , positioningStarted(false)
    , positioningState(trans_none_position_sm)
, ecn( _ecn), ec_idx(_idx)
    , ownVelocity (0)
    , offset(0)
    , targetPositionCopy(0)
    , targetVelocityCopy(0)
    , positionChangeRequested(false)
    , immediateCopy(false)
        , m2s(_m2s), s2m(_s2m)
    , pendingTransition(trans_none)
    , enabled(false), enableInProgress(false)
{
    // TODO complete this constructor
}

FestoController::~FestoController()
{
    // TODO
}

void FestoController::startEnabling()
{
//std::cout<<ec_idx<<" in startEnabling with EIP="<<enableInProgress<<std::endl;
    if (enableInProgress) return;
    if ((s2m.statusword & 0x6F) !=  0x27) {
        enabled = false;
    }
//std::cout<<ec_idx<<" in startEnabling with enabled == "<<enabled<<std::endl;
    if (enabled) return;
    enableInProgress = true;
//std::cout<<ec_idx<<"set EIP to true"<<std::endl;
    pendingTransition = trans_2;
    m2s.modes_of_operation = 3;
    
} 

void FestoController::updateOutputs()
{
    // TODO rework this place, as already a lot of "special"
    // cases are checked
    if (disablingMotor)
    {
        m2s.controlword = 0;
        enabled = false;
        enableInProgress = false;
        pendingTransition = trans_none;
        ownVelocity = 0;
        return;
    }

    if (s2m.error_register)
    {
        return errorStateHandling();
    }
    if (positionChangeRequested && not positioningStarted)
    {
        positionChangeRequested = false;
        startPositionChangeInternal(immediateCopy);
    }


    // TODO this function sets output fields based on state stored in this object
    if (pendingTransition != trans_none )
    {
        switch (pendingTransition) {
            case trans_2:
                // TODO
                m2s.controlword = 0x6;
                pendingTransition = trans_3;
                break;
            case trans_3:
                // TODO
                // if trans_3 allowed?
                if ((s2m.statusword & 0x6F) == 0x21) {
                    m2s.controlword = 0x7;
                    pendingTransition = trans_4;
                }
                break;
            case trans_4:
                // TODO
                // if trans_4 allowed
                if ((s2m.statusword &0x6F) == 0x23) {
                    m2s.controlword = 0xF;
                    pendingTransition = trans_none;
                }
                break;

            case trans_14:
                // TODO
                m2s.controlword |= (1<<7);
                pendingTransition = trans_15;
                break;
            case trans_15:
                //TODO
              if (s2m.statusword & 0x80) // (1<<7) == 0x80, controller confirmed Fault Error quoting
              {
                resetClearingErrorState();
              } else {
                m2s.controlword |= (1<<7);
              }
                break;

            case trans_start_homing:
                if (s2m.modes_of_operation_display == 6) {
                    m2s.controlword |= 0x10;
                    pendingTransition = trans_homing_await_conf;
                } 
                break;
            case trans_homing_await_conf:
                if (s2m.statusword & 0x1000) {
                    homingStarted = false;
                    pendingTransition = trans_none;
                    m2s.modes_of_operation = 3;
                }
        }
    }
    // TODO
    if ((s2m.statusword & 0x6F) == 0x27 ) {
        enabled = true;
        enableInProgress = false;
//std::cout<<ec_idx<<"set EIP to flase"<<std::endl;
    }
    if (enabled) {
        if (homingStarted && pendingTransition == trans_none) {
            pendingTransition = trans_start_homing;
            //m2s.controlword |= 0x100; // HALT
            //uint8_t homingMethod = 35;
            ecn.setSlotDataUINT8(ec_idx, 0x6098, 0, 35);
            m2s.modes_of_operation = 6;
        }
    }
    //m2s.modes_of_operation = 3;
    //m2s.target_velocity = 0;
    if (s2m.modes_of_operation_display == 3) {
        m2s.target_velocity = ownVelocity + getVelocityCorrections();
    }
    if (positioningStarted && enabled) {
        m2s.target_position = targetPositionCopy;
        m2s.profile_velocity = targetVelocityCopy;
        switch (positioningState) {
            case trans_confirm_enabling:
                if (s2m.modes_of_operation_display == 1) {
                    positioningState = trans_confirm_mode_of_operation;
                } else {
                    m2s.modes_of_operation = 1;
                }
                break;
            case trans_confirm_mode_of_operation:
                m2s.controlword &= ~CTRL_NEW_SET_POINT_MASK;
                positioningState = trans_confirm_clearence_SP_ack;
                break;
            case trans_confirm_clearence_SP_ack:
                if ((s2m.statusword & SET_POINT_ACK_STATUS_MASK) == 0) {
                    m2s.controlword |= CTRL_NEW_SET_POINT_MASK;
                    m2s.controlword |= CTRL_CNG_SET_IMMEDIAT_MASK;
                    positioningState = trans_confirm_SP_ack;
                }
                break;
            case trans_confirm_SP_ack:
                if (s2m.statusword & SET_POINT_ACK_STATUS_MASK) {
                    m2s.controlword &= ~(CTRL_NEW_SET_POINT_MASK | CTRL_CNG_SET_IMMEDIAT_MASK);
                    positioningState = trans_none_position_sm;
                    positioningStarted = false;
                }
        }
    }

    //ROS_INFO_STREAM("m2s.contwd ="<<m2s.controlword<<"   s2m.stwd="<<std::hex<<s2m.statusword<<"   m2s.mode_ope="<<static_cast<int>(m2s.modes_of_operation)<<"  s2m.mode_oper_d="<<static_cast<int>(s2m.modes_of_operation_display)<<std::dec<<"  m2s.tar_pos="<<m2s.target_position<<"  m2s.prof_vel="<<m2s.profile_velocity<<" positionState="<<positioningState<<"  pendingTransition="<<pendingTransition<<" clearingEr="<< clearingError<<" enabled="<<static_cast<int>(enabled)<<" error="<<static_cast<int>(s2m.error_register)  );
}

void FestoController::clearError()
{
    ROS_INFO_STREAM("Festo controller clear error called for id="<<ec_idx
            <<"  and flag ="<<static_cast<int>(clearingError)<<"   and pending trans ="<<
                static_cast<int>(pendingTransition));
    if (clearingError) return;
    if( pendingTransition != trans_none) return;

    // if there's no error flag on status - no need to
    // start error quiting procedure
    if (s2m.error_register == 0 and not (s2m.statusword & 0x80))
        return;
    // TODO check if error state is really present
    clearingError = true;
    pendingTransition = trans_14;
}

void FestoController::disableMotor()
{
    disablingMotor = true;
    if (not enabled)
        return;
    // TODO
//std::cout<<"disabling motor "<<ec_idx<<std::endl;
    m2s.controlword = 0;
    enabled = false;
    enableInProgress = false;
//std::cout<<ec_idx<<"set EIP to false"<<std::endl;
    pendingTransition = trans_none;
    ownVelocity = 0;
}

void FestoController::toggleHalt()
{
    if( m2s.controlword & CTRL_HALT_MASK ) {
        m2s.controlword &= ~(CTRL_HALT_MASK);
    } else {
         m2s.controlword  |= CTRL_HALT_MASK;
    }

}

int32_t FestoController::getTargetVelocity() const
{
    if ( m2s.controlword & CTRL_HALT_MASK)
        return 0;

    return ownVelocity;
};

/**
    TODO this is not Festo controller's responsibility
    This should be done on 1 abstract level higher
*/
int32_t FestoController::getVelocityCorrections()
{
    double preRv = 0.;
    // TODO
    for( auto const &i: correctors) {
        preRv += i.first->getTargetVelocity() * i.second;
    }
    return static_cast<int32_t>(preRv);
}

void FestoController::setTargetVelocity( double val)
{
    std::cout<<ec_idx<<"Setting velocity from "<<ownVelocity;
    ownVelocity = 60 *val;
    std::cout<<"   to "<<ownVelocity<<std::endl;
};

void FestoController::completeHoming()
{
    homingStarted = true;
    m2s.modes_of_operation = 1;
    pendingTransition = trans_2;
    m2s.home_offset = 100 * (1 + ec_idx);
    m2s.target_position = 0;
}

void FestoController::setTargetPosition(int32_t position)
{
    targetPositionCopy = position - offset;
    return;
    m2s.controlword &= ~CTRL_NEW_SET_POINT_MASK;
    m2s.target_position = position - offset;
}

void FestoController::setPositioningVelocity(uint32_t velocity)
{
    targetVelocityCopy = velocity;
    return;
    m2s.controlword &= ~CTRL_NEW_SET_POINT_MASK;
    m2s.profile_velocity = velocity;
}

void FestoController::startPositionChange(bool immediate)
{
    positionChangeRequested = true;
    immediateCopy = immediate;
}

void FestoController::startPositionChangeInternal(bool immediate)
{
    if (!enabled) {
        pendingTransition = trans_2;
        enableInProgress = true;
//std::cout<<ec_idx<<"set EIP to true"<<std::endl;
    }
    m2s.modes_of_operation = 1;
    m2s.controlword &= ~(CTRL_HALT_MASK);
    //m2s.controlword |= CTRL_NEW_SET_POINT_MASK;
    positioningStarted = true;
    positioningState = trans_confirm_enabling;
}

void FestoController::setEndVelocity(uint32_t veloc)
{
    m2s.controlword &= ~CTRL_NEW_SET_POINT_MASK;
    m2s.end_velocity = veloc;
}

bool FestoController::isTargetReached() const
{
    return positioningStarted == false
        && s2m.modes_of_operation_display == 1
        && (s2m.statusword & TARGET_REACHED_STATUS_MASK);
};

void FestoController::setHomeOffset(int v)
{
std::cout<<"For festo "<<ec_idx<<" setting offset from "<<offset
        <<"  "<<v<<std::endl;
    offset = v;
}

void FestoController::resetClearingErrorState()
{
    m2s.controlword &= ~(1<<7);
    pendingTransition = trans_none;
    clearingError = false;
}

void FestoController::errorStateHandling()
{
std::cout<<"FestoController errorStateHandling"<<std::endl;
    switch (pendingTransition)
    {
        case trans_14:
            m2s.controlword |= (1<<7);
            pendingTransition = trans_15;
            break;
        case trans_15:
            if (s2m.statusword & 0x80) // (1<<7) == 0x80, controller confirmed Fault Error quoting
            {
                resetClearingErrorState();
            } else {
                m2s.controlword |= (1<<7);
            }
            break;
    }
    ROS_INFO_STREAM("m2s.contwd ="<<m2s.controlword<<"   s2m.stwd="<<std::hex<<s2m.statusword<<"   m2s.mode_ope="<<static_cast<int>(m2s.modes_of_operation)<<"  s2m.mode_oper_d="<<static_cast<int>(s2m.modes_of_operation_display)<<std::dec<<"  m2s.tar_pos="<<m2s.target_position<<"  m2s.prof_vel="<<m2s.profile_velocity<<" positionState="<<positioningState<<"  pendingTransition="<<pendingTransition<<" clearingEr="<< clearingError<<" enabled="<<static_cast<int>(enabled)<<" error="<<static_cast<int>(s2m.error_register)  );
}

std::ostream & operator<<(std::ostream &ostr, const FestoController &ctl)
{
    ostr<<"    FestoObject "<<ctl.ec_idx<<":"<<std::endl<<
        "             offset="<<ctl.offset<<std::endl; 

    ostr<<"Master2Slave :"<<std::endl<<
          "     modes_of_operation ="<<
                static_cast<int>(ctl.m2s.modes_of_operation)<<std::endl<<
          "            controlword ="<<ctl.m2s.controlword<<std::endl<<
          "        target velocity ="<<ctl.m2s.target_velocity<<std::endl<<
          "          homing method ="<<
                static_cast<int>(ctl.m2s.homing_method)<<std::endl<<
          "           end velocity ="<<ctl.m2s.end_velocity<<std::endl<<
          "        target position ="<<ctl.m2s.target_position<<std::endl<<
          "       profile velocity ="<<ctl.m2s.profile_velocity<<std::endl<<
          "             home offset="<<ctl.m2s.home_offset<<std::endl;
    ostr<<"Slave2Master :"<<std::endl<<
            "   modes of operation display="<<
                    static_cast<int>(ctl.s2m.modes_of_operation_display)<<std::endl<<
            "                   statusword="<<ctl.s2m.statusword<<std::endl<<
            "        position actual value="<<ctl.s2m.position_actual_value<<std::endl<<
            "               error register="<<
                    static_cast<int>(ctl.s2m.error_register)<<std::endl<<
            "        velocity actual value="<<ctl.s2m.velocity_actual_value<<std::endl<<
            "         current actual value="<<ctl.s2m.current_actual_value<<std::endl<<
            "           last warning code="<<ctl.s2m.last_warning_code<<std::endl;
        
    return ostr;
}
