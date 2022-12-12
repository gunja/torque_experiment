#pragma once
#ifndef FESTO_OVER_ETHERCAT_H
#define FESTO_OVER_ETHERCAT_H

#include "exchange_regions.h"
#include <list>
#include <utility>
#include <iostream>

#define CTRL_HALT_MASK 0x0100
#define CTRL_NEW_SET_POINT_MASK 0x10
#define CTRL_CNG_SET_IMMEDIAT_MASK 0x20
#define SET_POINT_ACK_STATUS_MASK 0x1000
#define TARGET_REACHED_STATUS_MASK 0x400


#define INTERPOLATION_SUBMODE_SELECT_ADDR_SLOT 0x60c0
#define INTERPOLATION_SUBMODE_SELECT_ADDR_IDX 0
#define INTERPOLATION_DATA_RECORD_IP_DATA_POSITION_SLOT 0x60c1
#define INTERPOLATION_DATA_RECORD_IP_DATA_POSITION_IDX 1
#define INTERPOLATION_DATA_RECORD_IP_DATA_CONTROLWORD_SLOT 0x60c1
#define INTERPOLATION_DATA_RECORD_IP_DATA_CONTROLWORD_IDX 2
#define INTERPOLATION_TIME_PERIOD_IP_TIME_UNITS_SLOT 0X60C2
#define INTERPOLATION_TIME_PERIOD_IP_TIME_UNITS_IDX 1
#define INTERPOLATION_TIME_PERIOD_IP_TIME_INDEX_SLOT 0x60c2
#define INTERPOLATION_TIME_PERIOD_IP_TIME_INDEX_IDX 2
#define INTERPOLATION_DATA_CONFIGURATION_BUFFER_CLEAR_SLOT  0x60c4
#define INTERPOLATION_DATA_CONFIGURATION_BUFFER_CLEAR_IDX  6

#define STORE_PARAMETERS_SAVE_ALL_PARAMETERS_SLOT 0X1010
#define STORE_PARAMETERS_SAVE_ALL_PARAMETERS_IDX 1


#define MODE_OF_OPERATION_IP_MODE 7

class EtherCATNetwork;

class FestoController {
    enum ControllerTransitionStates {
        trans_none = 0,
        trans_0,
        trans_1,
        trans_2,
        trans_3,
        trans_4,

        trans_8,
        trans_9,
        trans_10,
        trans_14,
        trans_15

        , trans_start_homing
        , trans_homing_await_conf
    };


        bool clearingError;
        bool disablingMotor;

        bool homingStarted;

        bool positioningStarted;
    enum PositioningStateMachine {
        trans_none_position_sm = 0,
        trans_confirm_enabling,
        trans_confirm_mode_of_operation,
        trans_confirm_clearence_SP_ack,
        trans_confirm_SP_ack
    } positioningState; 

        EtherCATNetwork & ecn;
        int ec_idx;
        int32_t ownVelocity;
        int32_t offset;
        int32_t getVelocityCorrections();
        std::list< std::pair<const FestoController*, double > >  correctors;

        int32_t targetPositionCopy;
        uint32_t targetVelocityCopy;
        bool positionChangeRequested;
        bool immediateCopy;
        void startPositionChangeInternal(bool);
        void errorStateHandling();

    public:
        Master2Slave &m2s;
        Slave2Master &s2m;
        
    enum ControllerTransitionStates pendingTransition;
        bool enabled;
        bool enableInProgress;
    public:
        FestoController(Master2Slave &m2s, Slave2Master &_s2m, EtherCATNetwork &, int ecNetworkIdx);
        ~FestoController();
        void startEnabling();
        void updateOutputs();
        void clearError();
        void disableMotor();
        void toggleHalt();
        void setTargetVelocity( int val) {ownVelocity = val;};
        void setTargetVelocity( double val);
        int32_t getTargetVelocity() const;
        void addCorrector(const FestoController* f, double d) {
            correctors.push_back( { f,d});
        };
        void completeHoming();
        void setTargetPosition(int32_t position);
        void setPositioningVelocity(uint32_t velocity);
        void setEndVelocity(uint32_t veloc);
        void startPositionChange(bool = false);
        int32_t getPositionValue() const { return s2m.position_actual_value + offset; };
        int32_t getVelocityValue() const { return s2m.velocity_actual_value; };
        int16_t getCurrentValue() const { return s2m.current_actual_value; };
        bool isTargetReached() const;
        bool isHomingStarted() const { return homingStarted; };
        void clearEIP() { enableInProgress = false; };
        void setHomeOffset(int v);
        uint8_t getErrorRegister() const { return s2m.error_register;};
        void resetClearingErrorState();
    friend std::ostream & operator<<(std::ostream &ostr, const FestoController &ctl);
};

#endif // FESTO_OVER_ETHERCAT_H
