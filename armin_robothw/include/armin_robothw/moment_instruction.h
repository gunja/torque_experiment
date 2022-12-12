#pragma once

#include <list>
#include <utility>
#include <vector>

#include <sys/time.h>

struct MotorPositionSpeedEndSpeed {
    int idx;
    int targetPosition;
    int targetVel;
    int targetEndVelocity;
};

class MomentInstructions {
        enum recordType {
            type_velocity,
            type_positional,
            type_delay
        } instrType;
        bool _isScheduled;
        static MomentInstructions * positionalFromLine(const char *line);
        static MomentInstructions * delayFromLine(const char *line);
    public:
        double executionMoment;
        std::list< std::pair< int, double > > listOfMotorIDX_velocity;
        std::list< MotorPositionSpeedEndSpeed > listMotorPositionSpeedEndSpeed;
        bool operator<(const MomentInstructions &b);
        MomentInstructions();
        ~MomentInstructions();
        static MomentInstructions * fromLine(const char *line);
        bool isVelocityType() const { return type_velocity == instrType; };
        bool isPositionalType() const { return type_positional == instrType; };
        bool isDelayType() const { return type_delay == instrType; };
        bool isDone(std::vector<bool>) const;
        bool isScheduled() const { return _isScheduled;};
        void setScheduled();
        void clrScheduled() { _isScheduled = false;};
        struct timeval tv;
        double delayDuration;
};
