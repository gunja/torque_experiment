#pragma once
#ifndef BECKHOFF_H
#define BECKHOFF_H

#include <time.h>
#include <stdint.h>

class EtherCATNetwork;

class BECKHOFF
{
    EtherCATNetwork & ecn;
    int ec_idx;
    timespec endMomns[16];
public:
    BECKHOFF(EtherCATNetwork &ecn_, int ecNetworkIdx);
    virtual ~BECKHOFF();
    virtual bool powerOnChannel(int ch) = 0;
    virtual void powerOffChannel(int ch) = 0;
    virtual void regionsCandOutput() = 0;
    void powerOnChannelTimedOut(int ch, uint32_t dur_ns);
    void reflectOutput();
    friend const timespec operator+(const timespec&a, const uint32_t &deltans);
    friend bool operator<(const timespec&a, const timespec&b);
};

#endif // BECKHOFF_H
