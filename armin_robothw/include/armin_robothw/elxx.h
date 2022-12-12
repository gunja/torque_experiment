#pragma once
#ifndef ELXX_H
#define ELXX_H

#include "beckhoff.h"
#include "exchange_regions.h"

#define EL2819_NUMBER_OF_OUTPUTS_CHANNELS 16
class EtherCATNetwork;
class ELXX : public BECKHOFF
{
public:
    uint8_t outCand[2];
public:
    Elxx_Region_m2s &m2s;
    Elxx_Region_s2m &s2m;
public:
    ELXX(Elxx_Region_m2s &m2s_, Elxx_Region_s2m &s2m_, EtherCATNetwork &ecn_, int ecNetworkIdx);
    bool powerOnChannel(int ch);
    void powerOffChannel(int ch);
    void regionsCandOutput();
};

#endif // ELXX_H
