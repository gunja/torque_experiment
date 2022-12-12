#pragma once
#ifndef EKXX_H
#define EKXX_H

#include "exchange_regions.h"
#include "beckhoff.h"

#define EK1828_OPEN_CHANNEL_NUMBER 0
#define EK1828_CLOSE_CHANNEL_NUMBER 4
#define EK1828_POWER_ON_DELAY_NS 500000000 

class EtherCATNetwork;

class EKXX : public BECKHOFF
{
    uint8_t outCand[2];
public:
    Ekxx_Region_m2s &m2s;
    Ekxx_Region_s2m &s2m;
public:
    EKXX(Ekxx_Region_m2s &m2s, Ekxx_Region_s2m &s2m, EtherCATNetwork &, int ecNetworkIdx);
    ~EKXX();
    bool powerOnChannel(int ch);
    void powerOffChannel(int ch);
    void regionsCandOutput();
};

#endif // EKXX_H
