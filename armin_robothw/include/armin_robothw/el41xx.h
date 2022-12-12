#pragma once
#ifndef EL41XX_H
#define EL41XX_H

#include <stdint.h>
#include "exchange_regions.h"

class EtherCATNetwork;

class EL41XX
{
    EtherCATNetwork & ecn;
    int ec_idx;
    int16_t ch1OutCand;
    int16_t ch2OutCand;
public:
    El41xx_Region_m2s &m2s;
    El41xx_Region_s2m &s2m;
public:
    EL41XX(El41xx_Region_m2s &m2s_, El41xx_Region_s2m &s2m_, EtherCATNetwork &ecn_, int ecNetworkIdx);
    ~EL41XX();
    void powerChannel_1(int16_t powCh1);
    void powerChannel_2(int16_t powCh2);
    void reflectOutput();
};

#endif // EL41XX_H
