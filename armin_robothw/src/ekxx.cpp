#include "ekxx.h"

#include <iostream>

EKXX::EKXX(Ekxx_Region_m2s &m2s_, Ekxx_Region_s2m &s2m_, EtherCATNetwork &ecn_, int ecNetworkIdx):
    BECKHOFF (ecn_, ecNetworkIdx)
    , outCand {0}
    , m2s(m2s_), s2m(s2m_)
{

}
EKXX::~EKXX()
{

}

bool EKXX::powerOnChannel(int ch)
{
    if (ch <0 || ch > 7)
        return false;

    int byteNum = ch / 4;
    ch %= 4;
    int offset = (byteNum)?0:4;

    outCand[byteNum] |= ( 1<<(ch + offset));
    return true;
}

void EKXX::powerOffChannel(int ch)
{
    if (ch <0 || ch > 7)
        return;

    int byteNum = ch / 4;
    ch %= 4;
    int offset = (byteNum)?0:4;

    outCand[byteNum] &= ~( 1<<(ch + offset));
}



void EKXX::regionsCandOutput()
{
    m2s.out[0] = outCand[0];
    m2s.out[1] = outCand[1];
};

