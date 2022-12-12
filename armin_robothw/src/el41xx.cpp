#include "el41xx.h"

EL41XX::EL41XX(El41xx_Region_m2s &m2s_, El41xx_Region_s2m &s2m_, EtherCATNetwork &ecn_, int ecNetworkIdx ):
    ecn(ecn_)
    , ec_idx(ecNetworkIdx)
    , ch1OutCand(0), ch2OutCand(0)
    , m2s(m2s_), s2m(s2m_)
{

}




void EL41XX::reflectOutput()
{
    m2s.out_ch1 = ch1OutCand;
    m2s.out_ch2 = ch2OutCand;
}

void EL41XX::powerChannel_1(int16_t powCh1)
{
    ch1OutCand=powCh1;
}

void EL41XX::powerChannel_2(int16_t powCh2)
{
    ch1OutCand=powCh2;
}
