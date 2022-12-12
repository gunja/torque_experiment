#include "elxx.h"

ELXX::ELXX(Elxx_Region_m2s &m2s_, Elxx_Region_s2m &s2m_, EtherCATNetwork & ecn_, int ecNetworkIdx):
    BECKHOFF (ecn_, ecNetworkIdx)
    , outCand {0}
    , m2s(m2s_), s2m(s2m_)
{

}

/**
    \func bool ELXX::powerOnChannel(int ch)
    \brief sets HIGH on required channel num \param[in] ch

    Method checks if \arg ch within allowed ranges ( 0..15)
    and sets this channel for output HIGH

    \param[in] ch channel num to set HIGH
*/
bool ELXX::powerOnChannel(int ch)
{
    if (ch <0 || ch > (EL2819_NUMBER_OF_OUTPUTS_CHANNELS - 1) )
        return false;

    int byteNum = ch / 8;
    ch %= 8;
    outCand[byteNum] |= ( 1<<(ch));
    return true;
}

void ELXX::powerOffChannel(int ch)
{
    if (ch <0 || ch > (EL2819_NUMBER_OF_OUTPUTS_CHANNELS - 1) )
        return;

    int byteNum = ch / 8;
    ch %= 8;
    outCand[byteNum] &= ~( 1<<(ch));
}
void ELXX::regionsCandOutput()
{
    m2s.out[0] = outCand[0];
    m2s.out[1] = outCand[1];
}

