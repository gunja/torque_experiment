#include "beckhoff.h"

#include <iostream>

BECKHOFF::BECKHOFF(EtherCATNetwork & ecn_, int ecNetworkIdx):
    ecn(ecn_)
    , ec_idx(ecNetworkIdx)
{
    for(int i=0; i < 16; ++i)
    {
        endMomns[i] = {0, 0};
    }
}
BECKHOFF::~BECKHOFF()
{

}
bool operator<(const timespec&a, const timespec&b)
{
    if(a.tv_sec < b.tv_sec) return true;
    if (a.tv_sec > b.tv_sec) return false;
    return a.tv_nsec < b.tv_nsec;
}

const timespec operator+(const timespec&a, const uint32_t &deltans)
{
    timespec result;
    result.tv_sec = a.tv_sec + (a.tv_nsec + deltans)/1000000000;
    result.tv_nsec = (a.tv_nsec + deltans)%1000000000;
    return result;
 }

void BECKHOFF::powerOnChannelTimedOut(int ch, uint32_t dur_ns)
{
   if (powerOnChannel(ch))
   {
        timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        endMomns[ch] =  ts + dur_ns;
   }
}
void BECKHOFF::reflectOutput()
{
    timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    for(int i=0; i < 16; ++i)
        if  (endMomns[i] < now)
            if (endMomns[i].tv_sec > 0)
            {
                powerOffChannel(i);
                endMomns[i].tv_sec = 0;
            }
    regionsCandOutput();
 };

