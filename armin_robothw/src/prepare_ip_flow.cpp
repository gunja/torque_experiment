#include <iostream>
#include <ios>
#include <string>
#include <string.h>

#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

#include "ethercat.h"
#include "exchange_regions.h"
#include "festo_over_ec.h"

int main(int argc, char * argv[])
{
    if( argc < 2) {
        std::cerr<<"Interface name is expected"<<std::endl;
        exit(EXIT_FAILURE);
    }

    char save_word[] = "save";

    if( ec_init( argv[1]) < 0) {
        std::cerr<<"EC INIT failed. are you root? "<<std::endl;
        exit(EXIT_FAILURE);
    }

    if (ec_config_init(TRUE) < 1)
    {
        std::cerr<<"too less slaves found. not right"<<std::endl;
        exit(EXIT_FAILURE);
    }
    // сейчас мы должны уже быть в PRE-OP state
    std::cout<<"Network state =="<<ec_slave[0].state<<std::endl;
    std::cout<<ec_slave[1].name<<std::endl;
    uint8_t u8;
    int8_t s8;
    int sz = sizeof( u8);

    for(int i=1; i <= ec_slavecount; ++i)
    {
        std::cout<<"Slave at "<<i<<"  is "<<ec_slave[i].name<<std::endl;
        if (strcmp(ec_slave[i].name,"? M:0000001d I:502d4153") != 0 )
        {
            std::cout<<"Skipping not relevant device "<<ec_slave[i].name<<std::endl;
            continue;
        }
        int16_t interpolation_submode_select;
        sz = sizeof(interpolation_submode_select);
        ec_SDOread(i, INTERPOLATION_SUBMODE_SELECT_ADDR_SLOT, INTERPOLATION_SUBMODE_SELECT_ADDR_IDX,
                    FALSE, &sz, &interpolation_submode_select, EC_TIMEOUTRXM);
        int8_t ip_time_index;
        sz = sizeof(ip_time_index);
        ec_SDOread(i, INTERPOLATION_TIME_PERIOD_IP_TIME_INDEX_SLOT, INTERPOLATION_TIME_PERIOD_IP_TIME_INDEX_IDX,
                    FALSE, &sz, &ip_time_index, EC_TIMEOUTRXM);
        int8_t ip_time_units;
        sz = sizeof(ip_time_index);
        ec_SDOread(i, INTERPOLATION_TIME_PERIOD_IP_TIME_UNITS_SLOT, INTERPOLATION_TIME_PERIOD_IP_TIME_UNITS_IDX,
                    FALSE, &sz, &ip_time_units, EC_TIMEOUTRXM);

        std::cout<<"On reading current settings received:"<<
            "\ninterpolation_submode_select="<<interpolation_submode_select<<
            "\ninterpolation_time_index ="<<(int)ip_time_index<<
            "\ninterpolation_time_units="<<(int)ip_time_units<<std::endl;

        std::cout<<"Setting "<<SYNC_PERIOD_MS<<"ms synchronization period"<<std::endl;

        interpolation_submode_select = -2;
        sz = sizeof(interpolation_submode_select);
        ec_SDOwrite(i, INTERPOLATION_SUBMODE_SELECT_ADDR_SLOT, INTERPOLATION_SUBMODE_SELECT_ADDR_IDX,
                    FALSE, sz, &interpolation_submode_select, EC_TIMEOUTRXM);
        ip_time_index = -3;
        sz = sizeof(ip_time_index);
        ec_SDOwrite(i, INTERPOLATION_TIME_PERIOD_IP_TIME_INDEX_SLOT, INTERPOLATION_TIME_PERIOD_IP_TIME_INDEX_IDX,
                    FALSE, sz, &ip_time_index, EC_TIMEOUTRXM);
        ip_time_units= SYNC_PERIOD_MS;
        sz = sizeof(ip_time_index);
        ec_SDOwrite(i, INTERPOLATION_TIME_PERIOD_IP_TIME_UNITS_SLOT, INTERPOLATION_TIME_PERIOD_IP_TIME_UNITS_IDX,
                    FALSE, sz, &ip_time_units, EC_TIMEOUTRXM);

        std::cout<<"Writing changes to permament"<<std::endl;
        
        sz = 4;
        uint32_t t32 = 0x65766173;
        ec_SDOwrite( i, STORE_PARAMETERS_SAVE_ALL_PARAMETERS_SLOT, STORE_PARAMETERS_SAVE_ALL_PARAMETERS_IDX, FALSE, sz, &t32, EC_TIMEOUTRXM);

    }

    std::cout<<"Please, reboot all the devices"<<std::endl;

#if 0
    int rqSlave = atoi(argv[2]);

    if (ec_slavecount < rqSlave) {
        std::cerr<<"Requested slave "<<rqSlave<<" does not exist. Last known is "<<ec_slavecount<<std::endl;
        exit(EXIT_FAILURE);
    }

    uint32_t den, num;
    int slotNum = 0x6093;
    sz = sizeof( uint32_t);
    ec_SDOread(rqSlave, slotNum, 1, FALSE, &sz, &den, EC_TIMEOUTRXM);
    ec_SDOread(rqSlave, slotNum, 2, FALSE, &sz, &num, EC_TIMEOUTRXM);
    std::cout<<"Position factor: "<<den<<" / "<<num<<std::endl;

    slotNum = 0x6094;
    ec_SDOread(rqSlave, slotNum, 1, FALSE, &sz, &den, EC_TIMEOUTRXM);
    ec_SDOread(rqSlave, slotNum, 2, FALSE, &sz, &num, EC_TIMEOUTRXM);
    std::cout<<"Velocity factor: "<<den<<" / "<<num<<std::endl;

    slotNum = 0x6097;
    ec_SDOread(rqSlave, slotNum, 1, FALSE, &sz, &den, EC_TIMEOUTRXM);
    ec_SDOread(rqSlave, slotNum, 2, FALSE, &sz, &num, EC_TIMEOUTRXM);
    std::cout<<"Acceleration factor: "<<den<<" / "<<num<<std::endl;

    std::string vv;
    std::cout<<"Should values be re-written?"<<std::endl;
    std::cin>>vv;
    if (vv[0] != 'y') {
        std::cout<<"Bye."<<std::endl;
        exit(EXIT_SUCCESS);
    }

    den = 4096; num =225; slotNum = 0x6093;
    ec_SDOwrite( rqSlave, slotNum, 1, FALSE, sz, &den, EC_TIMEOUTRXM);
    ec_SDOwrite( rqSlave, slotNum, 2, FALSE, sz, &num, EC_TIMEOUTRXM);
    den = 1024; num =15; slotNum = 0x6094;
    ec_SDOwrite( rqSlave, slotNum, 1, FALSE, sz, &den, EC_TIMEOUTRXM);
    ec_SDOwrite( rqSlave, slotNum, 2, FALSE, sz, &num, EC_TIMEOUTRXM);
    den = 64; num =15; slotNum = 0x6097;
    ec_SDOwrite( rqSlave, slotNum, 1, FALSE, sz, &den, EC_TIMEOUTRXM);
    ec_SDOwrite( rqSlave, slotNum, 2, FALSE, sz, &num, EC_TIMEOUTRXM);

    char save[] = "save";
    ec_SDOwrite( rqSlave, 0x1010, 1, FALSE, sz, &save, EC_TIMEOUTRXM);

    std::cout<<"Done! type any key"<<std::endl;
    std::cin>>vv;

    slotNum = 0x6093; den =0; num = 0; sz = 4;
    ec_SDOread(rqSlave, slotNum, 1, FALSE, &sz, &den, EC_TIMEOUTRXM);
    ec_SDOread(rqSlave, slotNum, 2, FALSE, &sz, &num, EC_TIMEOUTRXM);
    std::cout<<"Position factor: "<<den<<" / "<<num<<std::endl;

    slotNum = 0x6094; den =0; num = 0;
    ec_SDOread(rqSlave, slotNum, 1, FALSE, &sz, &den, EC_TIMEOUTRXM);
    ec_SDOread(rqSlave, slotNum, 2, FALSE, &sz, &num, EC_TIMEOUTRXM);
    std::cout<<"Velocity factor: "<<den<<" / "<<num<<std::endl;

    slotNum = 0x6097; den =0; num = 0;
    ec_SDOread(rqSlave, slotNum, 1, FALSE, &sz, &den, EC_TIMEOUTRXM);
    ec_SDOread(rqSlave, slotNum, 2, FALSE, &sz, &num, EC_TIMEOUTRXM);
    std::cout<<"Acceleration factor: "<<den<<" / "<<num<<std::endl;
#endif
    return 0;
}

