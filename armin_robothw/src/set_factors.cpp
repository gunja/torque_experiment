#include <iostream>
#include <ios>
#include <string>

#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

#include "ethercat.h"
#include "exchange_regions.h"



#define CTRL_MASK   0x01ff
#define CTRL_START  0x0010
#define CTRL_IMMEDIATLY 0x0020
#define CTRL_RELATE 0x0040
#define CTRL_RESET  0x0080
#define CTRL_HALT   0x0100

#define STATE_SWITCH_NOT_READY    0x0000
#define STATE_SWITCH_DISABLE      0x0040
#define STATE_SWITCH_READY        0x0021
#define STATE_SWITCH_ON           0x0023
#define STATE_ENABLE       0x0027
#define STATE_QUICK_STOP   0x0007
#define STATE_FAULT_ACTIVE 0x000f
#define STATE_FAULT        0x0008

#define STATUS_VOLTAGE 0x0010
#define STATUS_WARNING 0x0080
#define STATUS_MOVING  0x0100
#define STATUS_REMOTE  0x0200
#define STATUS_DONE    0x0400
#define STATUS_LIMIT   0x0800
#define STATUS_ACK     0x1000
#define STATUS_ERROR   0x2000
#define STATUS_MANUFAC 0x4000
#define STATUS_HOME    0x8000

#define CMD_SHUTDOWN        0x0006
#define CMD_SWITCH_ON       0x0007
#define CMD_DISABLE_VOLTAGE 0x0000
#define CMD_QUICK_STOP      0x0002
#define CMD_DISABLE         0x0007
#define CMD_ENABLE          0x000f
#define CMD_RESET           0x0080


int main(int argc, char * argv[])
{
    if( argc < 2) {
        std::cerr<<"Interface name is expected"<<std::endl;
        exit(EXIT_FAILURE);
    }
    if (argc < 3) {
        std::cerr<<"Slave num is expected as 2-d argument"<<std::endl;
        exit(EXIT_FAILURE);
    }

    char save_word[] = "save";
    for(auto i =0; i < 4; ++i) {
        std::cout<<i<<" -> "<<std::hex<<static_cast<int>(save_word[i])<<std::dec<<" ";
    }
    std::cout<<std::endl;
    std::cout<<"as int "<<std::hex<<(*(reinterpret_cast<int*>(save_word)))<<std::dec<<std::endl;


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


}
