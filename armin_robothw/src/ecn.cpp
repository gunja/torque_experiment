#include <ncurses.h>
#include "ecn.h"
#include "ethercat.h"

#include <iostream>
#include <ios>
#include <unistd.h>
#include <string.h>

#include <ros/ros.h>

#define EC_TIMEOUTMON 500

#ifndef EC_STATE_NONE
#define EC_STATE_NONE 0
#endif


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


#define USED_FESTO_THINGS 2

EtherCATNetwork::EtherCATNetwork(): slavesFound(-1), started(false), errorCode(0)
    , operational(false)
    , usedmem(0)
    , cyclic_exchange_thread (0)
    , keepThreadRunning(true)
{
    memset(IOmap, 0, IOMAP_SIZE);
    memset(IOmapMirror, 0, IOMAP_SIZE);
}

EtherCATNetwork::~EtherCATNetwork()
{
    // TODO
    if (keepThreadRunning) {
        keepThreadRunning = false;
        pthread_join(cyclic_exchange_thread, nullptr);
        cyclic_exchange_thread =-1;
    }

    // TODO this saving has no relation to ethercat itself.
    // this should be done on a level higher: on controllers level
    // which encapsulates this communication channel
    int32_t *values = new int32_t[ec_slavecount];

    for(int slave = 1; slave <= ec_slavecount; ++slave) {
    }
    delete[] values;

    if (started) {
        ec_close();
        started = false;
    }
}

#ifdef SET_PDO_IN_ECN
void EtherCATNetwork::setPDOs()
{
    uint8_t t8;
    uint16_t t16;
    uint32_t t32;
    for( int s = 1; s <= ec_slavecount; ++s) {
        std::cout<<"Working on slave "<<s<<std::endl;
        int sz = sizeof( t8);

	std::cout<<"Getting 1400.1";
	uint32_t v1800_1; sz = sizeof(v1800_1);
	int rv = ec_SDOread(s, 0x1400, 1, FALSE, &sz, &v1800_1, EC_TIMEOUTRXM);
	std::cout<<" resulted in   "<<rv<<"  with altered sz="<<sz<<"   and value = "<<
		std::hex<<v1800_1<<std::dec<<std::endl;
	if ( v1800_1 & (1<<31) ) {
		std::cout<<"bit 31 is not set"<<std::endl;
	}
	v1800_1 ^= (1<<31); sz = sizeof(v1800_1);
	rv = ec_SDOwrite( s, 0x1400, 1, FALSE, sz, &v1800_1, EC_TIMEOUTRXM);
	std::cout<<"  wrote 1400.1  with "<<rv<<std::endl;
	sz = sizeof(v1800_1);
	rv = ec_SDOread(s, 0x1400, 1, FALSE, &sz, &v1800_1, EC_TIMEOUTRXM);
	std::cout<<" resulted in   "<<rv<<"  with altered sz="<<sz<<"   and value = "<<
		std::hex<<v1800_1<<std::dec<<std::endl;
	

        t8 = 0;
        std::cout<<"\tSetting 1600.00 to 0"<<ec_SDOwrite(s, 0x1600, 0, FALSE, sizeof(t8), &t8, EC_TIMEOUTRXM)<<std::endl;
        t32 = 0x60600008;
        std::cout<<"\tSetting 1600.01 to "<<std::hex<<t32<<std::dec<<"  results in "
                <<ec_SDOwrite(s, 0x1600, 1, FALSE, sizeof(t32), &t32, EC_TIMEOUTRXM)<<std::endl;
        t32 = 0x60400010;
        std::cout<<"\tSetting 1600.02 to "<<std::hex<<t32<<std::dec<<"  results in "
                <<ec_SDOwrite(s, 0x1600, 2, FALSE, sizeof(t32), &t32, EC_TIMEOUTRXM)<<std::endl;
        t32 = 0x60FF0020;
        std::cout<<"\tSetting 1600.03 to "<<std::hex<<t32<<std::dec<<"  results in "
                <<ec_SDOwrite(s, 0x1600, 3, FALSE, sizeof(t32), &t32, EC_TIMEOUTRXM)<<std::endl;
        t8 = 3;
        std::cout<<"\tSetting 1600.00 to 3"<<ec_SDOwrite(s, 0x1600, 0, FALSE, sizeof(t8), &t8, EC_TIMEOUTRXM)<<std::endl;
	v1800_1 &= ~(1<<31); sz = sizeof(v1800_1);
	rv = ec_SDOwrite( s, 0x1400, 1, FALSE, sz, &v1800_1, EC_TIMEOUTRXM);

	sz = sizeof(v1800_1);
	rv = ec_SDOread(s, 0x1401, 1, FALSE, &sz, &v1800_1, EC_TIMEOUTRXM);
	std::cout<<" resulted in   "<<rv<<"  with altered sz="<<sz<<"   and value = "<<
		std::hex<<v1800_1<<std::dec<<std::endl;
	v1800_1 ^= (1<<31); sz = sizeof(v1800_1);
	rv = ec_SDOwrite( s, 0x1401, 1, FALSE, sz, &v1800_1, EC_TIMEOUTRXM);
	std::cout<<"  wrote 1401.1  with "<<rv<<std::endl;
	sz = sizeof(v1800_1);
	rv = ec_SDOread(s, 0x1401, 1, FALSE, &sz, &v1800_1, EC_TIMEOUTRXM);
	std::cout<<" resulted in   "<<rv<<"  with altered sz="<<sz<<"   and value = "<<
		std::hex<<v1800_1<<std::dec<<std::endl;
        t8 = 0;
        std::cout<<"\tSetting 1601.00 to 0  ->"<<ec_SDOwrite(s, 0x1601, 0, FALSE, sizeof(t8), &t8, EC_TIMEOUTRXM)<<std::endl;
        t32 = 0x60820020;
        std::cout<<"\tSetting 1601.01 to "<<std::hex<<t32<<std::dec<<"  results in "
                <<ec_SDOwrite(s, 0x1601, 1, FALSE, sizeof(t32), &t32, EC_TIMEOUTRXM)<<std::endl;
        t8 = 1;
        std::cout<<"\tSetting 1601.00 to 1  ->"<<ec_SDOwrite(s, 0x1601, 0, FALSE, sizeof(t8), &t8, EC_TIMEOUTRXM)<<std::endl;
	v1800_1 &= ~(1<<31); sz = sizeof(v1800_1);
	rv = ec_SDOwrite( s, 0x1401, 1, FALSE, sz, &v1800_1, EC_TIMEOUTRXM);

        t8 = 2;
        std::cout<<"\tSetting 1C12.0 to 2 "<<ec_SDOwrite(s, 0x1C12, 0, FALSE, sizeof(t8), &t8, EC_TIMEOUTRXM)<<std::endl;





	sz = sizeof(v1800_1);
	rv = ec_SDOread(s, 0x1800, 1, FALSE, &sz, &v1800_1, EC_TIMEOUTRXM);
	std::cout<<" resulted in   "<<rv<<"  with altered sz="<<sz<<"   and value = "<<
		std::hex<<v1800_1<<std::dec<<std::endl;
	v1800_1 ^= (1<<31); sz = sizeof(v1800_1);
	rv = ec_SDOwrite( s, 0x1800, 1, FALSE, sz, &v1800_1, EC_TIMEOUTRXM);
	std::cout<<"  wrote 1800.1  with "<<rv<<std::endl;
	sz = sizeof(v1800_1);
	rv = ec_SDOread(s, 0x1800, 1, FALSE, &sz, &v1800_1, EC_TIMEOUTRXM);
	std::cout<<" resulted in   "<<rv<<"  with altered sz="<<sz<<"   and value = "<<
		std::hex<<v1800_1<<std::dec<<std::endl;
        t8 = 0;
        uint16_t slotNum = 0x1A00;
        std::cout<<"Setting 1A00.0 to 0  "<<ec_SDOwrite(s, slotNum, 0, FALSE, sizeof(t8), &t8, EC_TIMEOUTRXM)<<std::endl;
        t32 = 0x60610008;
        std::cout<<"\tSetting 1A00.01 to "<<std::hex<<t32<<std::dec<<"  results in "
            <<ec_SDOwrite(s, slotNum, 1, FALSE, sizeof( t32), &t32, EC_TIMEOUTRXM)<<std::endl;
        t32 = 0x60410010;
        std::cout<<"\tSetting 1A00.02 to "<<std::hex<<t32<<std::dec<<"  results in "
            <<ec_SDOwrite(s, slotNum, 2, FALSE, sizeof( t32), &t32, EC_TIMEOUTRXM)<<std::endl;
        t32 = 0x60640020;
        std::cout<<"\tSetting 1A00.03 to "<<std::hex<<t32<<std::dec<<"  results in "
            <<ec_SDOwrite(s, slotNum, 3, FALSE, sizeof( t32), &t32, EC_TIMEOUTRXM)<<std::endl;
        t8 = 3;
        std::cout<<"Setting 1A00.0 to 3  "<<ec_SDOwrite(s, slotNum, 0, FALSE, sizeof(t8), &t8, EC_TIMEOUTRXM)<<std::endl;

	v1800_1 &= ~(1<<31); sz = sizeof(v1800_1);
	rv = ec_SDOwrite( s, 0x1800, 1, FALSE, sz, &v1800_1, EC_TIMEOUTRXM);

	sz = sizeof(v1800_1);
	rv = ec_SDOread(s, 0x1801, 1, FALSE, &sz, &v1800_1, EC_TIMEOUTRXM);
	std::cout<<" resulted in   "<<rv<<"  with altered sz="<<sz<<"   and value = "<<
		std::hex<<v1800_1<<std::dec<<std::endl;
	v1800_1 ^= (1<<31); sz = sizeof(v1800_1);
	rv = ec_SDOwrite( s, 0x1801, 1, FALSE, sz, &v1800_1, EC_TIMEOUTRXM);
	std::cout<<"  wrote 1801.1  with "<<rv<<std::endl;
	sz = sizeof(v1800_1);
	rv = ec_SDOread(s, 0x1801, 1, FALSE, &sz, &v1800_1, EC_TIMEOUTRXM);
	std::cout<<" resulted in   "<<rv<<"  with altered sz="<<sz<<"   and value = "<<
		std::hex<<v1800_1<<std::dec<<std::endl;


        t8 = 0; slotNum++;
        std::cout<<"Setting 1A01.0 to 0  "<<ec_SDOwrite(s, slotNum, 0, FALSE, sizeof(t8), &t8, EC_TIMEOUTRXM)<<std::endl;
        t32 = 0x606C0020;
        std::cout<<"\tSetting 1A01.01 to "<<std::hex<<t32<<std::dec<<"  results in "
            <<ec_SDOwrite(s, slotNum, 1, FALSE, sizeof( t32), &t32, EC_TIMEOUTRXM)<<std::endl;
        t8 = 1;
        std::cout<<"Setting 1A01.0 to 1  "<<ec_SDOwrite(s, slotNum, 0, FALSE, sizeof(t8), &t8, EC_TIMEOUTRXM)<<std::endl;
	v1800_1 &= ~(1<<31); sz = sizeof(v1800_1);
	rv = ec_SDOwrite( s, 0x1801, 1, FALSE, sz, &v1800_1, EC_TIMEOUTRXM);

        t8 = 2;
        std::cout<<"\tSetting 1C13.0 to 2 "<<ec_SDOwrite(s, 0x1C13, 0, FALSE, sizeof(t8), &t8, EC_TIMEOUTRXM)<<std::endl;

        t32 = 0x65766173;
        std::cout<<"Calling save for settings ->"<<ec_SDOwrite(s, 0x1010, 1, FALSE, sizeof(t32), &t32, EC_TIMEOUTRXM) <<std::endl;
    }
}
#endif

#ifdef SET_1C1x_IN_ECN
void EtherCATNetwork::set1C1x()
{
    uint8_t v;
    for(int s = 1; s <= USED_FESTO_THINGS; ++s)
    {
        v= 4;
        ec_SDOwrite( s, 0x1C12, 0, FALSE, 1, &v, EC_TIMEOUTRXM);
        ec_SDOwrite( s, 0x1C13, 0, FALSE, 1, &v, EC_TIMEOUTRXM);
    }
}
#endif


int set1C1X__(uint16_t slave)
{
    uint8_t vOut;
    vOut = 4;
    int sz = sizeof(vOut);
    ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sz, &vOut, EC_TIMEOUTSAFE);
    uint8_t vIn;
    vIn = 4;
    sz = sizeof(vIn);
    ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sz, &vIn, EC_TIMEOUTSAFE);
}


#define S2_S3_manual

bool EtherCATNetwork::startNetwork(const char *interface)
{
    if (started) return true;

    int rv = ec_init(const_cast<char*>(interface));
    if (rv <= 0)
    {
        errorCode = EC_FAILED_IFACE_BRING_UP;
        return false;
    }

#ifndef S2_S3_manual
    slavesFound = ec_config_init(TRUE);
#else
    slavesFound = ec_config_init(FALSE);
    for(int i=1; i <= USED_FESTO_THINGS; ++i) {
        ec_slave[i].SM[2].StartAddr =0x1600;
        ec_slave[i].SM[3].StartAddr =0x1A00;
        ec_slave[i].SM[2].SMlength = sizeof(Master2Slave);
        ec_slave[i].SM[3].SMlength = sizeof(Slave2Master);
        ec_slave[i].SM[2].SMflags= 0x10024;
        ec_slave[i].SM[3].SMflags= 0x10022;

        ec_slave[i].FMMU[0].FMMUactive=1;
        ec_slave[i].FMMU[1].FMMUactive=1;
        ec_slave[i].FMMU0func= 1;
        ec_slave[i].FMMU1func= 2;


        ec_slave[i].Dtype = 8;
        ec_slave[i].Obits = sizeof(Master2Slave) * 8;
        ec_slave[i].Ibits = sizeof(Slave2Master) * 8;
        ec_slave[i].Obytes = sizeof(Master2Slave);
        ec_slave[i].Ibytes = sizeof(Slave2Master);

        ec_slave[i].configindex= 23;

        ec_slave[i].PO2SOconfig = set1C1X__;

    }
#endif
    started = ec_slavecount != 0;
/*
    for(int i=1; i <= ec_slavecount; ++i) {
        uint8_t u8 = 3;
        int sz = sizeof( u8);
        ec_SDOwrite( i, 0x6060, 0, FALSE, sz, &u8, EC_TIMEOUTRXM);
        u8 = 12;
        ec_SDOwrite( i, 0x2420, 0x03, FALSE, sz, &u8, EC_TIMEOUTRXM);
    }
*/
//    setPDOs();
    //set1C1x();
//    resetErrors();

    return started;
}

void EtherCATNetwork::resetErrors()
{
	for(int slave = 1; slave <= ec_slavecount; ++slave)
    {
        clearErrorState(slave);
    }
}

int EtherCATNetwork::getECKnownSlavesCount() const { if(started) return ec_slavecount; else return 0; };

const char * EtherCATNetwork::getSlaveName(int i)
{
    if (i < 1 || i > ec_slavecount)
        return nullptr;
    return ec_slave[i].name;
}

bool EtherCATNetwork::SwitchOperational()
{
    if (! started) return false;
    if( ! usedmem) {
        usedmem = ec_config_map(IOmap);
        ec_configdc();
        usedmem = true;
    }
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_send_processdata();
    int wks = ec_receive_processdata(EC_TIMEOUTRET);

	ec_writestate(0);

    int st = EC_STATE_NONE;
    int count = 0;
    do
    {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
        st = ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
        ++count;
        std::cerr<<"Network state now is "<<ec_slave[0].state<<std::endl;
    }  while(count < 50 && ec_slave[0].state != EC_STATE_OPERATIONAL);

    operational = (ec_slave[0].state == EC_STATE_OPERATIONAL);
    return ec_slave[0].state == EC_STATE_OPERATIONAL;
}

bool EtherCATNetwork::SwitchSafeOperational()
{
    if (! started) return false;
    if (! usedmem) {
        memset( IOmap, 0, sizeof(IOmap));
        memset( IOmapMirror, 0, sizeof(IOmap));
        usedmem = ec_config_map(IOmap);
    }
    int st = ec_readstate();
        for(int i=1; i < ec_slavecount; ++i) {
            uint8_t u8;
            u8 = 1;
            ec_SDOwrite( i, 0x60C4, 6, FALSE, sizeof(uint8_t), &u8, EC_TIMEOUTRXM);
            int8_t s8 = 3;
            ec_SDOwrite(i, MODE_OF_OPERATION_SLOT, 0, FALSE, sizeof(int8_t), &s8, EC_TIMEOUTRXM);
        }

    ec_configdc();
    //for(int slave = 1; slave <= ec_slavecount; slave++)
    for(int slave = 1; slave <= 2; slave++)
    {
        if (ec_slave[slave].hasdc) {
	     std::cerr<<"Setting dcsync of slave "<<slave<<" to value "<< SYNC_PERIOD_MS *1000000UL <<std::endl;
            ec_dcsync0( slave, TRUE, SYNC_PERIOD_MS *1000000, 0); // SYNC0 on slave
        }
    }


    st = EC_STATE_NONE;
    int count = 0;
    while(count < 30 && st != EC_STATE_SAFE_OP)
    {
        st = ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
        ++count;
    }
    if (st != EC_STATE_SAFE_OP)
    {
	    std::cerr<<"After "<<count<<" attempts, still have state == "<<st<<std::endl;
    }

    if (st == EC_STATE_SAFE_OP && cyclic_exchange_thread == 0) {
        for (int s = 1; s <= ec_slavecount; ++s) {
            Master2Slave &outVal( *reinterpret_cast<Master2Slave*>(
                    ec_slave[s].outputs));
            //written mode of operation
            int8_t s8; int sz = sizeof(s8);
            ec_SDOread(s, MODE_OF_OPERATION_SLOT, 0, FALSE, &sz, &s8, EC_TIMEOUTRXM);
            outVal.modes_of_operation = s8;
            uint16_t cw; sz = sizeof( cw);
            ec_SDOread(s, CONTROL_WORD_SLOT, 0, FALSE, &sz, &cw, EC_TIMEOUTRXM);
            outVal.controlword = cw;
            //target velocity we force to 0
            outVal.target_velocity = 0;
            // end velocity we force to 0
            outVal.end_velocity = 0; 
        }
        pthread_attr_t high_prio_attrs;
        pthread_attr_init(&high_prio_attrs);
        struct sched_param prio; prio.sched_priority = 10;
        pthread_attr_setschedparam(&high_prio_attrs, &prio);
        pthread_attr_setschedpolicy(&high_prio_attrs, SCHED_FIFO);
        if ( pthread_create(&cyclic_exchange_thread, &high_prio_attrs, CyclicThreadEntry, this))
        {
          ROS_FATAL_STREAM("Failed to call create with high priority settings. Error is "<<(errno==EAGAIN?"no resources":"")
            <<(errno==EINVAL?"invalid settings in attr":"")<<(errno==EPERM?"no permission to set policy":""));
          ROS_INFO("Starting normal priority");
          pthread_create( &cyclic_exchange_thread, NULL, CyclicThreadEntry, this);
        }
        // ATT return value is not checked
    }
   
    return st == EC_STATE_SAFE_OP;
}

bool EtherCATNetwork::switchOP_SO()
{
    if (!isOperational())
    {
        // we can not perform this transition as original state
        // is not Operational
        return false;
    }

    ec_slave[0].state = EC_STATE_SAFE_OP;
	ec_writestate(0);
    operational = false;
    for(int slave = 1; slave <= ec_slavecount; slave++)
        ec_dcsync0( slave, FALSE, 5000000, 0); // SYNC0 on slave

    // assuming that everything was done OK
    return true;
}

uint32_t EtherCATNetwork::getSlotDataUINT32(int slaveNum, int slotNum, int subslotNum)
{
    if (!started) return 0U;
    if ( slaveNum < 1 || slaveNum > ec_slavecount) return 0U;

    int size = sizeof(uint32_t);
    uint32_t rez;

    ec_SDOread(slaveNum, slotNum, subslotNum, FALSE, &size, &rez, EC_TIMEOUTRXM);
    return rez;
}

uint16_t EtherCATNetwork::getSlotDataUINT16(int slaveNum, int slotNum, int subslotNum)
{
    if (!started) return 0U;
    if ( slaveNum < 1 || slaveNum > ec_slavecount) return 0U;

    int size = sizeof(uint16_t);
    uint16_t rez;

    ec_SDOread(slaveNum, slotNum, subslotNum, FALSE, &size, &rez, EC_TIMEOUTRXM);
    return rez;
}

uint8_t EtherCATNetwork::getSlotDataUINT8(int slaveNum, int slotNum, int subslotNum)
{
    if (!started) return 0U;
    if ( slaveNum < 1 || slaveNum > ec_slavecount) return 0U;

    int size = sizeof(uint8_t);
    uint8_t rez;

    ec_SDOread(slaveNum, slotNum, subslotNum, FALSE, &size, &rez, EC_TIMEOUTRXM);
    return rez;
}

uint16_t EtherCATNetwork::getSlaveState(int s) const
{
    if( !started) return EC_STATE_NONE;

    return ec_statecheck(s, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
}

const char *errorTexts[] = {
    "No errors",
    "Failed to bring up interface",
    "Failed to discover any slave"
};

const char * EtherCATNetwork::ecStateMachineAsText(uint16_t s)
{
    switch(s) {
        case EC_STATE_NONE:
            return "No valid state";
        case EC_STATE_INIT:
            return "Init state";
        case EC_STATE_PRE_OP:
            return "Pre-operational";
        case EC_STATE_BOOT:
            return "Boot state";
        case EC_STATE_SAFE_OP:
            return "Safe-operational";
        case EC_STATE_OPERATIONAL:
            return "Operational";
        case EC_STATE_ACK:
            return "Error or ACK error";
        //case EC_STATE_ERROR:
        //    return "State ERROR";
    }
    return "Not known";
}

bool EtherCATNetwork::fillExchangeRegions(int slave, const Master2Slave &sending, Slave2Master & receiving, bool ignoreSending)
{
    if (!operational) return false;

    if (slave < 1 || slave > ec_slavecount) return false;

    // CRITICAL!!! enclose in case of multi-threaded application
    if (!ignoreSending) {
        memcpy( ec_slave[slave].outputs, &sending, sizeof(Master2Slave));
    }
    memcpy( &receiving, ec_slave[slave].inputs, sizeof(Slave2Master));

    return true;
}

void *EtherCATNetwork::getOutputPointer(int slaveNum)
{
    if (slaveNum < 1 || slaveNum > ec_slavecount)
    {
        return nullptr;
    }
    return (ec_slave[slaveNum].outputs - 
        reinterpret_cast<uint8*>(IOmap)) + IOmapMirror;
}

void* EtherCATNetwork::getInputPointer(int slaveNum)
{
    if (slaveNum < 1 || slaveNum > ec_slavecount)
    {
        return nullptr;
    }
    return (ec_slave[slaveNum].inputs - 
        reinterpret_cast<uint8*>(IOmap)) + IOmapMirror;
}

void EtherCATNetwork::clearErrorState(int slaveNum)
{
    if (slaveNum < 1 || slaveNum > ec_slavecount) return;

    int i;
    uint16 state, ctrl, cmd;
    int l16 = sizeof( uint16);
    ec_SDOread(slaveNum, 0x6041, 0x00, FALSE, &l16, &state, EC_TIMEOUTRXM);
    state &= 0x004f;
    if(( state != STATE_FAULT_ACTIVE) && ( state != STATE_FAULT))
        return;

    ec_SDOread( slaveNum, 0x6040, 0x00, FALSE, &l16, &ctrl, EC_TIMEOUTRXM);
    printf( "fd_reset [%d]: 1.ctrl=0x%04x\n", slaveNum, ctrl);

    if( ctrl & CTRL_RESET) {
//        cmd = ctrl & ~CTRL_RESET;
        cmd = 0x0000;
        ec_SDOwrite( slaveNum, 0x6040, 0x00, FALSE, l16, &cmd, EC_TIMEOUTRXM);
        ec_SDOread( slaveNum, 0x6040, 0x00, FALSE, &l16, &ctrl, EC_TIMEOUTRXM);
    }

    printf( "fd_reset [%d]: 2.ctrl=0x%04x\n", slaveNum, ctrl);

    i = 0;
//    cmd = ctrl | CTRL_RESET;
    cmd = CTRL_RESET;
    ec_SDOwrite( slaveNum, 0x6040, 0x00, FALSE, l16, &cmd, EC_TIMEOUTRXM);
    do {
        if( ++i > 50)
            return;
        ec_SDOread( slaveNum, 0x6041, 0x00, FALSE, &l16, &state, EC_TIMEOUTRXM);
        printf( "fd_reset [%d]: state=0x%04x\n", slaveNum, state);
        sleep( 1);
    } while(( state & 0x004f) & STATE_FAULT);

//    cmd = ctrl & ~CTRL_RESET;
    cmd = 0x0000;
    ec_SDOwrite( slaveNum, 0x6040, 0x00, FALSE, l16, &cmd, EC_TIMEOUTRXM);

    sleep( 1);

    ec_SDOread( slaveNum, 0x6040, 0x00, FALSE, &l16, &ctrl, EC_TIMEOUTRXM);
    ec_SDOread( slaveNum, 0x6041, 0x00, FALSE, &l16, &state, EC_TIMEOUTRXM);

    printf( "fd_reset [%d]: Done !!! ctrl=0x%04x, state=0x%04x\n", slaveNum, ctrl, state);

}

void EtherCATNetwork::enableVoltage(int sl)
{
	int slotControlWord = 0x6040;
	int slotStatusWord = 0x6041;
	int rdSize;

	uint16_t cw;
	cw = 6;
	ec_SDOwrite( sl, slotControlWord, 0, FALSE, sizeof(cw), &cw, EC_TIMEOUTRXM);

	int count = 10;
	while( count > 0 && (cw & 0x6F) != 0x21) {
		rdSize = sizeof( cw);
		ec_SDOread( sl, slotStatusWord, 0, FALSE, &rdSize, &cw, EC_TIMEOUTRXM);
		count--;
	}
	if ( (cw & 0x6F) != 0x21) return;

	cw = 0xF;
	ec_SDOwrite( sl, slotControlWord, 0, FALSE, sizeof(cw), &cw, EC_TIMEOUTRXM);
	count = 10;
	while( count > 0 && (cw & 0x6F) != 0x27) {
		rdSize = sizeof( cw);
		ec_SDOread( sl, slotStatusWord, 0, FALSE, &rdSize, &cw, EC_TIMEOUTRXM);
		count--;
	}
	if ( (cw & 0x6F) != 0x27) { printf("was not reached state\n");}
    else {
        Master2Slave &outVal( *reinterpret_cast<Master2Slave*>(
                    ec_slave[sl].outputs));
        outVal.controlword = cw;
    }

	return;
}

bool EtherCATNetwork::isSafeOperational() const 
{
    return ec_slave[0].state == EC_STATE_SAFE_OP;
}

void *EtherCATNetwork::CyclicThreadEntry( void * arg)
{
    EtherCATNetwork * self = reinterpret_cast<EtherCATNetwork *>(arg);
    return self->CyclicFunction();
}

struct timespec add_us(const struct timespec &b, const unsigned long usec)
{
    struct timespec rv { b};
    rv.tv_nsec += usec * 1000L;
    if (rv.tv_nsec > 999999999L) {
        ++rv.tv_sec;
        rv.tv_nsec -= 1000000000L;
    }
    return rv;
}

void * EtherCATNetwork::CyclicFunction()
{
    while(keepThreadRunning)
    {
        usleep(SYNC_PERIOD_MS * 1000);

        //copy latest required data to EtherCAT exchange region
        memcpy(IOmap, IOmapMirror, IOMAP_SIZE);
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        // copy just received data ArminHW latest data
        memcpy(IOmapMirror, IOmap, IOMAP_SIZE);

        notifyReadyCyclic.notify_all();
    }
    return nullptr;
}

void EtherCATNetwork::disableMotorASYNC(int slaveNum)
{
    if (slaveNum > ec_slavecount || slaveNum < 1)
        return;
    
	int slotControlWord = 0x6040;
	int slotStatusWord = 0x6041;
	int rdSize;

	uint16_t cw;
	cw = 0;
	ec_SDOwrite( slaveNum, slotControlWord, 0, FALSE,
            sizeof(cw), &cw, EC_TIMEOUTRXM);

	int count = 10;
	while( count > 0 && (cw & 0x6F) != 0x0) {
		rdSize = sizeof( cw);
		ec_SDOread( slaveNum, slotStatusWord, 0, FALSE,
            &rdSize, &cw, EC_TIMEOUTRXM);
		count--;
	}
	return;
}

int EtherCATNetwork::getNetState() const
{
    if (! started) return -1;
    return ec_slave[0].state;
}

void EtherCATNetwork::requestProfile(int slave, int mode)
{
    if (slave < 1 || slave > ec_slavecount) return;

    uint8_t rq = static_cast<uint8_t>(mode);
    int sz = sizeof(rq);
    ec_SDOwrite( slave, MODE_OF_OPERATION_SLOT, 0x00, FALSE, sz, &rq, EC_TIMEOUTRXM);

}

/*
template<typename r> inline void set_mask(r &val, const r &mask)
{
    val |= mask;
}

template<typename r> inline void clear_mask(r &val, const r &mask)
{
    val &= ~(mask);
}

using template<typename r> mask_oper = void  (*)(r &val, const r &mask);

template<typename r> void EtherCATNetwork::commonMask(int slave, int slot, int subslot, r mask, mask_oper op)
{
    if (slave < 1 || slave > ec_slavecount) return;
    auto cv = mask;
    int sz = sizeof(cv);
    ec_SDOread( slave, slot, subslot, FALSE, &sz, &cv, EC_TIMEOUTRXM);
    op(cv, mask);

    ec_SDOwrite( slave, slot, subslot, FALSE, sz, &cv, EC_TIMEOUTRXM);
    
}
*/

void EtherCATNetwork::setMaskUINT32(int slave, int slot, int subslot, uint32_t mask)
{
    //commonMask(slave, slot, subslot, mask, set_mask);

    if (slave < 1 || slave > ec_slavecount) return;
    auto cv = mask;
    int sz = sizeof(cv);
    ec_SDOread( slave, slot, subslot, FALSE, &sz, &cv, EC_TIMEOUTRXM);
    cv |= mask;
    ec_SDOwrite( slave, slot, subslot, FALSE, sz, &cv, EC_TIMEOUTRXM);
}

void EtherCATNetwork::clearMaskUINT32(int slave, int slot, int subslot, uint32_t mask)
{
    if (slave < 1 || slave > ec_slavecount) return;
    auto cv = mask;
    int sz = sizeof(cv);
    ec_SDOread( slave, slot, subslot, FALSE, &sz, &cv, EC_TIMEOUTRXM);
    cv &= ~mask;

    ec_SDOwrite( slave, slot, subslot, FALSE, sz, &cv, EC_TIMEOUTRXM);
}

void EtherCATNetwork::setHome(int slave)
{
    if (slave < 1 || slave > ec_slavecount) return;

    
}

void EtherCATNetwork::setSlotDataUINT32(int  slaveNum, int slotNum, int subslotNum, uint32_t newVal)
{
    if (slaveNum < 1 || slaveNum > ec_slavecount) return;
    int sz = sizeof(newVal);
    ec_SDOwrite( slaveNum, slotNum, subslotNum, FALSE, sz, &newVal, EC_TIMEOUTRXM);
}

void EtherCATNetwork::setSlotDataUINT8(int slaveNum, int slotNum, int subslotNum, uint8_t newVal)
{
    if (slaveNum < 1 || slaveNum > ec_slavecount) return;
    int sz = sizeof(newVal);
    ec_SDOwrite( slaveNum, slotNum, subslotNum, FALSE, sz, &newVal, EC_TIMEOUTRXM);
}

