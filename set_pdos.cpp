#include <iostream>
#include <ios>
#include <list>

#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>

#include <ethercat.h>

void configurePDO(int SlaveNum, uint16_t coveringSlot, uint16_t PDOslot, std::list<uint32_t> pdo_maps ){

        std::cout<<"Getting "<<std::hex<<coveringSlot<<".1";
	uint32_t v1800_1;
    int sz = sizeof(v1800_1);
	int rv = ec_SDOread(SlaveNum, coveringSlot, 1, FALSE, &sz, &v1800_1, EC_TIMEOUTRXM);
	std::cout<<" resulted in   "<<rv<<"  with altered sz="<<sz<<"   and value = "<<
		        std::hex<<v1800_1<<std::dec<<std::endl;
    if ( v1800_1 & (1<<31) ) {
		    std::cout<<"bit 31 is set"<<std::endl;
    }
	    v1800_1 |= (1<<31); sz = sizeof(v1800_1);
	    rv = ec_SDOwrite( SlaveNum, coveringSlot, 1, FALSE, sz, &v1800_1, EC_TIMEOUTRXM);
	  std::cout<<"  wrote "<<std::hex<<coveringSlot<<std::dec<<".1  with "<<rv<<std::endl;
	   sz = sizeof(v1800_1);
	   rv = ec_SDOread(SlaveNum, coveringSlot, 1, FALSE, &sz, &v1800_1, EC_TIMEOUTRXM);
	   std::cout<<" resulted in   "<<rv<<"  with altered sz="<<sz<<"   and value = "<<
		    std::hex<<v1800_1<<std::dec<<std::endl;
	
        int8_t t8 = 0;
            std::cout<<"\tSetting "<<std::hex<<PDOslot<<std::dec<<".00 to 0 ->"
             <<ec_SDOwrite(SlaveNum, PDOslot, 0, FALSE, sizeof(t8), &t8, EC_TIMEOUTRXM)<<std::endl;
        int c= 1;
        for(auto const &i: pdo_maps) {
            uint32_t t32 = i;
            std::cout<<"\tSetting "<<std::hex<<PDOslot<<std::dec<<"."<<c<<" to "
                <<std::hex<<t32<<std::dec<<"  results in "
                    <<ec_SDOwrite(SlaveNum, PDOslot, c, FALSE, sizeof(t32), &t32, EC_TIMEOUTRXM)<<std::endl;
            ++c;
        }
        --c;
            t8 = static_cast<uint8_t>(c);
            std::cout<<"\tSetting "<<std::hex<<PDOslot<<std::dec<<".0 to "<<(int)t8<<" ->"<<ec_SDOwrite(SlaveNum, PDOslot, 0, FALSE, sizeof(t8), &t8, EC_TIMEOUTRXM)<<std::endl;
    	v1800_1 &= ~(1<<31); sz = sizeof(v1800_1);
	    rv = ec_SDOwrite( SlaveNum, coveringSlot, 1, FALSE, sz, &v1800_1, EC_TIMEOUTRXM);
}

int main(int argc, char * argv[])
{
    char buff[200];
    uint8_t t8;
    uint16_t t16;
    uint32_t t32;

    if (argc < 2) {
        std::cerr<<"Required second argument with interface name"<<std::endl;
        exit(EXIT_FAILURE);
    }
    if (getuid() != 0) {
        std::cerr<<"Should be executed as root or with X-flags enabled"<<std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout<<"Starting EtherCAT"<<std::endl;

    int rv = ec_init(argv[1]);
    if(rv <= 0) {
        std::cerr<<"Failure on calling ec_init. Exiting"<<std::endl;
        exit(EXIT_FAILURE);
    }
    int slavesFound = ec_config_init(FALSE);
    std::cout<<"Discovered "<<slavesFound << " slaves on the network"<<std::endl;
    std::cout<<"Press any key to start writing settings to each slave"<<std::endl;
    std::cin>>buff;

    for( int s = 1; s <= ec_slavecount; ++s) {
        std::cout<<"Working on slave "<<s<<std::endl;
        int sz = sizeof( t8);

        t8 = 0;
        std::cout<<"\tSetting 1C12.0 to "<<(int)t8<<" -> "<<ec_SDOwrite(s, 0x1C12, 0, FALSE, sizeof(t8), &t8, EC_TIMEOUTRXM)<<std::endl;
        t8 = 0;
        std::cout<<"\tSetting 1C13.0 to "<<(int)t8<<" -> "<<ec_SDOwrite(s, 0x1C13, 0, FALSE, sizeof(t8), &t8, EC_TIMEOUTRXM)<<std::endl;

    configurePDO(s, 0x1400, 0x1600, { 0x60600008, 0x60400010, 0x60FF0020, 0x60980008 } );
    configurePDO(s, 0x1401, 0x1601, { 0x60820020, 0x607A0020 } );
    configurePDO(s, 0x1402, 0x1602, { 0x60810020, 0x607C0020 } );
    configurePDO(s, 0x1403, 0x1603, {  } );


    configurePDO(s, 0x1800, 0x1A00, { 0x60610008, 0x60410010, 0x60640020, 0x10010008 } );
    configurePDO(s, 0x1801, 0x1A01, { 0x606C0020, 0x60780010, 0x200F0010 } );
    configurePDO(s, 0x1802, 0x1A02, { 0x60FD0020 } );

        t8 = 3;
        std::cout<<"\tSetting 1C12.0 to "<<(int)t8<<" -> "<<ec_SDOwrite(s, 0x1C12, 0, FALSE, sizeof(t8), &t8, EC_TIMEOUTRXM)<<std::endl;

        t8 = 3;
        std::cout<<"\tSetting 1C13.0 to "<<(int)t8<<" -> "<<ec_SDOwrite(s, 0x1C13, 0, FALSE, sizeof(t8), &t8, EC_TIMEOUTRXM)<<std::endl;

        t32 = 0x65766173;
        std::cout<<"Calling save for settings ->"<<ec_SDOwrite(s, 0x1010, 1, FALSE, sizeof(t32), &t32, EC_TIMEOUTRXM) <<std::endl;
    }
}

