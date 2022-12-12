#include <iostream>
#include <ios>
#include <list>

#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>

#include <ethercat.h>

void configurePDO(int SlaveNum, uint16_t coveringSlot, uint16_t PDOslot, std::list<uint32_t> pdo_maps, bool read180x_2 = false ){

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

    std::cout<<"\t\tgetting "<<coveringSlot<<".0";
    uint8_t zero_pos;
    sz = sizeof(zero_pos);
    rv = ec_SDOread(SlaveNum, coveringSlot, 0, FALSE, &sz, &zero_pos, EC_TIMEOUTRXM);
    std::cout<<" rzlt as "<<rv<<"with decimal value = "<<static_cast<int>(zero_pos)<<std::endl;


if(read180x_2) {
        uint8_t trans_type;
        sz = sizeof(trans_type);
        rv = ec_SDOread(SlaveNum, coveringSlot, 2, FALSE, &sz, &trans_type, EC_TIMEOUTRXM);
        std::cout<<"\t\tgetting "<<std::hex<<coveringSlot<<std::dec<<".2 -> "<<static_cast<int>(trans_type)<<"   at rez ="<<rv<<std::endl;
    }

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

void readTPDO(int slaveNum, uint16_t slot)
{
    std::cout<<"Reading "<<slot<<"   on "<<slaveNum<<std::endl;

    uint8_t field0;
    uint32_t field1;
    uint32_t field2;
    int sz;
    int rv;

    sz = sizeof(field0);
    rv = ec_SDOread(slaveNum, slot, 0, FALSE, &sz, &field0, EC_TIMEOUTRXM);
    std::cout<<"\t\t"<<slot<<".0 = "<<static_cast<int>(field0)<<" at res ="<<rv<<std::endl;
     
    sz = sizeof(field1);
    rv = ec_SDOread(slaveNum, slot, 1, FALSE, &sz, &field1, EC_TIMEOUTRXM);
    std::cout<<"\t\t"<<slot<<".1 = "<<std::hex<<field1<<std::dec<<" at res ="<<rv<<std::endl;
     
    sz = sizeof(field2);
    rv = ec_SDOread(slaveNum, slot, 2, FALSE, &sz, &field2, EC_TIMEOUTRXM);
    std::cout<<"\t\t"<<slot<<".2 = "<<std::hex<<field2<<std::dec<<" at res ="<<rv<<std::endl;
     
}

void read1Cxx(int slaveNum, uint16_t slot_index)
{
    uint8_t index0;
    uint8_t nonzeroIndex;
    int sz, rv;
    uint16_t nonXeroIndex;

    uint16_t full[5];

    sz = sizeof(index0);

    rv = ec_SDOread(slaveNum, slot_index, 0, FALSE, &sz, &index0, EC_TIMEOUTRXM);
    std::cout<<"slave "<<slaveNum<<"  U8  at index "<<std::hex<<slot_index<<std::dec<<".0 = "
            <<std::hex<<static_cast<int>(index0)<<std::dec<<"  with rv ="<<rv<<std::endl;
    sz = sizeof(nonXeroIndex);
    rv = ec_SDOread(slaveNum, slot_index, 0, FALSE, &sz, &nonXeroIndex, EC_TIMEOUTRXM);
    std::cout<<"slave "<<slaveNum<<"  U16   at index "<<std::hex<<slot_index<<std::dec<<".0 = "
            <<std::hex<<static_cast<int>(nonXeroIndex)<<std::dec<<"  with rv ="<<rv<<std::endl;

    std::cout<<"Reading all the data"<<std::endl;
    sz = sizeof(full);
    rv = ec_SDOread(slaveNum, slot_index, 0, TRUE, &sz, &full[0], EC_TIMEOUTRXM);
    std::cout<<"slave "<<slaveNum<<"  at index "<<std::hex<<slot_index<<std::dec<<". = "<<std::endl;
        std::cout<<"\t0->"<<std::hex<<static_cast<int>(full[0])<<std::dec<<"  with rv ="<<rv<<std::endl;
        std::cout<<"\t1->"<<std::hex<<static_cast<int>(full[1])<<std::dec<<"  with rv ="<<rv<<std::endl;
        std::cout<<"\t2->"<<std::hex<<static_cast<int>(full[2])<<std::dec<<"  with rv ="<<rv<<std::endl;
        std::cout<<"\t3->"<<std::hex<<static_cast<int>(full[3])<<std::dec<<"  with rv ="<<rv<<std::endl;
        std::cout<<"\t4->"<<std::hex<<static_cast<int>(full[4])<<std::dec<<"  with rv ="<<rv<<std::endl;


  for(int i = 1; i <=4; ++i)
  {
    sz = sizeof(nonXeroIndex);
    rv = ec_SDOread(slaveNum, slot_index, i, FALSE, &sz, &nonXeroIndex, EC_TIMEOUTRXM);
    std::cout<<"slave "<<slaveNum<<"  U16   at index "<<std::hex<<slot_index<<std::dec<<"."<<i<<" = "
            <<std::hex<<static_cast<int>(nonXeroIndex)<<std::dec<<"  with rv ="<<rv<<std::endl;
  }


    std::cout<<"setting to 4 at .0"<<std::endl;
    
    sz = sizeof(index0);
    index0 = 4;

    rv = ec_SDOwrite(slaveNum, slot_index, 0, FALSE, sz, &index0, EC_TIMEOUTRXM);
    std::cout<<"slave "<<slaveNum<<"  U8  at index "<<std::hex<<slot_index<<std::dec<<"  with rv ="<<rv<<std::endl;
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

    for( int s = 1; s <= ec_slavecount; ++s) {
        std::cout<<"Reading out  slave "<<s<<std::endl;

            readTPDO(s, 0x2014);
            readTPDO(s, 0x2015);
            readTPDO(s, 0x2016);
            readTPDO(s, 0x2017);

            read1Cxx(s, 0x1c12);
            read1Cxx(s, 0x1c13);

    }


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
    configurePDO(s, 0x1403, 0x1603, {  0x60710010 } );


    configurePDO(s, 0x1800, 0x1A00, { 0x60610008, 0x60410010, 0x60640020, 0x10010008 }, true );
    configurePDO(s, 0x1801, 0x1A01, { 0x606C0020, 0x60780010, 0x200F0010 }, true );
    configurePDO(s, 0x1802, 0x1A02, { 0x60FD0020 }, true );
    configurePDO(s, 0x1803, 0x1A03, { 0x60770010 }, true );

        t8 = 4;
        std::cout<<"\tSetting 1C12.0 to "<<(int)t8<<" -> "<<ec_SDOwrite(s, 0x1C12, 0, FALSE, sizeof(t8), &t8, EC_TIMEOUTRXM)<<std::endl;

        t8 = 4;
        std::cout<<"\tSetting 1C13.0 to "<<(int)t8<<" -> "<<ec_SDOwrite(s, 0x1C13, 0, FALSE, sizeof(t8), &t8, EC_TIMEOUTRXM)<<std::endl;

        t32 = 0x65766173;
        std::cout<<"Calling save for settings ->"<<ec_SDOwrite(s, 0x1010, 1, FALSE, sizeof(t32), &t32, EC_TIMEOUTRXM) <<std::endl;
    }
}

