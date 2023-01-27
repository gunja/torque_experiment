#pragma once
#ifndef ETHERCAT_NETWORK_H
#define ETHERCAT_NETWORK_H
#include "exchange_regions.h"

#include <stdint.h>
#include <pthread.h>

#include <mutex>
#include <condition_variable>

#define EC_NO_ERROR 0
#define EC_FAILED_IFACE_BRING_UP 1
#define EC_FAILED_DISCOVER_SLAVES 2

#define IOMAP_SIZE 4096

extern const char *errorTexts[];
class EtherCATNetwork
{
        int slavesFound;
        bool started;
        bool operational;
        int errorCode;
        char IOmap[IOMAP_SIZE];
        char IOmapMirror[IOMAP_SIZE];
        int usedmem;

        pthread_t cyclic_exchange_thread;
        bool keepThreadRunning;
        //std::mutex exchange_Lock;
        std::condition_variable notifyChanges;
        std::mutex dataReadyMutex;
        std::condition_variable notifyReadyCyclic;

#ifdef SET_PDO_IN_ECN
        void setPDOs();
#endif
#define SET_1C1x_IN_ECN
#ifdef SET_1C1x_IN_ECN
        void set1C1x();
#endif
        void resetErrors();
        void * CyclicFunction();

    public:
        EtherCATNetwork();
        ~EtherCATNetwork();
        bool startNetwork(const char*);
        bool networkRunning() const { return started;};
        bool hasError() const { return errorCode != 0; };
        const char * getLastErrorText() const { return errorTexts[errorCode];};
        int getDiscoveredSlavesCount() const { return slavesFound; };
        int getECKnownSlavesCount() const;
        const char * getSlaveName(int i);
        bool SwitchOperational();
        bool SwitchSafeOperational();
        bool switchOP_SO();
        uint32_t getSlotDataUINT32(int slaveNum, int slotNum, int subslotNum);
        uint16_t getSlotDataUINT16(int slaveNum, int slotNum, int subslotNum);
        uint8_t getSlotDataUINT8(int slaveNum, int slotNum, int subslotNum);
        uint16_t getSlaveState(int s) const;
        bool isOperational() const { return operational;};
        bool isSafeOperational() const;
        bool fillExchangeRegions(int slave, const Master2Slave &, Slave2Master &, bool = true);
        void *getOutputPointer(int slaveNum);
        void* getInputPointer(int slaveNum);
        void clearErrorState(int slaveNum);
        void enableVoltage(int);

        static const char * ecStateMachineAsText(uint16_t);
        static void *CyclicThreadEntry( void *);
        void disableMotorASYNC(int slNum);
        int getNetState() const;
        void requestProfile(int slave, int mode);
        void setMaskUINT32( int slave, int slot, int subslot, uint32_t mask);
        void clearMaskUINT32( int slave, int slot, int subslot, uint32_t mask);
        void setHome(int slave);

        void setSlotDataUINT32(int slaveNum, int slotNum, int subslotNum, uint32_t newVal);
        void setSlotDataUINT8(int slaveNum, int slotNum, int subslotNum, uint8_t newVal);
        //std::mutex & getGuard() { return exchange_Lock;};
        std::condition_variable & getCondVar() { return notifyChanges;};
        std::mutex & getReadyMutex() { return dataReadyMutex;};
        std::condition_variable & getReadyDataCond() { return notifyReadyCyclic; };
};


#endif // ETHERCAT_NETWORK_H
