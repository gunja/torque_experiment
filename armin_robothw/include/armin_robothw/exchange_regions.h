#pragma once
#ifndef EXCHANGE_REGS_H
#define EXCHANGE_REGS_H
#include <stdint.h>

#pragma pack(push,1)

#define MODE_OF_OPERATION_SLOT 0x6060
#define CONTROL_WORD_SLOT 0x6040

typedef struct _master_slave_exchange {
    // slaves R-PDO 1
    int8_t modes_of_operation; // 0x60600008 Value Range:1, 3, 4, 6, 7
    uint16_t controlword; // 0x60400010 bits 9-15 should be set to 0
    int32_t target_velocity; // 0x60FF0020 Units: speed units
    int8_t homing_method;   //60980008  homing method
    // slaves R-PDO 2
    uint32_t end_velocity; // 0x60820020 end_velocity
    int32_t target_position;    //0x607A0020 target position; units: position units
    // slaves R-PDO 3
    uint32_t profile_velocity; // 0x60810020  units: speed units
    int32_t home_offset;  //0x607C0020
    //slaves R-PDO 4
    int16_t target_torque; // 0x6071:0x00 0x10    target_torque 
} Master2Slave;

typedef struct _slave_master_exchange {
    int8_t modes_of_operation_display; // 0x60610008 Value range: -1, -11, -13, -14, 1, 3, 4, 6, 7
    uint16_t statusword; // 0x60410010 - to be analyzed
    int32_t position_actual_value; // 0x60640020  in position units
    uint8_t error_register; // 0x10010008 TBC
    // slave T-PDO 2
    int32_t velocity_actual_value; // 0x606C0020  speed units
    int16_t current_actual_value;   //0x60780010   motor_rated_current (0x6075, uint32) / 1000
    uint16_t last_warning_code; // 0x200f0010
    // slave T-PDO 3
    uint32_t digital_inputs;    //0x60fd0020
    uint32_t padding;
    // slave T-PDO 4
    int16_t torque_actual_value; // 0x6077:0x00 0x10
    
} Slave2Master;

typedef struct _ekxx_master_slave_exchange {
    uint8_t out[2];
} Ekxx_Region_m2s;
typedef struct _ekxx_slave_master_exchange {
    uint8_t out[1];
} Ekxx_Region_s2m;
typedef struct _elxx_master_slave_exchange {
    uint8_t out[2];
} Elxx_Region_m2s;
typedef struct _elxx_slave_master_exchange {
    uint8_t out[8];
} Elxx_Region_s2m;
typedef struct _el41xx_master_slave_exchange {
    int16_t out_ch1;
    int16_t out_ch2;
} El41xx_Region_m2s;
typedef struct _el41xx_slave_master_exchange {
} El41xx_Region_s2m;
#pragma pack(pop)

#endif // EXCHANGE_REGS_H
