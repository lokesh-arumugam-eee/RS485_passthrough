#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_spiffs.h"
#include "cJSON.h"
#include "configuration/configuration.h"
#include "modbus_rtu.h"
#include "xmodem.h"


void apply_endianness(uint16_t *src_regs, uint8_t *dest, int bytes, int type) {

    uint8_t A=0,B=0,C=0,D=0;

    A = (uint8_t)(src_regs[0] >> 8) & 0xFF;
    B = (uint8_t)src_regs[0] & 0xFF;
    if (bytes == 4) {
        C = (uint8_t)(src_regs[1] >> 8) & 0xFF;
        D = (uint8_t)src_regs[1] & 0xFF;
    }

    switch (type) {
        case 0: // Big Endian (ABCD)
        	dest[0] = A;dest[1] = B;
        	if (bytes > 2) {dest[2] = C;dest[3] = D;}
            break;

        case 1: // Little Endian (DCBA)
        	if (bytes > 2) {dest[0] = D;dest[1] = C;dest[2] = B;dest[3] = A;}
            break;
        case 2: //BADC
        	if (bytes > 2) {dest[0] = B;dest[1] = A;dest[2] = D;dest[3] = C;}
        	break;
        case 3: // Word Swap (CDAB)
        	if (bytes > 2) {dest[0] = C;dest[1] = D;dest[2] = A;dest[3] = B;}
            break;
    }
}

float convert_to_value(uint8_t *buffer, profile_config_t *p) {

    uint16_t Val_u16t =0;
    int16_t Val_i16t =0;
     
    printf("case:%d\n",p->dta_uplnk_type);
    union {
        uint32_t u32;
        float f;
    } data_converter;
    
    float final_val = 0;
    
    switch (p->dta_uplnk_type) {
        case 0:  //FLoat
        case 1:
        case 2:
        // Combine bytes into a 32-bit integer (Big Endian order)
            data_converter.u32 = ((uint32_t)buffer[0] << 24) |
                                 ((uint32_t)buffer[1] << 16) |
                                 ((uint32_t)buffer[2] << 8)  |
                                 ((uint32_t)buffer[3]);
            final_val = data_converter.f;
            break;
        case 3: //Unsigned Integer
            Val_u16t = ((uint16_t)buffer[0] << 8) |((uint32_t)buffer[1]);
            final_val = (float)Val_u16t; 
            break;
        case 4: // signed Int 16-bit
            Val_i16t = ((uint16_t)buffer[0] << 8) |((uint32_t)buffer[1]);
            final_val = (float)Val_i16t; 
            break;
        case 5:
        case 6:  
             final_val = (float)buffer[0]; break;
        
        case 7: { // Bitwise
            uint32_t raw = (p->bytes == 4) ? *(uint32_t*)buffer : *(uint16_t*)buffer;
            final_val = (raw >> p->start_bit) & ((1 << p->bit_cnt) - 1);
            break;
        }
        case 17: { // TRINITYPOWFACTOR
            uint16_t raw = *(uint16_t*)buffer;
            final_val = (raw > 1000) ? (raw - 1000) * -1 : raw;
            break;
        }
        case 22: // HVAC FCU mapping
            final_val = -10.0 + ((float)(*(uint16_t*)buffer) / 120.0) * 70.0; break;
        default: final_val = *(uint16_t*)buffer; break;
    }
    return (float)(final_val * p->multiplier);
}

