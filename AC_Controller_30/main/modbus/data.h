#ifndef DATA_H
#define DATA_H

#include "configuration/configuration.h"

float convert_to_value(uint8_t *buffer, profile_config_t *p);
void apply_endianness(uint16_t *src_regs, uint8_t *dest, int bytes, int type);

#endif
