#ifndef _SIGNAL_GENERATOR_H_
#define _SIGNAL_GENERATOR_H_
#include <stdint.h>

#pragma pack(1)
typedef struct SIGNAL_GENERATOR_t {
    uint8_t mode;
    uint16_t Signal_Cnt;
    uint8_t Error_Analysis;
    float Error_Sum;
} signal_generator;
#pragma pack()

float Signal_Generator(signal_generator* obj);


#endif