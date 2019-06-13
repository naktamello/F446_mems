#include "utils.h"

uint32_t micros(void) {
    register uint32_t ms, cycle_cnt;
    do {
        ms = HAL_GetTick();
        cycle_cnt = TIMBASE->CNT;
    } while (ms != HAL_GetTick());

    return (ms * 1000) + cycle_cnt;
}

void delay_us(uint32_t us) {
    uint32_t start = micros();
    while (micros() - start < (uint32_t) us) {
        __ASM("nop");
    }
}

void serialize_int32(uint32_t integer, uint8_t *bytearray) {
    bytearray[0] = (uint8_t) (integer >> 24);
    bytearray[1] = (uint8_t) (integer >> 16);
    bytearray[2] = (uint8_t) (integer >> 8);
    bytearray[3] = (uint8_t) integer;
}