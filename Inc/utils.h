#ifndef F446_MEMS_UTILS_H
#define F446_MEMS_UTILS_H

#include <stdint.h>
#include <stm32f4xx_hal.h>

#define TIMBASE TIM14

uint32_t micros(void);
void delay_us(uint32_t us);
void serialize_int32(uint32_t integer, uint8_t *bytearray);

#endif //F446_MEMS_UTILS_H
