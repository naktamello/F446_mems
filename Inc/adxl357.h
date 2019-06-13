#ifndef F446_MEMS_ADXL357_H
#define F446_MEMS_ADXL357_H

#include <stdbool.h>
#include <memory.h>
#include <stm32f4xx_hal.h>

#define ADXL357_TX_LEN              10

#define ADXL357_VAL_PWRUP            0x00

#define ADXL357_RANGE_10G            0x81
#define ADXL357_RANGE_20G            0x82
#define ADXL357_RANGE_40G            0x83

#define ADXL357_VAL_Z_OFFSET_RAW     35475
#define ADXL357_VAL_Z_OFFSET_H       0x08
#define ADXL357_VAL_Z_OFFSET_L       0xA9

#define ADXL357_REG_DEVID_AD         0x00
#define ADXL357_REG_DEVID_MSG        0x01
#define ADXL357_REG_PARTID           0x02
#define ADXL357_REG_REVID            0x03
#define ADXL357_REG_STATUS           0x04
#define ADXL357_REG_FIFO_ENTRIES     0x05
#define ADXL357_REG_TEMP2            0x06
#define ADXL357_REG_TEMP1            0x07
#define ADXL357_REG_XDATA3           0x08
#define ADXL357_REG_XDATA2           0x09
#define ADXL357_REG_XDATA1           0x0A
#define ADXL357_REG_YDATA3           0x0B
#define ADXL357_REG_YDATA2           0x0C
#define ADXL357_REG_YDATA1           0x0D
#define ADXL357_REG_ZDATA3           0x0E
#define ADXL357_REG_ZDATA2           0x0F
#define ADXL357_REG_ZDATA1           0x10
#define ADXL357_REG_FIFO_DATA        0x11
#define ADXL357_REG_OFFSET_X_H       0x1E
#define ADXL357_REG_OFFSET_X_L       0x1F
#define ADXL357_REG_OFFSET_Y_H       0x20
#define ADXL357_REG_OFFSET_Y_L       0x21
#define ADXL357_REG_OFFSET_Z_H       0x22
#define ADXL357_REG_OFFSET_Z_L       0x23
#define ADXL357_REG_ACT_EN           0x24
#define ADXL357_REG_ACT_TRHESH_H     0x25
#define ADXL357_REG_ACT_THRESH_L     0x26
#define ADXL357_REG_ACT_COUNT        0x27
#define ADXL357_REG_FILTER           0x28
#define ADXL357_REG_FIFO_SAMPLES     0x29
#define ADXL357_REG_INT_MAP          0x2A
#define ADXL357_REG_SYNC             0x2B
#define ADXL357_REG_RANGE            0x2C
#define ADXL357_REG_POWER_CTL        0x2D
#define ADXL357_REG_SELF_TEST        0x2E
#define ADXL357_REG_RESET            0x2F

#endif //F446_MEMS_ADXL357_H

enum ADXL357_Range{
    range10g = ADXL357_RANGE_10G,
    range20g = ADXL357_RANGE_20G,
    range40g = ADXL357_RANGE_40G
};

void ADXL357_Init(SPI_HandleTypeDef* pspi, GPIO_TypeDef* spi_nss_port, uint16_t spi_nss_pin);
void ADXL357_PowerUp();
void ADXL357_AccData(uint8_t* rxbuf);
void ADXL357_SetRange(enum ADXL357_Range range);