#include "adxl357.h"

static SPI_HandleTypeDef *pspi_;
static GPIO_TypeDef *spi_nss_port_;
static uint16_t spi_nss_pin_;
static uint8_t txbuf_[ADXL357_TX_LEN];
static uint8_t dummy_[ADXL357_TX_LEN];
static bool read_ready_ = false;

static void ADXL357_SPI_TransmitReceive(uint8_t *rxbuf, uint16_t len){
    HAL_GPIO_WritePin(spi_nss_port_, spi_nss_pin_, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(pspi_, txbuf_, rxbuf, len, 100);
    HAL_GPIO_WritePin(spi_nss_port_, spi_nss_pin_, GPIO_PIN_SET);
}

static uint8_t ADXL357_ReadRegister(uint8_t reg){
    return (uint8_t ) ((reg << 1) | 0x01);
}

static uint8_t ADXL357_WriteRegister(uint8_t reg){
    return reg << 1;
}

void ADXL357_Init(SPI_HandleTypeDef *pspi, GPIO_TypeDef *spi_nss_port, uint16_t spi_nss_pin) {
    pspi_ = pspi;
    spi_nss_port_ = spi_nss_port;
    spi_nss_pin_ = spi_nss_pin;
}

void ADXL357_SetRange(enum ADXL357_Range range){
    txbuf_[0] = ADXL357_WriteRegister(ADXL357_REG_RANGE);
    txbuf_[1] = (uint8_t) range;
    ADXL357_SPI_TransmitReceive(dummy_, 2);
}

void ADXL357_PowerUp() {
    txbuf_[0] = ADXL357_WriteRegister(ADXL357_REG_POWER_CTL);
    txbuf_[1] = ADXL357_VAL_PWRUP;
    ADXL357_SPI_TransmitReceive(dummy_, 2);
}

void ADXL357_AccData(uint8_t *rxbuf){
    if (!read_ready_){
        memset(txbuf_, 0xFF, ADXL357_TX_LEN);
        txbuf_[0] = ADXL357_ReadRegister(ADXL357_REG_XDATA3);
    }
    ADXL357_SPI_TransmitReceive(rxbuf, 10);
}