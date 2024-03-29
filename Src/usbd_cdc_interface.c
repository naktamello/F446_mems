/**
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Src/usbd_cdc_interface.c
  * @author  MCD Application Team
  * @brief   Source file for USBD CDC interface
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBD_CDC_LineCodingTypeDef line_coding =
        {
                921600, /* baud rate*/
                0x00,   /* stop bits-1*/
                0x00,   /* parity - none*/
                0x08    /* nb. of bits 8*/
        };

uint8_t usb_rx_buf[APP_RX_DATA_SIZE];/* Received Data over USB are stored in this buffer */
uint8_t usb_tx_buf[APP_TX_DATA_SIZE];/* Received Data over UART (CDC interface) are stored in this buffer */
uint32_t usb_tx_ptr_in = 0;/* Increment this pointer or roll it back to
                               start address when data are received over USART */
uint32_t usb_tx_ptr_out = 0; /* Increment this pointer or roll it back to
                                 start address when data are sent over USB */
uint32_t usb_tx_ptr_tail = APP_TX_DATA_SIZE;

/* UART handler declaration */
UART_HandleTypeDef huart_usb;
/* TIM handler declaration */
TIM_HandleTypeDef htim_usb;
/* USB handler declaration */
extern USBD_HandleTypeDef husbd;

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_Itf_Init(void);

static int8_t CDC_Itf_DeInit(void);

static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length);

static int8_t CDC_Itf_Receive(uint8_t *pbuf, uint32_t *Len);

static void Set_LineCoding(const uint8_t *pbuf);

static void Get_LineCoding(uint8_t *pbuf);

static void Error_Handler(void);

static void ComPort_Config(void);

void USB_TIM_Config(void);

USBD_CDC_ItfTypeDef USBD_CDC_fops =
        {
                CDC_Itf_Init,
                CDC_Itf_DeInit,
                CDC_Itf_Control,
                CDC_Itf_Receive
        };

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CDC_Itf_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Init(void) {
    /*##-1- Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* USART configured as follow:
        - Word Length = 8 Bits
        - Stop Bit    = One Stop bit
        - Parity      = No parity
        - BaudRate    = 115200 baud
        - Hardware flow control disabled (RTS and CTS signals) */
    huart_usb.Instance = USARTx;
    huart_usb.Init.BaudRate = 115200;
    huart_usb.Init.WordLength = UART_WORDLENGTH_8B;
    huart_usb.Init.StopBits = UART_STOPBITS_1;
    huart_usb.Init.Parity = UART_PARITY_NONE;
    huart_usb.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart_usb.Init.Mode = UART_MODE_TX_RX;
    huart_usb.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart_usb) != HAL_OK) {
        /* Initialization Error */
        Error_Handler();
    }

    /*##-2- Put UART peripheral in IT reception process ########################*/
    /* Any data received will be stored in "usb_tx_buf" buffer  */
    if (HAL_UART_Receive_IT(&huart_usb, (uint8_t *) usb_tx_buf, 1) != HAL_OK) {
        /* Transfer error in reception process */
        Error_Handler();
    }

    /*##-3- Configure the TIM Base generation  #################################*/
    USB_TIM_Config();

    /*##-4- Start the TIM Base generation in interrupt mode ####################*/
    /* Start Channel1 */


    /*##-5- Set Application Buffers ############################################*/
    USBD_CDC_SetTxBuffer(&husbd, usb_tx_buf, 0);
    USBD_CDC_SetRxBuffer(&husbd, usb_rx_buf);

    return (USBD_OK);
}

/**
  * @brief  CDC_Itf_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_DeInit(void) {
    /* DeInitialize the UART peripheral */
    if (HAL_UART_DeInit(&huart_usb) != HAL_OK) {
        /* Initialization Error */
        Error_Handler();
    }
    return (USBD_OK);
}

/**
  * @brief  CDC_Itf_Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code            
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length) {
    switch (cmd) {
        case CDC_SEND_ENCAPSULATED_COMMAND:
            /* Add your code here */
            break;

        case CDC_GET_ENCAPSULATED_RESPONSE:
            /* Add your code here */
            break;

        case CDC_SET_COMM_FEATURE:
            /* Add your code here */
            break;

        case CDC_GET_COMM_FEATURE:
            /* Add your code here */
            break;

        case CDC_CLEAR_COMM_FEATURE:
            /* Add your code here */
            break;

        case CDC_SET_LINE_CODING:
            Set_LineCoding(pbuf);
            break;

        case CDC_GET_LINE_CODING:
            Get_LineCoding(pbuf);
            break;

        case CDC_SET_CONTROL_LINE_STATE:
            /* Add your code here */
            break;

        case CDC_SEND_BREAK:
            /* Add your code here */
            break;

        default:
            break;
    }

    return (USBD_OK);
}

/**
  * @brief  TIM period elapsed callback
  * @param  htim: TIM handle
  * @retval None
  */
void USB_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    uint32_t buffptr;
    uint32_t buffsize;

    if (husbd.pClassData != NULL) {
        if (usb_tx_ptr_out != usb_tx_ptr_in) {
            if (usb_tx_ptr_out > usb_tx_ptr_in) /* Rollback */
            {
                buffsize = usb_tx_ptr_tail - usb_tx_ptr_out;
            } else {
                buffsize = usb_tx_ptr_in - usb_tx_ptr_out;
            }

            buffptr = usb_tx_ptr_out;

            USBD_CDC_SetTxBuffer(&husbd, &usb_tx_buf[buffptr], (uint16_t) buffsize);

            if (USBD_CDC_TransmitPacket(&husbd) == USBD_OK) {
                usb_tx_ptr_out += buffsize;
                if (usb_tx_ptr_out >= usb_tx_ptr_tail) {
                    usb_tx_ptr_out = 0;
                }
            }
        }
    }
}

/**
  * @brief  Rx Transfer completed callback
  * @param  huart: UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    /* Increment Index for buffer writing */
    usb_tx_ptr_in++;

    /* To avoid buffer overflow */
    if (usb_tx_ptr_in == APP_RX_DATA_SIZE) {
        usb_tx_ptr_in = 0;
    }

    /* Start another reception: provide the buffer pointer with offset and the buffer size */
    HAL_UART_Receive_IT(huart, (uint8_t *) (usb_tx_buf + usb_tx_ptr_in), 1);
}

/**
  * @brief  CDC_Itf_DataRx
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  * @param  Buf: Buffer of data to be transmitted
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Receive(uint8_t *Buf, uint32_t *Len) {
    HAL_UART_Transmit_DMA(&huart_usb, Buf, (uint16_t) *Len);
    return (USBD_OK);
}


static void Set_LineCoding(const uint8_t *pbuf) {
    line_coding.bitrate = (uint32_t) (pbuf[0] | (pbuf[1] << 8) | \
                            (pbuf[2] << 16) | (pbuf[3] << 24));
    line_coding.format = pbuf[4];
    line_coding.paritytype = pbuf[5];
    line_coding.datatype = pbuf[6];

//    ComPort_Config();
}

static void Get_LineCoding(uint8_t *pbuf) {
    pbuf[0] = (uint8_t) (line_coding.bitrate);
    pbuf[1] = (uint8_t) (line_coding.bitrate >> 8);
    pbuf[2] = (uint8_t) (line_coding.bitrate >> 16);
    pbuf[3] = (uint8_t) (line_coding.bitrate >> 24);
    pbuf[4] = line_coding.format;
    pbuf[5] = line_coding.paritytype;
    pbuf[6] = line_coding.datatype;
}

/**
  * @brief  Tx Transfer completed callback
  * @param  huart: UART handle
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    /* Initiate next USB packet transfer once UART completes transfer (transmitting data over Tx line) */
    USBD_CDC_ReceivePacket(&husbd);
}

/**
  * @brief  ComPort_Config
  *         Configure the COM Port with the parameters received from host.
  * @param  None.
  * @retval None
  * @note   When a configuration is not supported, a default value is used.
  */
static void ComPort_Config(void) {
    if (HAL_UART_DeInit(&huart_usb) != HAL_OK) {
        /* Initialization Error */
        Error_Handler();
    }

    /* set the Stop bit */
    switch (line_coding.format) {
        case 0:
            huart_usb.Init.StopBits = UART_STOPBITS_1;
            break;
        case 2:
            huart_usb.Init.StopBits = UART_STOPBITS_2;
            break;
        default :
            huart_usb.Init.StopBits = UART_STOPBITS_1;
            break;
    }

    /* set the parity bit*/
    switch (line_coding.paritytype) {
        case 0:
            huart_usb.Init.Parity = UART_PARITY_NONE;
            break;
        case 1:
            huart_usb.Init.Parity = UART_PARITY_ODD;
            break;
        case 2:
            huart_usb.Init.Parity = UART_PARITY_EVEN;
            break;
        default :
            huart_usb.Init.Parity = UART_PARITY_NONE;
            break;
    }

    /*set the data type : only 8bits and 9bits is supported */
    switch (line_coding.datatype) {
        case 0x07:
            /* With this configuration a parity (Even or Odd) must be set */
            huart_usb.Init.WordLength = UART_WORDLENGTH_8B;
            break;
        case 0x08:
            if (huart_usb.Init.Parity == UART_PARITY_NONE) {
                huart_usb.Init.WordLength = UART_WORDLENGTH_8B;
            } else {
                huart_usb.Init.WordLength = UART_WORDLENGTH_9B;
            }

            break;
        default :
            huart_usb.Init.WordLength = UART_WORDLENGTH_8B;
            break;
    }

    huart_usb.Init.BaudRate = line_coding.bitrate;
    huart_usb.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart_usb.Init.Mode = UART_MODE_TX_RX;
    huart_usb.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart_usb) != HAL_OK) {
        /* Initialization Error */
        Error_Handler();
    }

    /* Start reception: provide the buffer pointer with offset and the buffer size */
    HAL_UART_Receive_IT(&huart_usb, (uint8_t *) (usb_tx_buf + usb_tx_ptr_in), 1);
}

/**
  * @brief  TIM_Config: Configure TIMx timer
  * @param  None.
  * @retval None
  */
void USB_TIM_Config(void) {
    /* Set TIMx instance */
    htim_usb.Instance = TIMx;

    /* Initialize TIM3 peripheral as follow:
         + Period = 10000 - 1
         + Prescaler = ((SystemCoreClock/2)/10000) - 1
         + ClockDivision = 0
         + Counter direction = Up
    */
    htim_usb.Init.Period = (CDC_POLLING_INTERVAL * 1000) - 1;
    htim_usb.Init.Prescaler = 84 - 1;
    htim_usb.Init.ClockDivision = 0;
    htim_usb.Init.CounterMode = TIM_COUNTERMODE_UP;


    /*##-6- Enable TIM peripherals Clock #######################################*/
    TIMx_CLK_ENABLE();



    if (HAL_TIM_Base_Init(&htim_usb) != HAL_OK) {
        /* Initialization Error */
        Error_Handler();
    }

    /*##-7- Configure the NVIC for TIMx ########################################*/
    /* Set Interrupt Group Priority */
    HAL_NVIC_SetPriority(TIMx_IRQn, 6, 0);

    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(TIMx_IRQn);

    if (HAL_TIM_Base_Start_IT(&htim_usb) != HAL_OK) {
        /* Starting Error */
        Error_Handler();
    }

}

/**
  * @brief  UART error callbacks
  * @param  huart_usb: UART handle
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart_usb) {
    /* Transfer error occurred in reception and/or transmission process */
    Error_Handler();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void) {
    /* Add your own code here */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
