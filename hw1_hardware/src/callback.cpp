#include "usart.h"
#include "../inc/remotecontrol.h"
#include "stm32f4xx_hal_dma.h"

extern volatile uint8_t sbus_rx_buffer[RC_FRAME_LENGTH];
extern RC_Ctl_t RC_CtrlData;


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART3) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *) sbus_rx_buffer, RC_FRAME_LENGTH);
        //HAL_UARTEx_ReceiveToIdle_IT(&huart3, (uint8_t *) sbus_rx_buffer, RC_FRAME_LENGTH);

        RemoteControlDataReceive();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
    RemoteControlDataReceive();

}

