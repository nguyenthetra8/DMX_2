#include "DMX512.h"
#include "main.h"
#include "stdio.h"
#include "stm32f1xx_hal.h"


//solt 24.752ms  40HZ refresh
//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
uint8_t dmxData[513]; // Data pool
extern UART_HandleTypeDef huart2;

/* Clear DMX Buffer */
void clrDmxData(uint8_t *dmxData) {
	int i;
	for (i = 0; i < 513; i++) {
		dmxData[i] = 0;
	}
}
/* Set Tx_GPIO_Mode */
//iomode 0:OUTPUT_PP
//iomode 1:AF_PP
void GPIO_Tx_Config_OUT(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	/*Configure GPIO pin : PtPin */
	GPIO_InitStruct.Pin = DMX_TX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(DMX_TX_GPIO_Port, &GPIO_InitStruct);
}
void GPIO_Tx_Config_AF(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	/*Configure GPIO pin : PtPin */
	GPIO_InitStruct.Pin = DMX_TX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(DMX_TX_GPIO_Port, &GPIO_InitStruct);
}

/* Delay nus for break */
void DMX_Delay_us(uint32_t nus) {
	uint32_t i = 16000*nus;
	while (i--);
}

/* Send Break sign and 00 Code */
void DMX_Break() {
	GPIO_Tx_Config_OUT();     //Set UART TX pin mode to OUTPUT
	DMX_TX_Low;
	delay_us(150);	//DMX512 BREAK >88us
	DMX_TX_High;
	delay_us(30);	//DMX512 Mark after break MAB >8us
	GPIO_Tx_Config_AF();		//Set UART TX pin mode to AF
	/* Send Start Code 00 */
	uint8_t startcode[1] = { 0 };
	HAL_UART_Transmit(&huart2, startcode, 1, 100);
}
/* Send packet data,tempnum must <512 */

void DMX_Delay1ms() {
	GPIO_Tx_Config_OUT();     //Set UART TX pin mode to OUTPUT
	DMX_TX_Low;
	DMX_Delay_us(1000);        //DMX512 BREAK >88us
	GPIO_Tx_Config_AF();		//Set UART TX pin mode to AF
}

void DMX_Send_Packet() {
	uint16_t i = 0;
	DMX_UART_INIT_SEND_DATA;
	DMX_Break();        //Break and Start Code
	for (i = 0; i < 512; i++) {
		HAL_UART_Transmit(&huart2, (uint8_t*) dmxData + i, 1, 100);
	}
}

void DMX_send_ms(int ms){
	DMX_Send_Packet();
	delay_ms(ms-3);
	clrDmxData(dmxData);
}

void DMX_Send_Add(unsigned char add) {
	uint8_t data_H, data_L;
	uint16_t i = 0;
	clrDmxData(dmxData);
	data_H = (add >> 6) & 63;     //Get High 8bit
	data_L = add & 63;		    //Get Low 8bit
	dmxData[0] = 0xc3;
	dmxData[1] = 0xf5;
	dmxData[2] = data_H + 1;
	dmxData[3] = data_H;
	dmxData[4] = data_L;
	dmxData[5] = data_L - 1;
	dmxData[6] = 0xf5;
	dmxData[7] = 0x5f;
	dmxData[8] = 0x00;
	DMX_Break();        //Break and Start Code
	DMX_UART_INIT_SEND_ADD;        //Macro 1 stop bit
	HAL_UART_Transmit(&huart2, dmxData, 1, 100);
	DMX_UART_INIT_SEND_DATA;        //Macro 2 stop bit
	for (i = 1; i < 513; i++) {
		HAL_UART_Transmit(&huart2, dmxData + i, 1, 100);
	}
	DMX_Notice(add);
}
/* Send Reset sign and 00Code */
void DMX_Reset() {
	GPIO_Tx_Config_OUT();     //Set UART TX pin mode to OUTPUT
	DMX_TX_Low;
	DMX_Delay_us(2000);              //UCS512 RESET >2s
	DMX_TX_High;
	//HAL_Delay(1);
	DMX_Delay_us(150);           //UCS512 Mark after RESET MAB >50us
	GPIO_Tx_Config_AF();
	/* Send Start Code 00 */
	uint8_t i = { 0 };
	HAL_UART_Transmit(&huart2, &i, 1, 100);
}
void DMX_ful() {
	int i = 0;
	clrDmxData(dmxData);
	for (i = 0; i < 513; i++) {
		dmxData[i] = 0xff;
	}
	DMX_Send_Packet();
} 
void DMX_Notice(uint16_t add) {
	clrDmxData(dmxData);
	dmxData[add] = 255;
	DMX_Send_Packet();
}
