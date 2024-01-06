#ifndef __DMX_H
#define __DMX_H
#include "main.h"


#define DMX_MODEL_NUM 3                          //IC num
#define DMX_MODEL_CHN 40                         //channel IC per UCS512=4*10 TM512=3*8
#define DMX_UART USART2							 //DMX UART PORT

#define DEF_R   255
#define DEF_G   0
#define DEF_B   0
#define DEF_W   0
#define DMX_TX_High  HAL_GPIO_WritePin(DMX_TX_GPIO_Port,DMX_TX_Pin,GPIO_PIN_SET)
#define DMX_TX_Low  HAL_GPIO_WritePin(DMX_TX_GPIO_Port,DMX_TX_Pin,GPIO_PIN_RESET)


extern void DMX_Reset();
extern uint8_t DMX_Transposition(uint8_t tempchar);
extern void clrDmxData(uint8_t* dmxData);
extern void GPIO_Tx_Config_OUT(void);
extern void GPIO_Tx_Config_AF(void);
extern void DMX_Delay_us(uint32_t nus);
extern void DMX_Break();
extern void DMX_Delay1ms();
extern void DMX_Send_Packet();
extern void DMX_Init();
extern void DMX_Send_Add(unsigned char add);
extern void DMX_Notice(uint16_t add);
extern void DMX_ful();
extern void DMX_send_ms(int ms);

#endif

