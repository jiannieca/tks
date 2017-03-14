#ifndef _CONFIG_H_
#define _CONFIG_H_

//#define EVAL_BOARD_Z
//#define EVAL_BOARD_V
#define BMS_SW_ID 0x0084
#define BMS_HW_ID 0x0022
#define DEFAULT_MOD_SN 0x1

#define PRO_TKS

#define PROTO_A
#ifdef PROTO_A
	#define FLASH_CSN      	GPIO_Pin_9
	#define GPIO_FLASH_CSN  GPIOC
	#define RCC_FLASH_CSN  	RCC_AHB1Periph_GPIOE
	#define FLASH_SPI      	SPI3
#endif
#ifdef EVAL_BOARD_Z
	#define FLASH_CSN      	GPIO_Pin_4
	#define GPIO_FLASH_CSN  GPIOE
	#define RCC_FLASH_CSN  	RCC_AHB1Periph_GPIOE
	#define FLASH_SPI      	SPI3
#endif

#ifdef EVAL_BOARD_V
	#define FLASH_CSN      	GPIO_Pin_0
	#define GPIO_FLASH_CSN  GPIOB
	#define RCC_FLASH_CSN  	RCC_AHB1Periph_GPIOB

	#define FLASH_SPI      	SPI1
#endif
//SPI CS 
#define SPI_FLASH_CS_LOW()     {GPIO_ResetBits(GPIO_FLASH_CSN, FLASH_CSN);}
#define SPI_FLASH_CS_HIGH()     {GPIO_SetBits(GPIO_FLASH_CSN, FLASH_CSN);}
#define LED_OFF()     {GPIO_SetBits(GPIOC,GPIO_Pin_6);}
#define LED_ON()    {GPIO_ResetBits(GPIOC,GPIO_Pin_6);}

#define ADDR_SYS_IN_NVM 0x1000
#define ADDR_BM_IN_NVM 0x2000
#define ADDR_BP_IN_NVM 0x3000
#define ADDR_LOG_IN_NVM 0x4000
#define ms10 0x1

#define SC_TORO_SW_ID 0x84
#define SC_TORO_HW_ID 0x22
#define BP_FW_VER_MAJOR 0x00
#define BP_FW_VER_MINOR 0x1
#define BP_FW_VER_PATCH 0x4

extern void SCInit(void);
#endif
