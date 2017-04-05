/***********************************************************************
***********************************************************************/
#ifndef _BATTERY_GT_H_
#define _BATTERY_GT_H_
void Task_CAN2MsgBuf_GT(void *pdata);

typedef enum{
	GT_READ_VOL,
	GT_READ_TEMP,
	GT_READ_SOC,
	GT_READ_CURR
}GT_OP_CMD;

#endif

