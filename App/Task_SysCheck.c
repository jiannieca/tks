/***********************************************************************
***********************************************************************/
#include "main.h"  
#include "Task_sysCheck.h"
#include "Task_main.h"

OS_EVENT  *fault_flag = 0;			//any fault detected
OS_STK Stk_TaskFaultCheck[TASK_FAULT_CHECK_STK_SIZE];
OS_EVENT *sem_vcu_can_rev;
OS_EVENT *sem_bmu_can_rev;
OS_EVENT *sem_fault_clr;

void Task_faultCheck(void);

/***********************************************************************
***********************************************************************/
void Task_FaultCheck(void *pdata)
{
	int16_t vcu_rev_flag,bmu_rev_flag;
	static uint16_t vcu_lose_cnt=0,bmu_lose_cnt=0;
	static uint32_t idleStartTime;
	sem_vcu_can_rev=OSSemCreate(0);
	sem_bmu_can_rev=OSSemCreate(0);
	while(1)
	{  
		OSTimeDlyHMSM(0, 0, 0, 10);//suspend at here for 10ms
		// VCU comm fault
		vcu_rev_flag=OSSemAccept(sem_vcu_can_rev);
		if(vcu_rev_flag){
			vcu_lose_cnt=0;
			sysInfo.f_vcu_lost=0;
		}else{
			if(vcu_lose_cnt++>600){
			//v1.02	if(vcu_lose_cnt++>60){
				sysInfo.f_vcu_lost=1;
			}
		}
		//BMU comm fault
		bmu_rev_flag=OSSemAccept(sem_bmu_can_rev);
		if(bmu_rev_flag){
			bmu_lose_cnt=0;
			bmInfo.f_bmu_lost=0;
		}else{
			if(bmu_lose_cnt++>600){
				bmInfo.f_bmu_lost=1;
			}
		}		
		//VCU comm fault
		if(0
			||sysInfo.f_vcu_lost
			||bmInfo.f_bmu_lost
		){
			sysInfo.f_fault=1;
	//		vcuInfo.keep_alive_req=VCMD_OPEN;
		}else{
			sysInfo.f_fault=0;
		}
		//IDLE when chargef
		if (bpInfo.bp_curr>300){
			bpInfo.w_idle=0;
			idleStartTime=OSTime;
		}else{
			if(timeDiff(idleStartTime,OSTime)>60000){
				bpInfo.w_idle=1;
			}
		}
			
	}
}

