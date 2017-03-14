/***********************************************************************
***********************************************************************/
#define _TASK_MAIN_DEFINE
#include "main.h"
#include "def.h"
#include "Task_main.h"
#include "Task_sysCheck.h"
#include "Task_ADC.h"
#include "wifi.h"
	 B_TKS_OP_MODE gBOpMd;
	 BATT_PACK_INFO bpInfo;
	 BATT_MODULE_INFO bmInfo;
	 UNS32  main_task_timer;
	UNS32 mTsk_rt;

	// OS_STK Stk_Task_Main[TASK_MAIN_STK_SIZE];
	WK_SOURCE WakeSrcCheck(void);
void wakeupBmu(void);
void setWhoAmI(void);
static void bmu_pwer_ctrl(UNS8 chg);
UNS32 abs(UNS32 a, UNS32 b);

BOOLEAN isCriticalFaultExist();
BOOLEAN isAlarmExist();
BOOLEAN isHWShutdownExist();
void Task_Main_Toro(void *pdata);
/***********************************************************************
***********************************************************************/
	void Task_Main(void *pdata)
	{		 
		unsigned  char	os_err,i,isCmdIn;
		Message RxMSG;
		
		//UNS32 static enterTime;
		UNS32 nowTime;
		unsigned char sdData[10]={1,2,3,4,6,7,8,9};
		SC_POWER_ON;
		PWR_12V_ON;
		while(1){
		OSTimeDlyHMSM(0, 0, 0, 10);//suspend at here for 10ms
		
		wdg_feed();		//feed dog
		mTsk_rt=OSTimeGet();	//get real time counter
		
		if(isHWShutdownExist()){
			gBOpMd=BP_FINAL;
		}else if(isCriticalFaultExist()){
			main_task_timer=mTsk_rt;
			gBOpMd=BP_ERROR;
		}
		switch(gBOpMd){
			case BP_INIT:   //wakeup bmu and make sure 2 BMU alive
				//initilize battery BSCInit();
				bpInfo.bmuWakeupCmd=0;
				bmInfo.bmuOnCmd=CMD_BMU_FET_OFF;
				//bmu_pwer_ctrl(1);	//enable 12V output for wake up BMU
				if(timeDiff(main_task_timer,mTsk_rt)>100000){ //
					gBOpMd=BP_FAULT;
				}else if(timeDiff(main_task_timer,mTsk_rt)>15000){
					main_task_timer=mTsk_rt;
					gBOpMd=BP_SYNC_OFF;
				}else if(timeDiff(main_task_timer,mTsk_rt)>5000){
					if(bmInfo.bmu_alive_num==2){
						main_task_timer=mTsk_rt;
						gBOpMd=BP_DO_NOT_ATTACH;
					}
					BMU_WAKE_LOW;
				}else if(timeDiff(main_task_timer,mTsk_rt)>2000){
					if(bmInfo.bmu_alive_num==0){
						BMU_WAKE_HIGH;
					}
				}else if(timeDiff(main_task_timer,mTsk_rt)>500){
					BMU_WAKE_LOW;
				}
				break;
			case BP_SYNC_OFF:
				bmInfo.bmuOnCmd=CMD_BMU_SHUTDOWN;
				if(timeDiff(main_task_timer,mTsk_rt)>2000){
					if(bmInfo.bmu_alive_num==0){
						gBOpMd=BP_INIT;			
						main_task_timer=mTsk_rt;
						
						BMU_WAKE_LOW;
					}
				}
				break;
			case BP_DO_NOT_ATTACH:  //no BMU has fault and FET is open
				bpInfo.bmuWakeupCmd=1;
				bmInfo.bmuOnCmd=CMD_BMU_FET_OFF;
				BMU_WAKE_LOW;
				if((bmInfo.bmu_ready_num>0)&&(bmInfo.bmu_alive_num==2)){
					gBOpMd=BP_READY_TO_ATTACH;
				}else if((timeDiff(main_task_timer,mTsk_rt)>10000)&&(bmInfo.bmu_alive_num<2)){
				
					bmInfo.bmuOnCmd=CMD_BMU_SHUTDOWN;		
					BMU_WAKE_LOW;
					gBOpMd=BP_INIT;
					main_task_timer=mTsk_rt;
				}
				break;
			case BP_READY_TO_ATTACH: 	//at least one BMU ready to work. wait for controller command
				LS3_OFF;
				if(vcuInfo.vcuCmd==VCMD_CLOSE){
					bpInfo.masterCmd=MCMD_CLOSE;
					gBOpMd=BP_NOMOR_OPERATION;
					if(bmInfo.bmu_on_num>0){
						gBOpMd=BP_NOMOR_OPERATION;
					}
				}
				break;
			case BP_NOMOR_OPERATION:
				bpInfo.masterCmd=MCMD_CLOSE;
				bmInfo.bmuOnCmd=CMD_BMU_FET_ON;
				if(vcuInfo.vcuCmd==VCMD_OPEN){
					gBOpMd=BP_DO_NOT_ATTACH;
				}else if(isAlarmExist()){
					gBOpMd=BP_REQUEST_DETACH;
				}
				break;
			case BP_REQUEST_DETACH:
				if(vcuInfo.vcuCmd==VCMD_OPEN){
					gBOpMd=BP_DO_NOT_ATTACH;
				}else if(isAlarmExist()==0){
					gBOpMd=BP_NOMOR_OPERATION;
				}
				break;
			case BP_ERROR:
				if((isCriticalFaultExist()==0)&&(isAlarmExist()==0)){
					gBOpMd=BP_DO_NOT_ATTACH;
				}
				if(timeDiff(main_task_timer,mTsk_rt)>60000){
					gBOpMd=BP_FINAL;
				}
				break;
			case BP_FINAL:  //turn off MBU power, and SC power
				bmInfo.bmuOnCmd==CMD_BMU_SHUTDOWN;
				if(bmInfo.bmu_alive_num==0){
					SC_POWER_OFF;
					
				}
				
				break;
			case BP_WARMUP:		//in low temperature. turn on heater by extenal power
				break;
				
			default:
				break;
		}
	}		
}

/***********************************************************************
***********************************************************************/
	void Task_Main_Toro(void *pdata)
	{		 
		unsigned  char	os_err,i,isCmdIn;
		Message RxMSG;
		
		//UNS32 static enterTime;
		UNS32 nowTime;
		unsigned char sdData[10]={1,2,3,4,6,7,8,9};
		SC_POWER_ON;
		PWR_12V_ON;
		while(1){
		OSTimeDlyHMSM(0, 0, 0, 10);//suspend at here for 10ms
		
				wdg_feed();
		bmu_pwer_ctrl(1);
		mTsk_rt=OSTimeGet();
		switch(gBOpMd){
			case B_BOOT:
				//initilize battery BSCInit();
				if(timeDiff(main_task_timer,mTsk_rt)>2){
				bpInfo.bmuWakeupCmd=1;
				gBOpMd=B_ARBITRATION;
				main_task_timer=mTsk_rt;
				}
				break;
			case B_ARBITRATION: 	//assign BMU id, set Master attribuate
					LS3_OFF;
				bmInfo.Inter_Role=ARTRIBITION;
				sysInfo.wkSrc=WakeSrcCheck();
				if(timeDiff(main_task_timer,mTsk_rt)>60){
					setWhoAmI();
					if(bmInfo.Inter_Role==MASTER){
						gBOpMd=B_ENGIZE_DC;
					}else if(bmInfo.Inter_Role==SLAVE){
							gBOpMd=B_WAIT_SLAVE;
					}
					main_task_timer=mTsk_rt;
				}
				break;
			case B_RUN_SLAVE:
				break;
			case B_WAIT_SLAVE:
				if(bpInfo.masterCmd==1){
					gBOpMd=B_PCHG_WAIT;
				}
				break;
			case B_PCHG_WAIT:
				bmInfo.bmuOnCmd=1;
				if((abs(ad_res.V_DC_Bus,bmInfo.mod_volt)<1000)&&(ad_res.V_DC_Bus>30000)){
					main_task_timer=mTsk_rt;
					gBOpMd=B_ENGIZE_DC;
				}
				break;
			case B_ENGIZE_DC:
				bmInfo.bmuOnCmd=1;
				if(ad_res.V_DC_Bus>30000){
					if(sysInfo.wkSrc==WK_CHARGE){
						gBOpMd=B_CHARGE;
					}else{
						gBOpMd=B_WAIT_VCU;
					}
					main_task_timer=mTsk_rt;
				}else if(timeDiff(main_task_timer,mTsk_rt)>10000){
					gBOpMd=B_FAULT;
					main_task_timer=mTsk_rt;
				}
				break;
			case B_WAIT_VCU:
				if(timeDiff(main_task_timer,mTsk_rt)>(ms10*1000)){
						gBOpMd=B_FAULT;
				}
				if(sysInfo.f_vcu_lost==0){
					sysInfo.f_fault=0;
					gBOpMd=B_RUN;
				}
				break;
			case B_RUN:
				bpInfo.bmuWakeupCmd=1;
				bmInfo.bmuOnCmd=1;
				//ver1.02 if(sysInfo.f_fault){
				 if(0){
				//	gBOpMd=B_FAULT;
				}else if((vcuInfo.keep_alive_req==VCMD_OPEN)||(vcuInfo.keep_alive_req==VCMD_SD_PRE)){
				//	gBOpMd=B_SHUTDOWN_PRE;
				}
				LS3_ON;
				break;
			case B_IDLE:
				bpInfo.bmuWakeupCmd=1;
				bmInfo.bmuOnCmd=0;
				if(vcuInfo.keep_alive_req==1){
					gBOpMd=B_RUN;
				}
				break;
			  /* turn off BMU. save data in NVM */
			case B_SHUTDOWN_PRE:
				if(vcuInfo.keep_alive_req==VCMD_CLOSE){
					//gBOpMd=B_RUN;
					NVM_AllInfoWrite();
					gBOpMd=B_SHUTDOWN;
				}else if(vcuInfo.keep_alive_req==VCMD_OPEN){
					NVM_AllInfoWrite();
					gBOpMd=B_SHUTDOWN;
				}else{
					NVM_AllInfoWrite();
					gBOpMd=B_SHUTDOWN;
				}
				bmInfo.bmuOnCmd=0;
				bpInfo.bmuWakeupCmd=0;
				break;
			case B_SHUTDOWN:
				bpInfo.bmuWakeupCmd=0;
				bmInfo.bmuOnCmd=0;
				if(bmInfo.f_bmu_lost){
					//cut off controller power
					SC_POWER_OFF;
				}
				break;
			case B_FAULT:
				if(OSSemAccept(sem_fault_clr)){
					gBOpMd=B_RUN;
				}else{
					gBOpMd=B_SHUTDOWN_PRE;
				}
				break;
			case B_CHARGE:
				if (bpInfo.w_idle){
					gBOpMd=B_SHUTDOWN_PRE;
				}
				break;
			case B_CHARGE_DONE:
				break;
			case B_DISCHARGE:
				break;
			
			case B_STORAGE:
				break;
			case B_STORAGE_CHARGE:
				break;
			default:
				break;
		}
	}		
}
WK_SOURCE WakeSrcCheck(void){
	int a;
	WK_SOURCE src;
	if((!DI_WAKE_BY_CHG)&&(sysInfo.f_vcu_lost)){
		src=WK_CHARGE;
	}else{
		src=WK_MACHINE;
	}
	
	
	return src;
}
void wakeupBmu(void){
	if (BMU_WAKE_STAT==1){
		PWR_12V_ON;
		BMU_WAKE_HIGH;
	}
}
void setWhoAmI(void){
	UNS16 mySn,sn;
	BM_POSISTION whoAmI=MASTER;
	UNS8 mid,rank=0;
	mySn=bmInfo.mod_sn;
	for(mid=0;mid<bpInfo.mod_num;mid++){
		sn=bpInfo.mod[mid].mod_sn;
		if(mySn>sn){
			whoAmI=SLAVE;
			rank++;
		}
	}
	bmInfo.Inter_Role=whoAmI;
	bmInfo.intAddr=rank+0xA9;
	bpInfo.mod[0].Inter_Role=whoAmI;	
	
}
void masterRun(void){
}
static void bmu_pwer_ctrl(UNS8 chg){
	static UNS8 lastCmd;
	static UNS8 pulse_stat=0;
	static UNS16 lp;
	static UNS32 tskTm=0;
	UNS8 bt[8];

	switch(pulse_stat){
		case 0:
			if(timeDiff(tskTm,mTsk_rt)>1000){
				pulse_stat=	1;
				tskTm=mTsk_rt;
			}
			BMU_WAKE_LOW;
			break;
		case 1:
			if(((bpInfo.bmuWakeupCmd && (bmInfo.bmu_alive_num<1))
				||((bpInfo.bmuWakeupCmd==0) && (bmInfo.bmu_alive_num==2)))
				&&(timeDiff(tskTm,mTsk_rt)>1000))
			{
 					pulse_stat=2;
					tskTm=mTsk_rt;
					lp=0;
				
			}
			break;
		case 2:  //send a pulse
			if(timeDiff(tskTm,mTsk_rt)>2000){
				pulse_stat=3;
				tskTm=mTsk_rt;
			}
			BMU_WAKE_HIGH;
			break;
		case 3:
			if(timeDiff(tskTm,mTsk_rt)>3000){
				pulse_stat=0;
				tskTm=mTsk_rt;
			}
			BMU_WAKE_LOW;
			break;
		case 4:
			break;
	}
}
	
UNS32 abs(UNS32 a, UNS32 b){
	if(a>b) return (a-b);
	else return (b-a);
}
BOOLEAN isCriticalFaultExist(){
	BOOLEAN ret;
	if((LID_SW_STAT==0)
		||(MANUAL_SW_STAT==0)
		){
		ret=1;
	}
		
	return ret;
}
BOOLEAN isAlarmExist(){
	BOOLEAN ret=0;
	return ret;
}
BOOLEAN isHWShutdownExist(){
	BOOLEAN ret=0;
	return ret;
}
