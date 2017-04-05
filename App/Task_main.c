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
TCODE_T m_tc[TC_NUM];
	// OS_STK Stk_Task_Main[TASK_MAIN_STK_SIZE];
	WK_SOURCE WakeSrcCheck(void);
void wakeupBmu(void);
void setWhoAmI(void);
static void bmu_pwer_ctrl(UNS8 chg);
UNS32 abs(UNS32 a, UNS32 b);
UNS32 abso(INT32 a);

BOOLEAN isCriticalFaultExist();
BOOLEAN isAlarmExist();
BOOLEAN isHWShutdownExist();
BOOLEAN modAlarmClr();
BOOLEAN modFaultClr();

void Task_Main_Toro(void *pdata);
void check_tc(TCODE_T *pt_tc);
static void TCTableInit(void);
static UNS32 getModPara(FAW_ID_T tcid);

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
		TCTableInit();
		while(1){
		OSTimeDlyHMSM(0, 0, 0, 10);//suspend at here for 10ms
		wdg_feed();		//feed dog

		
		mTsk_rt=OSTimeGet();	//get real time counter

		if(timeDiff(mTsk_rt,sysInfo.sysStartTime)>100000){ //
			check_tc((TCODE_T *)&m_tc[0]);
		}

		if(isHWShutdownExist()){
			gBOpMd=BP_FINAL;
		}else if((isCriticalFaultExist()||isAlarmExist())&&(gBOpMd!=BP_ERROR)&&(gBOpMd!=BP_FINAL)){
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
					// nie if(bmInfo.bmu_alive_num==2){
					if(bmInfo.bmu_alive_num>0){
						main_task_timer=mTsk_rt;
						gBOpMd=BP_DO_NOT_ATTACH;
					}
					BMU_WAKE_LOW;
				}else if(timeDiff(main_task_timer,mTsk_rt)>4000){
					if(bmInfo.bmu_alive_num==0){
						BMU_WAKE_HIGH;
					}
				}else if(timeDiff(main_task_timer,mTsk_rt)>3000){
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
				//nie if((bmInfo.bmu_ready_num>0)&&(bmInfo.bmu_alive_num==2)){
				if((bmInfo.bmu_ready_num>0)&&(bmInfo.bmu_alive_num>0)){
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
				bmInfo.bmuOnCmd=VCMD_OPEN;
				if((isCriticalFaultExist()==0)&&(isAlarmExist()==0)){
					gBOpMd=BP_DO_NOT_ATTACH;
				}
				if(timeDiff(main_task_timer,mTsk_rt)>600000){
					gBOpMd=BP_FINAL;
				}
				break;
			case BP_FINAL:  //turn off MBU power, and SC power
				bmInfo.bmuOnCmd=CMD_BMU_SHUTDOWN;
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
UNS32 abso(INT32 a){
	if(a>=0) return (a);
	else return (-a);
}
BOOLEAN isCriticalFaultExist(){
	BOOLEAN ret=0;
	if(isDelayExp(5000,sysInfo.sysStartTime)==0){
		return 0;
	}
	if(0
		//||(LID_SW_STAT==0)
		//||(MANUAL_SW_STAT==0)
		||(m_tc[F_MOD_OV].tc_stat)
		||(m_tc[F_MOD_UV].tc_stat)
		||(m_tc[F_MOD_OT].tc_stat)
		||(m_tc[F_MOD_UT].tc_stat)
		||(m_tc[F_MOD_DOC].tc_stat)
		||(m_tc[F_MOD_COC].tc_stat)
		){
		ret=1;
	}
		
	return ret;
}
BOOLEAN modFaultClr(){
	BOOLEAN ret=0;
	m_tc[F_MOD_OV].tc_stat=0;
	m_tc[F_MOD_UV].tc_stat=0;
	m_tc[F_MOD_OT].tc_stat=0;
	m_tc[F_MOD_UT].tc_stat=0;
	m_tc[F_MOD_DOC].tc_stat=0;
	m_tc[F_MOD_COC].tc_stat=0;
	modAlarmClr();
	ret=1;
	return ret;
}
BOOLEAN isAlarmExist(){
	BOOLEAN ret=0;
	if(isDelayExp(5000,sysInfo.sysStartTime)==0){
		return 0;
	}
	if(0
		//||(LID_SW_STAT==0)
		//||(MANUAL_SW_STAT==0)
		||(m_tc[F_MOD_OV].tc_stat)
		||(m_tc[A_MOD_OV].tc_stat)
		||(m_tc[F_MOD_UV].tc_stat)
		||(m_tc[A_MOD_UV].tc_stat)
		||(m_tc[F_MOD_OT].tc_stat)
		||(m_tc[A_MOD_OT].tc_stat)
		||(m_tc[F_MOD_UT].tc_stat)
		||(m_tc[A_MOD_UT].tc_stat)
		||(m_tc[F_MOD_DOC].tc_stat)
		||(m_tc[A_MOD_DOC].tc_stat)
		||(m_tc[F_MOD_COC].tc_stat)
		||(m_tc[A_MOD_COC].tc_stat)
		){
			ret=1;
		}else{
			ret=0;
		}

	return ret;
}
BOOLEAN modAlarmClr(){
	BOOLEAN ret=0;
		m_tc[F_MOD_OV].tc_stat=0;
		 m_tc[A_MOD_OV].tc_stat=0;
		 m_tc[F_MOD_UV].tc_stat=0;
		 m_tc[A_MOD_UV].tc_stat=0;
		 m_tc[F_MOD_OT].tc_stat=0;
		 m_tc[A_MOD_OT].tc_stat=0;
		 m_tc[F_MOD_UT].tc_stat=0;
		 m_tc[A_MOD_UT].tc_stat=0;
		 m_tc[F_MOD_DOC].tc_stat=0;
		 m_tc[A_MOD_DOC].tc_stat=0;
		 m_tc[F_MOD_COC].tc_stat=0;
		 m_tc[A_MOD_COC].tc_stat=0;
		ret=1;
	return ret;
}

BOOLEAN isWarningExist(){
	BOOLEAN ret=0;
	if(0
		||(m_tc[W_MOD_OV].tc_stat)
		||(m_tc[W_MOD_UV].tc_stat)
		||(m_tc[W_MOD_OT].tc_stat)
		||(m_tc[W_MOD_UT].tc_stat)
		||(m_tc[W_MOD_DOC].tc_stat)
		||(m_tc[W_MOD_COC].tc_stat))
		{
		ret=1;
	}else{
		ret=0;
	}
	return ret;
}
BOOLEAN isHWShutdownExist(){
	BOOLEAN ret=0;
	if(isDelayExp(5000,sysInfo.sysStartTime)==0){
		return 0;
	}
	if((0
		||(vcuInfo.vcuCmd==VCMD_SHUTDOWN)
		//||(LID_SW_STAT==0)
		//||(MANUAL_SW_STAT==0)
		)){
			ret=1;
		}else{
			ret=0;
	}
	return ret;
}
void check_tc(TCODE_T *pt_tc){
	FAW_ID_T tcid;
	TCODE_T *ptc;
	TCODE_T tc;
	UINT32 rtVal;
	static uint16_t tm_rem_s[TC_NUM]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	static uint16_t tm_rem_r[TC_NUM]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	ptc=pt_tc;
	for(tcid=0;tcid<20;tcid++){
		rtVal=getModPara(tcid);
		ptc=pt_tc+tcid;
		if (ptc->tc_stat==0){  //no fault
			if(ptc->dir>0){	/* upside threshold */
				if(rtVal > ptc->thd_s){
					tm_rem_s[tcid]++;
					if (tm_rem_s[tcid] > ptc->tm_s){
						ptc->tc_stat=1;
						tm_rem_r[tcid]=0;
					}
				}else{
					tm_rem_s[tcid]=0;
				}
			}else{
				if(rtVal < ptc->thd_s){
					tm_rem_s[tcid] ++;
					if (tm_rem_s[tcid] > ptc->tm_s){
						ptc->tc_stat=1;
						tm_rem_r[tcid]=0;
					}
				}else{
					tm_rem_s[tcid]=0;
				}
			}
		}else{	//fault exist
			if(ptc->tm_r !=0xFFFF){	//fault recoverable
			if(ptc->dir>0){	/* upside threshold reset when val<threshold */
				if(rtVal < ptc->thd_r){
					tm_rem_r[tcid] ++;
					if (tm_rem_r[tcid] > ptc->tm_r){
						ptc->tc_stat=0;
						tm_rem_s[tcid]=0;
					}
				}else{
					tm_rem_r[tcid]=0;
				}
			}else{
				if(rtVal > ptc->thd_r){	//lowside. reset when val>threshold
					tm_rem_r[tcid] ++;
					if (tm_rem_r[tcid] > ptc->tm_r){
						ptc->tc_stat=0;
						tm_rem_s[tcid]=0;
					}
				}else{
					tm_rem_r[tcid]=0;
				}
			}
			}
	}
	}
}
static void TCTableInit(void){
	
	FAW_ID_T fawId;
	int i;
	for(i=0;i<TC_NUM;i++){
		m_tc[i].id=i;
	}
	fawId=F_MOD_OV;
	m_tc[fawId].tc_stat=0;
	m_tc[fawId].dir=1;
	m_tc[fawId].thd_s=4150;
	m_tc[fawId].tm_s=300;
	m_tc[fawId].thd_r=3900;
	m_tc[fawId].tm_r=300;
	fawId=A_MOD_OV;
	m_tc[fawId].tc_stat=0;
	m_tc[fawId].dir=1;
	m_tc[fawId].thd_s=4050;
	m_tc[fawId].tm_s=1000;
	m_tc[fawId].thd_r=3900;
	m_tc[fawId].tm_r=300;
	fawId=W_MOD_OV;
	m_tc[fawId].tc_stat=0;
	m_tc[fawId].dir=1;
	m_tc[fawId].thd_s=4000;
	m_tc[fawId].tm_s=300;
	m_tc[fawId].thd_r=3900;
	m_tc[fawId].tm_r=100;
	fawId=F_MOD_UV;
	m_tc[fawId].tc_stat=0;
	m_tc[fawId].dir=0;
	m_tc[fawId].thd_s=2950;
	m_tc[fawId].tm_s=300;
	m_tc[fawId].thd_r=3200;
	m_tc[fawId].tm_r=300;
	fawId=A_MOD_UV;
	m_tc[fawId].tc_stat=0;
	m_tc[fawId].dir=0;
	m_tc[fawId].thd_s=3000;
	m_tc[fawId].tm_s=300;
	m_tc[fawId].thd_r=3150;
	m_tc[fawId].tm_r=300;
	fawId=W_MOD_UV;
	m_tc[fawId].tc_stat=0;
	m_tc[fawId].dir=0;
	m_tc[fawId].thd_s=3050;
	m_tc[fawId].tm_s=300;
	m_tc[fawId].thd_r=3100;
	m_tc[fawId].tm_r=100;
	fawId=F_MOD_OT;
	m_tc[fawId].tc_stat=0;
	m_tc[fawId].dir=1;
	m_tc[fawId].thd_s=550; // 0.1C
	m_tc[fawId].tm_s=100;
	m_tc[fawId].thd_r=400; // 0.1C
	m_tc[fawId].tm_r=500;
	fawId=A_MOD_OT;
	m_tc[fawId].tc_stat=0;
	m_tc[fawId].dir=1;
	m_tc[fawId].thd_s=500; // 0.1C
	m_tc[fawId].tm_s=300;
	m_tc[fawId].thd_r=450; // 0.1C
	m_tc[fawId].tm_r=100;
	fawId=W_MOD_OT;
	m_tc[fawId].tc_stat=0;
	m_tc[fawId].dir=1;
	m_tc[fawId].thd_s=480; // 0.1C
	m_tc[fawId].tm_s=46;
	m_tc[fawId].thd_r=460; // 0.1C
	m_tc[fawId].tm_r=100;
	fawId=F_MOD_UT;
	m_tc[fawId].tc_stat=0;
	m_tc[fawId].dir=0;
	m_tc[fawId].thd_s=(UINT32)-400; // 0.1C
	m_tc[fawId].tm_s=300;
	m_tc[fawId].thd_r=(UINT32)-100; // 0.1C
	m_tc[fawId].tm_r=100;
	fawId=A_MOD_UT;
	m_tc[fawId].tc_stat=0;
	m_tc[fawId].dir=0;
	m_tc[fawId].thd_s=(UINT32)-300; // 0.1C
	m_tc[fawId].tm_s=300;
	m_tc[fawId].thd_r=(UINT32)-100; // 0.1C
	m_tc[fawId].tm_s=100;
	fawId=W_MOD_UT;
	m_tc[fawId].tc_stat=0;
	m_tc[fawId].dir=0;
	m_tc[fawId].thd_s=(UINT32)-100; // 0.1C
	m_tc[fawId].tm_s=300;
	m_tc[fawId].thd_r=(UINT32)-80; // 0.1C
	m_tc[fawId].tm_r=100;
	
	fawId=F_MOD_DOC;
	m_tc[fawId].tc_stat=0;
	m_tc[fawId].dir=1;
	m_tc[fawId].thd_s=50000; //mA
	m_tc[fawId].tm_s=300;
	m_tc[fawId].thd_r=2000; //mA
	m_tc[fawId].tm_r=100;
	fawId=A_MOD_DOC;
	m_tc[fawId].tc_stat=0;
	m_tc[fawId].dir=1;
	m_tc[fawId].thd_s=40000; //mA
	m_tc[fawId].tm_s=300;
	m_tc[fawId].thd_r=35000; //mA
	m_tc[fawId].tm_r=0;
	fawId=W_MOD_DOC;
	m_tc[fawId].tc_stat=0;
	m_tc[fawId].dir=1;
	m_tc[fawId].thd_s=30000; //mA
	m_tc[fawId].tm_s=300;
	m_tc[fawId].thd_r=30000; //mA
	m_tc[fawId].tm_r=100;
	fawId=F_MOD_COC;
	m_tc[fawId].tc_stat=0;
	m_tc[fawId].dir=1;
	m_tc[fawId].thd_s=35000; //mA
	m_tc[fawId].tm_s=300;	//10ms 
	m_tc[fawId].thd_r=30000; //mA
	m_tc[fawId].tm_r=100;
	fawId=A_MOD_COC;
	m_tc[fawId].tc_stat=0;
	m_tc[fawId].dir=1;
	m_tc[fawId].thd_s=30000; //mA
	m_tc[fawId].tm_s=300;
	m_tc[fawId].thd_r=25000; //mA
	m_tc[fawId].tm_r=100;
	fawId=W_MOD_COC;
	m_tc[fawId].tc_stat=0;
	m_tc[fawId].dir=1;
	m_tc[fawId].thd_s=25000; //mA
	m_tc[fawId].tm_s=300;
	m_tc[fawId].thd_r=25000; //mA
	m_tc[fawId].tm_r=0;
}
static UNS32 getModPara(FAW_ID_T tcid){
	UNS32 val;
	switch(tcid){
		case F_MOD_OV:
		case A_MOD_OV:
		case W_MOD_OV:
			val=bmInfo.m_cv_max;
			break;
		case F_MOD_UV:
		case A_MOD_UV:
		case W_MOD_UV:
			val=bmInfo.m_cv_min;
			break;
		case F_MOD_OT:
		case A_MOD_OT:
		case W_MOD_OT:
			if(bmInfo.m_ct_max<=0) val=0;
			val=(UINT32)bmInfo.m_ct_max;
			break;
		case F_MOD_UT:
		case A_MOD_UT:
		case W_MOD_UT:
			if(bmInfo.m_ct_min>=0) val=0xFFFFFFFF;
			else val=(UINT32)bmInfo.m_ct_min;
			break;
		case F_MOD_DOC:
		case A_MOD_DOC:
		case W_MOD_DOC:
			if(bmInfo.mod_curr>=0) val=0;	//charging
			else val=(UINT32)(-bmInfo.mod_curr);
			break;
		case F_MOD_COC:
		case A_MOD_COC:
		case W_MOD_COC:
			if(bmInfo.mod_curr>=0) val=bmInfo.mod_curr;	//charging
			else val=(UINT32)(0);
			break;
		case F_CV_DIFF:
		case A_CV_DIFF:
		case W_CV_DIFF:
			val=(bmInfo.m_cv_max>bmInfo.m_cv_min)?(bmInfo.m_cv_max-bmInfo.m_cv_min):0;
			break;
		case F_HEATER_OT:
		case A_HEATER_OT:
		case W_HEATER_OT:
			//bpInfo.
			break;
		default:
			break;
	}
	return val;
}

