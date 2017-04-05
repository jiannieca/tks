#include "bmu.h"
#include "main.h"
#include "can.h"
#include "appBMU.h"
#include "Task_main.h"
#include "Task_sysCheck.h"
#include "sysTick.h"
parseBroadMsg(Message *m, UNS8 nodeId);
UINT16 SOC_OCV_Cal(BATT_MODULE_INFO *bm,UINT16 savedSOC);
UINT16 SOC_OCV_Lookup(INT8 temp,UINT16 Vcell);
void strPlimSet(BATT_MODULE_INFO *bm);

void bmuInit(void);
BMU_INFO bmu[MAX_BMU_NUM];
parseBroadMsg(Message *m, UNS8 nodeId){
	UNS16 idx;
	UNS8 subIdx,dcnt;
	UNS32 da;
	UNS8 ds[8];
	static int32_t last_curr,now_curr;
	void *d;
	UNS8 nid;
	float flVolt;
	OSSemPost(sem_bmu_can_rev);
	if (nodeId>0){ nid=1;} else{ nid=nodeId-1;}
	if (nodeId>bmInfo.bmu_total) bmInfo.bmu_total=nodeId;
	dcnt=4-getSDOn2(m->data[0]);
	memcpy((void *)&da,m->data+4,dcnt);
	idx=getSDOindex(m->data[1],m->data[2]);
	subIdx=m->data[3];
	Task_ADC();
	switch(idx){
		case 0x6D01:	
			bmu[nodeId-1].aliveBuf=1;
			bmu[nodeId-1].SOC=da;
			break;
		case 0x6D02:	
			bmu[nodeId-1].aliveBuf=1;
			memcpy((void *)&bmu[nodeId-1].SOH,m->data+4,2);
			bmu[nodeId-1].SOH*=10;
			break;
		case 0x6D03:	
			bmu[nodeId-1].aliveBuf=1;
			memcpy((void *)&bmu[nodeId-1].I_bmu,m->data+4,dcnt);
		/*	now_curr=bmu[nodeId-1].I_bmu;
			if((((last_curr-bmu[nodeId-1].I_bmu)>5000)&&(last_curr>bmu[nodeId-1].I_bmu))
				||(((bmu[nodeId-1].I_bmu>last_curr)<5000)&&(last_curr<bmu[nodeId-1].I_bmu))
				){
				bmu[nodeId-1].I_bmu=last_curr;
			}
			last_curr=now_curr;
			*/
			break;

		case 0x6D04:	
			memcpy((void *)&flVolt,m->data+4,dcnt);
			bmu[nodeId-1].V_bmu=(uint32_t)flVolt;
			break;
		case 0x6D05:	
			memcpy((void *)&bmu[nodeId-1].ct_max,m->data+4,dcnt);
			break;
		case 0x6D06:	
			memcpy((void *)&bmu[nodeId-1].ct_min,m->data+4,dcnt);
			break;
		case 0x6D07:	
			memcpy((void *)&bmu[nodeId-1].bmuSafeFlag_1,m->data+4,dcnt);
			break;
		case 0x6D08:	
			memcpy((void *)&bmu[nodeId-1].bmuSafeFlag_2,m->data+4,dcnt);
			break;
		case 0x6D09:	
			memcpy((void *)&bmu[nodeId-1].bmuStatus,m->data+4,dcnt);
			break;
		case 0x6D0A:	
			memcpy((void *)&bmu[nodeId-1].fetFlag,m->data+4,dcnt);
			break;
		case 0x6D0B:	
			memcpy((void *)&bmu[nodeId-1].parllFlag,m->data+4,dcnt);
			break;
		case 0x6024:
			memcpy((void *)&bmu[nodeId-1].cc_max,m->data+4,dcnt);
			break;
		case 0x6025:
			memcpy((void *)&bmu[nodeId-1].dc_max,m->data+4,dcnt);
			break;
		case 0x6E80:
			memcpy((void *)&bmu[nodeId-1].dc_max,m->data+4,dcnt);
				break;
		default:
			break;

	}
}
void updateBP_BM(void){
	UNS8 bmuid,mid,onNum=0,offNum=0,aliveNum=0,readyNum=0,cid;
	UNS16 soc=0,soh=0,cc_max=0,dc_max=0,cv_max=0,cv_min=0xffff;
	float ct_max=-100,ct_min=200;
	UNS32 vbmu=0,cap,cv_total;
	int32_t Ibmu=0,ct_total;
	BMU_INFO tbmu;
	UNS8 e_m_ov,e_m_ot,e_m_uv,e_m_ut;
	UNS8 cv_min_cid,cv_max_cid,cv_min_mid,cv_max_mid,ct_min_cid,ct_min_mid,ct_max_cid,ct_max_mid;
	static u32_t loseBMU_tm;
	//bmInfo.bmu_total=1;
	for(bmuid=0;bmuid<bmInfo.bmu_total;bmuid++){
		if(bmu[bmuid].alive>0){
			soc+=bmu[bmuid].SOC;
			soh+=bmu[bmuid].SOH;
			cc_max+=bmu[bmuid].cc_max/100;
			dc_max+=bmu[bmuid].dc_max/100;
			if((bmu[bmuid].I_bmu<(-200000)) ||(bmu[bmuid].I_bmu>400000)){
				bmu[bmuid].I_bmu=0;
			}
			Ibmu+=bmu[bmuid].I_bmu;
			vbmu=bmu[bmuid].V_bmu;
			if((bmInfo.mod_volt<vbmu)||(bmInfo.mod_volt>0xFF000000)) bmInfo.mod_volt=vbmu;
			
			tbmu=bmu[bmuid];
			if((tbmu.bmuSafeFlag_2.w_flag2)==0){	//chargerable or dischargable
					readyNum++;
				}else{
					offNum++;
				}
			if(((tbmu.bmuSafeFlag_1.w_flag1)& 0x1C0000)>0){	//charging/discharging/idle
				onNum++;
			}
		}
	}
		
	if(timeDiff(loseBMU_tm,OSTimeGet())>2500){
		for(bmuid=0;bmuid<bmInfo.bmu_total;bmuid++){
			tbmu=bmu[bmuid];
			if(((bmInfo.ptBMU+bmuid)->aliveBuf)>0){ //received boradcast message
				(bmInfo.ptBMU+bmuid)->aliveBuf=0;
				(bmInfo.ptBMU+bmuid)->alive=1;
				aliveNum++;
				for(cid=0;cid<BMU_CV_NUM;cid++){
					if(cv_max<tbmu.cv[cid]){
						cv_max=tbmu.cv[cid];
						cv_max_cid=cid;
						cv_max_mid=bmuid;
					}
					if(cv_min>tbmu.cv[cid]){
						cv_min=tbmu.cv[cid];
						cv_min_cid=cid;
						cv_min_mid=bmuid;
					}
					cv_total+=tbmu.cv[cid];
				}
				
				for(cid=0;cid<BMU_CT_NUM;cid++){
					if(ct_max<tbmu.ct[cid]){
						ct_max=tbmu.ct[cid];
						ct_max_cid=cid;
						ct_max_mid=bmuid;
					}
					if(ct_min>tbmu.ct[cid]){
						ct_min=tbmu.ct[cid];
						ct_min_cid=cid;
						ct_min_mid=bmuid;
					}
					ct_total+=tbmu.ct[cid];
				}
				/*ct_max=(ct_max>tbmu.ct_1)?ct_max:tbmu.ct_1;
				ct_max=(ct_max>tbmu.ct_2)?ct_max:tbmu.ct_2;
				ct_max=(ct_max>tbmu.ct_3)?ct_max:tbmu.ct_3;
				ct_min=(ct_min<tbmu.ct_1)?ct_min:tbmu.ct_1;
				ct_min=(ct_min<tbmu.ct_2)?ct_min:tbmu.ct_2;
				ct_min=(ct_min<tbmu.ct_3)?ct_min:tbmu.ct_3;*/
				
			}else{
				(bmInfo.ptBMU+bmuid)->alive=0;
			}
		}
		bmInfo.m_cv_max=cv_max;
		bmInfo.m_cv_min=cv_min;
		bmInfo.m_cv_avg=cv_total/(bmInfo.bmu_total*BMU_CV_NUM);
		bmInfo.m_ct_avg=ct_total/(bmInfo.bmu_total*BMU_CT_NUM);
		bmInfo.m_ct_max=(INT16)(ct_max*10);
		bmInfo.m_ct_min=(INT16)(ct_min*10);
		bmInfo.m_cvmin_cid=cv_min_cid;
		bmInfo.m_cvmax_cid=cv_max_cid;
		bmInfo.m_cvmin_mid=cv_min_mid;
		bmInfo.m_cvmax_mid=cv_max_mid;
		bmInfo.m_ctmin_cid=ct_min_cid;
		bmInfo.m_ctmax_cid=ct_max_cid;
		bmInfo.m_ctmin_mid=ct_min_mid;
		bmInfo.m_ctmax_mid=ct_max_mid;
		
		loseBMU_tm=OSTimeGet();
		bmInfo.bmu_alive_num=aliveNum;
	}
	bmInfo.bmu_ready_num=readyNum;
	bmInfo.bmu_on_num=onNum;
	bmInfo.bmu_off_num=offNum;
	
	bmInfo.mod_curr=Ibmu;
	if (bmInfo.mod_curr>0)
		bmInfo.currentDirection=1; //charge
	else
		bmInfo.currentDirection=0; //discharge
		
	if(bmInfo.bmu_total>0){
		//nie bmInfo.mod_soc=soc/bmInfo.bmu_total;
		bmInfo.mod_soh=soh/bmInfo.bmu_total;
		bmInfo.mod_clc=cc_max/10;
		bmInfo.mod_cld=dc_max/10;
		bmInfo.mod_curr=Ibmu;
	}
	bmInfo.m_err.err_m_uv=1;
	if(Ibmu>=0){	//charge
		bmInfo.mod_life_ahr_c+=Ibmu;
		bmInfo.mod_life_whr_c+=((UNS64)Ibmu*vbmu)/1000;
	}else{
		bmInfo.mod_life_ahr_d+=(UNS16)(-Ibmu);
		bmInfo.mod_life_whr_d+=((UNS64)(-Ibmu)*vbmu)/1000;
	
	}

	
	memcpy((void *)&bpInfo.mod[0],(void *)&bmInfo,sizeof(bmInfo));
	bpInfo.extAddr=0xA9;
	bpInfo.mod_num=1;
	bpInfo.sc_hw_version[0]=1; 
	bpInfo.sc_hw_version[1]=2; 
	bpInfo.sc_hw_version[2]=3; 

	soc=0;soh=0;Ibmu=0;vbmu=0;cc_max=0;dc_max=0;
	for(mid=0;mid<bpInfo.mod_num;mid++){
		soc+=bpInfo.mod[mid].mod_soc;
		soh+=bpInfo.mod[mid].mod_soh;
		Ibmu+=bpInfo.mod[mid].mod_curr;
		cc_max+=bpInfo.mod[mid].mod_clc;
		dc_max=bpInfo.mod[mid].mod_cld;
		if(bpInfo.mod[mid].mod_volt>vbmu) vbmu=bpInfo.mod[mid].mod_volt;
		cap+=(bpInfo.mod[mid].mod_life_whr_c+bpInfo.mod[mid].mod_life_whr_d)/3600000;
	}
	if(bpInfo.mod_num>0){
		bpInfo.bp_soc=soc/bpInfo.mod_num;
		bpInfo.bp_soh=soh/bpInfo.mod_num;
		bpInfo.bp_curr=Ibmu;
		bpInfo.bp_VBus=vbmu;
		bpInfo.bp_clc=cc_max;
		bpInfo.bp_cld=dc_max;
		bpInfo.bp_lifw_wh=cap;
	}
}
	
void bmuInit(){
	uint8_t bid;
	for(bid=0;bid<MAX_BMU_NUM;bid++){
		memset(&bmu[bid],0,sizeof(BMU_INFO));
	}
}
void updateBMSoc(BATT_MODULE_INFO * bm,UNS16 interval_ms){
	
	
	//interval: 100ms
//	UINT16 updateSOC(UINT8 interval_ms){
		UINT16 totalSOC=0;
		UINT8 packNo,mid,cid;
		UINT8 bleedingCellNum;
		UINT16 bleedingEnergyLose;	//0.001VAS
		INT16 I_last_loop_5min,I_last_loop_1min,I_last_loop_30sec,I_last_loop_2sec; //current at 5min, 1min,30sec ago
		//average current at last 5min,1min,30sec. flow out:>0, flow in:<0
	//	static TICK currCbeginTick[STR_NUM_IN_CAB];
		UINT8 interval;
		static UINT8 SOC_OCV_AjuestAllowed=TRUE;
		interval=interval_ms/100;		//interVal_ms unit is ms, so all calculation based on 0.1S (100ms)

	//Added  by Amanda for record balance time per key cycle
	//	static UINT64 UpdateLoopCount=0;	//add one per 0.1s
	
	//Added  by Amanda for record balance time per key cycle
	//	UpdateLoopCount++;
	//	interval=interval_ms/100;		//interVal_ms unit is ms, so all calculation based on 0.1S (100ms)
	
	//temperatary	updateBatteryFullCapcity();
	
#if 0
		//update the value in currentHistory[][] and CAP_LOSE_x_min
		if(currRecordPos >= CURRENT_SAVE_COUNT) currRecordPos=0;
		for(packNo=0;packNo<STR_NUM_IN_CAB;packNo++){
	
			I_last_loop_5min=currentHistory[packNo][currRecordPos]; //get current last loop
			if(currRecordPos>=600) I_last_loop_1min=currentHistory[packNo][currRecordPos-600];
			else I_last_loop_1min=currentHistory[packNo][CURRENT_SAVE_COUNT+currRecordPos-600];
			if(currRecordPos>=300) I_last_loop_30sec=currentHistory[packNo][currRecordPos-300];
			else I_last_loop_30sec=currentHistory[packNo][CURRENT_SAVE_COUNT+currRecordPos-300];
			if(currRecordPos>=20) I_last_loop_2sec=currentHistory[packNo][currRecordPos-20];
			else I_last_loop_2sec=currentHistory[packNo][CURRENT_SAVE_COUNT+currRecordPos-20];
	
	
				if(I_last_loop_5min>0) CAP_lose_5_min[packNo] -= I_last_loop_5min;
				else CAP_gain_5_min[packNo]  += I_last_loop_5min;
				if(I_last_loop_1min>0) CAP_lose_1_min[packNo] -= I_last_loop_1min;
				else CAP_gain_1_min[packNo]  += I_last_loop_1min;
				if(I_last_loop_30sec>0) CAP_lose_30_sec[packNo] -= I_last_loop_30sec;
				else CAP_gain_30_sec[packNo]  += I_last_loop_30sec;
				if(I_last_loop_2sec>0) CAP_lose_2_sec[packNo] -= I_last_loop_2sec;
				else CAP_gain_2_sec[packNo]  += I_last_loop_2sec;
			if(bm->mod_currDirection>0){
				currentHistory[packNo][currRecordPos]=bm->mod_curr;
				Inow=bm->mod_curr;
				CAP_lose_5_min[packNo] += bm->mod_curr;
				CAP_lose_1_min[packNo] += bm->mod_curr;
				CAP_lose_30_sec[packNo] += bm->mod_curr;
				CAP_lose_2_sec[packNo] += bm->mod_curr;
			}else{
				currentHistory[packNo][currRecordPos]=-bm->mod_curr;
				Inow=-bm->mod_curr;
				CAP_gain_5_min[packNo] += bm->mod_curr;
				CAP_gain_1_min[packNo] += bm->mod_curr;
				CAP_gain_30_sec[packNo] += bm->mod_curr;
				CAP_gain_2_sec[packNo] += bm->mod_curr;
			}
				
				Iavg_d_5min[packNo]=CAP_lose_5_min[packNo]/3000;
				Iavg_d_1min[packNo]=CAP_lose_1_min[packNo]/600;
				Iavg_d_30sec[packNo]=CAP_lose_30_sec[packNo]/300;
				Iavg_d_2sec[packNo]=CAP_lose_2_sec[packNo]/20;
				Iavg_c_5min[packNo]=(CAP_gain_5_min[packNo]/3000);
				Iavg_c_1min[packNo]=(CAP_gain_1_min[packNo]/600);
				Iavg_c_30sec[packNo]=(CAP_gain_30_sec[packNo]/300);
				Iavg_c_2sec[packNo]=(CAP_gain_2_sec[packNo]/20);
	
			capaChange_5min=capaChange_5min+currentHistory[packNo][currRecordPos]-I_last_loop_5min;
			capaChange_1min=capaChange_1min+currentHistory[packNo][currRecordPos]-I_last_loop_1min;
			capaChange_30sec=capaChange_30sec+currentHistory[packNo][currRecordPos]-I_last_loop_30sec;
			capaChange_2sec=capaChange_2sec+currentHistory[packNo][currRecordPos]-I_last_loop_2sec;
			Iavg_5min=capaChange_5min/3000;
			Iavg_1min=capaChange_1min/600;
			Iavg_30sec=capaChange_30sec/300;
			Iavg_2sec=capaChange_2sec/20;
			
			
		}
		currRecordPos++;	//next write point
	
	
	
	
					//take account in  bleeding energy 
					bleedingCellNum=0;
					for(mid=0;mid<SERIAL_BOARD_NUM;mid++){
						for(cid=0;cid<CELL_NUM_PER_MODULE;cid++){
							if(sstrinfo[0].cellBleedingFalg[mid]>>cid)
							bleedingCellNum++;
						}
					}
	
#endif
			
			/*battery in discharge status. or idle status(in this situation, maybe have current between two packs*/
			if(1
				&& (abso(bm->mod_curr)>10) ////flow out and I>1A (for test mistake)
				&&(isDelayExp(SECONDS(5),sysInfo.sysStartTime))
				
			){ 
				UINT32 capaDiff=0;	// pack capacity change 
				if(bm->currentDirection==0){		//discharge
					capaDiff=(UINT32)(-bm->mod_curr/100)*interval;
					bm->capacityLose += capaDiff;
					bm->capacityLoseTotal+= capaDiff;
					bm->capacity-=capaDiff;
					if(bm->capacity >bm->fullCapacity) bm->capacity=0; //capacity < 0
				//	eeprom_ram_para.sstrCapacity[packNo]=bm->capacity; //save a copy in eeprom
					
					// for record Energy lost to balance and power
					bleedingEnergyLose=(UINT32)bleedingCellNum*38*38*interval/10;	//0.001VAS
					
				//nie	bm->BalancePower=bleedingEnergyLose/interval;//0.001watt
				//nie	bm->EngLostToBalance+=bleedingEnergyLose/10;//0.01VAS
	
					bm->powerLose += (UINT32)(-bm->mod_curr/100)*bm->mod_volt/100*interval;
					bm->powerLose +=bleedingEnergyLose;	//0.001VAS
	
					bm->powerLoseTotal+= (UINT64)(bm->mod_curr/100)*bm->mod_volt/100*interval;
					bm->powerLoseTotal+= bleedingEnergyLose;	//0.001VAS
					
	
					
				}else{		//charge
					capaDiff=(UINT32)(abs(bm->mod_curr)/100) *interval;
					bm->capacityGain+=capaDiff;
					bm->capacityGainTotal+=capaDiff;
					bm->capacity=bm->capacity + capaDiff;
					if(bm->capacity >bm->fullCapacity) bm->capacity=bm->fullCapacity; 
	
					bm->powerGain+=(UINT64)bm->mod_curr/100*bm->mod_volt/100*interval;
					bm->powerGainTotal+=(UINT64)bm->mod_curr/100*bm->mod_volt/100*interval;
				}
				bm->mod_soc=(UNS16)((UINT64)bm->capacity*1000/bm->fullCapacity);
				#if 0 //nie
				//save in eeprom
				eeprom_ram_para.capacityGainTotal[STR_NUM_IN_CAB]=bm->capacityGainTotal;
				eeprom_ram_para.capacityLoseTotal[STR_NUM_IN_CAB]=bm->capacityLoseTotal;
				//kwallt_dischg
				eeprom_ram_para.KWattHrs_Chg[STR_NUM_IN_CAB]=bm->powerGainTotal/360000000;
				//DisChgAH
				eeprom_ram_para.KWattHrs_DisChg[STR_NUM_IN_CAB]=bm->powerLoseTotal/360000000;
				//DisChgAH
				eeprom_ram_para.DisChgAH[STR_NUM_IN_CAB]=bm->capacityLoseTotal/36000;
				//chgAH 
				eeprom_ram_para.ChgAH[STR_NUM_IN_CAB]=(UINT64)bm->capacityGainTotal/36000;
				#endif
			}
	

				//SOC-OCV calibrate
				if((bmInfo.bmu_alive_num==bmInfo.bmu_total)
		//		&&(systemInfo.CVErrCnt==0)
		//		&&(systemInfo.CTErrCnt==0)
		//		&&(systemStatus!=SYS_WAIT)
		//		&&(systemStatus != SYS_ALL_PACK_CHG)
				 &&(isDelayExp(SECONDS(4),sysInfo.sysStartTime))
				 &&(!isDelayExp(SECONDS(10),sysInfo.sysStartTime))
				 &&(bmInfo.SOC_OCV_adjusted==0)
				 &&(abso(bmInfo.mod_curr)<10)
				 &&(bmInfo.m_cv_min >0) 
				 &&(bmInfo.m_cv_min<5000)
				 &&(bmInfo.m_cv_max<5000)
				){
						UINT16 soc;
					//nie	systemInfo.HVBBattSOCAdjCriMetOCV=1;
						soc=SOC_OCV_Cal(&bmInfo,bmInfo.mod_soc);
						bm->mod_soc=soc;
						SOC_OCV_AjuestAllowed=FALSE;
						bm->SOC_OCV_adjusted=1;
						bm->capacity=((UINT64)(bm->mod_soc)*bm->fullCapacity)/1000;
				}
	
	
	#if 0
		//update SYSTEMINFO
		UINT32 cg=0,cl=0,cgl=0,cll=0;
		UINT64 pg=0,pl=0,pgl=0,pll=0;
				cg+=bm->capacityGain;
				cl+=bm->capacityLose;
				cgl+=bm->capacityGainTotal;
				cll+=bm->capacityLoseTotal;
				pg+=bm->powerGain;
				pgl+=bm->powerGainTotal;
				pl+=bm->powerLose;
				pll+=bm->powerLoseTotal;
				bm->SOC=((UINT64)(bm->capacity)*1000/bm->fullCapacity);
			systemInfo.powerGain=pg;
			systemInfo.powerGainTotal=pgl;
			systemInfo.powerLose=pl;
			systemInfo.powerLoseTotal=pll;
			systemInfo.capacityGain=cg;
			systemInfo.capacityLose=cl;
			systemInfo.capacityGainTotal=cgl;
			systemInfo.capacityLoseTotal=cll;
			
			eeprom_ram_para.syscapacityGainTotal=systemInfo.capacityGainTotal;
			eeprom_ram_para.syscapacityLoseTotal=systemInfo.capacityLoseTotal;
			//kwallt_dischg
			eeprom_ram_para.sysKWattHrs_Chg=systemInfo.powerGainTotal/360000000;
			//DisChgAH
			eeprom_ram_para.sysKWattHrs_DisChg=systemInfo.powerLoseTotal/360000000;
	
			//DisChgAH
			eeprom_ram_para.sysDisChgAH=systemInfo.capacityLoseTotal/36000;
			//chgAH 
			eeprom_ram_para.sysChgAH=(UINT64)systemInfo.capacityGainTotal/36000;
	
	
			if(systemInfo.currentSensorValidity==V_VALID){
				int currentDirection;
				if(systemInfo.currentDirection) currentDirection=1;
				else currentDirection=-1;
	
				if(systemInfo.voltBatteryValidity==V_VALID) systemInfo.power=(INT64)currentDirection*systemInfo.voltBattery*systemInfo.current/100;
				else if(HVBMsg.HVBBusVoltageValidity==V_VALID) systemInfo.power=(INT64)currentDirection*systemInfo.voltBus*systemInfo.current/100;
				else systemInfo.power=0;
			}else{
				systemInfo.power=0;
			}
	
	
		 
	
	
	
		totalSOC=0;
		for(packNo=0;packNo<STR_NUM_IN_CAB;packNo++){
			if(isACPresent(0)){
				totalSOC += bm->SOC;
			}else{
				totalSOC += bm->SOC;
			}
			eeprom_ram_para.sstrSOC[packNo]=bm->SOC;
		}
		systemInfo.SOC =totalSOC/STR_NUM_IN_CAB;
		uiBattTaskStage=206;
	#endif
	//	return 1;
	
	}
	
	



UINT16 SOC_OCV_Cal(BATT_MODULE_INFO *bm,UINT16 savedSOC){
	UINT16 minSOC, maxSOC, avgSOC,deltaMinSOC, deltaMaxSOC, blendedSOC,retSOC,ampSOC;
	UINT8 socadj;

	
	minSOC=SOC_OCV_Lookup(bm->m_ct_min/10,bm->m_cv_min);
	maxSOC=SOC_OCV_Lookup(bm->m_ct_min/10,bm->m_cv_max);
	avgSOC=SOC_OCV_Lookup(bm->m_ct_min/10,bm->m_cv_max);
	
	//cal SOCamp
		deltaMinSOC=avgSOC-minSOC;
		deltaMaxSOC=maxSOC-avgSOC;
		ampSOC=avgSOC;
	
	//cal SOCblended
	if(ampSOC<SOC_CAL_LOW){
		if(ampSOC>deltaMinSOC) blendedSOC=ampSOC-deltaMinSOC;
		else blendedSOC=0;
	
	}else if(ampSOC>SOC_CAL_HIGH){
		blendedSOC=ampSOC+deltaMaxSOC;
		if(blendedSOC>1000) blendedSOC=1000;
	}else{
		blendedSOC=(UINT32)(ampSOC+deltaMaxSOC)*(ampSOC-SOC_CAL_LOW)/(SOC_CAL_HIGH-SOC_CAL_LOW);
		blendedSOC+=(UINT32)(ampSOC-deltaMinSOC)*(SOC_CAL_HIGH - ampSOC)/(SOC_CAL_HIGH-SOC_CAL_LOW);
	}

	retSOC=savedSOC;
	socadj=1;
	if(blendedSOC>=(savedSOC+250)){
		retSOC=blendedSOC;
//nie		bm->AHrAdjTgt=((INT32)(systemInfo.fullCapacity/1000)*(blendedSOC - savedSOC));	//AHrAdj is 0.01A.S, fullCapacity is A.S
//nie		bm->AHrAdjLeft=0;
//		sstrinfo[0].HVBBattSOCAdj=1;
	}else if(blendedSOC>=(savedSOC+100)){
		retSOC=blendedSOC;
//nie		bm->AHrAdjTgt=((INT32)(systemInfo.fullCapacity/1000)*(blendedSOC - savedSOC));	//AHrAdj is 0.01A.S, fullCapacity is A.S
//nie		bm->AHrAdjLeft=0;
/* from 11.8.2    not adjust SOC if target SOC big than exist SOC and difference lower than 25%
		systemInfo.AHrAdjTgt=((INT32)(systemInfo.fullCapacity/1000)*(blendedSOC - savedSOC));	//AHrAdj is 0.01A.S, fullCapacity is A.S
		systemInfo.AHrAdjLeft=systemInfo.AHrAdjTgt;
		systemInfo.HVBBattSOCAdj=1;
*/
//10.51.5	}else if(blendedSOC>=(savedSOC+50)){
	}else if(blendedSOC>=(savedSOC+80)){
/* from 11.8.2    not adjust SOC if target SOC big than exist SOC and difference lower than 25%
		systemInfo.AHrAdjTgt=((INT32)(systemInfo.fullCapacity/1000)*(blendedSOC - savedSOC));	//AHrAdj is 0.01A.S, fullCapacity is A.S
		systemInfo.AHrAdjLeft=systemInfo.AHrAdjTgt;
		systemInfo.HVBBattSOCAdj=0;
*/
	}else if(savedSOC >= (blendedSOC+250)){
//nie		bm->AHrAdjTgt=((INT32)(systemInfo.fullCapacity/1000)*(savedSOC- blendedSOC));	//AHrAdj is 0.01A.S, fullCapacity is A.S
//nie		bm->AHrAdjLeft=0;
		retSOC=blendedSOC;
//nie		systemInfo.HVBBattSOCAdj=1;
//10.51.5	}else if(savedSOC>=(blendedSOC+50)){
	}else if(savedSOC>=(blendedSOC+80)){
		retSOC=blendedSOC;
	//nei	bm->AHrAdjTgt=((INT32)(systemInfo.fullCapacity/1000)*(savedSOC - blendedSOC));	//AHrAdj is 0.01A.S, fullCapacity is A.S
	//nie	bm->AHrAdjLeft=0;

/*		systemInfo.AHrAdjTgt=-((INT32)(systemInfo.fullCapacity/1000)*(savedSOC - blendedSOC));	//AHrAdj is 0.01A.S, fullCapacity is A.S
		systemInfo.AHrAdjLeft=systemInfo.AHrAdjTgt;
		systemInfo.HVBBattSOCAdj=0;
*/
	}else{
		socadj=0;
		bm->AHrAdjTgt=0;
		bm->AHrAdjLeft=0;
//		bm->HVBBattSOCAdj=0;
	}



	retSOC=minSOC;	// to be sample
	
	return retSOC;
	
}

/*
	UINT16 current : 0.1A
	UINT8 currentDirection: 0 - in, 1- out
	INT8 temp: -40 to 85
	unsigned int Vmin: mv
*/
UINT16 SOC_OCV_Lookup(INT8 temp,UINT16 Vcell)
{
	int intSOC;
	#define TEMP_N40_LOW	0
	#define TEMP_N40_N30	1
	#define TEMP_N30_N20	2
	

	UINT16 SOCTable[11]={3500,3670,3740,3810,3880,3930,4000,4050,4100,4140,4200};
 	int i,tem,j,k,index_table_i,index_table_j,index_table_k;
 	int count=0;
	float fSOC;
	UINT16 I;
	UINT8   Idir;
	UINT16 * socTbl_1, *socTbl_2, *socTbl;

	if(temp<-40) {
		SOCTable[0]=3320;SOCTable[1]=3470;SOCTable[2]=3570;SOCTable[3]=3620;SOCTable[4]=3650;
		SOCTable[5]=3690;SOCTable[6]=3730;SOCTable[7]=3790;SOCTable[8]=3860;SOCTable[9]=3940;SOCTable[10]=4050;
	}else if(temp<-30){
		SOCTable[0]=3320;SOCTable[1]=3470;SOCTable[2]=3570;SOCTable[3]=3620;SOCTable[4]=3650;
		SOCTable[5]=3690;SOCTable[6]=3730;SOCTable[7]=3790;SOCTable[8]=3860;SOCTable[9]=3940;SOCTable[10]=4050;
	}else if(temp<5){
		SOCTable[0]=3320;SOCTable[1]=3470;SOCTable[2]=3570;SOCTable[3]=3620;SOCTable[4]=3650;
		SOCTable[5]=3690;SOCTable[6]=3730;SOCTable[7]=3790;SOCTable[8]=3860;SOCTable[9]=3940;SOCTable[10]=4050;
	}else if(temp<40){
		SOCTable[0]=3320;SOCTable[1]=3470;SOCTable[2]=3570;SOCTable[3]=3620;SOCTable[4]=3650;
		SOCTable[5]=3690;SOCTable[6]=3730;SOCTable[7]=3790;SOCTable[8]=3860;SOCTable[9]=3940;SOCTable[10]=4050;
	}else if(temp<50){
		SOCTable[0]=3320;SOCTable[1]=3470;SOCTable[2]=3570;SOCTable[3]=3620;SOCTable[4]=3650;
		SOCTable[5]=3690;SOCTable[6]=3730;SOCTable[7]=3790;SOCTable[8]=3860;SOCTable[9]=3940;SOCTable[10]=4050;
	}else if(temp<60){
		SOCTable[0]=3320;SOCTable[1]=3470;SOCTable[2]=3570;SOCTable[3]=3620;SOCTable[4]=3650;
		SOCTable[5]=3690;SOCTable[6]=3730;SOCTable[7]=3790;SOCTable[8]=3860;SOCTable[9]=3940;SOCTable[10]=4050;
	}
		

  i=0;  j=10;
  if (Vcell>SOCTable[10]) {
     	intSOC=100;
  }else  if (Vcell<SOCTable[0]) {
  	intSOC=0;
  }else{
	  for (;;){
		    count++;
		    index_table_i=i;
		    index_table_j=j;
		    index_table_k=(index_table_i+index_table_j)/2;
		    k=index_table_k;
		    if (j-i<=1) {
		      	tem=i*10+(float)(Vcell-SOCTable[index_table_i])/(float)(SOCTable[index_table_j]-SOCTable[index_table_i])*10;
				intSOC=tem;
				break;
	          }else{
			    if  (Vcell>SOCTable[index_table_k])      i=k;
			    else if (Vcell<SOCTable[index_table_k])      j=k;
			    else{
					intSOC=index_table_k*10;
					break;
				}

		    }  
	    }
   }
	return intSOC*10;
		
}
//new version power limit

void strPlimSet(BATT_MODULE_INFO *bm){

	#define PL_DERATE_MODE 1
	#define PL_NORMAL_MODE 0
// Minimum Charge Power at low SOC
#define MPchg 15000 /* minimum charge power at low SOC (W) */
static UINT8 plimcMode=PL_NORMAL_MODE;
static UINT8 plimdMode=PL_NORMAL_MODE;
	UINT16  PowLimC, PowLimD, contPowD, contPowC ;
	UINT16 plimcT = (UINT16)FPchg ;
	UINT16 plimcV =(UINT16) FPchg ;
	UINT16 plimcSOC = (UINT16)FPchg ;
	UINT16 plimdT =(UINT16) FPdis ;
	UINT16 plimdV = (UINT16)FPdis ;
	UINT16 plimdSOC = (UINT16)FPdis ;
	UINT16 CA,CB,CV,CP,CQ;	//some constant
	//static TICK cDerateBeginTick,cNomrolBeginTick;
	//static TICK dDerateBeginTick,dNomrolBeginTick;
	UINT16 NPL4,NPL3,NPL2,NPL1,NPL;
	UINT32 avgPow30sec,avgPow1_8sec;

//UINT16 plim[101];
INT8 i,j,tempSensorFailCount,voltSensorFailCount;
//nie	if((Iavg_d_5min[0]>600) ||(Iavg_d_1min[0]>750) ||(Iavg_d_30sec[0]>900)){	// Iavg_5min>60A or Iavg_1min>75A or Iavg_30sec>90A
if(1){
		CA=48; CB=25;
	}else{
		CA=60; CB=28;
	}
/*
for(i=0;i<=100;i++){plim[i]=0;}
systemInfo.cellVltMin=3000;
systemInfo.cellVltMax=3000;

for(i=0;i<=100;i++){
	plimcT = (UINT16)FPchg ;
	plimcV =(UINT16) FPchg ;
	plimcSOC = (UINT16)FPchg ;
	plimdT =(UINT16) FPdis ;
	plimdV = (UINT16)FPdis ;
	plimdSOC = (UINT16)FPdis ;

	systemInfo.cellVltMin+=i*10;
	systemInfo.cellVltMax+=i*10;
	systemInfo.SOC=(UINT16)i*10;
	systemInfo.current=100;
	systemInfo.cellTempMax=40;
	systemInfo.voltBattery=3700;

*/

/*power limit calibrated by cell voltage */
	//discharge power limit by cell voltage
	NPL4=FPdis;
		if((bm->m_cv_min) >3360){ 
			NPL4=FPdis;
		}else if(bm->m_cv_min>3200){
//			NPL4=FPdis -(UINT32)FPdis*(100-CA)*(3360-bm->m_cv_min)/16000;
			NPL4=FPdis -(UINT32)FPdis*(3360-bm->m_cv_min)/250;	//when Vcell_min is 3.2V, power limit change to 36% of full pl
		}else if(bm->m_cv_min>3150){
			NPL4=(UINT32)FPdis*(bm->m_cv_min - 3150)/150;
		}else NPL4=0;
	NPL1=FPdis;
	NPL2=FPdis;
#if 1

/* SOC calication */
	
	//discharge power limit
		if (bm->mod_soc  <=30){
			NPL3=0;
		}else if (bm->mod_soc  <=300){
			NPL3  =(UINT32) 35000-(UINT32)(30 - bm->mod_soc/10)*(30 - bm->mod_soc/10)*464/10; 
		}else if (bm->mod_soc  <=550){
			NPL3  =(UINT32) 35000+(UINT32)(bm->mod_soc/10-30)*(bm->mod_soc/10-30)*464/10; 
		}else if (bm->mod_soc  <= 1000) {
			NPL3=FPdis;
		}
#endif	
/* temperature calibration */		
NPL=NPL1;
if(NPL>NPL2) NPL=NPL2;
if(NPL>NPL3) NPL=NPL3;
if(NPL>NPL4) NPL=NPL4;

	if(bm->m_ct_max>ThighH){
		NPL=0;
	}else if(bm->m_ct_max>ThighS){
		NPL=(UINT32)NPL*(ThighH-bm->m_ct_max)/(ThighH-ThighS);
	}else if(bm->m_ct_max<25){
		if(bm->m_ct_min<-40){
		 	NPL=0;
		}else if(bm->m_ct_min<15){
	 		NPL=(UINT32)NPL*(bm->m_ct_min+40)*(bm->m_ct_min+40)/3025;
		}
	}


	PowLimD = NPL;


//charge power limit

/*power limit calibrated by cell voltage */
	plimcV=FPchg;

			if (bm->m_cv_max>4100) plimcV = 0 ;
			else if (bm->m_cv_max > 3950) {
//				plimcV=FPchg - (UINT32)(bm->m_cv_max-V40)/4; //*250/1000
#ifdef PRO_HATCH
				plimcV=FPchg - (UINT32)(bm->m_cv_max-3950)*120; //*250/1000
#else
				plimcV=FPchg - (UINT32)(bm->m_cv_max-3950)*100; //*250/1000
#endif
			}else if(bm->m_cv_max>3250){
//				plimcV=(UINT32)sstrinfo[strid].strVolt*13;
				//plimcV=(UINT32)bm->mod_volt*4;
				if(plimcV>FPchg) plimcV=FPchg;
					
			}else if(bm->m_cv_max<=3250){
			
				plimcV =FPchg - (UINT32)FPchg*(3250-bm->m_cv_max)/250;
				//nie if(plimcV>(UINT32)(bm->mod_volt)*13) plimcV=(UINT32)bm->mod_volt*13;
			}
#if 1
/* SOC calication */
	if (bm->mod_soc >900) plimcSOC = (UINT32)FPchg*(1000-bm->mod_soc)*3/550;
	
	else if (bm->mod_soc > 600) plimcSOC = (UINT32)FPchg*(1260-bm->mod_soc)/660;
	else if (bm->mod_soc  > 200) {
		plimcSOC = FPchg ;
	}else{
		plimcSOC=(UINT32)FPchg*(900 + bm->mod_soc)/1100;
	}
#endif

    plimcT=plimcV;
   if(plimcT>plimcSOC) plimcT=plimcSOC;
    
/* temperature calication */
	if(bm->m_ct_max>ThighH) PowLimC=0;
	else if(bm->m_ct_max>ThighS){
		PowLimC=(UINT32)plimcT*(ThighH-bm->m_ct_max)/(ThighH-ThighS); 
	}else if(bm->m_ct_max>15){
		PowLimC=plimcT;
	}else if(bm->m_ct_min>(-20)){
		PowLimC=(UINT32)plimcT*(bm->m_ct_max+20)*(bm->m_ct_max+20)/35/35;
	}else{
		PowLimC=0;
	}
		
	

//get temperature sensor fail count
tempSensorFailCount=0;
for(i=0;i<4;i++){
	for(j=0;j<24;j++){
//		if(!((sstrinfo[0].cellTmpValidity[i]>>j) & 0x01)) tempSensorFailCount++;
	}
}
voltSensorFailCount++;
for(i=0;i<4;i++){
	for(j=0;j<24;j++){
//		if(!((sstrinfo[0].cellVolValidity[i]>>j) & 0x01)) voltSensorFailCount++;
	}
}

#if 0
	if(systemInfo.bmsDebugMode==1){
		if((PowLimD>PowLimD_Last[strid]) && (sstrinfo[strid].current>20) && (sstrinfo[strid].currentDirection ==1)){
			PowLimD=PowLimD_Last[strid];
		}else{
			PowLimD_Last[strid]=PowLimD;
		}
		if((PowLimC>PowLimC_Last[strid]) && (sstrinfo[strid].current<(-20))  && (sstrinfo[strid].currentDirection ==0)){
			PowLimC=PowLimC_Last[strid];
		}else{
			PowLimC_Last[strid]=PowLimC;
		}

	}
#endif
	
	bm->mod_pld=PowLimD;
	bm->mod_plc=PowLimC;



	
}

