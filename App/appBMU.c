
#include "bmu.h"
#include "main.h"
#include "can.h"
#include "appBMU.h"
#include "Task_main.h"
#include "Task_sysCheck.h"
parseBroadMsg(Message *m, UNS8 nodeId);

BMU_INFO bmu[MAX_BMU_NUM];
parseBroadMsg(Message *m, UNS8 nodeId){
	UNS16 idx;
	UNS8 subIdx,dcnt;
	UNS32 da;
	UNS8 ds[8];
	static int32_t last_curr,now_curr;
	void *d;
	UNS8 nid;
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
			bmu[nodeId-1].SOC=da;
			break;
		case 0x6D02:	
			memcpy((void *)&bmu[nodeId-1].SOH,m->data+4,2);
			bmu[nodeId-1].SOH*=10;
			break;
		case 0x6D03:	
		 bmu[nodeId-1].alive=1;
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
			memcpy((void *)&bmu[nodeId-1].V_bmu,m->data+4,dcnt);
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
	UNS8 bmuid,mid,onNum=0,offNum=0,aliveNum=0,readyNum=0;
	UNS16 soc=0,soh=0,cc_max=0,dc_max=0;
	UNS32 vbmu=0,cap;
	int32_t Ibmu=0;
	BMU_INFO tbmu;
	static u32_t loseBMU_tm[TOTAL_BMU_NUM];
	//bmInfo.bmu_total=1;
	for(bmuid=0;bmuid<bmInfo.bmu_total;bmuid++){
		soc+=bmInfo.mod_soc=bmu[bmuid].SOC;
		soh+=bmInfo.mod_soh=bmu[bmuid].SOH;
		cc_max+=bmInfo.mod_clc=bmu[bmuid].cc_max/100;
		dc_max+=bmInfo.mod_cld=bmu[bmuid].dc_max/100;
		Ibmu+=bmInfo.mod_curr=bmu[bmuid].I_bmu;
		vbmu=bmu[bmuid].V_bmu;
		if((bmInfo.mod_volt<vbmu)||(bmInfo.mod_volt>0xFF000000)) bmInfo.mod_volt=vbmu;
		bmInfo.ptBMU=(BMU_INFO *)&bmu[0];
		
		tbmu=bmu[bmuid];
		

		if(tbmu.alive>0){	//received boradcast message
			(bmInfo.ptBMU+bmuid)->alive=0;
			aliveNum++;
			loseBMU_tm[bmuid]=OSTimeGet();
		}else if(timeDiff(loseBMU_tm[bmuid],OSTimeGet())<5000){
			aliveNum++;
		}
		
		if((tbmu.bmuSafeFlag_2.w_flag2)==0){	//chargerable or dischargable
				readyNum++;
			}else{
				offNum++;
			}
		if(((tbmu.bmuSafeFlag_1.w_flag1)& 0x1C0000)>0){	//charging/discharging/idle
			onNum++;
		}
	}
	bmInfo.bmu_alive_num=aliveNum;
	bmInfo.bmu_ready_num=readyNum;
	bmInfo.bmu_on_num=onNum;
	bmInfo.bmu_off_num=offNum;
	
	if(bmInfo.bmu_total>0){
		bmInfo.mod_soc=soc/bmInfo.bmu_total;
		bmInfo.mod_soh=soh/bmInfo.bmu_total;
		bmInfo.mod_clc=cc_max/100;
		bmInfo.mod_cld=dc_max/100;
		bmInfo.mod_curr=Ibmu;
	}
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
	

