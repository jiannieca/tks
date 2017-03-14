/***********************************************************************
***********************************************************************/
#include "main.h"
#include "def.h"
#include "bsp_can.h"
#include "Task_ADC.h"
#include "task_main.h"
extern	AD_RESULT ad_res;
	
	void parseCOLong(CO_Data * d,uint8_t nodeid);
	OS_STK Stk_Task_CAN1[TASK_TEST_CAN1_STK_SIZE];
	OS_STK Stk_Task_CO[TASK_TEST_CO_STK_SIZE];
	
	OS_EVENT  *sem_CAN1_rec_flag;			//CAN1接收完一桢数据信号量定义
/***********************************************************************
***********************************************************************/
	void Task_CAN1(void *pdata)
	{		 
		unsigned  char	os_err,i;
		Message RxMSG;
		sem_CAN1_rec_flag = OSSemCreate(1); //create a semphere. 

		while(1)
		{  
			OSSemPend(sem_CAN1_rec_flag,0,&os_err); 		//wait sem_CAN1_rec_flag being set in ISR
			if(can1_rec_flag == 1)							//check if received valid message
			{
				if(timeDiff(sysInfo.sysStartTime,OSTime)>50){

					if((CAN1RxMsg.IDE==0)&&(CAN1RxMsg.StdId>=0x580)){
						RxMSG.cob_id=CAN1RxMsg.StdId;
						RxMSG.len=CAN1RxMsg.DLC;
						RxMSG.rtr=CAN1RxMsg.RTR;
						for (i=0;i<8;i++){
							RxMSG.data[i]=CAN1RxMsg.Data[i];
						}
						canDispatch(CANOpenMasterObject, &(RxMSG));
					}
				}
			}
		}
	}
	void Task_CO(void *pdata){
		static uint32_t t_co;
		static uint8_t CO_tid=0;
		static uint32_t t_stat;
		uint32_t tnow;
		static uint32_t loop_cnt=0;
		uint8_t sndData[8];
		uint8_t *pData,*pt;
		uint8_t i,j,k;
		static uint8_t pollBmuNum=1;
		static uint8_t svrNodeId=0;
		UNS16 sdo_id;
		UNS8 bt[8];
		UNS16 wd[4];
		CanopenInit();		
		while(1){
	//			OSTimeDlyHMSM(0, 0, 0, 1000);//1000ms

		switch(CO_tid){
			case 0:
				CO_tid=2;
				break;
			case 1:
				OSTimeDly(100);//100ms
					setState(CANOpenMasterObject,Pre_operational);
					CO_tid+=1;
				break;
			case 2:
					setState(CANOpenMasterObject,Operational);
					CO_tid+=1;
				break;
			case 3:  //read long parameter
				OSTimeDly(100);//100ms
					// pollBmuNum=3;
					initSDOline (CANOpenMasterObject, 0, svrNodeId, 0x6E80, pollBmuNum,SDO_UPLOAD_IN_PROGRESS );
					CANOpenMasterObject->transfers[0].whoami=SDO_CLIENT;
					CANOpenMasterObject->transfers[0].Callback=(void *)parseCOLong;
					sndData[0]=0x40;sndData[1]=0x80;sndData[2]=0x6E;sndData[3]=pollBmuNum;
					sndData[4]=0x00;sndData[5]=0x00;sndData[6]=0x00;sndData[7]=0x00;
					pData=sndData;
					sendSDO (CANOpenMasterObject, SDO_CLIENT, svrNodeId,  pData);
					//if(pollBmuNum++>6) pollBmuNum=1;
					if(pollBmuNum++>5){
						pollBmuNum=3;
						svrNodeId++;
						if(svrNodeId>bmInfo.bmu_total) svrNodeId=1;
					}
					CO_tid+=1;
				break;
			case 4:  //read short parameter
						loop_cnt++;
						OSTimeDly(100);//100ms
						initSDOline (CANOpenMasterObject, 0, svrNodeId, 0x6024+loop_cnt%2, 0,SDO_UPLOAD_IN_PROGRESS );
						CANOpenMasterObject->transfers[0].whoami=SDO_CLIENT;
						CANOpenMasterObject->transfers[0].Callback=(void *)parseCOLong;
						sndData[0]=0x40;sndData[1]=0x24+loop_cnt%2;sndData[2]=0x60;sndData[3]=00;
						sndData[4]=0x00;sndData[5]=0x00;sndData[6]=0x00;sndData[7]=0x00;
						pData=sndData;
						sendSDO (CANOpenMasterObject, SDO_CLIENT, svrNodeId,  pData);
						CO_tid+=1;
					break;
			case 5:  //control BMU
					OSTimeDly(100);//1000ms
					if(bmInfo.bmuOnCmd==CMD_BMU_FET_ON){
						if((bmu[0].bmuSafeFlag_1.b_flag1.PF==1)
							||(bmu[0].bmuSafeFlag_1.b_flag1.PFAleady==1)
							||(bmu[1].bmuSafeFlag_1.b_flag1.PF==1)
							||(bmu[1].bmuSafeFlag_1.b_flag1.PFAleady==1)
						){
							sdo_id=0x6A03; //clear fault
							bmu[0].bmuSafeFlag_1.b_flag1.PF=0;
							bmu[0].bmuSafeFlag_1.b_flag1.PFAleady=0;
							bmu[1].bmuSafeFlag_1.b_flag1.PF=0;
							bmu[1].bmuSafeFlag_1.b_flag1.PFAleady=0;
						}else{
							sdo_id=0x6A04; //close FETs
						}
					}else if(bmInfo.bmuOnCmd==CMD_BMU_FET_OFF){
							sdo_id=0x6A05; // open FETs
					}
					if ((bmInfo.bmuOnCmd==CMD_BMU_SHUTDOWN) ||(bpInfo.masterCmd==CMD_BMU_SHUTDOWN)){
						
						sdo_id=0x6C04;   //shutdown system
					}
					
					initSDOline (CANOpenMasterObject, 0, svrNodeId, sdo_id, 0,SDO_UPLOAD_IN_PROGRESS );
					CANOpenMasterObject->transfers[0].whoami=SDO_CLIENT;
					CANOpenMasterObject->transfers[0].Callback=(void *)parseCOLong;
					//ver 1.01 use 0x40 sndData[0]=0x2F;sndData[1]=sdo_id & 0xFF;sndData[2]=(sdo_id>>8) & 0xFF;sndData[3]=00;
					sndData[0]=0x40;sndData[1]=sdo_id & 0xFF;sndData[2]=(sdo_id>>8) & 0xFF;sndData[3]=00;
					sndData[4]=0x01;sndData[5]=0x00;sndData[6]=0x00;sndData[7]=0x00;
					pData=sndData;
					sendSDO (CANOpenMasterObject, SDO_CLIENT, svrNodeId,  pData);
					CO_tid+=2;
				break;

			case 6:  //write parameters
				
				OSTimeDly(100);//1000ms
					//pollBmuNum=2;
					initSDOline (CANOpenMasterObject, 0, 1, 0x6E00, pollBmuNum,SDO_DOWNLOAD_IN_PROGRESS );
					CANOpenMasterObject->transfers[0].whoami=SDO_CLIENT;
					CANOpenMasterObject->transfers[0].count=80;
					bmu[0].DesignVoltage=32;
					bmu[0].PackID=1;
					bmu[0].ManufactureDate[0]='1';
					bmu[0].ManufactureDate[1]='2';
					bmu[0].ManufactureDate[2]='/';
					bmu[0].ManufactureDate[3]='1';
					bmu[0].ManufactureDate[4]='2';
					bmu[0].ManufactureDate[5]='/';
					bmu[0].ManufactureDate[6]='1';
					bmu[0].ManufactureDate[7]='2';
					bmu[0].SN=123;
					
					pt=(void *)&(bmu[0].FWVer);
					for(i=0;i<0x50;i++){
						CANOpenMasterObject->transfers[0].data[i]=*pt++;
					}
					CANOpenMasterObject->transfers[0].Callback=NULL;
					sndData[0]=0x21;sndData[1]=0x00;sndData[2]=0x6E;sndData[3]=pollBmuNum;
					sndData[4]=0x50;sndData[5]=0x00;sndData[6]=0x00;sndData[7]=0x00;
					pData=sndData;
					sendSDO (CANOpenMasterObject, SDO_CLIENT, 1,  pData);
					CO_tid+=1;
				break;
			case 7:
					/*send ADC  result*/
				wd[0]=ADC_RCVTab[0];wd[1]=ADC_RCVTab[1];wd[2]=ADC_RCVTab[2];wd[3]=ADC_RCVTab[3];
				CAN1_WriteData(0X100,(UNS8 *)wd,8,CAN_Id_Standard); 	
					CO_tid++;
					break;
			default:
					CO_tid=3;
				break;
		}
	}
	}
	void parseCOLong(CO_Data * d,uint8_t nodeid){
		uint32_t idx;
		uint8_t sidx;
		idx=d->transfers[0].index;
		sidx=d->transfers[0].subIndex;
		if(idx=0x6E80){
			switch(sidx){
				case 1:
					memcpy((void *)&(bmu[nodeid-1].gBatteryStatus),d->transfers[0].data,(uint32_t)d->transfers[0].offset);
					break;
				case 2:
					memcpy((void *)&(bmu[nodeid-1].FWVer),d->transfers[0].data,(uint32_t)d->transfers[0].offset);
					break;
				case 3:
					memcpy((void *)&(bmu[nodeid-1].Balancing),d->transfers[0].data,(uint32_t)d->transfers[0].offset);
					break;
				case 4:
					memcpy((void *)&(bmu[nodeid-1].PackVolt),d->transfers[0].data,(uint32_t)d->transfers[0].offset);
					break;
				case 5:
					memcpy((void *)&(bmu[nodeid-1].ct_1),d->transfers[0].data,(uint32_t)d->transfers[0].offset);					
					break;		
				case 6:
					memcpy((void *)&(bmu[nodeid-1].COCI),d->transfers[0].data,(uint32_t)d->transfers[0].offset);
					break;
				case 7:
					break;
				case 8:
					break;
				case 9:
					break;
				default:
					break;
			}
		}
	}

