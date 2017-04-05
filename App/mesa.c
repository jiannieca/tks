/***********************************************************************
***********************************************************************/
#include "main.h"
#include "def.h"
#include "mesa.h"
#include "task_main.h"
OS_STK Stk_Task_Mesa[TASK_MESA_STK_SIZE];
MESA_DATA_801 mesa_801;
MESA_DATA_802 mesa_802;
MESA_DATA_803 mesa_803;
extern void fillMbReg(uint16_t dstAddr,uint16_t * ptData,uint8_t len);

static void mesaVarInit(void);
static void mesaVarUpdata(void);
void Task_Mesa(void *pdata);
void Task_Mesa(void *pdata){
	int lp;
	mesaVarInit();
	while(1){
		OSTimeDly(100);//100ms
		
		mesaVarUpdata();
		lp++;
	}
}
static void mesaVarUpdata(void){
	mesa_803.BMinCellVol=bmInfo.m_cv_min;
	mesa_803.BMaxCellVol=bmInfo.m_cv_max;
	
	mesa_803.ID_803=803;		//		Lithium-Ion Battery Model	803 uint16			R	M		
	mesa_803.L_803=16; 	//			16	uint16			R	M	Model Length	
										
	mesa_803.BConStrCt=1; 	//		Connected String Count		uint16			R	M	Number of strings with contactor closed.	
	mesa_803.BMaxCellVol=bmInfo.m_cv_max;		//		Max Cell Voltage		uint16	V	BCellVol_SF R	M	Maximum voltage for all cells in the bank.	Measurement
	mesa_803.BMaxCellVolLoc=bmInfo.m_cvmax_mid*BMU_CV_NUM+bmInfo.m_cvmax_cid;		//		Max Cell Voltage Location		uint16			R	O	Location of the cell with maximum voltage	Bit0:Bit7 = String Number  Bit8:Bit15 = Module Number
	mesa_803.BMinCellVol=bmInfo.m_cv_min;		//		Min Cell Voltage		uint16	V	BCellVol_SF R	M	Minimum voltage for all cells in the bank.	Measurement
	mesa_803.BMinCellVolLoc=bmInfo.m_cvmin_mid*BMU_CV_NUM+bmInfo.m_cvmin_cid;		//		Min Cell Voltage Location		uint16			R	O	Location of the cell with minimum voltage	Bit0:Bit7 = String Number  Bit8:Bit15 = Module Number
	mesa_803.BMaxModTmp=bmInfo.m_ct_max; 	//		Max Module Temp 	int16		BModTmp_SF	R	M	Maximum temperature for all modules in the bank.	Measurement
	mesa_803.BMaxModTmpLoc=bmInfo.m_ctmax_mid*BMU_CT_NUM+bmInfo.m_ctmax_cid;; 	//		Max Module Temp Location		uint16			R	O	Location of the module with max temperature.	Bit0:Bit7 = String Number  Bit8:Bit15 = Module Number
	mesa_803.BMinModTmp=bmInfo.m_ct_min; 	//		Min Module Temp 	int16	C	BModTmp_SF	R	M	Minimum temperature for all modules in the bank.	Measurement
	mesa_803.BMinModTmpLoc=bmInfo.m_ctmin_mid*BMU_CT_NUM+bmInfo.m_ctmin_cid; 	//		Min Module Temp Location		uint16			R	O	Location of the module with min temperature.	Bit0:Bit7 = String Number  Bit8:Bit15 = Module Number
	mesa_803.BTotDCCur=bmInfo.mod_curr;		//		Total DC Current		int16	A	BCurrent_SF R	M	Total DC current flowing to/from the battery bank.	Measurement
	mesa_803.BMaxStrCur; 	//		Max String Current		int16	A	BCurrent_SF R	O	Maximum current of any string in the bank.	Measurement
	mesa_803.BMinStrCur; 	//		Min String Current		int16	A	BCurrent_SF R	O	Minimm current of any string in the bank.	Measurement
	mesa_803.BCellVol_SF=1000;		//				sunssf			R	M	Scale factor for cell voltage.	
	mesa_803.BModTmp_SF=1;		//				sunssf			R	M	Scale factor for module temperatures.	
	mesa_803.BCurrent_SF=100;		//				sunssf			R	M	Scale factor for Total DC Current, Max String Current and Min String Current.	
	mesa_803.StrSoH_SF=1; 	//				sunssf			R	O	Scale factor for String State of Health.	
										
	mesa_803.StrModCt=MAX_BMU_NUM;		//		Module Count		uint16			R	M	Count of modules in the string. 
	mesa_803.StrSoC=bmInfo.mod_soc;		//		String SoC		uint16	%		R	M	Battery string state of charge, expressed as a percentage.	Measurement
	mesa_803.StrSoH=bmInfo.mod_soh;		//		String SoH		uint16	%	StrSoH_SF	R	O	Battery string state of health, expressed as a percentage.	Measurement
	mesa_803.StrCur=bmInfo.mod_curr; 	//		String Current		int16	A	BCurrent_SF R	M	String current measurement. Measurement
	mesa_803.StrMaxCellVol=bmInfo.m_cv_max; 	//		Max Cell Voltage		uint16	V	BCellVol_SF R	M	Maximum voltage for all cells in the string.	Measurement
	mesa_803.StrMinCellVol=bmInfo.m_cv_min; 	//		Min Cell Voltage		uint16	V	BCellVol_SF R	M	Minimum voltage for all cells in the string.	Measurement
	mesa_803.StrCellVolLoc=(bmInfo.m_cvmin_mid<<8)+bmInfo.m_cvmax_mid; 	//		Max/Min Cell Voltage Loc		uint16			R	O	Location of maximum and minimum cell voltages.	Bit0:Bit7 = Max Module Num	Bit8:Bit15 = Min Module Num
	mesa_803.StrMaxModTmp=bmInfo.m_ct_max;		//		Max Module Temp 	int16	C	BModTmp_SF	R	M	Maximum temperature for all modules in the bank.	Measurement
	mesa_803.StrMinModTmp=bmInfo.m_ct_min;		//		Min Module Temp 	int16	C	BModTmp_SF	R	M	Minimum temperature for all modules in the bank.	Measurement
	mesa_803.StrModTmpLoc=(bmInfo.m_ctmin_mid<<8)+bmInfo.m_ctmax_mid;		//		Max/Min Mod Temp Loc		uint16			R	O	Location of maximum and minimum module temperatures.	Bit0:Bit7 = Max Module Num	Bit8:Bit15 = Min Module Num
	mesa_803.StrEvt1;		//		String Event 1; 	//			bitfield32			R	M	Alarms, warnings and status values.  Bit flags. 
	mesa_803.StrEvt2;		//		String Event 2		bitfield32			R	O	Alarms, warnings and status values.  Bit flags. Reserved for future use.
	mesa_803.StrConFail;		//		Connection Failure Reason		enum16			R	O		
	fillMbReg(70,(uint16_t *)&mesa_801,sizeof(mesa_801));
	fillMbReg(94,(uint16_t *)&mesa_802,sizeof(mesa_802));
	fillMbReg(116,(uint16_t *)&mesa_803,sizeof(mesa_803));
}
static void mesaVarInit(void){
	uint16_t tmp[40];
	uint8_t i;
	for(i=0;i<40;i++){tmp[i]=i;}
	memcpy((uint16_t *)&mesa_801,(uint16_t *)tmp,sizeof(mesa_801));
	memcpy((uint16_t *)&mesa_802,(uint16_t *)tmp,sizeof(mesa_802));
	memcpy((uint16_t *)&mesa_803,(uint16_t *)tmp,sizeof(mesa_803));
}
	
