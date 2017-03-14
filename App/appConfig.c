/***********************************************************************
***********************************************************************/
#include "main.h"  
#include "config.h"
#include "task_main.h"
void SCInit(void );
void BMInit(void);
void SysVarInit(void);
void BPInit(void);



/***********************************************************************
 ***********************************************************************/
void SCInit(void){
	BPInit();
	//BMinit must after BPInit
	BMInit();
	SysVarInit();
	vcuInfo.keep_alive_req=VCMD_NA;
	sysInfo.sysStartTime=OSTime;
}
/***********************************************************************
function: fetch battery module infromation from NVM
 ***********************************************************************/
void BMInit(void){
	UNS16 bmsize;
	
	bmsize=sizeof(bmInfo);
	df_read_open(ADDR_BM_IN_NVM);
	SPI_FLASH_CS_HIGH();
	df_read((UNS8 *)&bmInfo,sizeof(bmInfo));
	bmInfo.Inter_Role=ARTRIBITION;	//always set to slave node when start
	bmInfo.intAddr=0xA8;
	if(bmInfo.mod_sn>0xFFFFFF) bmInfo.mod_sn=DEFAULT_MOD_SN;
	if(bmInfo.mod_soc>1000) bmInfo.mod_soc=1000;
	if(bmInfo.mod_soh>1000) bmInfo.mod_soc=1000;
	/*get 32bit random number */		 
//	bmInfo.rand_num=rand();
	if(bmInfo.mod_life_ahr_c>0xFFFFFFFFFFFFFFF) bmInfo.mod_life_ahr_c=0;
	if(bmInfo.mod_life_ahr_d>0xFFFFFFFFFFFFFFF) bmInfo.mod_life_ahr_d=0;
	
	if(bmInfo.mod_life_whr_c>0xFFFFFFFFFFFFFFF) bmInfo.mod_life_whr_c=0;
	if(bmInfo.mod_life_whr_d>0xFFFFFFFFFFFFFFF) bmInfo.mod_life_whr_d=0;
	bmInfo.rand_num=RNG_GetRandomNumber();
	memcpy((void *)&bpInfo.mod[0],(void *)&bmInfo,sizeof(bmInfo));
		SPI_FLASH_CS_HIGH();
		bmInfo.f_bmu_lost=1;
		bmInfo.bmuOnCmd=0;
		bmInfo.mod_fw_ver_major=BP_FW_VER_MAJOR;
		bmInfo.mod_fw_ver_minor=BP_FW_VER_MINOR;
		bmInfo.mod_fw_ver_patch=BP_FW_VER_PATCH;

}
void BPInit(void){
	UNS16 bpsize;
	bpsize=sizeof(bpInfo);
		SPI_FLASH_CS_HIGH();

	df_read_open(ADDR_BP_IN_NVM);
	df_read((UNS8 *)&bpInfo,sizeof(bpInfo));
	bpInfo.w_idle=0;
	bpInfo.mod_num=1;
	SPI_FLASH_CS_HIGH();
	bpInfo.sc_fw_ver_major=BP_FW_VER_MAJOR;
	bpInfo.sc_fw_ver_minor=BP_FW_VER_MINOR;
	bpInfo.sc_fw_ver_patch=BP_FW_VER_PATCH;

}
void SysVarInit(void){
	UNS16 bsyssize;
	bsyssize=sizeof(sysInfo);
	SPI_FLASH_CS_HIGH();
	df_read_open(ADDR_SYS_IN_NVM);
	df_read((UNS8 *)&sysInfo,sizeof(sysInfo));
	SPI_FLASH_CS_HIGH();
	sysInfo.f_alarm=0;
	sysInfo.f_vcu_lost=0;
	sysInfo.f_fault=0;
	sysInfo.sysResetCnt+=1;

}

