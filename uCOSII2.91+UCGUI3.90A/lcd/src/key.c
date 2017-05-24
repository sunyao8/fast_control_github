/* Includes ------------------------------------------------------------------*/
#include <includes.h>

/* Private variables ---------------------------------------------------------*/	
u8 Work_num=0;	  //   设置参数页面数
u8 Display_num=1;	  //正常显示页面数
u8 L1_L2_L3_COS=1;
u8 L1_L2_L3_KAR=1;
 u8 COMMCAT_para=0;
u8 CAPA_num=1,capa1_value,capa2_value;

//#endif
/*************************************/
//#endif
extern u8 Work_Flag;//设置界面的闪烁标志位
extern u16 dianya_zhi;
extern u8 hguestnum,gonglvshishu;
extern u32 idle_time,scan_time,dianliuzhi;
extern u16 wugongkvar,allkvar;
extern float32_t HV,HI,A_HV,B_HV,C_HV,A_HI,B_HI,C_HI;
extern s8 L_C_flag;
extern u16 dianya_zhi_A,dianya_zhi_B,dianya_zhi_C,wugongkvar_A,wugongkvar_B,wugongkvar_C,allkvar_A,allkvar_B,allkvar_C;
extern u32	dianliuzhi_A,dianliuzhi_B,dianliuzhi_C;
extern u8 gonglvshishu_A,gonglvshishu_B,gonglvshishu_C;
extern void LIGHT_backligt_on(void);
extern void LIGHT_backligt_off(void);
extern u8 phase_flag;
extern u8 L_C_flag_A;//感性容性标准变量
extern u8 L_C_flag_B;
extern u8 L_C_flag_C;
extern u8 light_time;


extern   status_comm_node comm_list[34];

//////////////////////////////////////////////////////////////////////////////////	 
//本程序为控制器设计，未经许可，不得复制外传
//实验板栋达电子V3.0-1
//KEY 代码 PA11为显示板设置按键；PA12为手动投切开关	   
//修改日期:2013/3/13
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 济宁市栋达电子科技有限公司 2013-2023
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////  
								    
//按键初始化函数
void KEY_Init(void)
{
		 GPIO_InitTypeDef      GPIO_InitStructure;
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8|GPIO_Pin_9| GPIO_Pin_10| GPIO_Pin_11| GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

/*************************拨码开关*********************************/
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

} 



void ParaSet(void)
{



extern u8 L_C_flag_A;//感性容性标准变量
	 u8 CT_para_qianwei,CT_para_baiwei,CT_para_shiwei,CT_para_gewei;
	 u8	DELAY_ON_para_baiwei,DELAY_ON_para_shiwei,DELAY_ON_para_gewei;
	 u8	DELAY_OFF_para_baiwei,DELAY_OFF_para_shiwei,DELAY_OFF_para_gewei;
	 u8	COS_ON_para_baiwei,COS_ON_para_shiwei,COS_ON_para_gewei;
	 u8	COS_OFF_para_baiwei,COS_OFF_para_shiwei,COS_OFF_para_gewei;
	 u8	V_PROT_para_baiwei,V_PROT_para_shiwei,V_PROT_para_gewei;
	 u8	ON_HOLD_para_shiwei,ON_HOLD_para_gewei;
	 u8	OFF_HOLD_para_shiwei,OFF_HOLD_para_gewei;
	 u8 HU_PROT_para_baiwei,HU_PROT_para_shiwei,HU_PROT_para_gewei;
	 u8 HI_PROT_para_baiwei,HI_PROT_para_shiwei,HI_PROT_para_gewei;
	 u8 COMMCAT_para_baiwei,COMMCAT_para_shiwei,COMMCAT_para_gewei;
         s8 a=0;
	u8 ON_HOLD_para=1;
 u8 OFF_HOLD_para=1;	 
u16 TR[]={1,2,3,4,5,6,8,10,12,16,20,24,30,40,50,60,80,100,120};

u8 num1_5Seg[]=   {0X0F,0X05,0X06,0X00,0X0B,0X06,0X0F,0X02,0X06,0X03,0X0D,0X03,0X0D,0X07,0X07,0X00,0X0F,0X07,0X0F,0X03};
u8 num6_12Seg[]={0X05,0X0F,0X00,0X06,0X06,0X0B,0X02,0X0F,0X03,0X06,0X03,0X0D,0X07,0X0D,0X00,0X07,0X07,0X0F,0X03,0X0F};
u8 num12dpSeg[]=  {0X0F,0X0D,0X06,0X08,0X0B,0X0E,0X0F,0X0A,0X06,0X0B,0X0D,0X0B,0X0D,0X0F,0X07,0X08,0X0F,0X0F,0X0F,0X0B};
u8 num34dpSeg[]=   {0X0D,0X0F,0X08,0X06,0X0E,0X0B,0X0A,0X0F,0X0B,0X06,0X0B,0X0D,0X0F,0X0D,0X08,0X07,0X0F,0X0F,0X0B,0X0F};
u8 num_SET_Seg[]={0X0F,0X0F,0X06,0X0D,0X0F,0X06,0X0B,0X0F,0X06,0X07,0X0F,0X06,0X0F,0X07,0X06,0X0F,0X0E,0X06,0X0F,0X0D,0X06,0X0F,0X0B,0X06,0X0F,0X0F,0X04,0X0F,0X0F,0X02,0X0F,0X0F,0X06};
// 全显，CT互感器,延时时间,目标cos,电压保护, 投,切,门限, 谐波保护,通讯设置,温度设置,
u8 numHU_prot[]={0x0e,0x05,0x06,0x07};
u8 numHI_prot[]={0x00,0x05,0x06,0x07};
 u8 com_ID=1;
u8 DELAY_ON_para;
 u8 DELAY_OFF_para;
 u8 COS_ON_para=90;
 u8 COS_OFF_para=95;
 u8 V_PROT_para_L=40;
 u8 V_PROT_para_tri=40;
 u8 HU_PROT_para=100;
 u8 HI_PROT_para=100;


 
if(light_time==0)
{

	 if(KEY_set==0)
	 {
	 LIGHT_backligt_on();
	   while(KEY_set==0);
	 }
	 if(KEY_hand==0)
	 {
	 LIGHT_backligt_on();
	   while(KEY_hand==0);
	 }

		 if(KEY_right==0)
	 {
	 LIGHT_backligt_on();
	   while(KEY_right==0);
	 }

	   if(KEY_left==0)
	 {
	 LIGHT_backligt_on();
	   while(KEY_left==0);
	 } 
	 	 if(KEY_up==0)
	 {
	 LIGHT_backligt_on();
	   while(KEY_up==0);
	 }
		 if(KEY_down==0)
	 {
	 LIGHT_backligt_on();
	   while(KEY_down==0);
	}	 
if(Work_num==0)Graf_display();		 
}
else
{
	 if(KEY_set==0)
	 {
	 LIGHT_backligt_on();
	    Work_num++;
	   while(KEY_set==0);
	   	if(Work_num>11)
		Work_num=0;
	 }

	 if(KEY_hand==0)
	 {
	 	 LIGHT_backligt_on();
	    COMMCAT_para++;
	   while(KEY_hand==0);
	   	if(COMMCAT_para>1)COMMCAT_para=0;
	 }
	 
	 switch(Work_num)
	 {
	 
	  case 1:								//CT互感器参数设置
	  {
		 Clera_lcd();
		 if(Work_Flag==1)WriteAll_1621(28,num_SET_Seg,3);
		 if(Work_Flag==0)WriteAll_1621(28,num_SET_Seg+3,3);
   			//CT_para从eeprom读出
a=AT24CXX_ReadOneByte(0x0000);

		  CT_para_qianwei=TR[a]*50/1000;
		  CT_para_baiwei=(TR[a]*50%1000)/100;
		  CT_para_shiwei=(TR[a]*50%100)/10;
		  CT_para_gewei=0;
		  WriteAll_1621(14,num1_5Seg+2*CT_para_qianwei,2);	//
		  WriteAll_1621(8,num6_12Seg+2*CT_para_baiwei,2);	//
  		  WriteAll_1621(10,num6_12Seg+2*CT_para_shiwei,2);	//
		  WriteAll_1621(12,num6_12Seg+2*CT_para_gewei,2);	//

		  if(KEY_up==0)
		 {
		 	 LIGHT_backligt_on();
			a++;
			while(KEY_up==0);
		    if(a>18)a=0;
		 }
		 if(KEY_down==0)
		 {
		 	 LIGHT_backligt_on();
		    a--;
			while(KEY_down==0);
			if(a<0)a=18;
			
		 }
		  
		  AT24CXX_WriteOneByte(0x0000,a);   //存储CT_para到eeprom
		 break;

	  }
	  case 2:				//投入延时参数设置
	  {
	  	 Clera_lcd();
		 if(Work_Flag==1)WriteAll_1621(28,num_SET_Seg,3);
		 if(Work_Flag==0)WriteAll_1621(28,num_SET_Seg+2*3,3);

		 Write_1621(24,0x04);	//投入

		 DELAY_ON_para=AT24CXX_ReadOneByte(0x0001);  //存储DELAY_ON_para到eeprom


		  
		  DELAY_ON_para_baiwei=DELAY_ON_para/100;
		  DELAY_ON_para_shiwei=(DELAY_ON_para%100)/10;
		  DELAY_ON_para_gewei=DELAY_ON_para%10;

		  WriteAll_1621(8,num6_12Seg+2*DELAY_ON_para_baiwei,2);	//
  		  WriteAll_1621(10,num6_12Seg+2*DELAY_ON_para_shiwei,2);	//
		  WriteAll_1621(12,num6_12Seg+2*DELAY_ON_para_gewei,2);	//

		  if(KEY_up==0)
		 {
		 	 LIGHT_backligt_on();
			DELAY_ON_para++;
			while(KEY_up==0);
			if(DELAY_ON_para>255)DELAY_ON_para=0;
		 }
		 if(KEY_down==0)
		 {
		 	 LIGHT_backligt_on();
			DELAY_ON_para--;
			while(KEY_down==0);
			if(DELAY_ON_para<0)DELAY_ON_para=255;
		 }
		  
		 AT24CXX_WriteOneByte(0x0001,DELAY_ON_para);  //存储DELAY_ON_para到eeprom
		 break;
	  }
	  case 3:		  //切除延时参数设置
	  {
	  	 Clera_lcd();
	   	 if(Work_Flag==1)WriteAll_1621(28,num_SET_Seg,3);
		 if(Work_Flag==0)WriteAll_1621(28,num_SET_Seg+2*3,3);

		  Write_1621(24,0x02);	//切除
		 DELAY_OFF_para=AT24CXX_ReadOneByte(0x0002);  //存储DELAY_OFF_para到eeprom

		
		  
		  DELAY_OFF_para_baiwei=DELAY_OFF_para/100;
		  DELAY_OFF_para_shiwei=(DELAY_OFF_para%100)/10;
		  DELAY_OFF_para_gewei=DELAY_OFF_para%10;

		  WriteAll_1621(8,num6_12Seg+2*DELAY_OFF_para_baiwei,2);	//
  		  WriteAll_1621(10,num6_12Seg+2*DELAY_OFF_para_shiwei,2);	//
		  WriteAll_1621(12,num6_12Seg+2*DELAY_OFF_para_gewei,2);	//

		   if(KEY_up==0)
		 {
		 	 LIGHT_backligt_on();
			DELAY_OFF_para++;
			while(KEY_up==0);
			if(DELAY_OFF_para>255)DELAY_OFF_para=0;
		 }
		 if(KEY_down==0)
		 {
		 	 LIGHT_backligt_on();
			DELAY_OFF_para--;
			while(KEY_down==0);
			if(DELAY_OFF_para<0)DELAY_OFF_para=255;
		 }

			 AT24CXX_WriteOneByte(0x0002,DELAY_OFF_para);  //存储DELAY_ON_para到eeprom

		  break;
	  }
	  case 4:	  //投入COS参数设置
	  {
	  	 Clera_lcd();

		 if(Work_Flag==1)WriteAll_1621(28,num_SET_Seg,3);
		 if(Work_Flag==0)WriteAll_1621(28,num_SET_Seg+3*3,3);

				 Write_1621(24,0x04);	//投入

		 COS_ON_para=AT24CXX_ReadOneByte(0x0003);  //存储DELAY_OFF_para到eeprom

		  
		  COS_ON_para_baiwei=COS_ON_para/100;
		  COS_ON_para_shiwei=(COS_ON_para%100)/10;
		  COS_ON_para_gewei=COS_ON_para%10;

 		  WriteAll_1621(22,num1_5Seg+2*COS_ON_para_baiwei,2);	//
  		  WriteAll_1621(20,num12dpSeg+2*COS_ON_para_shiwei,2);	//
		  WriteAll_1621(18,num1_5Seg+2*COS_ON_para_gewei,2);	//

  if(KEY_up==0)
		 {
		 	 LIGHT_backligt_on();
			COS_ON_para++;
			while(KEY_up==0);
			if(COS_ON_para>100)COS_ON_para=80;
		 }
		 if(KEY_down==0)
		 {
		 	 LIGHT_backligt_on();
			COS_ON_para--;
			while(KEY_down==0);
			if(COS_ON_para<80)COS_ON_para=100;
		 }
			 AT24CXX_WriteOneByte(0x0003,COS_ON_para);  //存储DELAY_ON_para到eeprom
		 break;

	  }
	  case 5:		//切除COS参数设置
	  {
	  	 Clera_lcd();
		 if(Work_Flag==1)WriteAll_1621(28,num_SET_Seg,3);
		 if(Work_Flag==0)WriteAll_1621(28,num_SET_Seg+3*3,3);
		 
		 Write_1621(24,0x02);	//切除

				 COS_OFF_para=AT24CXX_ReadOneByte(0x0004);  //存储DELAY_OFF_para到eeprom
 

		  
		  COS_OFF_para_baiwei=COS_OFF_para/100;
		  COS_OFF_para_shiwei=(COS_OFF_para%100)/10;
		  COS_OFF_para_gewei=COS_OFF_para%10;

		  WriteAll_1621(22,num1_5Seg+2*COS_OFF_para_baiwei,2);	//
  		  WriteAll_1621(20,num12dpSeg+2*COS_OFF_para_shiwei,2);	//
		  WriteAll_1621(18,num1_5Seg+2*COS_OFF_para_gewei,2);	//

		 if(KEY_up==0)
		 {
		 	 LIGHT_backligt_on();
			COS_OFF_para++;
			while(KEY_up==0);
			if(COS_OFF_para>100)COS_OFF_para=80;
		 }
		 if(KEY_down==0)
		 {
		 	 LIGHT_backligt_on();
			COS_OFF_para--;
			while(KEY_down==0);
            if(COS_OFF_para<80)COS_OFF_para=100;
		 } 
					 AT24CXX_WriteOneByte(0x0004,COS_OFF_para);  //存储DELAY_ON_para到eeprom
 
		 break;

	  }
	  case 6:	   //电压保护参数设置
   if(KEY_3==1) 
	  {
	  	Clera_lcd();
		if(Work_Flag==1)WriteAll_1621(28,num_SET_Seg,3);
		if(Work_Flag==0)WriteAll_1621(28,num_SET_Seg+4*3,3);

		//CT_para从eeprom读出

		V_PROT_para_L=AT24CXX_ReadOneByte(0x0009);

		  	
		  {
		  V_PROT_para_baiwei=(V_PROT_para_L+200)/100;
		  V_PROT_para_shiwei=((V_PROT_para_L+200)%100)/10;
		  V_PROT_para_gewei=(V_PROT_para_L+200)%10;
		  	}
		  WriteAll_1621(8,num6_12Seg+2*V_PROT_para_baiwei,2);	//
  		  WriteAll_1621(10,num6_12Seg+2*V_PROT_para_shiwei,2);	//
		  WriteAll_1621(12,num6_12Seg+2*V_PROT_para_gewei,2);	//
		  
		   if(KEY_up==0)
		 {
		 	 LIGHT_backligt_on();
			V_PROT_para_L++;
			while(KEY_up==0);
			if(V_PROT_para_L>70)V_PROT_para_L=1;
		 }
		 if(KEY_down==0)
		 {
		 	 LIGHT_backligt_on();
					V_PROT_para_L--;
			while(KEY_down==0);
			if(V_PROT_para_L<1)V_PROT_para_L=70;

		 }
		  //存储CT_para到eeprom
 AT24CXX_WriteOneByte(0x0009,V_PROT_para_L);

		 break;
	  }
	   if(KEY_3==0) 
	  {
	  	Clera_lcd();
		if(Work_Flag==1)WriteAll_1621(28,num_SET_Seg,3);
		if(Work_Flag==0)WriteAll_1621(28,num_SET_Seg+4*3,3);

		//CT_para从eeprom读出

		V_PROT_para_tri=AT24CXX_ReadOneByte(0x000a);

		  	
		  {
		  V_PROT_para_baiwei=(V_PROT_para_tri+400)/100;
		  V_PROT_para_shiwei=((V_PROT_para_tri+400)%100)/10;
		  V_PROT_para_gewei=(V_PROT_para_tri+400)%10;
		  	}
			  WriteAll_1621(8,num6_12Seg+2*V_PROT_para_baiwei,2);	//
  		  WriteAll_1621(10,num6_12Seg+2*V_PROT_para_shiwei,2);	//
		  WriteAll_1621(12,num6_12Seg+2*V_PROT_para_gewei,2);	//
		  
		   if(KEY_up==0)
		 {	 LIGHT_backligt_on();
			V_PROT_para_tri++;
			while(KEY_up==0);
			if(V_PROT_para_tri>60)V_PROT_para_tri=1;
		 }
		 if(KEY_down==0)
		 {
		 	 LIGHT_backligt_on();
			V_PROT_para_tri--;
			while(KEY_down==0);
			if(V_PROT_para_tri<1)V_PROT_para_tri=60;

		 }
		  //存储CT_para到eeprom
 AT24CXX_WriteOneByte(0x000a,V_PROT_para_tri);

		 break;
	  }  
	  case 7:	   //投入门限参数设置
	  {
	  	Clera_lcd();
		if(Work_Flag==1)WriteAll_1621(28,num_SET_Seg,3);
		if(Work_Flag==0)WriteAll_1621(28,num_SET_Seg+5*3,3);

		ON_HOLD_para=AT24CXX_ReadOneByte(0x0005); 
 

		  
		  ON_HOLD_para_shiwei=ON_HOLD_para/10;
		  ON_HOLD_para_gewei=ON_HOLD_para%10;

  		  WriteAll_1621(10,num6_12Seg+2*ON_HOLD_para_shiwei,2);	//
		  WriteAll_1621(12,num34dpSeg+2*ON_HOLD_para_gewei,2);	//
		  
			if(KEY_up==0)
		 {
		 	 LIGHT_backligt_on();
		 	delay_ms(10); 
			ON_HOLD_para++;
			while(KEY_up==0);
			delay_ms(10);
			if(ON_HOLD_para>12)ON_HOLD_para=2;
		 }
		 if(KEY_down==0)
		 {
		 	 LIGHT_backligt_on();
			ON_HOLD_para--;
			while(KEY_down==0);
			if(ON_HOLD_para<2)ON_HOLD_para=12;
		 }
		  //存储CT_para到eeprom
		 AT24CXX_WriteOneByte(0x0005,ON_HOLD_para);  //存储DELAY_ON_para到eeprom

		break;

	  }
	  case 8:	  //切除门限参数设置
	  {
	  	Clera_lcd();
		if(Work_Flag==1)WriteAll_1621(28,num_SET_Seg,3);
		if(Work_Flag==0)WriteAll_1621(28,num_SET_Seg+6*3,3);

	

		 
		OFF_HOLD_para=AT24CXX_ReadOneByte(0x0006); 

		  
		  OFF_HOLD_para_shiwei=OFF_HOLD_para/10;
		  OFF_HOLD_para_gewei=OFF_HOLD_para%10;

  		  WriteAll_1621(10,num6_12Seg+2*OFF_HOLD_para_shiwei,2);	//
		  WriteAll_1621(12,num34dpSeg+2*OFF_HOLD_para_gewei,2);	//
		  
		    if(KEY_up==0)
		 {
		 	 LIGHT_backligt_on();
			OFF_HOLD_para++;
			while(KEY_up==0);
			if(OFF_HOLD_para>12)OFF_HOLD_para=2;
		 }
		 if(KEY_down==0)
		 {
		 	 LIGHT_backligt_on();
			OFF_HOLD_para--;
			while(KEY_down==0);
			if(OFF_HOLD_para<2)OFF_HOLD_para=12;
		 }
		  //存储CT_para到eeprom
		 AT24CXX_WriteOneByte(0x0006,OFF_HOLD_para);  //存储DELAY_ON_para到eeprom

		break;

	  }
	  case 9:	 //谐波电压参数设置
	  {
	  	Clera_lcd();
		if(Work_Flag==1)WriteAll_1621(28,num_SET_Seg,3);
		if(Work_Flag==0)WriteAll_1621(28,num_SET_Seg+8*3,3);

		WriteAll_1621(20,numHU_prot,4);
	//	WriteAll_1621(22,numHU_prot+2,2);

			HU_PROT_para=AT24CXX_ReadOneByte(0x0007); 
	  

		  HU_PROT_para_baiwei=HU_PROT_para/100;
		  HU_PROT_para_shiwei=(HU_PROT_para%100)/10;
		  HU_PROT_para_gewei=HU_PROT_para%10;

		  WriteAll_1621(8,num6_12Seg+2*HU_PROT_para_baiwei,2);	//
  		  WriteAll_1621(10,num6_12Seg+2*HU_PROT_para_shiwei,2);	//
		  WriteAll_1621(12,num6_12Seg+2*HU_PROT_para_gewei,2);	//

		   if(KEY_up==0)
		 {
		 	 LIGHT_backligt_on();
			HU_PROT_para++;
		    while(KEY_up==0);
			if(HU_PROT_para>100)HU_PROT_para=1;
		 }
		 if(KEY_down==0)
		 {
		 	 LIGHT_backligt_on();
			HU_PROT_para--;
			while(KEY_down==0);
			if(HU_PROT_para<1)HU_PROT_para=100;
		 }
		 		 AT24CXX_WriteOneByte(0x0007,HU_PROT_para);  //存储DELAY_ON_para到eeprom

		break;

	  }

	  case 10:		 //谐波电流参数设置
	  {
	   Clera_lcd();
	   if(Work_Flag==1)WriteAll_1621(28,num_SET_Seg,3);
	   if(Work_Flag==0)WriteAll_1621(28,num_SET_Seg+8*3,3);

	   WriteAll_1621(20,numHI_prot,4);
	//   WriteAll_1621(22,numHI_prot+2,2);

	   			HI_PROT_para=AT24CXX_ReadOneByte(0x0008); 


		  HI_PROT_para_baiwei=HI_PROT_para/100;
		  HI_PROT_para_shiwei=(HI_PROT_para%100)/10;
		  HI_PROT_para_gewei=HI_PROT_para%10;

		  WriteAll_1621(8,num6_12Seg+2*HI_PROT_para_baiwei,2);	//
  		  WriteAll_1621(10,num6_12Seg+2*HI_PROT_para_shiwei,2);	//
		  WriteAll_1621(12,num6_12Seg+2*HI_PROT_para_gewei,2);	//

		  if(KEY_up==0)
		 {	 LIGHT_backligt_on();
			HI_PROT_para++;
			while(KEY_up==0);
			if(HI_PROT_para>100)HI_PROT_para=0;
		 }
		 if(KEY_down==0)
		 {
		 	 LIGHT_backligt_on();
			HI_PROT_para--;
			while(KEY_down==0);
			if(HI_PROT_para<1)HI_PROT_para=100;
		 }
		 		 		 AT24CXX_WriteOneByte(0x0008,HI_PROT_para);  //存储DELAY_ON_para到eeprom

	   break;

	  }
	  case 11:	  //通信地址参数设置
	  {
	  	Clera_lcd();
		if(Work_Flag==1)WriteAll_1621(28,num_SET_Seg,3);
		if(Work_Flag==0)WriteAll_1621(28,num_SET_Seg+9*3,3);
		
			
		 		 com_ID=AT24CXX_ReadOneByte(0x000b);  //存储DELAY_OFF_para到eeprom

		 COMMCAT_para_baiwei=com_ID/100;
		 COMMCAT_para_shiwei=(com_ID%100)/10;
		 COMMCAT_para_gewei=com_ID%10;

		 WriteAll_1621(8,num6_12Seg+2*COMMCAT_para_baiwei,2);	//
  		 WriteAll_1621(10,num6_12Seg+2*COMMCAT_para_shiwei,2);	//
		 WriteAll_1621(12,num6_12Seg+2*COMMCAT_para_gewei,2);	//

		   if(KEY_up==0)
		 {
			com_ID++;
				 LIGHT_backligt_on();
			while(KEY_up==0);
			if(com_ID>64)com_ID=1;
		 }
		 if(KEY_down==0)
		 {
			com_ID--;
			 LIGHT_backligt_on();
			while(KEY_down==0);
			if(com_ID<1)com_ID=64;
		 }
		  AT24CXX_WriteOneByte(0x000b,com_ID);  //存储DELAY_ON_para到eeprom

		break;

	  }

	  default:	{Graf_display();break;}
	 
	}

}
}
void Graf_display(void)
{
	u8 CAPA_num_shiwei,CAPA_num_gewei,capa1_value_shiwei,capa1_value_gewei,capa2_value_shiwei,capa2_value_gewei;
	u8 numCAP[]={0X00,0X02,0X09,0X05};
u8 num1_5Seg[]=   {0X0F,0X05,0X06,0X00,0X0B,0X06,0X0F,0X02,0X06,0X03,0X0D,0X03,0X0D,0X07,0X07,0X00,0X0F,0X07,0X0F,0X03};
u8 num6_12Seg[]={0X05,0X0F,0X00,0X06,0X06,0X0B,0X02,0X0F,0X03,0X06,0X03,0X0D,0X07,0X0D,0X00,0X07,0X07,0X0F,0X03,0X0F};
static u16 count=1;
if(light_time>0)
{

	if(KEY_down==0)
 		{
 		if(COMMCAT_para==0)
		{ Display_num++;LIGHT_backligt_on();}
		 while(KEY_down==0);
		 if(Display_num>6)Display_num=1;
	    }
	if(KEY_up==0)
	  { 	if(COMMCAT_para==0)
		 {Display_num--;LIGHT_backligt_on();}
		 while(KEY_up==0);
		 if(Display_num<1)Display_num=6;
	  }
}	
   switch(Display_num)
   {
   	 case 1:										   //显示功率因数电流电压
     {
  if(KEY_3==0) 
              {
		 Clera_lcd();
		 Graf_cos_volt_current(gonglvshishu*10,dianya_zhi*10,dianliuzhi);
            	}
   if(KEY_3==1) 
   	{
	if(KEY_right==0)
	if(COMMCAT_para==0)
 		{	 
 		LIGHT_backligt_on();
		 L1_L2_L3_COS++;
		 while(KEY_right==0);
		 if(L1_L2_L3_COS>3)L1_L2_L3_COS=1;
	    }
	if(KEY_left==0)
	   if(COMMCAT_para==0)	
	  {
	  	 LIGHT_backligt_on();
		 L1_L2_L3_COS--;
		 while(KEY_left==0);
		 if(L1_L2_L3_COS<1)L1_L2_L3_COS=3;
	  }
  if(L1_L2_L3_COS==1)
   	{
   	if(phase_flag==0)//第一屏正序显示L1
		{ Clera_lcd();
		 Graf_cos_volt_current_L1(gonglvshishu_A*10,dianya_zhi_A*10,dianliuzhi_A,L_C_flag_A);
  	    }
   	if(phase_flag==1)//反序第一屏显示L3
   	{		 
   	       Clera_lcd();
		 Graf_cos_volt_current_L3(gonglvshishu_C*10,dianya_zhi_C*10,dianliuzhi_C,L_C_flag_C);
  	}
  }
  if(L1_L2_L3_COS==2)
   		{ 
   		Clera_lcd();
		 Graf_cos_volt_current_L2(gonglvshishu_B*10,dianya_zhi_B*10,dianliuzhi_B,L_C_flag_B);
		}
  if(L1_L2_L3_COS==3)
  	{

	if(phase_flag==1)//反序第三屏显示L1
		{ Clera_lcd();
		 Graf_cos_volt_current_L1(gonglvshishu_A*10,dianya_zhi_A*10,dianliuzhi_A,L_C_flag_A);
  	    }
	
   	if(phase_flag==0)//正序第三屏显示L3
   	{		 
   	       Clera_lcd();
		 Graf_cos_volt_current_L3(gonglvshishu_C*10,dianya_zhi_C*10,dianliuzhi_C,L_C_flag_C);
  	}
  }
   }

break;
     }
	 case 2:	//显示无功有功频率
  if(KEY_3==0) 
		{
		 Clera_lcd();
		 Graf_powuse_poweunuse_freq(wugongkvar,allkvar,501);
	 	}
  if(KEY_3==1) 
              {
	if(KEY_right==0)
	  	if(COMMCAT_para==0)	
 		{
 			 LIGHT_backligt_on();
		 L1_L2_L3_KAR++;
		 while(KEY_right==0);
		 if(L1_L2_L3_KAR>3)L1_L2_L3_KAR=1;
	    }
	if(KEY_left==0)
	  	if(COMMCAT_para==0)	
	  {
	  	 LIGHT_backligt_on();
		 L1_L2_L3_KAR--;
		 while(KEY_left==0);
		 if(L1_L2_L3_KAR<1)L1_L2_L3_KAR=3;
	  }

		if(L1_L2_L3_KAR==1)
			{				
		 Clera_lcd();
		 if(phase_flag==0)
		 Graf_powuse_poweunuse_freq_L1(wugongkvar_A,allkvar_A,501);
               else
		 Graf_powuse_poweunuse_freq_L3(wugongkvar_C,allkvar_C,501);              
		}
		if(L1_L2_L3_KAR==2)
			{
		 Clera_lcd();
		 Graf_powuse_poweunuse_freq_L2(wugongkvar_B,allkvar_B,501);
			}
		if(L1_L2_L3_KAR==3)
			{
		 Clera_lcd();
		 if(phase_flag==0)
		 Graf_powuse_poweunuse_freq_L3(wugongkvar_C,allkvar_C,501);
               else
		 Graf_powuse_poweunuse_freq_L1(wugongkvar_A,allkvar_A,501);
		}
		}
  
		 break;
	 case 3:											  //显示温度电压谐波电流谐波
           if(KEY_3==0)
           	{
		   Clera_lcd();
		 Graf_temp_hv_hi(0,HV*10,HI*10);
           	}
	  if(KEY_3==1)
           	{
           		if(KEY_right==0)
		  	if(COMMCAT_para==0)				
 		{
 			 LIGHT_backligt_on();
		 L1_L2_L3_KAR++;
		 while(KEY_right==0);
		 if(L1_L2_L3_KAR>3)L1_L2_L3_KAR=1;
	    }
	if(KEY_left==0)
		 if(COMMCAT_para==0)	
	  {
	  	 LIGHT_backligt_on();
                 L1_L2_L3_KAR--;
		 while(KEY_left==0);
		 if(L1_L2_L3_KAR<1)L1_L2_L3_KAR=3;
	  }
	
		 if(L1_L2_L3_KAR==1)
		 	{
		 	Clera_lcd();
			if(phase_flag==0)
		 Graf_temp_hv_hi_L1(0,A_HV*10,A_HI*10);
                 else
		 Graf_temp_hv_hi_L3(0,C_HV*10,C_HI*10);
		 }
		  if(L1_L2_L3_KAR==2)
		 	{
		 	Clera_lcd();
		 Graf_temp_hv_hi_L2(0,B_HV*10,B_HI*10);
		 	}
		  if(L1_L2_L3_KAR==3)
		 	{
		 	Clera_lcd();
			if(phase_flag==0)
		 Graf_temp_hv_hi_L3(0,C_HV*10,C_HI*10);
                   else 
		 Graf_temp_hv_hi_L1(0,A_HV*10,A_HI*10);
		  }
		 }	   
		 break;
	 case 4:											//显示版本号
		  Clera_lcd();
		  Graf_VER(2);
		 break;

	 case 5:											//显示电容组号及容量
		  Clera_lcd();

    Write_1621(28,0x01);//LOG
    Write_1621(31,0x01);//	显示P14功率因数电压电流和P12“动控制”符号
   //Write_1621(24,0x08);	//	带△符号

   if(COMMCAT_para==0)
	  	  	{
	  	  	Write_1621(31,0x0a);//自
	  	  	}
		  if(COMMCAT_para==1)
	  	  	{
	  	  Write_1621(31,0x0c);//手

	  	  	}			


  if(KEY_conceal==0)
		  	{
                  LIGHT_backligt_on();
			comm_list[CAPA_num].size=comm_list[CAPA_num].size+1;	  
			  delay_ms(150);

		      }
		  while(KEY_conceal==0){
                                       count++;
						   if(count==200)
						   	{
						   	count=1;                  
                  LIGHT_backligt_on();

	 Clera_lcd();

    Write_1621(28,0x01);//LOG
    Write_1621(31,0x01);//	显示P14功率因数电压电流和P12“动控制”符号
   //Write_1621(24,0x08);	//	带△符号

   if(COMMCAT_para==0)
	  	  	{
	  	  	Write_1621(31,0x0a);//自
	  	  	}
		  if(COMMCAT_para==1)
	  	  	{
	  	  Write_1621(31,0x0c);//手

	  	  	}
			comm_list[CAPA_num].size=comm_list[CAPA_num].size+1;
			if(comm_list[CAPA_num].size>60)comm_list[CAPA_num].size=0;
	capa1_value=comm_list[CAPA_num].size;

		  WriteAll_1621(20,numCAP,4);

   		  CAPA_num_shiwei=CAPA_num/10;
		  CAPA_num_gewei=CAPA_num%10;
	   	  capa1_value_shiwei=capa1_value/10;
		  capa1_value_gewei=capa1_value%10;
		  capa2_value_shiwei=capa1_value/10;
		  capa2_value_gewei=capa1_value%10;

		  WriteAll_1621(18,num1_5Seg+2*CAPA_num_shiwei,2);
		  WriteAll_1621(16,num1_5Seg+2*CAPA_num_gewei,2);
		  WriteAll_1621(10,num6_12Seg+2*capa1_value_shiwei,2);
		  WriteAll_1621(12,num6_12Seg+2*capa1_value_gewei,2);
delay_ms(150);
                                                    }

		  }		
		  count=1;
if(comm_list[CAPA_num].size>60)comm_list[CAPA_num].size=0;
				capa1_value=comm_list[CAPA_num].size;
		  AT24CXX_WriteOneByte(0x0010+(CAPA_num-1)*2,comm_list[CAPA_num].size);  //存储DELAY_ON_para到eeprom

		  WriteAll_1621(20,numCAP,4);

   		  CAPA_num_shiwei=CAPA_num/10;
		  CAPA_num_gewei=CAPA_num%10;
	   	  capa1_value_shiwei=capa1_value/10;
		  capa1_value_gewei=capa1_value%10;
		  capa2_value_shiwei=capa1_value/10;
		  capa2_value_gewei=capa1_value%10;

		  WriteAll_1621(18,num1_5Seg+2*CAPA_num_shiwei,2);
		  WriteAll_1621(16,num1_5Seg+2*CAPA_num_gewei,2);
		  WriteAll_1621(10,num6_12Seg+2*capa1_value_shiwei,2);
		  WriteAll_1621(12,num6_12Seg+2*capa1_value_gewei,2);	  	  	
//	  WriteAll_1621(4,num6_12Seg+2*capa2_value_shiwei,2);					  
	//	  WriteAll_1621(6,num6_12Seg+2*capa2_value_gewei,2);
		  
		  if(KEY_right==0)
		  	if(COMMCAT_para==0)	
		  {
		  	 LIGHT_backligt_on();
                       CAPA_num++;
			while(KEY_right==0);
		  	if(CAPA_num>32)CAPA_num=1;
		

		  }
		  if(KEY_left==0)
		  if(COMMCAT_para==0)
		  {
		  	 LIGHT_backligt_on();
			CAPA_num--;
			while(KEY_left==0);
		  	if(CAPA_num<1)CAPA_num=32;
			capa1_value=comm_list[CAPA_num].size;
			capa2_value=comm_list[CAPA_num].size;

			
		  }

		  break;

	 case 6:										   //显示投入的电容电流

	 Clera_lcd();
	 Graf_current_value(0,0,0);
	 
//	 Graf_current_value(dianya_zhi_A*10,dianya_zhi_B*10,dianya_zhi_C*10);
	 break;
   
   }

}

