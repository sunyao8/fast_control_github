//#include <stm32f10x_lib.h>
#include "ht1621.h"
#include "delay.h"
#include "key.h"
//#include "led.h"

 //u8 num1234Seg[]={0X0A,0X0F,0X00,0X06,0X06,0X0D,0X04,0X0F,0X0C,0X06,0X0C,0X0B,0X0E,0X0B,0X00,0X0E,0X0E,0X0F,0X0C,0X0F};
//u8 num567Seg[]={0X0F,0X05,0X06,0X00,0X0B,0X06,0X0F,0X02,0X06,0X03,0X0D,0X03,0X0D,0X07,0X07,0X00,0X0F,0X07,0X0F,0X03};
//u8 num8Seg[]={0X05,0X0F,0X00,0X06,0X06,0X0B,0X02,0X0F,0X03,0X06,0X03,0X0D,0X07,0X0D,0X00,0X07,0X07,0X0F,0X03,0X0F};
//u8 num9_12Seg[]={0X06,0X05,0X06,0X06,0X00,0X00,0X04,0X07,0X02,0X06,0X07,0X00,0X06,0X02,0X04,0X02,0X07,0X04,0X02,0X07,0X06,0X06,0X04,0X00,0X06,0X07,0X06,0X06,0X07,0X04};

//u8 num8_dp7Seg[]={0X0D,0X0F,0X08,0X06,0X0E,0X0B,0X0A,0X0F,0X0B,0X06,0X0B,0X0D,0X0F,0X0D,0X08,0X07,0X0F,0X0F,0X0B,0X0F};
//u8 num10dp9_12dp11Seg[]={0X06,0X05,0X07,0X06,0X00,0X01,0X04,0X07,0X03,0X06,0X07,0X01,0X06,0X02,0X05,0X02,0X07,0X05,0X02,0X07,0X07,0X06,0X04,0X01,0X06,0X07,0X07,0X06,0X07,0X05};
//u8 num11_dp22Seg[]={0X0E,0X05,0X06,0X0E,0X00,0X00,0X0C,0X07,0X02,0X0E,0X07,0X00,0X0E,0X02,0X04,0X0A,0X07,0X04,0X0A,0X07,0X06,0X0E,0X04,0X00,0X0E,0X07,0X06,0X0E,0X07,0X04};
//u8 num4_dp3Seg[]={0X0B,0X0F,0X01,0X06,0X07,0X0D,0X05,0X0F,0X0D,0X06,0X0D,0X0B,0X0F,0X0B,0X01,0X0E,0X0F,0X0F,0X0D,0X0F};
//u8 num3_p11Seg[]={0X0B,0X0F,0X01,0X06,0X07,0X0D,0X05,0X0F,0X0C,0X06,0X0D,0X0B,0X0F,0X0B,0X01,0X0E,0X0F,0X0F,0X0D,0X0F};
//u8 num11_p21_p22Seg[]={0X0E,0X0D,0X06,0X0E,0X08,0X00,0X0C,0X0F,0X02,0X0E,0X0F,0X00,0X0E,0X0A,0X04,0X0A,0X0F,0X04,0X0A,0X0F,0X06,0X0E,0X0C,0X00,0X0E,0X0F,0X06,0X0E,0X0F,0X04};
//u8 num11_p21Seg[]={0X06,0X0D,0X06,0X06,0X08,0X00,0X04,0X0F,0X02,0X06,0X0F,0X00,0X06,0X0A,0X04,0X02,0X0F,0X04,0X02,0X0F,0X06,0X06,0X0C,0X00,0X06,0X0F,0X06,0X06,0X0F,0X04};





//////////////////////////////////////////////////////////////////////////////////	 
//本程序为控制器设计，未经许可，不得复制外传
//实验板栋达电子V3.0-1
//LCD驱动HT1621代码 PB3为595RCLK;PB4为1621WR;PB5为1621CS;PB6为1621DATA和595SCLK	   
//修改日期:2013/3/13
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 济宁市栋达电子科技有限公司 2013-2023
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   

//初始化PB4 PB5和PB6为输出口.并使能这PE时钟		    
//HT1621 IO初始化
extern u8 L_C_flag_B;
extern u8 COMMCAT_para;
extern u8 display_nothing_close_open_warn;

void HT1621_Init(void)
{		

  GPIO_InitTypeDef      GPIO_InitStructure;
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
   GPIO_SetBits( GPIOB, GPIO_Pin_5);
  GPIO_SetBits( GPIOB, GPIO_Pin_6);
  GPIO_SetBits( GPIOB, GPIO_Pin_7);
 
	SendCmd(LCDOFF);
	SendCmd(BIAS);			
	SendCmd(SYSEN);
	SendCmd(LCDON);
}

void SendBit_1621(u8 data,u8 cnt)	 //data高cnt位写入HT1621,高位在前
{
  u8 i;
  for(i=0;i<cnt;i++)
  {
   if((data&0x80)==0)DATA_0;
   else DATA_1;
   WR_0;
   delay_us(1);
   WR_1;
      delay_us(1);
   data<<=1;
  }
}
void SendDataBit_1621(u8 data,u8 cnt)	 //data低cnt位写入HT1621,低位在前
{
  u8 i;
  for(i=0;i<cnt;i++)
  {
   if((data&0x01)==0)DATA_0;
   else DATA_1;
   WR_0;
   delay_us(1);
   WR_1;
      delay_us(1);
   data>>=1;
  }
}

void SendCmd(u8 command)
{
   CS_0;
   SendBit_1621(0x80,3);
   SendBit_1621(command,9);
   CS_1;
   
}

void Write_1621(u8 addr,u8 data)
{
   CS_0;
   SendBit_1621(0xa0,3);
   SendBit_1621(addr<<2,6);
   SendDataBit_1621(data,4);
   CS_1;
}

void WriteAll_1621(u8 addr,u8 *p,u8 cnt)
{
   u8 i;
   CS_0;
   SendBit_1621(0xa0,3);
   SendBit_1621(addr<<2,6);
   for(i=0;i<cnt;i++,p++)
   {
   	   SendDataBit_1621(*p,4);
   }
   CS_1;
}

void Clera_lcd(void)
{
	 u8 t;
	 for(t=0;t<32;t++)
	 {
	   u8 i;
	   for(i=0;i<4;i++)
	   {		  
	   	Write_1621(t,0x00<<i);
	   }
	 }
}



void Graf_cos_volt_current(u16 PF,u16 volt_para,u32 current_para)		 //显示功率因数电压电流
{
  u8 PF_zhengshu,PF_shifenwei,PF_baifenwei,PF_qianfenwei;
  u8 volt_para_qianwei,volt_para_baiwei,volt_para_shiwei,volt_para_gewei;
  u8 current_para_qianwei,current_para_baiwei,current_para_shiwei,current_para_gewei;


u8 num1_5Seg[]=   {0X0F,0X05,0X06,0X00,0X0B,0X06,0X0F,0X02,0X06,0X03,0X0D,0X03,0X0D,0X07,0X07,0X00,0X0F,0X07,0X0F,0X03};
u8 num6_12Seg[]={0X05,0X0F,0X00,0X06,0X06,0X0B,0X02,0X0F,0X03,0X06,0X03,0X0D,0X07,0X0D,0X00,0X07,0X07,0X0F,0X03,0X0F};
u8 num12dpSeg[]=  {0X0F,0X0D,0X06,0X08,0X0B,0X0E,0X0F,0X0A,0X06,0X0B,0X0D,0X0B,0X0D,0X0F,0X07,0X08,0X0F,0X0F,0X0F,0X0B};
u8 num34dpSeg[]=   {0X0D,0X0F,0X08,0X06,0X0E,0X0B,0X0A,0X0F,0X0B,0X06,0X0B,0X0D,0X0F,0X0D,0X08,0X07,0X0F,0X0F,0X0B,0X0F};


Write_1621(28,0x01);//LOG
  Write_1621(31,0x01);//	显示P14功率因数电压电流和P12“动控制”符号
  if(display_nothing_close_open_warn==0)
 Write_1621(24,0x08);	//	带△符号
 if(display_nothing_close_open_warn==1)
 Write_1621(24,0x0c);	//	带△符号和投入
 if(display_nothing_close_open_warn==2)
 Write_1621(24,0x0a);	//	带△符号和切除
 if(display_nothing_close_open_warn==3)
 Write_1621(24,0x09);	//	带△符号和报警
 
  Write_1621(27,0x08);//cos 电压电流
  PF_zhengshu=PF/1000;
  PF_shifenwei=(PF%1000)/100;
  PF_baifenwei=	(PF%100)/10;
  PF_qianfenwei=PF%10;

  
  if(L_C_flag_B==1)WriteAll_1621(22,num1_5Seg+2*PF_zhengshu,2);				//
	  if(L_C_flag_B==0)Write_1621(23,0x02);	//	负号
   
  WriteAll_1621(20,num12dpSeg+2*PF_shifenwei,2);	//
WriteAll_1621(18,num1_5Seg+2*PF_baifenwei,2);	
  	
  WriteAll_1621(16,num1_5Seg+2*PF_qianfenwei,2);			//

  volt_para_qianwei=volt_para/1000;
  volt_para_baiwei=(volt_para%1000)/100;
  volt_para_shiwei=(volt_para%100)/10;
  volt_para_gewei=volt_para%10;


  if(volt_para_qianwei>0)
    {
	  WriteAll_1621(14,num1_5Seg+2*volt_para_qianwei,2);	//
	  WriteAll_1621(8,num6_12Seg+2*volt_para_baiwei,2);	//
	  WriteAll_1621(10,num6_12Seg+2*volt_para_shiwei,2);	//
	  WriteAll_1621(12,num34dpSeg+2*volt_para_gewei,2);	//
  	}
	else if(volt_para_baiwei>0)
	{
	 	  WriteAll_1621(8,num6_12Seg+2*volt_para_baiwei,2);	//
	  WriteAll_1621(10,num6_12Seg+2*volt_para_shiwei,2);	//
	  WriteAll_1621(12,num34dpSeg+2*volt_para_gewei,2);	//
	}
	 else
	 {
	  WriteAll_1621(10,num6_12Seg+2*volt_para_shiwei,2);	//
	  WriteAll_1621(12,num34dpSeg+2*volt_para_gewei,2);	//
	 }
  
  current_para_qianwei=current_para/1000;
  current_para_baiwei=(current_para%1000)/100;
  current_para_shiwei=(current_para%100)/10;
  current_para_gewei=current_para%10;
   if(COMMCAT_para==0)
	  	  	{
	  	  	Write_1621(31,0x0a);//自
	  	  	}
		  if(COMMCAT_para==1)
	  	  	{
	  	  Write_1621(31,0x0c);//手

	  	  	}

  if(current_para_qianwei>0)
    {
	  WriteAll_1621(0,num6_12Seg+2*current_para_qianwei,2);	//
	    WriteAll_1621(2,num6_12Seg+2*current_para_baiwei,2);	//
  WriteAll_1621(4,num6_12Seg+2*current_para_shiwei,2);	//
  WriteAll_1621(6,num6_12Seg+2*current_para_gewei,2);	//	  	
	  	  		  
    }
	 else if(current_para_baiwei>0)
	 	{
  WriteAll_1621(2,num6_12Seg+2*current_para_baiwei,2);	//
  WriteAll_1621(4,num6_12Seg+2*current_para_shiwei,2);	//
  WriteAll_1621(6,num6_12Seg+2*current_para_gewei,2);	//

	 }
	 else
	 	{
  WriteAll_1621(4,num6_12Seg+2*current_para_shiwei,2);	//
  WriteAll_1621(6,num6_12Seg+2*current_para_gewei,2);	//

	 }

}


void Graf_cos_volt_current_L1(u16 PF,u16 volt_para,u32 current_para,u8 L_C)		 //显示功率因数电压电流

{
  u8 PF_zhengshu,PF_shifenwei,PF_baifenwei,PF_qianfenwei;
  u8 volt_para_qianwei,volt_para_baiwei,volt_para_shiwei,volt_para_gewei;
  u8 current_para_qianwei,current_para_baiwei,current_para_shiwei,current_para_gewei;


u8 num1_5Seg[]=   {0X0F,0X05,0X06,0X00,0X0B,0X06,0X0F,0X02,0X06,0X03,0X0D,0X03,0X0D,0X07,0X07,0X00,0X0F,0X07,0X0F,0X03};
u8 num6_12Seg[]={0X05,0X0F,0X00,0X06,0X06,0X0B,0X02,0X0F,0X03,0X06,0X03,0X0D,0X07,0X0D,0X00,0X07,0X07,0X0F,0X03,0X0F};
u8 num12dpSeg[]=  {0X0F,0X0D,0X06,0X08,0X0B,0X0E,0X0F,0X0A,0X06,0X0B,0X0D,0X0B,0X0D,0X0F,0X07,0X08,0X0F,0X0F,0X0F,0X0B};
u8 num34dpSeg[]=   {0X0D,0X0F,0X08,0X06,0X0E,0X0B,0X0A,0X0F,0X0B,0X06,0X0B,0X0D,0X0F,0X0D,0X08,0X07,0X0F,0X0F,0X0B,0X0F};

Write_1621(28,0x01);//LOG
  Write_1621(31,0x01);//	显示P14功率因数电压电流和P12“动控制”符号
  Write_1621(27,0x0a);//cos 电压电流带L1符号
if(display_nothing_close_open_warn==0)
 Write_1621(24,0x00);	//	空  
if(display_nothing_close_open_warn==1)
 Write_1621(24,0x04);	//	投入
 if(display_nothing_close_open_warn==2)
 Write_1621(24,0x02);	//	切除
 if(display_nothing_close_open_warn==3)
 Write_1621(24,0x01);	//	报警
  PF_zhengshu=PF/1000;
  PF_shifenwei=(PF%1000)/100;
  PF_baifenwei=	(PF%100)/10;
  PF_qianfenwei=PF%10;

  
  if(L_C_flag_B==1)WriteAll_1621(22,num1_5Seg+2*PF_zhengshu,2);				//
	  if(L_C_flag_B==0)Write_1621(23,0x02);	//	负号
   
  WriteAll_1621(20,num12dpSeg+2*PF_shifenwei,2);	//
WriteAll_1621(18,num1_5Seg+2*PF_baifenwei,2);	
  	
  WriteAll_1621(16,num1_5Seg+2*PF_qianfenwei,2);			//

  volt_para_qianwei=volt_para/1000;
  volt_para_baiwei=(volt_para%1000)/100;
  volt_para_shiwei=(volt_para%100)/10;
  volt_para_gewei=volt_para%10;


  if(volt_para_qianwei>0)
    {
	  WriteAll_1621(14,num1_5Seg+2*volt_para_qianwei,2);	//
	  WriteAll_1621(8,num6_12Seg+2*volt_para_baiwei,2);	//
	  WriteAll_1621(10,num6_12Seg+2*volt_para_shiwei,2);	//
	  WriteAll_1621(12,num34dpSeg+2*volt_para_gewei,2);	//
  	}
	else if(volt_para_baiwei>0)
	{
	 	  WriteAll_1621(8,num6_12Seg+2*volt_para_baiwei,2);	//
	  WriteAll_1621(10,num6_12Seg+2*volt_para_shiwei,2);	//
	  WriteAll_1621(12,num34dpSeg+2*volt_para_gewei,2);	//
	}
	 else
	 {
	  WriteAll_1621(10,num6_12Seg+2*volt_para_shiwei,2);	//
	  WriteAll_1621(12,num34dpSeg+2*volt_para_gewei,2);	//
	 }
  
  current_para_qianwei=current_para/1000;
  current_para_baiwei=(current_para%1000)/100;
  current_para_shiwei=(current_para%100)/10;
  current_para_gewei=current_para%10;
   if(COMMCAT_para==0)
	  	  	{
	  	  	Write_1621(31,0x0a);//自
	  	  	}
		  if(COMMCAT_para==1)
	  	  	{
	  	  Write_1621(31,0x0c);//手

	  	  	}

  if(current_para_qianwei>0)
    {
	  WriteAll_1621(0,num6_12Seg+2*current_para_qianwei,2);	//
	    WriteAll_1621(2,num6_12Seg+2*current_para_baiwei,2);	//
  WriteAll_1621(4,num6_12Seg+2*current_para_shiwei,2);	//
  WriteAll_1621(6,num6_12Seg+2*current_para_gewei,2);	//	  	
	  	  		  
    }
	 else if(current_para_baiwei>0)
	 	{
  WriteAll_1621(2,num6_12Seg+2*current_para_baiwei,2);	//
  WriteAll_1621(4,num6_12Seg+2*current_para_shiwei,2);	//
  WriteAll_1621(6,num6_12Seg+2*current_para_gewei,2);	//

	 }
	 else
	 	{
  WriteAll_1621(4,num6_12Seg+2*current_para_shiwei,2);	//
  WriteAll_1621(6,num6_12Seg+2*current_para_gewei,2);	//

	 }

}


void Graf_cos_volt_current_L2(u16 PF,u16 volt_para,u32 current_para,u8 L_C)		 //显示功率因数电压电流

{
  u8 PF_zhengshu,PF_shifenwei,PF_baifenwei,PF_qianfenwei;
  u8 volt_para_qianwei,volt_para_baiwei,volt_para_shiwei,volt_para_gewei;
  u8 current_para_qianwei,current_para_baiwei,current_para_shiwei,current_para_gewei;


u8 num1_5Seg[]=   {0X0F,0X05,0X06,0X00,0X0B,0X06,0X0F,0X02,0X06,0X03,0X0D,0X03,0X0D,0X07,0X07,0X00,0X0F,0X07,0X0F,0X03};
u8 num6_12Seg[]={0X05,0X0F,0X00,0X06,0X06,0X0B,0X02,0X0F,0X03,0X06,0X03,0X0D,0X07,0X0D,0X00,0X07,0X07,0X0F,0X03,0X0F};
u8 num12dpSeg[]=  {0X0F,0X0D,0X06,0X08,0X0B,0X0E,0X0F,0X0A,0X06,0X0B,0X0D,0X0B,0X0D,0X0F,0X07,0X08,0X0F,0X0F,0X0F,0X0B};
u8 num34dpSeg[]=   {0X0D,0X0F,0X08,0X06,0X0E,0X0B,0X0A,0X0F,0X0B,0X06,0X0B,0X0D,0X0F,0X0D,0X08,0X07,0X0F,0X0F,0X0B,0X0F};

Write_1621(28,0x01);//LOG
  Write_1621(31,0x01);//	显示P14功率因数电压电流和P12“动控制”符号
  Write_1621(27,0x0C);//cos 电压电流带L2符号
if(display_nothing_close_open_warn==0)
 Write_1621(24,0x00);	//	空  
if(display_nothing_close_open_warn==1)
 Write_1621(24,0x04);	//	投入
 if(display_nothing_close_open_warn==2)
 Write_1621(24,0x02);	//	切除
 if(display_nothing_close_open_warn==3)
 Write_1621(24,0x01);	//	报警 
  PF_zhengshu=PF/1000;
  PF_shifenwei=(PF%1000)/100;
  PF_baifenwei=	(PF%100)/10;
  PF_qianfenwei=PF%10;

  
  if(L_C_flag_B==1)WriteAll_1621(22,num1_5Seg+2*PF_zhengshu,2);				//
	  if(L_C_flag_B==0)Write_1621(23,0x02);	//	负号
   
  WriteAll_1621(20,num12dpSeg+2*PF_shifenwei,2);	//
WriteAll_1621(18,num1_5Seg+2*PF_baifenwei,2);	
  	
  WriteAll_1621(16,num1_5Seg+2*PF_qianfenwei,2);			//

  volt_para_qianwei=volt_para/1000;
  volt_para_baiwei=(volt_para%1000)/100;
  volt_para_shiwei=(volt_para%100)/10;
  volt_para_gewei=volt_para%10;


  if(volt_para_qianwei>0)
    {
	  WriteAll_1621(14,num1_5Seg+2*volt_para_qianwei,2);	//
	  WriteAll_1621(8,num6_12Seg+2*volt_para_baiwei,2);	//
	  WriteAll_1621(10,num6_12Seg+2*volt_para_shiwei,2);	//
	  WriteAll_1621(12,num34dpSeg+2*volt_para_gewei,2);	//
  	}
	else if(volt_para_baiwei>0)
	{
	 	  WriteAll_1621(8,num6_12Seg+2*volt_para_baiwei,2);	//
	  WriteAll_1621(10,num6_12Seg+2*volt_para_shiwei,2);	//
	  WriteAll_1621(12,num34dpSeg+2*volt_para_gewei,2);	//
	}
	 else
	 {
	  WriteAll_1621(10,num6_12Seg+2*volt_para_shiwei,2);	//
	  WriteAll_1621(12,num34dpSeg+2*volt_para_gewei,2);	//
	 }
  
  current_para_qianwei=current_para/1000;
  current_para_baiwei=(current_para%1000)/100;
  current_para_shiwei=(current_para%100)/10;
  current_para_gewei=current_para%10;
   if(COMMCAT_para==0)
	  	  	{
	  	  	Write_1621(31,0x0a);//自
	  	  	}
		  if(COMMCAT_para==1)
	  	  	{
	  	  Write_1621(31,0x0c);//手

	  	  	}

  if(current_para_qianwei>0)
    {
	  WriteAll_1621(0,num6_12Seg+2*current_para_qianwei,2);	//
	    WriteAll_1621(2,num6_12Seg+2*current_para_baiwei,2);	//
  WriteAll_1621(4,num6_12Seg+2*current_para_shiwei,2);	//
  WriteAll_1621(6,num6_12Seg+2*current_para_gewei,2);	//	  	
	  	  		  
    }
	 else if(current_para_baiwei>0)
	 	{
  WriteAll_1621(2,num6_12Seg+2*current_para_baiwei,2);	//
  WriteAll_1621(4,num6_12Seg+2*current_para_shiwei,2);	//
  WriteAll_1621(6,num6_12Seg+2*current_para_gewei,2);	//

	 }
	 else
	 	{
  WriteAll_1621(4,num6_12Seg+2*current_para_shiwei,2);	//
  WriteAll_1621(6,num6_12Seg+2*current_para_gewei,2);	//

	 }

}


void Graf_cos_volt_current_L3(u16 PF,u16 volt_para,u32 current_para,u8 L_C)		 //显示功率因数电压电流

{
  u8 PF_zhengshu,PF_shifenwei,PF_baifenwei,PF_qianfenwei;
  u8 volt_para_qianwei,volt_para_baiwei,volt_para_shiwei,volt_para_gewei;
  u8 current_para_qianwei,current_para_baiwei,current_para_shiwei,current_para_gewei;


u8 num1_5Seg[]=   {0X0F,0X05,0X06,0X00,0X0B,0X06,0X0F,0X02,0X06,0X03,0X0D,0X03,0X0D,0X07,0X07,0X00,0X0F,0X07,0X0F,0X03};
u8 num6_12Seg[]={0X05,0X0F,0X00,0X06,0X06,0X0B,0X02,0X0F,0X03,0X06,0X03,0X0D,0X07,0X0D,0X00,0X07,0X07,0X0F,0X03,0X0F};
u8 num12dpSeg[]=  {0X0F,0X0D,0X06,0X08,0X0B,0X0E,0X0F,0X0A,0X06,0X0B,0X0D,0X0B,0X0D,0X0F,0X07,0X08,0X0F,0X0F,0X0F,0X0B};
u8 num34dpSeg[]=   {0X0D,0X0F,0X08,0X06,0X0E,0X0B,0X0A,0X0F,0X0B,0X06,0X0B,0X0D,0X0F,0X0D,0X08,0X07,0X0F,0X0F,0X0B,0X0F};

Write_1621(28,0x01);//LOG
  Write_1621(31,0x01);//	显示P14功率因数电压电流和P12“动控制”符号
 Write_1621(26,0x04);	//	带L3符号
if(display_nothing_close_open_warn==0)
 Write_1621(24,0x00);	//	空  
if(display_nothing_close_open_warn==1)
 Write_1621(24,0x04);	//	投入
 if(display_nothing_close_open_warn==2)
 Write_1621(24,0x02);	//	切除
 if(display_nothing_close_open_warn==3)
 Write_1621(24,0x01);	//	报警 
  Write_1621(27,0x08);//cos 电压电流
  PF_zhengshu=PF/1000;
  PF_shifenwei=(PF%1000)/100;
  PF_baifenwei=	(PF%100)/10;
  PF_qianfenwei=PF%10;

  
  if(L_C_flag_B==1)WriteAll_1621(22,num1_5Seg+2*PF_zhengshu,2);				//
	  if(L_C_flag_B==0)Write_1621(23,0x02);	//	负号
   
  WriteAll_1621(20,num12dpSeg+2*PF_shifenwei,2);	//
WriteAll_1621(18,num1_5Seg+2*PF_baifenwei,2);	
  	
  WriteAll_1621(16,num1_5Seg+2*PF_qianfenwei,2);			//

  volt_para_qianwei=volt_para/1000;
  volt_para_baiwei=(volt_para%1000)/100;
  volt_para_shiwei=(volt_para%100)/10;
  volt_para_gewei=volt_para%10;


  if(volt_para_qianwei>0)
    {
	  WriteAll_1621(14,num1_5Seg+2*volt_para_qianwei,2);	//
	  WriteAll_1621(8,num6_12Seg+2*volt_para_baiwei,2);	//
	  WriteAll_1621(10,num6_12Seg+2*volt_para_shiwei,2);	//
	  WriteAll_1621(12,num34dpSeg+2*volt_para_gewei,2);	//
  	}
	else if(volt_para_baiwei>0)
	{
	 	  WriteAll_1621(8,num6_12Seg+2*volt_para_baiwei,2);	//
	  WriteAll_1621(10,num6_12Seg+2*volt_para_shiwei,2);	//
	  WriteAll_1621(12,num34dpSeg+2*volt_para_gewei,2);	//
	}
	 else
	 {
	  WriteAll_1621(10,num6_12Seg+2*volt_para_shiwei,2);	//
	  WriteAll_1621(12,num34dpSeg+2*volt_para_gewei,2);	//
	 }
  
  current_para_qianwei=current_para/1000;
  current_para_baiwei=(current_para%1000)/100;
  current_para_shiwei=(current_para%100)/10;
  current_para_gewei=current_para%10;
   if(COMMCAT_para==0)
	  	  	{
	  	  	Write_1621(31,0x0a);//自
	  	  	}
		  if(COMMCAT_para==1)
	  	  	{
	  	  Write_1621(31,0x0c);//手

	  	  	}

  if(current_para_qianwei>0)
    {
	  WriteAll_1621(0,num6_12Seg+2*current_para_qianwei,2);	//
	    WriteAll_1621(2,num6_12Seg+2*current_para_baiwei,2);	//
  WriteAll_1621(4,num6_12Seg+2*current_para_shiwei,2);	//
  WriteAll_1621(6,num6_12Seg+2*current_para_gewei,2);	//	  	
	  	  		  
    }
	 else if(current_para_baiwei>0)
	 	{
  WriteAll_1621(2,num6_12Seg+2*current_para_baiwei,2);	//
  WriteAll_1621(4,num6_12Seg+2*current_para_shiwei,2);	//
  WriteAll_1621(6,num6_12Seg+2*current_para_gewei,2);	//

	 }
	 else
	 	{
  WriteAll_1621(4,num6_12Seg+2*current_para_shiwei,2);	//
  WriteAll_1621(6,num6_12Seg+2*current_para_gewei,2);	//

	 }

}
void Graf_powuse_poweunuse_freq(u16 powunuse,u16 powuse,u16 freq)
{
	u8 powunuse_baiwei,powunuse_shiwei,powunuse_gewei,powunuse_shifenwei;
	u8 powuse_baiwei,powuse_shiwei,powuse_gewei,powuse_shifenwei;
	u8 freq_shiwei,freq_gewei,freq_shifenwei;
u8 num1_5Seg[]=   {0X0F,0X05,0X06,0X00,0X0B,0X06,0X0F,0X02,0X06,0X03,0X0D,0X03,0X0D,0X07,0X07,0X00,0X0F,0X07,0X0F,0X03};
u8 num6_12Seg[]={0X05,0X0F,0X00,0X06,0X06,0X0B,0X02,0X0F,0X03,0X06,0X03,0X0D,0X07,0X0D,0X00,0X07,0X07,0X0F,0X03,0X0F};
u8 num12dpSeg[]=  {0X0F,0X0D,0X06,0X08,0X0B,0X0E,0X0F,0X0A,0X06,0X0B,0X0D,0X0B,0X0D,0X0F,0X07,0X08,0X0F,0X0F,0X0F,0X0B};
u8 num34dpSeg[]=   {0X0D,0X0F,0X08,0X06,0X0E,0X0B,0X0A,0X0F,0X0B,0X06,0X0B,0X0D,0X0F,0X0D,0X08,0X07,0X0F,0X0F,0X0B,0X0F};
Write_1621(28,0x01);//LOG
  Write_1621(31,0x01);//	显示P14功率因数电压电流和P12“动控制”符号
 if(display_nothing_close_open_warn==0)
 Write_1621(24,0x08);	//	带△符号
 if(display_nothing_close_open_warn==1)
 Write_1621(24,0x0c);	//	带△符号和投入
 if(display_nothing_close_open_warn==2)
 Write_1621(24,0x0a);	//	带△符号和切除
 if(display_nothing_close_open_warn==3)
 Write_1621(24,0x09);	//	带△符号和报警
  Write_1621(25,0x08);//cos 电压电流

   if(COMMCAT_para==0)
	  	  	{
	  	  	Write_1621(31,0x0a);//自
	  	  	}
		  if(COMMCAT_para==1)
	  	  	{
	  	  Write_1621(31,0x0c);//手

	  	  	}
		  
	powunuse_baiwei=powunuse/1000;
	powunuse_shiwei=(powunuse%1000)/100;
	powunuse_gewei=(powunuse%100)/10;
	powunuse_shifenwei=powunuse%10;
	if(powunuse_baiwei>0)
	 {
	  WriteAll_1621(22,num1_5Seg+2*powunuse_baiwei,2);	
      WriteAll_1621(20,num1_5Seg+2*powunuse_shiwei,2);	
      WriteAll_1621(18,num1_5Seg+2*powunuse_gewei,2);	
	  WriteAll_1621(16,num1_5Seg+2*powunuse_shifenwei,2);	
	 }
	 else if(powunuse_shiwei>0)
	 {
	       WriteAll_1621(20,num1_5Seg+2*powunuse_shiwei,2);	//
      WriteAll_1621(18,num1_5Seg+2*powunuse_gewei,2);	//
	  WriteAll_1621(16,num1_5Seg+2*powunuse_shifenwei,2);	//
	 }
	  else
	   {
	      WriteAll_1621(18,num1_5Seg+2*powunuse_gewei,2);	//
	  WriteAll_1621(16,num1_5Seg+2*powunuse_shifenwei,2);	//

	   }
	powuse_baiwei=powuse/1000;
	powuse_shiwei=(powuse%1000)/100;
	powuse_gewei=(powuse%100)/10;
	powuse_shifenwei=powuse%10;
	if(powuse_baiwei>0)
	  {
		WriteAll_1621(14,num1_5Seg+2*powuse_baiwei,2);	//
		WriteAll_1621(8,num6_12Seg+2*powuse_shiwei,2);	//
		WriteAll_1621(10,num6_12Seg+2*powuse_gewei,2);	//
		WriteAll_1621(12,num6_12Seg+2*powuse_shifenwei,2);	//
	  }
	  else if(powuse_shiwei>0)
	   {
	  			WriteAll_1621(8,num6_12Seg+2*powuse_shiwei,2);	//
		WriteAll_1621(10,num6_12Seg+2*powuse_gewei,2);	//
		WriteAll_1621(12,num6_12Seg+2*powuse_shifenwei,2);	//
	   }
	    else 
		{
		WriteAll_1621(10,num6_12Seg+2*powuse_gewei,2);	//
		WriteAll_1621(12,num6_12Seg+2*powuse_shifenwei,2);	
		}

	freq_shiwei=freq/100;
	freq_gewei=(freq%100)/10;
	freq_shifenwei=freq%10;
		  WriteAll_1621(2,num6_12Seg+2*freq_shiwei,2);	//
	  WriteAll_1621(4,num6_12Seg+2*freq_gewei,2);	//
	      WriteAll_1621(6,num34dpSeg+2*freq_shifenwei,2);
	  	  	
	  	  	

}

void Graf_powuse_poweunuse_freq_L1(u16 powunuse,u16 powuse,u16 freq)

{
	u8 powunuse_baiwei,powunuse_shiwei,powunuse_gewei,powunuse_shifenwei;
	u8 powuse_baiwei,powuse_shiwei,powuse_gewei,powuse_shifenwei;
	u8 freq_shiwei,freq_gewei,freq_shifenwei;
u8 num1_5Seg[]=   {0X0F,0X05,0X06,0X00,0X0B,0X06,0X0F,0X02,0X06,0X03,0X0D,0X03,0X0D,0X07,0X07,0X00,0X0F,0X07,0X0F,0X03};
u8 num6_12Seg[]={0X05,0X0F,0X00,0X06,0X06,0X0B,0X02,0X0F,0X03,0X06,0X03,0X0D,0X07,0X0D,0X00,0X07,0X07,0X0F,0X03,0X0F};
u8 num12dpSeg[]=  {0X0F,0X0D,0X06,0X08,0X0B,0X0E,0X0F,0X0A,0X06,0X0B,0X0D,0X0B,0X0D,0X0F,0X07,0X08,0X0F,0X0F,0X0F,0X0B};
u8 num34dpSeg[]=   {0X0D,0X0F,0X08,0X06,0X0E,0X0B,0X0A,0X0F,0X0B,0X06,0X0B,0X0D,0X0F,0X0D,0X08,0X07,0X0F,0X0F,0X0B,0X0F};
Write_1621(28,0x01);//LOG
  Write_1621(31,0x01);//	显示P14功率因数电压电流和P12“动控制”符号
 Write_1621(27,0x02);	//	带L1符号
 if(display_nothing_close_open_warn==0)
 Write_1621(24,0x00);	//	空  
 if(display_nothing_close_open_warn==1)
 Write_1621(24,0x04);	//	投入
 if(display_nothing_close_open_warn==2)
 Write_1621(24,0x02);	//	切除
 if(display_nothing_close_open_warn==3)
 Write_1621(24,0x01);	//	报警 
  Write_1621(25,0x08);//无功有功

   if(COMMCAT_para==0)
	  	  	{
	  	  	Write_1621(31,0x0a);//自
	  	  	}
		  if(COMMCAT_para==1)
	  	  	{
	  	  Write_1621(31,0x0c);//手

	  	  	}
		  
	powunuse_baiwei=powunuse/1000;
	powunuse_shiwei=(powunuse%1000)/100;
	powunuse_gewei=(powunuse%100)/10;
	powunuse_shifenwei=powunuse%10;
	if(powunuse_baiwei>0)
	 {
	  WriteAll_1621(22,num1_5Seg+2*powunuse_baiwei,2);	
      WriteAll_1621(20,num1_5Seg+2*powunuse_shiwei,2);	
      WriteAll_1621(18,num1_5Seg+2*powunuse_gewei,2);	
	  WriteAll_1621(16,num1_5Seg+2*powunuse_shifenwei,2);	
	 }
	 else if(powunuse_shiwei>0)
	 {
	       WriteAll_1621(20,num1_5Seg+2*powunuse_shiwei,2);	//
      WriteAll_1621(18,num1_5Seg+2*powunuse_gewei,2);	//
	  WriteAll_1621(16,num1_5Seg+2*powunuse_shifenwei,2);	//
	 }
	  else
	   {
	      WriteAll_1621(18,num1_5Seg+2*powunuse_gewei,2);	//
	  WriteAll_1621(16,num1_5Seg+2*powunuse_shifenwei,2);	//

	   }
	powuse_baiwei=powuse/1000;
	powuse_shiwei=(powuse%1000)/100;
	powuse_gewei=(powuse%100)/10;
	powuse_shifenwei=powuse%10;
	if(powuse_baiwei>0)
	  {
		WriteAll_1621(14,num1_5Seg+2*powuse_baiwei,2);	//
		WriteAll_1621(8,num6_12Seg+2*powuse_shiwei,2);	//
		WriteAll_1621(10,num6_12Seg+2*powuse_gewei,2);	//
		WriteAll_1621(12,num6_12Seg+2*powuse_shifenwei,2);	//
	  }
	  else if(powuse_shiwei>0)
	   {
	  			WriteAll_1621(8,num6_12Seg+2*powuse_shiwei,2);	//
		WriteAll_1621(10,num6_12Seg+2*powuse_gewei,2);	//
		WriteAll_1621(12,num6_12Seg+2*powuse_shifenwei,2);	//
	   }
	    else 
		{
		WriteAll_1621(10,num6_12Seg+2*powuse_gewei,2);	//
		WriteAll_1621(12,num6_12Seg+2*powuse_shifenwei,2);	
		}

	freq_shiwei=freq/100;
	freq_gewei=(freq%100)/10;
	freq_shifenwei=freq%10;
		  WriteAll_1621(2,num6_12Seg+2*freq_shiwei,2);	//
	  WriteAll_1621(4,num6_12Seg+2*freq_gewei,2);	//
	      WriteAll_1621(6,num34dpSeg+2*freq_shifenwei,2);
	  	  	
	  	  	

}


void Graf_powuse_poweunuse_freq_L2(u16 powunuse,u16 powuse,u16 freq)

{
	u8 powunuse_baiwei,powunuse_shiwei,powunuse_gewei,powunuse_shifenwei;
	u8 powuse_baiwei,powuse_shiwei,powuse_gewei,powuse_shifenwei;
	u8 freq_shiwei,freq_gewei,freq_shifenwei;
u8 num1_5Seg[]=   {0X0F,0X05,0X06,0X00,0X0B,0X06,0X0F,0X02,0X06,0X03,0X0D,0X03,0X0D,0X07,0X07,0X00,0X0F,0X07,0X0F,0X03};
u8 num6_12Seg[]={0X05,0X0F,0X00,0X06,0X06,0X0B,0X02,0X0F,0X03,0X06,0X03,0X0D,0X07,0X0D,0X00,0X07,0X07,0X0F,0X03,0X0F};
u8 num12dpSeg[]=  {0X0F,0X0D,0X06,0X08,0X0B,0X0E,0X0F,0X0A,0X06,0X0B,0X0D,0X0B,0X0D,0X0F,0X07,0X08,0X0F,0X0F,0X0F,0X0B};
u8 num34dpSeg[]=   {0X0D,0X0F,0X08,0X06,0X0E,0X0B,0X0A,0X0F,0X0B,0X06,0X0B,0X0D,0X0F,0X0D,0X08,0X07,0X0F,0X0F,0X0B,0X0F};
Write_1621(28,0x01);//LOG
  Write_1621(31,0x01);//	显示P14功率因数电压电流和P12“动控制”符号
 Write_1621(27,0x04);	//	带L2符号
 if(display_nothing_close_open_warn==0)
 Write_1621(24,0x00);	//	空  
 if(display_nothing_close_open_warn==1)
 Write_1621(24,0x04);	//	投入
 if(display_nothing_close_open_warn==2)
 Write_1621(24,0x02);	//	切除
 if(display_nothing_close_open_warn==3)
 Write_1621(24,0x01);	//	报警 
  Write_1621(25,0x08);//无功有功

   if(COMMCAT_para==0)
	  	  	{
	  	  	Write_1621(31,0x0a);//自
	  	  	}
		  if(COMMCAT_para==1)
	  	  	{
	  	  Write_1621(31,0x0c);//手

	  	  	}
		  
	powunuse_baiwei=powunuse/1000;
	powunuse_shiwei=(powunuse%1000)/100;
	powunuse_gewei=(powunuse%100)/10;
	powunuse_shifenwei=powunuse%10;
	if(powunuse_baiwei>0)
	 {
	  WriteAll_1621(22,num1_5Seg+2*powunuse_baiwei,2);	
      WriteAll_1621(20,num1_5Seg+2*powunuse_shiwei,2);	
      WriteAll_1621(18,num1_5Seg+2*powunuse_gewei,2);	
	  WriteAll_1621(16,num1_5Seg+2*powunuse_shifenwei,2);	
	 }
	 else if(powunuse_shiwei>0)
	 {
	       WriteAll_1621(20,num1_5Seg+2*powunuse_shiwei,2);	//
      WriteAll_1621(18,num1_5Seg+2*powunuse_gewei,2);	//
	  WriteAll_1621(16,num1_5Seg+2*powunuse_shifenwei,2);	//
	 }
	  else
	   {
	      WriteAll_1621(18,num1_5Seg+2*powunuse_gewei,2);	//
	  WriteAll_1621(16,num1_5Seg+2*powunuse_shifenwei,2);	//

	   }
	powuse_baiwei=powuse/1000;
	powuse_shiwei=(powuse%1000)/100;
	powuse_gewei=(powuse%100)/10;
	powuse_shifenwei=powuse%10;
	if(powuse_baiwei>0)
	  {
		WriteAll_1621(14,num1_5Seg+2*powuse_baiwei,2);	//
		WriteAll_1621(8,num6_12Seg+2*powuse_shiwei,2);	//
		WriteAll_1621(10,num6_12Seg+2*powuse_gewei,2);	//
		WriteAll_1621(12,num6_12Seg+2*powuse_shifenwei,2);	//
	  }
	  else if(powuse_shiwei>0)
	   {
	  			WriteAll_1621(8,num6_12Seg+2*powuse_shiwei,2);	//
		WriteAll_1621(10,num6_12Seg+2*powuse_gewei,2);	//
		WriteAll_1621(12,num6_12Seg+2*powuse_shifenwei,2);	//
	   }
	    else 
		{
		WriteAll_1621(10,num6_12Seg+2*powuse_gewei,2);	//
		WriteAll_1621(12,num6_12Seg+2*powuse_shifenwei,2);	
		}

	freq_shiwei=freq/100;
	freq_gewei=(freq%100)/10;
	freq_shifenwei=freq%10;
		  WriteAll_1621(2,num6_12Seg+2*freq_shiwei,2);	//
	  WriteAll_1621(4,num6_12Seg+2*freq_gewei,2);	//
	      WriteAll_1621(6,num34dpSeg+2*freq_shifenwei,2);
	  	  	
	  	  	

}


void Graf_powuse_poweunuse_freq_L3(u16 powunuse,u16 powuse,u16 freq)

{
	u8 powunuse_baiwei,powunuse_shiwei,powunuse_gewei,powunuse_shifenwei;
	u8 powuse_baiwei,powuse_shiwei,powuse_gewei,powuse_shifenwei;
	u8 freq_shiwei,freq_gewei,freq_shifenwei;
u8 num1_5Seg[]=   {0X0F,0X05,0X06,0X00,0X0B,0X06,0X0F,0X02,0X06,0X03,0X0D,0X03,0X0D,0X07,0X07,0X00,0X0F,0X07,0X0F,0X03};
u8 num6_12Seg[]={0X05,0X0F,0X00,0X06,0X06,0X0B,0X02,0X0F,0X03,0X06,0X03,0X0D,0X07,0X0D,0X00,0X07,0X07,0X0F,0X03,0X0F};
u8 num12dpSeg[]=  {0X0F,0X0D,0X06,0X08,0X0B,0X0E,0X0F,0X0A,0X06,0X0B,0X0D,0X0B,0X0D,0X0F,0X07,0X08,0X0F,0X0F,0X0F,0X0B};
u8 num34dpSeg[]=   {0X0D,0X0F,0X08,0X06,0X0E,0X0B,0X0A,0X0F,0X0B,0X06,0X0B,0X0D,0X0F,0X0D,0X08,0X07,0X0F,0X0F,0X0B,0X0F};
Write_1621(28,0x01);//LOG
  Write_1621(31,0x01);//	显示P14功率因数电压电流和P12“动控制”符号
 Write_1621(26,0x04);	//	带L3符号
 if(display_nothing_close_open_warn==0)
 Write_1621(24,0x00);	//	空  
 if(display_nothing_close_open_warn==1)
 Write_1621(24,0x04);	//	投入
 if(display_nothing_close_open_warn==2)
 Write_1621(24,0x02);	//	切除
 if(display_nothing_close_open_warn==3)
 Write_1621(24,0x01);	//	报警 
  Write_1621(25,0x08);//无功有功

   if(COMMCAT_para==0)
	  	  	{
	  	  	Write_1621(31,0x0a);//自
	  	  	}
		  if(COMMCAT_para==1)
	  	  	{
	  	  Write_1621(31,0x0c);//手

	  	  	}
		  
	powunuse_baiwei=powunuse/1000;
	powunuse_shiwei=(powunuse%1000)/100;
	powunuse_gewei=(powunuse%100)/10;
	powunuse_shifenwei=powunuse%10;
	if(powunuse_baiwei>0)
	 {
	  WriteAll_1621(22,num1_5Seg+2*powunuse_baiwei,2);	
      WriteAll_1621(20,num1_5Seg+2*powunuse_shiwei,2);	
      WriteAll_1621(18,num1_5Seg+2*powunuse_gewei,2);	
	  WriteAll_1621(16,num1_5Seg+2*powunuse_shifenwei,2);	
	 }
	 else if(powunuse_shiwei>0)
	 {
	       WriteAll_1621(20,num1_5Seg+2*powunuse_shiwei,2);	//
      WriteAll_1621(18,num1_5Seg+2*powunuse_gewei,2);	//
	  WriteAll_1621(16,num1_5Seg+2*powunuse_shifenwei,2);	//
	 }
	  else
	   {
	      WriteAll_1621(18,num1_5Seg+2*powunuse_gewei,2);	//
	  WriteAll_1621(16,num1_5Seg+2*powunuse_shifenwei,2);	//

	   }
	powuse_baiwei=powuse/1000;
	powuse_shiwei=(powuse%1000)/100;
	powuse_gewei=(powuse%100)/10;
	powuse_shifenwei=powuse%10;
	if(powuse_baiwei>0)
	  {
		WriteAll_1621(14,num1_5Seg+2*powuse_baiwei,2);	//
		WriteAll_1621(8,num6_12Seg+2*powuse_shiwei,2);	//
		WriteAll_1621(10,num6_12Seg+2*powuse_gewei,2);	//
		WriteAll_1621(12,num6_12Seg+2*powuse_shifenwei,2);	//
	  }
	  else if(powuse_shiwei>0)
	   {
	  			WriteAll_1621(8,num6_12Seg+2*powuse_shiwei,2);	//
		WriteAll_1621(10,num6_12Seg+2*powuse_gewei,2);	//
		WriteAll_1621(12,num6_12Seg+2*powuse_shifenwei,2);	//
	   }
	    else 
		{
		WriteAll_1621(10,num6_12Seg+2*powuse_gewei,2);	//
		WriteAll_1621(12,num6_12Seg+2*powuse_shifenwei,2);	
		}

	freq_shiwei=freq/100;
	freq_gewei=(freq%100)/10;
	freq_shifenwei=freq%10;
		  WriteAll_1621(2,num6_12Seg+2*freq_shiwei,2);	//
	  WriteAll_1621(4,num6_12Seg+2*freq_gewei,2);	//
	      WriteAll_1621(6,num34dpSeg+2*freq_shifenwei,2);
	  	  	
	  	  	

}
void Graf_temp_hv_hi(u16 TEMP,u16 HV,u16 HI)	   //显示温度电压谐波电流谐波
{
	u8 	TEMP_baiwei,TEMP_shiwei,TEMP_gewei,TEMP_shifenwei;
	u8  HV_shiwei, HV_gewei,HV_shifenwei;
	u8  HI_shiwei, HI_gewei,HI_shifenwei;
u8 num1_5Seg[]=   {0X0F,0X05,0X06,0X00,0X0B,0X06,0X0F,0X02,0X06,0X03,0X0D,0X03,0X0D,0X07,0X07,0X00,0X0F,0X07,0X0F,0X03};
u8 num6_12Seg[]={0X05,0X0F,0X00,0X06,0X06,0X0B,0X02,0X0F,0X03,0X06,0X03,0X0D,0X07,0X0D,0X00,0X07,0X07,0X0F,0X03,0X0F};
u8 num12dpSeg[]=  {0X0F,0X0D,0X06,0X08,0X0B,0X0E,0X0F,0X0A,0X06,0X0B,0X0D,0X0B,0X0D,0X0F,0X07,0X08,0X0F,0X0F,0X0F,0X0B};
u8 num34dpSeg[]=   {0X0D,0X0F,0X08,0X06,0X0E,0X0B,0X0A,0X0F,0X0B,0X06,0X0B,0X0D,0X0F,0X0D,0X08,0X07,0X0F,0X0F,0X0B,0X0F};
Write_1621(28,0x01);//LOG
  Write_1621(31,0x01);//	显示P14功率因数电压电流和P12“动控制”符号
 if(display_nothing_close_open_warn==0)
 Write_1621(24,0x08);	//	带△符号
 if(display_nothing_close_open_warn==1)
 Write_1621(24,0x0c);	//	带△符号和投入
 if(display_nothing_close_open_warn==2)
 Write_1621(24,0x0a);	//	带△符号和切除
 if(display_nothing_close_open_warn==3)
 Write_1621(24,0x09);	//	带△符号和报警  Write_1621(26,0x08);//温度谐波
  Write_1621(26,0x08);//温度谐波

   if(COMMCAT_para==0)
	  	  	{
	  	  	Write_1621(31,0x0a);//自
	  	  	}
		  if(COMMCAT_para==1)
	  	  	{
	  	  Write_1621(31,0x0c);//手

	  	  	}
	TEMP_baiwei=TEMP/1000;
	TEMP_shiwei=(TEMP%1000)/100;
	TEMP_gewei=(TEMP%100)/10;
	TEMP_shifenwei=TEMP%10;
	if(TEMP_baiwei>0)
	  {
		WriteAll_1621(22,num1_5Seg+2*TEMP_baiwei,2);	//
    	WriteAll_1621(20,num1_5Seg+2*TEMP_shiwei,2);	//
    	WriteAll_1621(18,num1_5Seg+2*TEMP_gewei,2);	//
		WriteAll_1621(16,num12dpSeg+2*TEMP_shifenwei,2);	//
	  }
	  else if(TEMP_shiwei>0)
	  {
    	WriteAll_1621(20,num1_5Seg+2*TEMP_shiwei,2);	//
    	WriteAll_1621(18,num1_5Seg+2*TEMP_gewei,2);	//
		WriteAll_1621(16,num12dpSeg+2*TEMP_shifenwei,2);	//
	  }	  
	  else
	  {
    	WriteAll_1621(18,num1_5Seg+2*TEMP_gewei,2);	//
		WriteAll_1621(16,num12dpSeg+2*TEMP_shifenwei,2);	//
	  }
	  
	HV_shiwei=HV/100;
	HV_gewei=(HV%100)/10;
	HV_shifenwei=HV%10;
	if(HV_shiwei>0)
	 {
	  WriteAll_1621(8,num6_12Seg+2*HV_shiwei,2);	//
	  WriteAll_1621(10,num6_12Seg+2*HV_gewei,2);	//
	  WriteAll_1621(12,num34dpSeg+2*HV_shifenwei,2);	//
	 }
	 else 
	 {
	  WriteAll_1621(10,num6_12Seg+2*HV_gewei,2);	//
	  WriteAll_1621(12,num34dpSeg+2*HV_shifenwei,2);	//
	 }
	HI_shiwei=HI/100;
	HI_gewei=(HI%100)/10;
	HI_shifenwei=HI%10;
	if(HI_shiwei>0)
	{
	  	  	
	  	  	
	  WriteAll_1621(2,num6_12Seg+2*HI_shiwei,2);	//
	  WriteAll_1621(4,num6_12Seg+2*HI_gewei,2);	//	  	  	
	WriteAll_1621(6,num34dpSeg+2*HI_shifenwei,2);	//
	}
else
{
	  WriteAll_1621(4,num6_12Seg+2*HI_gewei,2);	//	  	  	
	WriteAll_1621(6,num34dpSeg+2*HI_shifenwei,2);	//
}


}

void Graf_temp_hv_hi_L1(u16 TEMP,u16 HV,u16 HI)	   //显示温度电压谐波电流谐波
{
	u8 	TEMP_baiwei,TEMP_shiwei,TEMP_gewei,TEMP_shifenwei;
	u8  HV_shiwei, HV_gewei,HV_shifenwei;
	u8  HI_shiwei, HI_gewei,HI_shifenwei;
u8 num1_5Seg[]=   {0X0F,0X05,0X06,0X00,0X0B,0X06,0X0F,0X02,0X06,0X03,0X0D,0X03,0X0D,0X07,0X07,0X00,0X0F,0X07,0X0F,0X03};
u8 num6_12Seg[]={0X05,0X0F,0X00,0X06,0X06,0X0B,0X02,0X0F,0X03,0X06,0X03,0X0D,0X07,0X0D,0X00,0X07,0X07,0X0F,0X03,0X0F};
u8 num12dpSeg[]=  {0X0F,0X0D,0X06,0X08,0X0B,0X0E,0X0F,0X0A,0X06,0X0B,0X0D,0X0B,0X0D,0X0F,0X07,0X08,0X0F,0X0F,0X0F,0X0B};
u8 num34dpSeg[]=   {0X0D,0X0F,0X08,0X06,0X0E,0X0B,0X0A,0X0F,0X0B,0X06,0X0B,0X0D,0X0F,0X0D,0X08,0X07,0X0F,0X0F,0X0B,0X0F};
Write_1621(28,0x01);//LOG
  Write_1621(31,0x01);//	显示P14功率因数电压电流和P12“动控制”符号
 Write_1621(27,0x02);	//	带L1符号
 if(display_nothing_close_open_warn==0)
 Write_1621(24,0x00);	//	空  
 if(display_nothing_close_open_warn==1)
 Write_1621(24,0x04);	//	投入
 if(display_nothing_close_open_warn==2)
 Write_1621(24,0x02);	//	切除
 if(display_nothing_close_open_warn==3)
 Write_1621(24,0x01);	//	报警 
  Write_1621(26,0x08);//温度谐波

   if(COMMCAT_para==0)
	  	  	{
	  	  	Write_1621(31,0x0a);//自
	  	  	}
		  if(COMMCAT_para==1)
	  	  	{
	  	  Write_1621(31,0x0c);//手

	  	  	}
	TEMP_baiwei=TEMP/1000;
	TEMP_shiwei=(TEMP%1000)/100;
	TEMP_gewei=(TEMP%100)/10;
	TEMP_shifenwei=TEMP%10;
	if(TEMP_baiwei>0)
	  {
		WriteAll_1621(22,num1_5Seg+2*TEMP_baiwei,2);	//
    	WriteAll_1621(20,num1_5Seg+2*TEMP_shiwei,2);	//
    	WriteAll_1621(18,num1_5Seg+2*TEMP_gewei,2);	//
		WriteAll_1621(16,num12dpSeg+2*TEMP_shifenwei,2);	//
	  }
	  else if(TEMP_shiwei>0)
	  {
    	WriteAll_1621(20,num1_5Seg+2*TEMP_shiwei,2);	//
    	WriteAll_1621(18,num1_5Seg+2*TEMP_gewei,2);	//
		WriteAll_1621(16,num12dpSeg+2*TEMP_shifenwei,2);	//
	  }	  
	  else
	  {
    	WriteAll_1621(18,num1_5Seg+2*TEMP_gewei,2);	//
		WriteAll_1621(16,num12dpSeg+2*TEMP_shifenwei,2);	//
	  }
	  
	HV_shiwei=HV/100;
	HV_gewei=(HV%100)/10;
	HV_shifenwei=HV%10;
	if(HV_shiwei>0)
	 {
	  WriteAll_1621(8,num6_12Seg+2*HV_shiwei,2);	//
	  WriteAll_1621(10,num6_12Seg+2*HV_gewei,2);	//
	  WriteAll_1621(12,num34dpSeg+2*HV_shifenwei,2);	//
	 }
	 else 
	 {
	  WriteAll_1621(10,num6_12Seg+2*HV_gewei,2);	//
	  WriteAll_1621(12,num34dpSeg+2*HV_shifenwei,2);	//
	 }
	HI_shiwei=HI/100;
	HI_gewei=(HI%100)/10;
	HI_shifenwei=HI%10;
	if(HI_shiwei>0)
	{
	  	  	
	  	  	
	  WriteAll_1621(2,num6_12Seg+2*HI_shiwei,2);	//
	  WriteAll_1621(4,num6_12Seg+2*HI_gewei,2);	//	  	  	
	WriteAll_1621(6,num34dpSeg+2*HI_shifenwei,2);	//
	}
else
{
	  WriteAll_1621(4,num6_12Seg+2*HI_gewei,2);	//	  	  	
	WriteAll_1621(6,num34dpSeg+2*HI_shifenwei,2);	//
}


}

void Graf_temp_hv_hi_L2(u16 TEMP,u16 HV,u16 HI)	   //显示温度电压谐波电流谐波
{
	u8 	TEMP_baiwei,TEMP_shiwei,TEMP_gewei,TEMP_shifenwei;
	u8  HV_shiwei, HV_gewei,HV_shifenwei;
	u8  HI_shiwei, HI_gewei,HI_shifenwei;
u8 num1_5Seg[]=   {0X0F,0X05,0X06,0X00,0X0B,0X06,0X0F,0X02,0X06,0X03,0X0D,0X03,0X0D,0X07,0X07,0X00,0X0F,0X07,0X0F,0X03};
u8 num6_12Seg[]={0X05,0X0F,0X00,0X06,0X06,0X0B,0X02,0X0F,0X03,0X06,0X03,0X0D,0X07,0X0D,0X00,0X07,0X07,0X0F,0X03,0X0F};
u8 num12dpSeg[]=  {0X0F,0X0D,0X06,0X08,0X0B,0X0E,0X0F,0X0A,0X06,0X0B,0X0D,0X0B,0X0D,0X0F,0X07,0X08,0X0F,0X0F,0X0F,0X0B};
u8 num34dpSeg[]=   {0X0D,0X0F,0X08,0X06,0X0E,0X0B,0X0A,0X0F,0X0B,0X06,0X0B,0X0D,0X0F,0X0D,0X08,0X07,0X0F,0X0F,0X0B,0X0F};
Write_1621(28,0x01);//LOG
  Write_1621(31,0x01);//	显示P14功率因数电压电流和P12“动控制”符号
 Write_1621(27,0x04);	//	带L2符号
 if(display_nothing_close_open_warn==0)
 Write_1621(24,0x00);	//	空  
 if(display_nothing_close_open_warn==1)
 Write_1621(24,0x04);	//	投入
 if(display_nothing_close_open_warn==2)
 Write_1621(24,0x02);	//	切除
 if(display_nothing_close_open_warn==3)
 Write_1621(24,0x01);	//	报警 
  Write_1621(26,0x08);//温度谐波

   if(COMMCAT_para==0)
	  	  	{
	  	  	Write_1621(31,0x0a);//自
	  	  	}
		  if(COMMCAT_para==1)
	  	  	{
	  	  Write_1621(31,0x0c);//手

	  	  	}
	TEMP_baiwei=TEMP/1000;
	TEMP_shiwei=(TEMP%1000)/100;
	TEMP_gewei=(TEMP%100)/10;
	TEMP_shifenwei=TEMP%10;
	if(TEMP_baiwei>0)
	  {
		WriteAll_1621(22,num1_5Seg+2*TEMP_baiwei,2);	//
    	WriteAll_1621(20,num1_5Seg+2*TEMP_shiwei,2);	//
    	WriteAll_1621(18,num1_5Seg+2*TEMP_gewei,2);	//
		WriteAll_1621(16,num12dpSeg+2*TEMP_shifenwei,2);	//
	  }
	  else if(TEMP_shiwei>0)
	  {
    	WriteAll_1621(20,num1_5Seg+2*TEMP_shiwei,2);	//
    	WriteAll_1621(18,num1_5Seg+2*TEMP_gewei,2);	//
		WriteAll_1621(16,num12dpSeg+2*TEMP_shifenwei,2);	//
	  }	  
	  else
	  {
    	WriteAll_1621(18,num1_5Seg+2*TEMP_gewei,2);	//
		WriteAll_1621(16,num12dpSeg+2*TEMP_shifenwei,2);	//
	  }
	  
	HV_shiwei=HV/100;
	HV_gewei=(HV%100)/10;
	HV_shifenwei=HV%10;
	if(HV_shiwei>0)
	 {
	  WriteAll_1621(8,num6_12Seg+2*HV_shiwei,2);	//
	  WriteAll_1621(10,num6_12Seg+2*HV_gewei,2);	//
	  WriteAll_1621(12,num34dpSeg+2*HV_shifenwei,2);	//
	 }
	 else 
	 {
	  WriteAll_1621(10,num6_12Seg+2*HV_gewei,2);	//
	  WriteAll_1621(12,num34dpSeg+2*HV_shifenwei,2);	//
	 }
	HI_shiwei=HI/100;
	HI_gewei=(HI%100)/10;
	HI_shifenwei=HI%10;
	if(HI_shiwei>0)
	{
	  	  	
	  	  	
	  WriteAll_1621(2,num6_12Seg+2*HI_shiwei,2);	//
	  WriteAll_1621(4,num6_12Seg+2*HI_gewei,2);	//	  	  	
	WriteAll_1621(6,num34dpSeg+2*HI_shifenwei,2);	//
	}
else
{
	  WriteAll_1621(4,num6_12Seg+2*HI_gewei,2);	//	  	  	
	WriteAll_1621(6,num34dpSeg+2*HI_shifenwei,2);	//
}


}
void Graf_temp_hv_hi_L3(u16 TEMP,u16 HV,u16 HI)	   //显示温度电压谐波电流谐波
{
	u8 	TEMP_baiwei,TEMP_shiwei,TEMP_gewei,TEMP_shifenwei;
	u8  HV_shiwei, HV_gewei,HV_shifenwei;
	u8  HI_shiwei, HI_gewei,HI_shifenwei;
u8 num1_5Seg[]=   {0X0F,0X05,0X06,0X00,0X0B,0X06,0X0F,0X02,0X06,0X03,0X0D,0X03,0X0D,0X07,0X07,0X00,0X0F,0X07,0X0F,0X03};
u8 num6_12Seg[]={0X05,0X0F,0X00,0X06,0X06,0X0B,0X02,0X0F,0X03,0X06,0X03,0X0D,0X07,0X0D,0X00,0X07,0X07,0X0F,0X03,0X0F};
u8 num12dpSeg[]=  {0X0F,0X0D,0X06,0X08,0X0B,0X0E,0X0F,0X0A,0X06,0X0B,0X0D,0X0B,0X0D,0X0F,0X07,0X08,0X0F,0X0F,0X0F,0X0B};
u8 num34dpSeg[]=   {0X0D,0X0F,0X08,0X06,0X0E,0X0B,0X0A,0X0F,0X0B,0X06,0X0B,0X0D,0X0F,0X0D,0X08,0X07,0X0F,0X0F,0X0B,0X0F};
Write_1621(28,0x01);//LOG
  Write_1621(31,0x01);//	显示P14功率因数电压电流和P12“动控制”符号
  Write_1621(26,0x0C);//温度谐波 L3 
  if(display_nothing_close_open_warn==0)
 Write_1621(24,0x00);	//	空  
  if(display_nothing_close_open_warn==1)
 Write_1621(24,0x04);	//	投入
 if(display_nothing_close_open_warn==2)
 Write_1621(24,0x02);	//	切除
 if(display_nothing_close_open_warn==3)
 Write_1621(24,0x01);	//	报警 

   if(COMMCAT_para==0)
	  	  	{
	  	  	Write_1621(31,0x0a);//自
	  	  	}
		  if(COMMCAT_para==1)
	  	  	{
	  	  Write_1621(31,0x0c);//手

	  	  	}
	TEMP_baiwei=TEMP/1000;
	TEMP_shiwei=(TEMP%1000)/100;
	TEMP_gewei=(TEMP%100)/10;
	TEMP_shifenwei=TEMP%10;
	if(TEMP_baiwei>0)
	  {
		WriteAll_1621(22,num1_5Seg+2*TEMP_baiwei,2);	//
    	WriteAll_1621(20,num1_5Seg+2*TEMP_shiwei,2);	//
    	WriteAll_1621(18,num1_5Seg+2*TEMP_gewei,2);	//
		WriteAll_1621(16,num12dpSeg+2*TEMP_shifenwei,2);	//
	  }
	  else if(TEMP_shiwei>0)
	  {
    	WriteAll_1621(20,num1_5Seg+2*TEMP_shiwei,2);	//
    	WriteAll_1621(18,num1_5Seg+2*TEMP_gewei,2);	//
		WriteAll_1621(16,num12dpSeg+2*TEMP_shifenwei,2);	//
	  }	  
	  else
	  {
    	WriteAll_1621(18,num1_5Seg+2*TEMP_gewei,2);	//
		WriteAll_1621(16,num12dpSeg+2*TEMP_shifenwei,2);	//
	  }
	  
	HV_shiwei=HV/100;
	HV_gewei=(HV%100)/10;
	HV_shifenwei=HV%10;
	if(HV_shiwei>0)
	 {
	  WriteAll_1621(8,num6_12Seg+2*HV_shiwei,2);	//
	  WriteAll_1621(10,num6_12Seg+2*HV_gewei,2);	//
	  WriteAll_1621(12,num34dpSeg+2*HV_shifenwei,2);	//
	 }
	 else 
	 {
	  WriteAll_1621(10,num6_12Seg+2*HV_gewei,2);	//
	  WriteAll_1621(12,num34dpSeg+2*HV_shifenwei,2);	//
	 }
	HI_shiwei=HI/100;
	HI_gewei=(HI%100)/10;
	HI_shifenwei=HI%10;
	if(HI_shiwei>0)
	{
	  	  	
	  	  	
	  WriteAll_1621(2,num6_12Seg+2*HI_shiwei,2);	//
	  WriteAll_1621(4,num6_12Seg+2*HI_gewei,2);	//	  	  	
	WriteAll_1621(6,num34dpSeg+2*HI_shifenwei,2);	//
	}
else
{
	  WriteAll_1621(4,num6_12Seg+2*HI_gewei,2);	//	  	  	
	WriteAll_1621(6,num34dpSeg+2*HI_shifenwei,2);	//
}


}
void Graf_VER(u8 VER)
{
 	u8 VER_baiwei,VER_shiwei,VER_gewei;
u8 num6_12Seg[]={0X05,0X0F,0X00,0X06,0X06,0X0B,0X02,0X0F,0X03,0X06,0X03,0X0D,0X07,0X0D,0X00,0X07,0X07,0X0F,0X03,0X0F};
u8 num34dpSeg[]=   {0X0D,0X0F,0X08,0X06,0X0E,0X0B,0X0A,0X0F,0X0B,0X06,0X0B,0X0D,0X0F,0X0D,0X08,0X07,0X0F,0X0F,0X0B,0X0F};

Write_1621(28,0x01);//LOG
  Write_1621(31,0x01);//	显示P14功率因数电压电流和P12“动控制”符号
  Write_1621(25,0x01);//版本

   if(COMMCAT_para==0)
	  	  	{
	  	  	Write_1621(31,0x0a);//自
	  	  	}
		  if(COMMCAT_para==1)
	  	  	{
	  	  Write_1621(31,0x0c);//手

	  	  	}
	VER_baiwei=VER/100;
	VER_shiwei=(VER%100)/10;
	VER_gewei=VER%10;
	WriteAll_1621(8,num6_12Seg+2*VER_baiwei,2);	//
    WriteAll_1621(10,num6_12Seg+2*VER_shiwei,2);	//
    WriteAll_1621(12,num34dpSeg+2*VER_gewei,2);	//

}
void Graf_current_value(u32 Current_A,u32 Current_B,u32 Current_C)
{
   	u8 Current_A_qianwei,Current_A_baiwei,Current_A_shiwei,Current_A_gewei;
	u8 Current_B_qianwei,Current_B_baiwei,Current_B_shiwei,Current_B_gewei;
	u8 Current_C_qianwei,Current_C_baiwei,Current_C_shiwei,Current_C_gewei;
 u8 num1_5Seg[]=   {0X0F,0X05,0X06,0X00,0X0B,0X06,0X0F,0X02,0X06,0X03,0X0D,0X03,0X0D,0X07,0X07,0X00,0X0F,0X07,0X0F,0X03};
u8 num6_12Seg[]={0X05,0X0F,0X00,0X06,0X06,0X0B,0X02,0X0F,0X03,0X06,0X03,0X0D,0X07,0X0D,0X00,0X07,0X07,0X0F,0X03,0X0F};

Write_1621(28,0x01);//LOG
  Write_1621(31,0x01);//	显示P14功率因数电压电流和P12“动控制”符号

   if(COMMCAT_para==0)
	  	  	{
	  	  	Write_1621(31,0x0a);//自
	  	  	}
		  if(COMMCAT_para==1)
	  	  	{
	  	  Write_1621(31,0x0c);//手

	  	  	}




	Current_A_qianwei=Current_A/1000;
	Current_A_baiwei=(Current_A%1000)/100;
	Current_A_shiwei=(Current_A%100)/10;
	Current_A_gewei=Current_A%10;
	
	Current_B_qianwei=Current_B/1000;
	Current_B_baiwei=(Current_B%1000)/100;
	Current_B_shiwei=(Current_B%100)/10;
	Current_B_gewei=Current_B%10;
	
	Current_C_qianwei=Current_C/1000;
	Current_C_baiwei=(Current_C%1000)/100;
	Current_C_shiwei=(Current_C%100)/10;
	Current_C_gewei=Current_C%10;

	if(Current_A_qianwei>0)
	{
	  WriteAll_1621(22,num1_5Seg+2*Current_A_qianwei,2);
	  WriteAll_1621(20,num1_5Seg+2*Current_A_baiwei,2);
	  WriteAll_1621(18,num1_5Seg+2*Current_A_shiwei,2);
	  WriteAll_1621(16,num1_5Seg+2*Current_A_gewei,2);
	}
		else if(Current_A_baiwei>0)
		{
	  WriteAll_1621(20,num1_5Seg+2*Current_A_baiwei,2);
	  WriteAll_1621(18,num1_5Seg+2*Current_A_shiwei,2);
	  WriteAll_1621(16,num1_5Seg+2*Current_A_gewei,2);
		}
		else if(Current_A_shiwei>0)
			{
	  WriteAll_1621(18,num1_5Seg+2*Current_A_shiwei,2);
	  WriteAll_1621(16,num1_5Seg+2*Current_A_gewei,2);			
	  }
			else WriteAll_1621(16,num1_5Seg+2*Current_A_gewei,2);			
	
			
		if(Current_B_qianwei>0)
		{
		  WriteAll_1621(14,num1_5Seg+2*Current_B_qianwei,2);
		  WriteAll_1621(8,num6_12Seg+2*Current_B_baiwei,2);
		  WriteAll_1621(10,num6_12Seg+2*Current_B_shiwei,2);
		  WriteAll_1621(12,num6_12Seg+2*Current_B_gewei,2);
		}
			else if(Current_B_baiwei>0)
			{
		  WriteAll_1621(8,num6_12Seg+2*Current_B_baiwei,2);
		  WriteAll_1621(10,num6_12Seg+2*Current_B_shiwei,2);
		  WriteAll_1621(12,num6_12Seg+2*Current_B_gewei,2);
			}
				else if(Current_B_shiwei>0)
				{
		  WriteAll_1621(10,num6_12Seg+2*Current_B_shiwei,2);
		  WriteAll_1621(12,num6_12Seg+2*Current_B_gewei,2);
				}
					else  WriteAll_1621(12,num6_12Seg+2*Current_B_gewei,2);

	  if(Current_C_qianwei>0)
		{
		  WriteAll_1621(0,num6_12Seg+2*Current_C_qianwei,2);	  	  		  	  	
	  WriteAll_1621(2,num6_12Seg+2*Current_C_baiwei,2);	//
	  WriteAll_1621(4,num6_12Seg+2*Current_C_shiwei,2);	//	  	  	
		  WriteAll_1621(6,num6_12Seg+2*Current_C_gewei,2);
		}
			else if(Current_C_baiwei>0)
			{
	  WriteAll_1621(2,num6_12Seg+2*Current_C_baiwei,2);	//
	  WriteAll_1621(4,num6_12Seg+2*Current_C_shiwei,2);	//	  	  	
		  WriteAll_1621(6,num6_12Seg+2*Current_C_gewei,2);
		}
				else if(Current_C_shiwei>0)
				{
	  WriteAll_1621(4,num6_12Seg+2*Current_C_shiwei,2);	  	  	
		  WriteAll_1621(6,num6_12Seg+2*Current_C_gewei,2);
		}
					else 
					{
		  WriteAll_1621(6,num6_12Seg+2*Current_C_gewei,2);
		}


}
void HT595_Send_Byte(u8 state)
{                        
    u8 t; 
    for(t=0;t<8;t++)
    {    
		DATA_0;          
        if((state&0x80)==0)
		WR_0;
		else WR_1;
		delay_us(3);
		DATA_1;
        state<<=1; 	  

    }

	delay_us(10);
}

u8 remenber[33];
void set_74hc273(int number,int onoff)
{

GPIO_InitTypeDef GPIO_InitStructure;

int number_in;

 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);


 	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5| GPIO_Pin_6;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 //推挽输出
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 		GPIO_Init(GPIOE, &GPIO_InitStructure);	   //本工程使用 


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 //推挽输出
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 		GPIO_Init(GPIOB, &GPIO_InitStructure);	   //本工程使用 

if(onoff==1) remenber[number]=1;
if(onoff==0) remenber[number]=0;

 if(number<=8&&number>0)
 {
number_in=0;

 }
if(number<=16&&number>8)
	{
	number_in=8;
}
if(number<=24&&number>16)
	{
	number_in=16;
}

if(number<=32&&number>24)
{
number_in=24;
	}

if (remenber[1+number_in]==1)GPIO_SetBits(GPIOE, GPIO_Pin_0);	
 if(remenber[1+number_in]==0)GPIO_ResetBits(GPIOE, GPIO_Pin_0);	

if(remenber[2+number_in]==1)GPIO_SetBits(GPIOC, GPIO_Pin_9);	
if(remenber[2+number_in]==0)GPIO_ResetBits(GPIOC, GPIO_Pin_9);	


if(remenber[3+number_in]==1)GPIO_SetBits(GPIOE, GPIO_Pin_1);	
if(remenber[3+number_in]==0)GPIO_ResetBits(GPIOE, GPIO_Pin_1);	

if(remenber[4+number_in]==1)GPIO_SetBits(GPIOE, GPIO_Pin_2);	
if(remenber[4+number_in]==0)GPIO_ResetBits(GPIOE, GPIO_Pin_2);	

if(remenber[5+number_in]==1)GPIO_SetBits(GPIOE, GPIO_Pin_3);	
if(remenber[5+number_in]==0)GPIO_ResetBits(GPIOE, GPIO_Pin_3);	

if(remenber[6+number_in]==1)GPIO_SetBits(GPIOE, GPIO_Pin_4);	
if(remenber[6+number_in]==0)GPIO_ResetBits(GPIOE, GPIO_Pin_4);	

if(remenber[7+number_in]==1)GPIO_SetBits(GPIOE, GPIO_Pin_5);	
if(remenber[7+number_in]==0)GPIO_ResetBits(GPIOE, GPIO_Pin_5);	

if(remenber[8+number_in]==1)GPIO_SetBits(GPIOE, GPIO_Pin_6);	
if(remenber[8+number_in]==0)GPIO_ResetBits(GPIOE, GPIO_Pin_6);	
{
if(number<=8&&number>0)
{
GPIO_ResetBits(GPIOB, GPIO_Pin_1);
delay_us(2);
 GPIO_SetBits(GPIOB, GPIO_Pin_1);	
}

if(number<=16&&number>8)
{
GPIO_ResetBits(GPIOB, GPIO_Pin_2);
delay_us(2);
 GPIO_SetBits(GPIOB, GPIO_Pin_2);	

}
if(number<=24&&number>16)
{
GPIO_ResetBits(GPIOB, GPIO_Pin_3);
delay_us(2);
 GPIO_SetBits(GPIOB, GPIO_Pin_3);	

}

if(number<=32&&number>24)
{
GPIO_ResetBits(GPIOB, GPIO_Pin_4);
delay_us(2);
 GPIO_SetBits(GPIOB, GPIO_Pin_4);	

}

}
}
void init_74hc273()
{

GPIO_InitTypeDef GPIO_InitStructure;

 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);


 	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5| GPIO_Pin_6;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 //推挽输出
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 		GPIO_Init(GPIOE, &GPIO_InitStructure);	   //本工程使用 


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 //推挽输出
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 		GPIO_Init(GPIOB, &GPIO_InitStructure);	   //本工程使用 



 GPIO_ResetBits(GPIOE, GPIO_Pin_0);	

GPIO_ResetBits(GPIOC, GPIO_Pin_9);	


GPIO_ResetBits(GPIOE, GPIO_Pin_1);	

GPIO_ResetBits(GPIOE, GPIO_Pin_2);	

GPIO_ResetBits(GPIOE, GPIO_Pin_3);	

GPIO_ResetBits(GPIOE, GPIO_Pin_4);	

GPIO_ResetBits(GPIOE, GPIO_Pin_5);	

GPIO_ResetBits(GPIOE, GPIO_Pin_6);	

GPIO_ResetBits(GPIOB, GPIO_Pin_1);
GPIO_ResetBits(GPIOB, GPIO_Pin_2);
GPIO_ResetBits(GPIOB, GPIO_Pin_3);
GPIO_ResetBits(GPIOB, GPIO_Pin_4);

}


