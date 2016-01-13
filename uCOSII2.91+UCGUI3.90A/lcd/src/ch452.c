#include "ch452.h"
#include "delay.h"

void CH452_I2c_WrByte(u16 dat,u8 load_num)	//写一个字节数据
{
	u8 i;
	if(load_num>=1&&load_num<=6)
		{
	for(i=0;i<12;i++)  // 输出12位数据
	{
		if(dat&0x01) {CH452_SDA_SET;}
		else {CH452_SDA_CLR;}
		CH452_SCL_CLR;
		dat>>=1;
		delay_us(4);
		CH452_SCL_SET;
				delay_us(4);

	}
		}
	if(load_num==1)
		{
	LOAD1_SET;
	delay_us(4);
	LOAD1_CLR;
	delay_us(4);
		}
		if(load_num==2)
		{
	LOAD2_SET;
	delay_us(4);
	LOAD2_CLR;
	delay_us(4);
		}
		if(load_num==3)
		{
	LOAD3_SET;
	delay_us(4);
	LOAD3_CLR;
	delay_us(4);
		}
		if(load_num==4)
		{
	LOAD4_SET;
	delay_us(4);
	LOAD4_CLR;
	delay_us(4);
		}
		if(load_num==5)
		{
	LOAD5_SET;
	delay_us(4);
	LOAD5_CLR;
	delay_us(4);
		}

	if(load_num==6)
		{
	LOAD6_SET;
	delay_us(4);
	LOAD6_CLR;
	delay_us(4);
		}
			
}

void CH452_Init(void)
{
	 u8 i;  	 
  GPIO_InitTypeDef      GPIO_InitStructure;
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);

RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOD, &GPIO_InitStructure);

for(i=1;i<=5;i++)
{
//	CH452_I2c_WrByte(CH452_RESET,i);
//	CH452_I2c_WrByte(CH452_NOP,i);
//	CH452_I2c_WrByte(CH452_CLR_BIT,i);
	CH452_I2c_WrByte(CH452_SYSON1,i);
	//CH452_I2c_WrByte(CH452_TWINKLE,i); 
//	CH452_I2c_WrByte(CH452_CLR_BIT,i);
	//CH452_I2c_WrByte(CH452_RESET,i);
}


}
void init_light_off(void)
{
u8 i;
for(i=1;i<=6;i++)
{

 	 CH452_I2c_WrByte(CH452_DIG0|OFF_OFF,i);
	 delay_ms(1);
	 CH452_I2c_WrByte(CH452_DIG1|OFF_OFF,i);
	 delay_ms(1);
	 CH452_I2c_WrByte(CH452_DIG2|OFF_OFF,i);
	 delay_ms(1);
	 CH452_I2c_WrByte(CH452_DIG3|OFF_OFF,i);
	 delay_ms(1);
	 CH452_I2c_WrByte(CH452_DIG4|OFF_OFF,i);
	 delay_ms(1);
	 CH452_I2c_WrByte(CH452_DIG5|OFF_OFF,i);
	 delay_ms(1);
	 CH452_I2c_WrByte(CH452_DIG6|OFF_OFF,i);
	 delay_ms(1);
	 CH452_I2c_WrByte(CH452_DIG7|OFF_OFF,i);
	 delay_ms(1);
}
}
/************************************************************************
开led灯功能函数void Light_pad_on(u8 comm_dis ,u8 num,u8 status_1,u8 status_2,u8 status_3)
参数说明:
comm_dis =0 为分补 comm_dis=1为共补
num表示第几行亮灯

u8 RED[]={0X38,0X3B,0X30,0X33,0X28,0X2B,0X20,0X23,0X18,0X1B,0X10,0X13,0X08,0X0B,0X00,0X03};

u8 GREEN[]={0X39,0X3C,0X31,0X34,0X29,0X2C,0X21,0X24,0X19,0X1C,0X11,0X14,0X09,0X0C,0X01H,0X04};

u8 YELLOW[]={0X3A,0X3D,0X32,0X35,0X2A,0X2D,0X22,0X25,0X1A,0X1D,0X12,0X15,0X0A,0X0D,0X02,0X05};
以上数组表示红绿 黄灯的地址

CH452_SET_BIT表示灭灯 CH452_CLR_BIT表示开灯

先关其他两种灯，再开要开的灯
{CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],1);}
***********************************************************************/
void Light_pad_on(u8 comm_dis ,u8 num,u8 status_1,u8 status_2,u8 status_3)
{
/*****************旧屏指示灯******************************************************

u8 RED[]={0X38,0X3B,0X30,0X33,0X28,0X2B,0X20,0X23,0X18,0X1B,0X10,0X13,0X08,0X0B,0X00,0X03};

u8 GREEN[]={0X39,0X3C,0X31,0X34,0X29,0X2C,0X21,0X24,0X19,0X1C,0X11,0X14,0X09,0X0C,0X01H,0X04};

u8 YELLOW[]={0X3A,0X3D,0X32,0X35,0X2A,0X2D,0X22,0X25,0X1A,0X1D,0X12,0X15,0X0A,0X0D,0X02,0X05};
*****************旧屏指示灯******************************************************/


/*****************新屏指示灯******************************************************/
u8 YELLOW[]={0X38,0X3B,0X30,0X33,0X28,0X2B,0X20,0X23,0X18,0X1B,0X10,0X13,0X08,0X0B,0X00,0X03};

u8 GREEN[]={0X39,0X3C,0X31,0X34,0X29,0X2C,0X21,0X24,0X19,0X1C,0X11,0X14,0X09,0X0C,0X01,0X04};

u8 RED[]={0X3A,0X3D,0X32,0X35,0X2A,0X2D,0X22,0X25,0X1A,0X1D,0X12,0X15,0X0A,0X0D,0X02,0X05};
/*****************新屏指示灯******************************************************/
if(comm_dis==0)//分补
{
{
if(num>=1&&num<=16)
{
num=num-1;
if(status_1==0||status_1==3){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],1);}
if(status_2==0||status_2==3){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],2);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],2);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],2);}
if(status_3==0||status_3==3){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],3);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],3);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],3);}

if(status_1==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],1);}
if(status_2==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],2);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],2);CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],2);}
if(status_3==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],3);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],3);CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],3);}

//if(status_1==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],1);CH452_I2c_WrByte(CH452_CLR_BIT|YELLOW[num],1);}
//if(status_2==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],2);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],2);CH452_I2c_WrByte(CH452_CLR_BIT|YELLOW[num],2);}
//if(status_3==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],3);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],3);CH452_I2c_WrByte(CH452_CLR_BIT|YELLOW[num],3);}


if(status_1==2){CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],1);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],1);}
if(status_2==2){CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],2);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],2);}
if(status_3==2){CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],3);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],3);}

}
}	      



}
if(comm_dis==1)//共补
{
{
if(num>=1&&num<=16)
{
num=num-1;
if(status_1==0||status_1==3){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],1);}
if(status_2==0||status_2==3){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],2);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],2);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],2);}

if(status_1==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],1);}
if(status_2==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],2);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],2);CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],2);}

//if(status_1==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],1);CH452_I2c_WrByte(CH452_CLR_BIT|YELLOW[num],1);}
//if(status_2==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],2);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],2);CH452_I2c_WrByte(CH452_CLR_BIT|YELLOW[num],2);}


if(status_1==2){CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],1);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],1);}
if(status_2==2){CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],2);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],2);}

{CH452_I2c_WrByte(CH452_SET_BIT|RED[num],3);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],3);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],3);}

}

if(num>16&&num<=32)
	{
	num=num-16-1;
if(status_1==0||status_1==3){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],4);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],4);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],4);}
if(status_2==0||status_1==3){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],5);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],5);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],5);}

if(status_1==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],4);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],4);CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],4);}
if(status_2==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],5);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],5);CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],5);}

//if(status_1==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],4);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],4);CH452_I2c_WrByte(CH452_CLR_BIT|YELLOW[num],4);}
//if(status_2==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],5);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],5);CH452_I2c_WrByte(CH452_CLR_BIT|YELLOW[num],5);}

if(status_1==2){CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],4);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],4);}
if(status_2==2){CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],5);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],5);}

{CH452_I2c_WrByte(CH452_SET_BIT|RED[num],6);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],6);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],6);}

    }

}

}
}

void Light_pad_twinkle(u8 comm_dis ,u8 num,u8 status_1,u8 status_2,u8 status_3,u16 time)
{

/*****************旧屏指示灯******************************************************

u8 RED[]={0X38,0X3B,0X30,0X33,0X28,0X2B,0X20,0X23,0X18,0X1B,0X10,0X13,0X08,0X0B,0X00,0X03};

u8 GREEN[]={0X39,0X3C,0X31,0X34,0X29,0X2C,0X21,0X24,0X19,0X1C,0X11,0X14,0X09,0X0C,0X01H,0X04};

u8 YELLOW[]={0X3A,0X3D,0X32,0X35,0X2A,0X2D,0X22,0X25,0X1A,0X1D,0X12,0X15,0X0A,0X0D,0X02,0X05};
*****************旧屏指示灯******************************************************/


/*****************新屏指示灯******************************************************/
u8 YELLOW[]={0X38,0X3B,0X30,0X33,0X28,0X2B,0X20,0X23,0X18,0X1B,0X10,0X13,0X08,0X0B,0X00,0X03};

u8 GREEN[]={0X39,0X3C,0X31,0X34,0X29,0X2C,0X21,0X24,0X19,0X1C,0X11,0X14,0X09,0X0C,0X01,0X04};

u8 RED[]={0X3A,0X3D,0X32,0X35,0X2A,0X2D,0X22,0X25,0X1A,0X1D,0X12,0X15,0X0A,0X0D,0X02,0X05};
/*****************新屏指示灯******************************************************/
if(comm_dis==0)//分补
{
if(num>=1&&num<=16)
{
num=num-1;
{
if(status_1==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],1);}
if(status_2==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],2);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],2);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],2);}
if(status_3==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],3);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],3);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],3);}

if(status_1==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],1);}
if(status_2==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],2);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],2);CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],2);}
if(status_3==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],3);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],3);CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],3);}

if(status_1==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],1);CH452_I2c_WrByte(CH452_CLR_BIT|YELLOW[num],1);}
if(status_2==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],2);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],2);CH452_I2c_WrByte(CH452_CLR_BIT|YELLOW[num],2);}
if(status_3==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],3);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],3);CH452_I2c_WrByte(CH452_CLR_BIT|YELLOW[num],3);}
}

delay_ms(time);
{
if(status_1==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],1);}
if(status_2==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],2);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],2);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],2);}
if(status_3==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],3);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],3);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],3);}

if(status_1==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);}
if(status_2==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],2);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],2);CH452_I2c_WrByte(CH452_SET_BIT|RED[num],2);}
if(status_3==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],3);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],3);CH452_I2c_WrByte(CH452_SET_BIT|RED[num],3);}

if(status_1==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);}
if(status_2==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],2);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],2);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],2);}
if(status_3==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],3);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],3);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],3);}
}
}
}

if(comm_dis==1)//分补
{
if(num>=1&&num<=16)
{
num=num-1;
{
if(status_1==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],1);}
if(status_2==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],2);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],2);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],2);}

if(status_1==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],1);}
if(status_2==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],2);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],2);CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],2);}

if(status_1==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],1);CH452_I2c_WrByte(CH452_CLR_BIT|YELLOW[num],1);}
if(status_2==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],2);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],2);CH452_I2c_WrByte(CH452_CLR_BIT|YELLOW[num],2);}
}
delay_ms(time);

{
if(status_1==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],1);}
if(status_2==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],2);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],2);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],2);}


if(status_1==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);}
if(status_2==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],2);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],2);CH452_I2c_WrByte(CH452_SET_BIT|RED[num],2);}

if(status_1==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);}
if(status_2==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],2);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],2);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],2);}
}
}

if(num>16&&num<=32)
	{
	num=num-16-1;
		{
if(status_1==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],4);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],4);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],4);}
if(status_2==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],5);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],5);CH452_I2c_WrByte(CH452_CLR_BIT|GREEN[num],5);}

if(status_1==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],4);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],4);CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],4);}
if(status_2==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],5);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],5);CH452_I2c_WrByte(CH452_CLR_BIT|RED[num],5);}

if(status_1==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],4);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],4);CH452_I2c_WrByte(CH452_CLR_BIT|YELLOW[num],4);}
if(status_2==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],5);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],5);CH452_I2c_WrByte(CH452_CLR_BIT|YELLOW[num],5);}
		}
delay_ms(time);
	
		{
if(status_1==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],4);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],4);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],4);}
if(status_2==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],5);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],5);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],5);}

if(status_1==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],4);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],4);CH452_I2c_WrByte(CH452_SET_BIT|RED[num],4);}
if(status_2==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],5);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],5);CH452_I2c_WrByte(CH452_SET_BIT|RED[num],5);}

if(status_1==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],4);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],4);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],4);}
if(status_2==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],5);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],5);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],5);}
			}

    }

}
delay_ms(time);

}


void Light_pad_off(u8 comm_dis ,u8 num,u8 status_1,u8 status_2,u8 status_3)
{

/*****************旧屏指示灯******************************************************

u8 RED[]={0X38,0X3B,0X30,0X33,0X28,0X2B,0X20,0X23,0X18,0X1B,0X10,0X13,0X08,0X0B,0X00,0X03};

u8 GREEN[]={0X39,0X3C,0X31,0X34,0X29,0X2C,0X21,0X24,0X19,0X1C,0X11,0X14,0X09,0X0C,0X01H,0X04};

u8 YELLOW[]={0X3A,0X3D,0X32,0X35,0X2A,0X2D,0X22,0X25,0X1A,0X1D,0X12,0X15,0X0A,0X0D,0X02,0X05};
*****************旧屏指示灯******************************************************/


/*****************新屏指示灯******************************************************/
u8 YELLOW[]={0X38,0X3B,0X30,0X33,0X28,0X2B,0X20,0X23,0X18,0X1B,0X10,0X13,0X08,0X0B,0X00,0X03};

u8 GREEN[]={0X39,0X3C,0X31,0X34,0X29,0X2C,0X21,0X24,0X19,0X1C,0X11,0X14,0X09,0X0C,0X01,0X04};

u8 RED[]={0X3A,0X3D,0X32,0X35,0X2A,0X2D,0X22,0X25,0X1A,0X1D,0X12,0X15,0X0A,0X0D,0X02,0X05};
/*****************新屏指示灯******************************************************/
if(comm_dis==0)//分补
{
if(num>=1&&num<=16)
{
num=num-1;

{
if(status_1==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],1);}
if(status_2==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],2);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],2);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],2);}
if(status_3==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],3);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],3);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],3);}

if(status_1==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);}
if(status_2==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],2);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],2);CH452_I2c_WrByte(CH452_SET_BIT|RED[num],2);}
if(status_3==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],3);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],3);CH452_I2c_WrByte(CH452_SET_BIT|RED[num],3);}

if(status_1==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);}
if(status_2==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],2);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],2);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],2);}
if(status_3==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],3);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],3);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],3);}
}
}
}

if(comm_dis==1)//分补
{
if(num>=1&&num<=16)
{
num=num-1;


{
if(status_1==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],1);}
if(status_2==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],2);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],2);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],2);}


if(status_1==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);}
if(status_2==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],2);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],2);CH452_I2c_WrByte(CH452_SET_BIT|RED[num],2);}

if(status_1==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],1);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],1);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],1);}
if(status_2==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],2);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],2);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],2);}
}
}

if(num>16&&num<=32)
	{
	num=num-16-1;
		
	
		{
if(status_1==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],4);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],4);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],4);}
if(status_2==0){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],5);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],5);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],5);}

if(status_1==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],4);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],4);CH452_I2c_WrByte(CH452_SET_BIT|RED[num],4);}
if(status_2==1){CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],5);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],5);CH452_I2c_WrByte(CH452_SET_BIT|RED[num],5);}

if(status_1==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],4);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],4);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],4);}
if(status_2==2){CH452_I2c_WrByte(CH452_SET_BIT|RED[num],5);CH452_I2c_WrByte(CH452_SET_BIT|GREEN[num],5);CH452_I2c_WrByte(CH452_SET_BIT|YELLOW[num],5);}
			}

    }

}

}
