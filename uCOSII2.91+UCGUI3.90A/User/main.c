/*********************************************************************************************************
*
* File                : main.c
* Hardware Environment: 
* Build Environment   : RealView MDK-ARM  Version: 4.20
* Version             : V1.0
* By                  : 
*
*                                  (c) Copyright 2005-2011, WaveShare
*                                       http://www.waveshare.net
*                                          All Rights Reserved
*
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <includes.h>

/* Private variables ---------------------------------------------------------*/
#define ON_time 13400
#define OFF_time 15000		   //18500
#define  k 0.8	//0.8
#define PI2  6.28318530717959
#define cruccent_ratio  1.07//电流校正系数
#define zero_limit 1900         //1000为电流门限0.1           1500为电流门限0.15






//任务函数



/***************************************************/
/*
typedef struct  
{ 
  u8 dis_comm;//dis=0 comm=1
  u8 myid;      //本电容箱ID号
  u8 size;      //容量单位千法
  u8 work_status;    //工作状态 1 为投入工作；0 为没有工作
  u8 work_time;     //工作时间
  u8 group;// 第几组表示
}status_comm_node;
*/
 typedef struct  
{ 
  u8 dis_comm;//dis=0 comm=1
  u8 myid[3];      //本电容箱ID号
  u8 size[3];      //容量单位千法
  u8 work_status[3];    //工作状态 1 为投入工作；0 为没有工作
}status_dis_node;

 typedef struct  
{   u32 dis_comm;//dis=0 comm=1
  u32 work_status[4];   //工作状态 1 为投入工作；0 为没有工作
}light_status_node;
 light_status_node light_status;

/***************************************************/
/* Private function prototypes -----------------------------------------------*/


static  void  App_TaskLCD		() ;
void sort_list();
void qeen(  u8 d,u8 n);


/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Argument(s) : none.
*
* Return(s)   : none.
*********************************************************************************************************
*/
#define ADC1_DR_Address    ((u32)0x4001204C)
#define ADC2_DR_Address    ((u32)0x4001214C)
#define ADC3_DR_ADDRESS    ((uint32_t)0x4001224C)

u16 ADC_Converted_VValue=0;
u16 ADC_Converted_CValue=0;
u16 ADC_Converted_base=0;

u8 vernum=101,gonglvshishu=0;
u16 dianya_zhi=0,wugongkvar=0,allkvar=0;
float32_t HV=0,HI=0,A_HV=0,A_HI=0,B_HV=0,B_HI=0,C_HV=0,C_HI=0;
u32	dianliuzhi=0;
//#if (FUNCTION_MODULE == DF_THREE)
u16 dianya_zhi_A=0,dianya_zhi_B=0,dianya_zhi_C=0,wugongkvar_A=0,wugongkvar_B=0,wugongkvar_C=0;
u16 allkvar_A=0,allkvar_B=0,allkvar_C=0;
u32	dianliuzhi_A=0,dianliuzhi_B=0	,dianliuzhi_C=0;
u8 gonglvshishu_A=0,gonglvshishu_B=0,gonglvshishu_C=0;
u8 display_nothing_close_open_warn=0;

void ADC3_CH10_DMA_Config_VA(void);
void ADC2_CH8_DMA_Config_VEE(void);
void ADC1_CH1_DMA_Config_CA(void);
void ADC3_CH11_DMA_Config_VB(void);
void ADC1_CH4_DMA_Config_CB(void);
void ADC3_CH12_DMA_Config_VC(void);
void ADC1_CH7_DMA_Config_CC(void);
void ADC2_CH13_DMA_Config_A1(void);
void ADC2_CH14_DMA_Config_B1(void);
void ADC2_CH15_DMA_Config_C1(void);

void ADC1_CH1_DMA_Config_VC_phase(void);

//void init_para(void);
void Init_ADC(void);

static  void  GPIO_Configuration    (void);
void allphase(float32_t *V,float32_t *I);
 computer_gonglu(float32_t testInput_V[],float32_t testInput_C[],float32_t testOutput[],float32_t reslut[]);


/*****************************485_start*********************************************************/

#define delay_time_base 1000                   //延时数量级
#define time_out 3000
#define LEN_control 3
#define EN_USART2_RX 	1			//0,不接收;1,接收.
#define RS485_TX_EN_1		GPIO_SetBits(GPIOC, GPIO_Pin_12)	// 485模式控制.0,接收;1,发送.本工程用PB15
#define RS485_TX_EN_0		GPIO_ResetBits(GPIOC, GPIO_Pin_12)	// 485模式控制.0,接收;1,发送.本工程用PB15

//发送控制信息


u32 hand_light_existence;


//接收到的数据长度
u8 RS485_RX_CNT=0;  



u8 auto_on=1;

void EXTI9_5_IRQHandler(void);
u16 comp_16(u16 a,u16 b);
void NVIC_Configuration(void);
void EXTI_Configuration(void);//初始化函数

/***********************************485_end****************************************************/
 u8 Work_Flag=0;
 void TIM2_Int_Init(u16 arr,u16 psc);
 void TIM2_IRQHandler(void);   //TIM2

 void TIM4_Int_Init(u16 arr,u16 psc);

/********************************switch_A_B_C**************************************************/
//#define ON_time 13400//60
//#define OFF_time 15600//60

#define ON_time 13400                 //100
#define OFF_time 15000		   //1//100

u16 var=0;

void LIGHT_backligt_on(void);
void LIGHT_backligt_off(void);

/***********************************end*******************************************************/


/************************************TIME******************************************************/
void delay_time(u32 time);
 void heartbeat(u8 t);



/************************************TIME_end******************************************************/

/************************************MAster data structure*******************/




void set_bit(u8 b, u8 dis_com,light_status_node *light_status,u8 status_1,u8 status_2,u8 status_3,u8 status_4);
u8 clear_bit(u8 b,u32 light_pad);
void set_clear_existence(u8 true_false,u8 b,u32 *exist);
void init_cap();

/*************************************MAster data structure_end***************/





u8 L_C_flag_A=1;//感性容性标准变量
u8 L_C_flag_B=1;//感性容性标准变量
u8 L_C_flag_C=1;//感性容性标准变量

#define TEST_LENGTH_SAMPLES 512*2 
 
u8 phase_flag=0;
u8 light_time=100;
u8 light_time_led=50;
u8 delay_on=0,delay_off=0;
u8 delay_on_cont=1,delay_off_cont=1;
/********************控制器设置参数*************************/



extern u8 COMMCAT_para;
//extern u8 CT_para;
   status_comm_node comm_list[34];

u8 hand_id=1;
u8 dis_com=1;

 float32_t testInput_V[TEST_LENGTH_SAMPLES]; 
 float32_t testInput_C[TEST_LENGTH_SAMPLES]; 

float32_t testOutput[TEST_LENGTH_SAMPLES*2/2]; 
float32_t reslut[TEST_LENGTH_SAMPLES/2]; 
//u8 free_timeout_20=100;//轮休时间控制变量
//u8 free_timeout_10=100;//轮休时间控制变量
//u8 free_timeout_5=100;//轮休时间控制变量

/**********************************************/
u8 first=1;
u8 end=32;
INT32S main (void)
{
CPU_INT08U  os_err;
u8 i;
u32 j;
//CPU_IntDis();                   
/***************  Init hardware ***************/
//u8 i;

   // OS_CPU_SysTickInit();/* Initialize the SysTick.                              */
for(j=0;j<=100000000;j++);
delay_init();
//delay_ms(10000);

//	delay_us(500000);
NVIC_Configuration();
GPIO_Configuration();
IWDG_Init(6,625); 
 //EXTI_Configuration();//初始化函数

 init_light_off();
 LIGHT_backligt_on();
init_cap();
//sort_list();
while(1)
{
 App_TaskLCD();
  {

  if(KEY_3==0) 

{
for(i=1;i<=32;i++)
{

/*更新指示灯*/
if(comm_list[i].size==0)
{
Light_pad_off(1,i,0,0,0);//指示灯使用
	  	 set_bit(i, 1, &light_status, 0,0, 0,2);//手动投切使用
		set_clear_existence(0,i,&hand_light_existence);


}
else
{
	 Light_pad_on(1,i,comm_list[i].work_status,comm_list[i].work_status,0);
	 set_bit(i, 1, &light_status, comm_list[i].work_status,comm_list[i].work_status,0,0);//手动投切使用
	 set_clear_existence(1,i,&hand_light_existence);

}

}

 	}

  if(KEY_3==1) 
  	{

for(i=1;i<=26;i++)
{

/*更新指示灯*/
if(comm_list[i].size==0)
{
Light_pad_off(1,i,0,0,0);//指示灯使用
	  	 set_bit(i, 1, &light_status, 0,0, 0,2);//手动投切使用
		set_clear_existence(0,i,&hand_light_existence);


}
else
{
	 Light_pad_on(1,i,comm_list[i].work_status,comm_list[i].work_status,0);
	 set_bit(i, 1, &light_status, comm_list[i].work_status,comm_list[i].work_status,0,0);//手动投切使用
	 set_clear_existence(1,i,&hand_light_existence);

}

}

for(i=27;i<=32;i++)
{

/*更新指示灯*/
if(comm_list[i].size==0)
{
Light_pad_off(1,i,0,0,0);//指示灯使用
	  	 set_bit(i, 1, &light_status, 0,0, 0,2);//手动投切使用
		set_clear_existence(0,i,&hand_light_existence);


}
else
{
	 Light_pad_on(1,i,comm_list[i].work_status,comm_list[i].work_status,0);
	 set_bit(i, 1, &light_status, comm_list[i].work_status,comm_list[i].work_status,0,0);//手动投切使用
	 set_clear_existence(1,i,&hand_light_existence);

}

}


  }



if(COMMCAT_para==0) //自动模式
{
//sort_list();

 computer_gonglu(testInput_V,testInput_C,testOutput,reslut);
//delay_ms(1000);

}

    }

//	delay_ms(100);

}

   
    
/************************************************/



}


/*
*********************************************************************************************************
*                                          App_TaskStart()
*
* Description : The startup task.  The uC/OS-II ticker should only be initialize once multitasking starts.
*
* Argument(s) : p_arg       Argument passed to 'App_TaskStart()' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Note(s)     : none.
*********************************************************************************************************
*/	  


/*
*********************************************************************************************************
*                                            App_TaskCreate()
*
* Description : Create the application tasks.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : App_TaskStart().
*
* Note(s)     : none.
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                          App_TaskMaster	 (void		*p_arg )
*
* Description : The startup task.  The uC/OS-II ticker should only be initialize once multitasking starts.
*
* Argument(s) : p_arg       Argument passed to 'App_TaskStart()' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Note(s)     : none.
*********************************************************************************************************
*/	  



/**********************************************************************************/



/*
*********************************************************************************************************
*                                          App_TaskLCD	 (void		*p_arg )
*
* Description : The startup task.  The uC/OS-II ticker should only be initialize once multitasking starts.
*
* Argument(s) : p_arg       Argument passed to 'App_TaskStart()' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Note(s)     : none.
*********************************************************************************************************
*/	  
static  void  App_TaskLCD	 ()
{  

u8 status_1,status_2,status_3,status_4;
u8 exist;
u8 i;
	//for(;;)
		{  ParaSet();
	if(COMMCAT_para==1)
{


	if(KEY_right==0)
 		{
        exist=clear_bit(hand_id, hand_light_existence);
		if(exist==1){
           status_4=clear_bit(hand_id,light_status.work_status[3]);
 //	dis_com= clear_bit(hand_id,light_status.dis_comm);

	if(status_4==0)   
	{
	status_1= clear_bit(hand_id,light_status.work_status[0]);
	status_2= clear_bit(hand_id,light_status.work_status[1]);
	status_3= clear_bit(hand_id,light_status.work_status[2]);
 Light_pad_on(dis_com,hand_id,status_1,status_2,status_3);
			}
	if(status_4==1)Light_pad_on(dis_com,hand_id,2,2,2);

			

		}
		 hand_id++;
		 while(KEY_right==0);
		 if(hand_id>32)hand_id=1;
		 	for(i=hand_id;i<=32;i++)
		{exist=clear_bit(i, hand_light_existence);
	         if(exist==1)
			 	{
			 	hand_id=i;
				break;
				}
	 
	  }	
				//		if(i==33)hand_id=1;

	    }
	if(KEY_left==0)
	  {
	          exist=clear_bit(hand_id, hand_light_existence);
		if(exist==1){
           status_4=clear_bit(hand_id,light_status.work_status[3]);
 //	dis_com= clear_bit(hand_id,light_status.dis_comm);

	if(status_4==0)   
	{
	status_1= clear_bit(hand_id,light_status.work_status[0]);
	status_2= clear_bit(hand_id,light_status.work_status[1]);
	status_3= clear_bit(hand_id,light_status.work_status[2]);
 Light_pad_on(dis_com,hand_id,status_1,status_2,status_3);
			}
	if(status_4==1)Light_pad_on(dis_com,hand_id,2,2,2);

			

		}
		 hand_id--;
		 while(KEY_left==0);
		 if(hand_id<1)hand_id=32;
		 		 	for(i=hand_id;i>=1;i--)
		{exist=clear_bit(i, hand_light_existence);
	         if(exist==1)
			 	{
			 	hand_id=i;
				break;
				}
	 
	  }
		//	if(i==0)hand_id=32;

	  }

	   //if(exist==1)
	   	{
           status_4=clear_bit(hand_id,light_status.work_status[3]);
	//	   dis_com= clear_bit(hand_id,light_status.dis_comm);
	//	if(status_4==0)   
			{
	status_1= clear_bit(hand_id,light_status.work_status[0]);
	status_2= clear_bit(hand_id,light_status.work_status[1]);
	status_3= clear_bit(hand_id,light_status.work_status[2]);
//if(Work_Flag==1)
	Light_pad_on(dis_com,hand_id,status_1,status_2,status_3);
delay_ms(50);
//if(Work_Flag==0)
	Light_pad_off(dis_com,hand_id,status_1,status_2,status_3);
delay_ms(50);

				}
				
	   	}
		  
		  

}


	        }
   	
}

/*
*********************************************************************************************************
*                                          App_Taskcomputer	 (void		*p_arg )
*
* Description : The startup task.  The uC/OS-II ticker should only be initialize once multitasking starts.
*
* Argument(s) : p_arg       Argument passed to 'App_TaskStart()' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Note(s)     : none.
*********************************************************************************************************
*/	  




/*
*********************************************************************************************************
*                                          App_Taskcomputer	 (void		*p_arg )
*
* Description : The startup task.  The uC/OS-II ticker should only be initialize once multitasking starts.
*
* Argument(s) : p_arg       Argument passed to 'App_TaskStart()' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Note(s)     : none.
*********************************************************************************************************
*/	  








/*
*********************************************************************************************************
*                                          App_Taskcomputer	 (void		*p_arg )
*
* Description : The startup task.  The uC/OS-II ticker should only be initialize once multitasking starts.
*
* Argument(s) : p_arg       Argument passed to 'App_TaskStart()' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Note(s)     : none.
*********************************************************************************************************
*/








/*
*********************************************************************************************************
*                                          App_Taskcomputer	 (void		*p_arg )
*
* Description : The startup task.  The uC/OS-II ticker should only be initialize once multitasking starts.
*
* Argument(s) : p_arg       Argument passed to 'App_TaskStart()' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Note(s)     : none.
*********************************************************************************************************
*/
void init_cap()
{
int i;
for(i=1;i<=32;i++)
{
comm_list[i].size= AT24CXX_ReadOneByte(0x0010+(i-1)*2);
comm_list[i].work_status=0;
comm_list[i].id=i;
 Light_pad_on(dis_com,i,0,0,0);
}
}



/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configure GPIO Pin
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void GPIO_Configuration(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	/* Configure PF6 PF7 PF8 PF9 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);


{  
 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);


 	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 //推挽输出
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 		GPIO_Init(GPIOC, &GPIO_InitStructure);	   //本工程使用 

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);


 	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 //推挽输出
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 		GPIO_Init(GPIOA, &GPIO_InitStructure);	   //本工程使用 

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 //推挽输出
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 		GPIO_Init(GPIOB, &GPIO_InitStructure);	   //本工程使用 
 		
 	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 //推挽输出
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 		GPIO_Init(GPIOB, &GPIO_InitStructure);	   //本工程使用 

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 //推挽输出
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 		GPIO_Init(GPIOB, &GPIO_InitStructure);	   //本工程使用 

}
	

/*********************屏幕和按键*****************************************/

HT1621_Init();
AT24CXX_Init();
	KEY_Init();          //初始化与按键连接的硬件接口  
	CH452_Init();

/***********************采样和DMA**************************************/	
/********************485****************************************/	
/************************************************************/

/*************************TIME*******************************/
TIM4_Int_Init(4999,7199);//10Khz的计数频率，计数10K次为1000ms 
	TIM2_Int_Init(4999+500,7199);//10Khz的计数频率，计数10K次为1000ms 

EXTI_Configuration();
Init_ADC();
ADC2_CH8_DMA_Config_VEE();


}


void allphase(float32_t *V,float32_t *I)
{
int i=0;
int NPT=TEST_LENGTH_SAMPLES;
for(i=0;i<=NPT/2-1;i++)
{
V[i]=(i+1)*V[i];
I[i]=(i+1)*I[i];
}
for(i=NPT/2;i<NPT-1;i++)
{
V[i]=(NPT-(i+1))*V[i];
I[i]=(NPT-(i+1))*I[i];

}

for(i=0;i<NPT/2-1;i++)
{
V[i+NPT/2]=V[i]+V[i+NPT/2];
I[i+NPT/2]=I[i]+I[i+NPT/2];

}

for(i=0;i<=NPT/2-1;i++)
{
V[i]=V[NPT/2-1+i];
I[i]=I[NPT/2-1+i];

}
}
void ADC1_CH1_DMA_Config_VC_phase(void)
{
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC1);

}

void ADC3_CH10_DMA_Config_VA(void)
{
  ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC3);

}



void ADC1_CH1_DMA_Config_CA(void)
{
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC1);

}

void ADC2_CH8_DMA_Config_VEE(void)

{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOB, ENABLE);

  /* Configure ADC1 Channel10 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC1 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC2, &ADC_InitStructure);

  /* ADC1 regular channe6 configuration *************************************/
  ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 1, ADC_SampleTime_3Cycles);

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC2, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC2, ENABLE);
ADC_SoftwareStartConv(ADC2);

}

/*******************************B_phase***************************************/

void ADC3_CH11_DMA_Config_VB(void)
{
  ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC3);

}

void ADC1_CH4_DMA_Config_CB(void)
{
 ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC1);

}
/*******************************B_phase_end***********************************/




/********************************C_phase**************************************/
void ADC3_CH12_DMA_Config_VC(void)
{
  ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC3);

}

void ADC1_CH7_DMA_Config_CC(void)
{
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC1);

}
/******************************C_phase_end*********************************/



/********************************A1***************************************/
void ADC2_CH13_DMA_Config_A1(void)
{
  ADC_RegularChannelConfig(ADC2, ADC_Channel_13, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC2);

}

/********************************A1_end***********************************/

/********************************B1***************************************/
void ADC2_CH14_DMA_Config_B1(void)
{
  ADC_RegularChannelConfig(ADC2, ADC_Channel_14, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC2);

}

/********************************B1_end***********************************/

/********************************C1***************************************/
void ADC2_CH15_DMA_Config_C1(void)
{
  ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC2);

}

/********************************C1_end***********************************/





void Init_ADC(void)
{
{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_ADC2|RCC_APB2Periph_ADC3, ENABLE);

//  DMA_DeInit(DMA2_Stream0);
  /* DMA2 Stream0 channe0 configuration *************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Converted_VValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream1, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream1, ENABLE);

DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Converted_CValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);

  DMA_InitStructure.DMA_Channel = DMA_Channel_1;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC2_DR_Address;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Converted_base;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream2, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream2, ENABLE);

  /* Configure ADC1 Channel10 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC1 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC3, &ADC_InitStructure);
  ADC_Init(ADC2, &ADC_InitStructure);
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channe6 configuration *************************************/

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
    ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC3, ENABLE);
  ADC_DMACmd(ADC2, ENABLE);
  ADC_DMACmd(ADC1, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
    ADC_Cmd(ADC2, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

}

}



/********************************C_phase_end*********************************/


u16 comp_16(u16 a,u16 b)
{
u16 value=0;
value=((a&0x00FF)+((b<<8)&0xFF00));
return value;
}






/**************/
void sort_list()
{
u8 i,j;
u8 g,s,q;
for(j=2;j<=32;j++)
{
g=comm_list[j].size;
s=comm_list[j].work_status;
q=comm_list[j].id;
for(i=j-1;i>0;i--)
{
if(comm_list[i].size<g)
{
comm_list[i+1].size=comm_list[i].size;
comm_list[i+1].work_status=comm_list[i].work_status;
comm_list[i+1].id=comm_list[i].id;
}
}
comm_list[i+1].size=g;
comm_list[i+1].work_status=s;
comm_list[i+1].id=q;



}

}
 	
void qeen(u8 d,u8 n)
{
u8 j;
u8 g,s,q;
g=comm_list[d].size;
s=comm_list[d].work_status;
q=comm_list[d].id;

for(j=d;j<n;j++)
{
comm_list[j].size=comm_list[j+1].size;
comm_list[j].work_status=comm_list[j+1].work_status;
comm_list[j].id=comm_list[j+1].id;
}
comm_list[n].size=g;
comm_list[n].work_status=s;
comm_list[n].id=q;
}

/*********************************/
 computer_gonglu(float32_t testInput_V[],float32_t testInput_C[],float32_t testOutput[],float32_t reslut[])
{
int i=0;
arm_status status; 
arm_rfft_instance_f32 S;
arm_cfft_radix4_instance_f32  S_CFFT;
float32_t maxValue=0.0,maxValue_C=0.0; 

u16 TR[]={1,2,3,4,5,6,8,10,12,16,20,24,30,40,50,60,80,100,120};
/* ------------------------------------------------------------------ 
* Global variables for FFT Bin Example 
* ------------------------------------------------------------------- */ 
uint32_t fftSize = 512; 
uint32_t ifftFlag = 0; 
uint32_t doBitReverse = 1; 
 
/* Reference index at which max energy of bin ocuurs */ 
uint32_t  testIndex = 0,a,b,c; 
 double angle[4]; 
float32_t sine=0,cose=0;
u16 phase;
s32 gl[2];
u16 wugongkvar_95,wugongkvar_95A,wugongkvar_95B,wugongkvar_95C;

float32_t HU_SUM_A=0,HI_SUM_A=0,HU_A=0,HI_A=0;
float32_t HU_SUM_B=0,HI_SUM_B=0,HU_B=0,HI_B=0;
float32_t HU_SUM_C=0,HI_SUM_C=0,HU_C=0,HI_C=0;
u8 flag_phase=1;
//u8 DELAY_ON_para=10;
 //u8 DELAY_OFF_para=10;
 u8 COS_ON_para=90;
 u8 COS_OFF_para=95;
 u8 V_PROT_para_L=40;
 u8 V_PROT_para_tri=40;
 u8 HU_PROT_para=100;
 u8 HI_PROT_para=100;
 u8 ON_HOLD_para;
 u8 dis_num=0;
u8 T;
static u8 warning_flag=0;


{
{
		// DELAY_ON_para=AT24CXX_ReadOneByte(0x1000);  //存储DELAY_ON_para到eeprom
		// DELAY_OFF_para=AT24CXX_ReadOneByte(0x2000);  //存储DELAY_OFF_para到eeprom
		 COS_ON_para=AT24CXX_ReadOneByte(0x0003);  //存储DELAY_OFF_para到eeprom
				 COS_OFF_para=AT24CXX_ReadOneByte(0x0004);  //存储DELAY_OFF_para到eeprom

		V_PROT_para_L=AT24CXX_ReadOneByte(0x0009);
	   			HI_PROT_para=AT24CXX_ReadOneByte(0x0008); 

		V_PROT_para_tri=AT24CXX_ReadOneByte(0x000a);

		ON_HOLD_para=AT24CXX_ReadOneByte(0x0005); 
			HU_PROT_para=AT24CXX_ReadOneByte(0x0007); 

}	  
a=AT24CXX_ReadOneByte(0x0000);
T=TR[a];
/*
ADC_Converted_CValue=0;
ADC_Converted_VValue=0;
ADC_Converted_base=0;

 for(i=0;i<TEST_LENGTH_SAMPLES;i++)
	 	{	 	
testInput_C[i]=0;
testInput_V[i]=0;

        }
 */
 }


   if(KEY_3==0) 
   	{

{
ADC3_CH10_DMA_Config_VA();
ADC1_CH4_DMA_Config_CB();

 maxValue=0.0;
 maxValue_C=0.0; 

 for(i=0;i<TEST_LENGTH_SAMPLES;i++)
	 	{
	 	
	 	
testInput_C[i]=(float32_t)((ADC_Converted_CValue-ADC_Converted_base)*3.3/4096);///  1550

testInput_V[i]=(float32_t)((ADC_Converted_VValue-ADC_Converted_base)*3.3/4096);///  1550

delay_us(36);//36->512

        }

 
allphase(testInput_V,testInput_C);

 
	status = arm_rfft_init_f32(&S,&S_CFFT, fftSize,  
	  								ifftFlag, doBitReverse); 
	 
	/* Process the data through the CFFT/CIFFT module */ 
	arm_rfft_f32(&S, testInput_V,testOutput); 

             testIndex=1;
	 angle[0]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//电压初始相位

	/* Process the data through the Complex Magnitude Module for  
	calculating the magnitude at each bin */ 

	/*******通过原始数据计算电压值***********/
		//arm_rfft_f32(&S, testInput_V_source,testOutput); 

	arm_cmplx_mag_f32(testOutput, reslut,  
	  				fftSize);  
	/* Calculates maxValue and returns corresponding BIN value */ 

	arm_max_f32(reslut, fftSize/2, &maxValue, &testIndex);
dianya_zhi=maxValue/1000;
dianya_zhi=dianya_zhi/4.35;
if(dianya_zhi<=100)dianya_zhi=0;
/*************************电压谐波率****************************************/

{
for(i=3;i<=21;i=i+2){HU_SUM_B=(reslut[i]*reslut[i])+HU_SUM_B;}
arm_sqrt_f32(HU_SUM_B,&HU_B);
HV=(HU_B/maxValue)*100;
}
/******************************************************************/

/******************************************************************/
	arm_rfft_f32(&S, testInput_C,testOutput); 
         
	angle[1]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//电流初始相位

	/*******通过原始数据计算电压值***********/

	//	arm_rfft_f32(&S, testInput_C_source,testOutput); 

	arm_cmplx_mag_f32(testOutput, reslut,  
	  				fftSize);  
	 
	/* Calculates maxValue and returns corresponding BIN value */ 
	arm_max_f32(reslut, fftSize/2, &maxValue_C, &testIndex);
/****************************************************************/
/************************************phase******************/
				angle[3]=((angle[1]-angle[0])*360)/PI2;
                         	{
		

			{	if(angle[3]>0){while(1){if(angle[3]>360){angle[3]=angle[3]-360;} else break;}}
				else if(angle[3]<0){while(1){if(angle[3]<-360){angle[3]=angle[3]+360;} else break;}}
				
				if((angle[3]>3.0&&angle[3]<178.0)||(angle[3]>-357.0&&angle[3]<-182.0))flag_phase=1;//正序
				
			else if((angle[3]>182.0&&angle[3]<357.0)||(angle[3]>-178.0&&angle[3]<-3.0))flag_phase=0;//反序

				}

				}
/************************************phase_end*******************/
						angle[2]=((angle[1]-angle[0])*360)/PI2-90;					

				if(angle[2]>0){while(1){if(angle[2]>360){angle[2]=angle[2]-360;} else break;}}
			else if(angle[2]<0){while(1){if(angle[2]<-360){angle[2]=angle[2]+360;} else break;}} 
if((flag_phase==1))
{
if(((angle[2]>2.0)&&(angle[2]<90))||((angle[2]>-358.0&&angle[2]<-270.0))){L_C_flag_B=1;}
else if(((angle[2]<-2.0)&&(angle[2]>-90.0))||(angle[2]>270&&angle[2]<358)){ L_C_flag_B=0;}
}
if(flag_phase==0)
{
if(((angle[2]>180.0)&&(angle[2]<270))||((angle[2]>-180.0&&angle[2]<-90.0))){L_C_flag_B=1;}
else if(((angle[2]<180.0)&&(angle[2]>90.0))||(angle[2]>-270&&angle[2]<-180)){L_C_flag_B=0;}
}

dianliuzhi=T*maxValue_C*cruccent_ratio/16.8;
arm_sqrt_f32(1-(arm_cos_f32(angle[0]-angle[1]))*(arm_cos_f32(angle[0]-angle[1])),&sine);
gonglvshishu=sine*100;
if(dianliuzhi<zero_limit*T){gonglvshishu=100;dianliuzhi=0;L_C_flag_B=1;}//电流小于0.1A 时，电流就清零
else dianliuzhi=dianliuzhi/1000;
arm_sqrt_f32(1-sine*sine,&cose);

			 wugongkvar=((1.732*dianliuzhi*dianya_zhi*cose)/1000);
			 allkvar=((1.732*dianliuzhi*dianya_zhi*sine)/1000);
                    //wugongkvar=wugong_computer;
		


 
/*************************电流谐波率****************************************/
if((dianliuzhi==0)&&(gonglvshishu==100))HI=0;
else
{
for(i=3;i<=21;i=i+2){HI_SUM_B=(reslut[i]*reslut[i])+HI_SUM_B;}
arm_sqrt_f32(HI_SUM_B,&HI_B);
HI=(HI_B/maxValue_C)*1.03*100;
}
/******************************************************************/

gonglvshishu_A=gonglvshishu;
gonglvshishu_B=gonglvshishu;
gonglvshishu_C=gonglvshishu;
wugongkvar_A=wugongkvar/3;
wugongkvar_B=wugongkvar/3;
wugongkvar_C=wugongkvar/3;
L_C_flag_A=L_C_flag_B;
L_C_flag_C=L_C_flag_B;

}







   }




   if(KEY_3==1) 

{
/*********************判断相序*******************************/
{

ADC3_CH10_DMA_Config_VA();
ADC1_CH1_DMA_Config_VC_phase();

{
 for(i=0;i<TEST_LENGTH_SAMPLES;i++)
	 	{
	 	
	 	
testInput_C[i]=(float32_t)((ADC_Converted_CValue-ADC_Converted_base)*3.3/4096);///  1550

testInput_V[i]=(float32_t)((ADC_Converted_VValue-ADC_Converted_base)*3.3/4096);///  1550

delay_us(36);//36->512

        }

 
allphase(testInput_V,testInput_C);

 
	status = arm_rfft_init_f32(&S,&S_CFFT, fftSize,  
	  								ifftFlag, doBitReverse); 
	 
	/* Process the data through the CFFT/CIFFT module */ 
	arm_rfft_f32(&S, testInput_V,testOutput); 

             testIndex=1;
		angle[0]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//A相初始相位

/******************************************************************/
	arm_rfft_f32(&S, testInput_C,testOutput); 
         
	angle[1]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//C相初始相位


}
if((angle[0]-angle[1])>0)
{
phase=((angle[0]-angle[1])*360)/PI2;
if(phase>=118&&phase<=122)phase_flag=0;//正序
else phase_flag=1;
}
else 
	{
	phase=((angle[1]-angle[0])*360)/PI2;
if(phase>=238&&phase<=242)phase_flag=0;//正序
else phase_flag=1;


     }
}
/************************判断相序end**************************/

/*********************A_phase*********************************/
//for(s=1;s<=9;s++)
{
	if(phase_flag==0)
		{
ADC3_CH10_DMA_Config_VA();
ADC1_CH1_DMA_Config_CA();
		}
		if(phase_flag==1)
			{
ADC3_CH12_DMA_Config_VC();
ADC1_CH7_DMA_Config_CC();
		}


{
 for(i=0;i<TEST_LENGTH_SAMPLES;i++)
	 	{
	 	
	 	
testInput_C[i]=(float32_t)((ADC_Converted_CValue-ADC_Converted_base)*3.3/4096);///  1550

testInput_V[i]=(float32_t)((ADC_Converted_VValue-ADC_Converted_base)*3.3/4096);///  1550

delay_us(36);//36->512

        }

 
allphase(testInput_V,testInput_C);

 
	status = arm_rfft_init_f32(&S,&S_CFFT, fftSize,  
	  								ifftFlag, doBitReverse); 
	 
	/* Process the data through the CFFT/CIFFT module */ 
	arm_rfft_f32(&S, testInput_V,testOutput); 

             testIndex=1;
		angle[0]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//电压初始相位
	/* Process the data through the Complex Magnitude Module for  
	calculating the magnitude at each bin */ 

	/*******通过原始数据计算电压值***********/
//		arm_rfft_f32(&S, testInput_V_source,testOutput); 

	arm_cmplx_mag_f32(testOutput, reslut,  
	  				fftSize);  
	/* Calculates maxValue and returns corresponding BIN value */ 

	arm_max_f32(reslut, fftSize/2, &maxValue, &testIndex);
dianya_zhi_A=maxValue/1000;
dianya_zhi_A=dianya_zhi_A/4.35;
if(dianya_zhi_A<=100)dianya_zhi_A=0;

/*************************电压谐波率****************************************/

{
for(i=3;i<=21;i=i+2){HU_SUM_A=(reslut[i]*reslut[i])+HU_SUM_A;}
arm_sqrt_f32(HU_SUM_A,&HU_A);
A_HV=(HU_A/maxValue)*100;

}
/******************************************************************/

/******************************************************************/
	arm_rfft_f32(&S, testInput_C,testOutput); 
         
	angle[1]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//电流初始相位

	/*******通过原始数据计算电压值***********/

	//	arm_rfft_f32(&S, testInput_C_source,testOutput); 

	arm_cmplx_mag_f32(testOutput, reslut,  
	  				fftSize);  
	 
	/* Calculates maxValue and returns corresponding BIN value */ 
	arm_max_f32(reslut, fftSize/2, &maxValue_C, &testIndex);

/****************************************************************/
				angle[2]=((angle[1]-angle[0])*360)/PI2;

				  if(angle[2]>0){while(1){if(angle[2]>360){angle[2]=angle[2]-360;} else break;}}
				else if(angle[2]<0){while(1){if(angle[2]<-360){angle[2]=angle[2]+360;} else break;}}

					  
				if(angle[2]>0.0)
                               {
				if(angle[2]<90)L_C_flag_A=1;
				if(angle[2]>90&&angle[2]<180)L_C_flag_A=0;				
				if(angle[2]>180&&angle[2]<270)L_C_flag_A=1;
				if(angle[2]>270&&angle[2]<360)L_C_flag_A=0;


				}

				else if(angle[2]<=0.0)
				{
					if((angle[2]>-90.0&&angle[2]<=0.0))L_C_flag_A=0;
					if((angle[2]>-180.0&&angle[2]<-90.0))L_C_flag_A=1;
					if((angle[2]>-270.0&&angle[2]<-180.0))L_C_flag_A=0;
					if((angle[2]>-360.0&&angle[2]<-270.0))L_C_flag_A=1;

			     }

			if(angle[2]>0.0)
                               {
				if(angle[2]>90&&angle[2]<180)angle[2]=-(angle[2]-180);				
				if(angle[2]>180&&angle[2]<270)angle[2]=angle[2]-180;
				if(angle[2]>270&&angle[2]<360)angle[2]=-(angle[2]-360);


				}

				else if(angle[2]<=0.0)
				{
					if((angle[2]>-90.0&&angle[2]<=0.0))angle[2]=-angle[2];
					if((angle[2]>-180.0&&angle[2]<-90.0))angle[2]=(angle[2]+180);
					if((angle[2]>-270.0&&angle[2]<-180.0))angle[2]=-(angle[2]+180);
					if((angle[2]>-360.0&&angle[2]<-270.0))angle[2]=(angle[2]+360);

			     }


        
angle[2]=((angle[2])*PI2)/360;


/***************************************************************/
 dianliuzhi_A=T*maxValue_C*cruccent_ratio/16.8;
 if(dianliuzhi_A<=zero_limit*T){dianliuzhi_A=0;gonglvshishu_A=100;L_C_flag_A=1;}
else{ 
	dianliuzhi_A=dianliuzhi_A/1000;
	gonglvshishu_A=arm_cos_f32(angle[2])*100;//功率因素
}
/*************************电流谐波率****************************************/
if((dianliuzhi_A==0)&&(gonglvshishu_A==100))A_HI=0;
else
{
for(i=3;i<=21;i=i+2){HI_SUM_A=(reslut[i]*reslut[i])+HI_SUM_A;}
arm_sqrt_f32(HI_SUM_A,&HI_A);
A_HI=(HI_A/maxValue_C)*1.03*100;

}
/******************************************************************/

arm_sqrt_f32(1-(arm_cos_f32(angle[2]))*(arm_cos_f32(angle[2])),&sine);
        a=dianya_zhi_A*dianliuzhi_A*sine;
	wugongkvar_A=dianya_zhi_A*dianliuzhi_A*sine/1000;
      wugongkvar_95A=dianya_zhi_A*dianliuzhi_A*0.3122/1000;
allkvar_A=dianya_zhi_A*dianliuzhi_A*(arm_cos_f32(angle[2]))/1000;
}
				
//computer_trans_rs485(0,33,0,0,0,CPT_A);

/*********************B_phase*********************************/

{
ADC3_CH11_DMA_Config_VB();
ADC1_CH4_DMA_Config_CB();
 maxValue=0.0;
 maxValue_C=0.0; 

 for(i=0;i<TEST_LENGTH_SAMPLES;i++)
	 	{
	 	
	 	
testInput_C[i]=(float32_t)((ADC_Converted_CValue-ADC_Converted_base)*3.3/4096);///  1550

testInput_V[i]=(float32_t)((ADC_Converted_VValue-ADC_Converted_base)*3.3/4096);///  1550

delay_us(36);//36->512

        }

 
allphase(testInput_V,testInput_C);

 
	status = arm_rfft_init_f32(&S,&S_CFFT, fftSize,  
	  								ifftFlag, doBitReverse); 
	 
	/* Process the data through the CFFT/CIFFT module */ 
	arm_rfft_f32(&S, testInput_V,testOutput); 

             testIndex=1;
	 angle[0]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//电压初始相位

	/* Process the data through the Complex Magnitude Module for  
	calculating the magnitude at each bin */ 

	/*******通过原始数据计算电压值***********/
		//arm_rfft_f32(&S, testInput_V_source,testOutput); 

	arm_cmplx_mag_f32(testOutput, reslut,  
	  				fftSize);  
	/* Calculates maxValue and returns corresponding BIN value */ 

	arm_max_f32(reslut, fftSize/2, &maxValue, &testIndex);
dianya_zhi_B=maxValue/1000;
dianya_zhi_B=dianya_zhi_B/4.35;
if(dianya_zhi_B<=100)dianya_zhi_B=0;

/*************************电压谐波率****************************************/

{
for(i=3;i<=21;i=i+2){HU_SUM_B=(reslut[i]*reslut[i])+HU_SUM_B;}
arm_sqrt_f32(HU_SUM_B,&HU_B);
B_HV=(HU_B/maxValue)*100;
}
/******************************************************************/

/******************************************************************/
	arm_rfft_f32(&S, testInput_C,testOutput); 
         
	angle[1]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//电流初始相位

	/*******通过原始数据计算电压值***********/

	//	arm_rfft_f32(&S, testInput_C_source,testOutput); 

	arm_cmplx_mag_f32(testOutput, reslut,  
	  				fftSize);  
	 
	/* Calculates maxValue and returns corresponding BIN value */ 
	arm_max_f32(reslut, fftSize/2, &maxValue_C, &testIndex);
/****************************************************************/
				angle[2]=((angle[1]-angle[0])*360)/PI2;

				  if(angle[2]>0){while(1){if(angle[2]>360){angle[2]=angle[2]-360;} else break;}}
				else if(angle[2]<0){while(1){if(angle[2]<-360){angle[2]=angle[2]+360;} else break;}}

					  
				if(angle[2]>0.0)
                               {
				if(angle[2]<90)L_C_flag_B=1;
				if(angle[2]>90&&angle[2]<180)L_C_flag_B=0;				
				if(angle[2]>180&&angle[2]<270)L_C_flag_B=1;
				if(angle[2]>270&&angle[2]<360)L_C_flag_B=0;


				}

				else if(angle[2]<=0.0)
				{
					if((angle[2]>-90.0&&angle[2]<=0.0))L_C_flag_B=0;
					if((angle[2]>-180.0&&angle[2]<-90.0))L_C_flag_B=1;
					if((angle[2]>-270.0&&angle[2]<-180.0))L_C_flag_B=0;
					if((angle[2]>-360.0&&angle[2]<-270.0))L_C_flag_B=1;

			     }

			if(angle[2]>0.0)
                               {
				if(angle[2]>90&&angle[2]<180)angle[2]=-(angle[2]-180);				
				if(angle[2]>180&&angle[2]<270)angle[2]=angle[2]-180;
				if(angle[2]>270&&angle[2]<360)angle[2]=-(angle[2]-360);


				}

				else if(angle[2]<=0.0)
				{
					if((angle[2]>-90.0&&angle[2]<=0.0))angle[2]=-angle[2];
					if((angle[2]>-180.0&&angle[2]<-90.0))angle[2]=(angle[2]+180);
					if((angle[2]>-270.0&&angle[2]<-180.0))angle[2]=-(angle[2]+180);
					if((angle[2]>-360.0&&angle[2]<-270.0))angle[2]=(angle[2]+360);

			     }


        
angle[2]=((angle[2])*PI2)/360;


/***************************************************************/
dianliuzhi_B=T*maxValue_C*cruccent_ratio/16.8;
 if(dianliuzhi_B<=zero_limit*T){dianliuzhi_B=0;gonglvshishu_B=100;L_C_flag_B=1;}
else {
        dianliuzhi_B=dianliuzhi_B/1000;
	gonglvshishu_B=arm_cos_f32(angle[2])*100;//功率因素
}
/*************************电流谐波率****************************************/
if((dianliuzhi_B==0)&&(gonglvshishu_B==100))B_HI=0;
else
{
for(i=3;i<=21;i=i+2){HI_SUM_B=(reslut[i]*reslut[i])+HI_SUM_B;}
arm_sqrt_f32(HI_SUM_B,&HI_B);
B_HI=(HI_B/maxValue_C)*1.03*100;

}
/******************************************************************/

arm_sqrt_f32(1-(arm_cos_f32(angle[2]))*(arm_cos_f32(angle[2])),&sine);
         b=dianya_zhi_B*dianliuzhi_B*sine;
	wugongkvar_B=dianya_zhi_B*dianliuzhi_B*sine/1000;
      wugongkvar_95B=dianya_zhi_B*dianliuzhi_B*0.3122/1000;
allkvar_B=dianya_zhi_B*dianliuzhi_B*(arm_cos_f32(angle[2]))/1000;



}
//computer_trans_rs485(0,33,0,0,0,CPT_B);

/*********************C_phase*********************************/

{
	if(phase_flag==1)
		{
ADC3_CH10_DMA_Config_VA();
ADC1_CH1_DMA_Config_CA();
		}
		if(phase_flag==0)
			{
ADC3_CH12_DMA_Config_VC();
ADC1_CH7_DMA_Config_CC();
		}

 maxValue=0.0;
 maxValue_C=0.0; 

 for(i=0;i<TEST_LENGTH_SAMPLES;i++)
	 	{
	 	
	 	
testInput_C[i]=(float32_t)((ADC_Converted_CValue-ADC_Converted_base)*3.3/4096);///  1550

testInput_V[i]=(float32_t)((ADC_Converted_VValue-ADC_Converted_base)*3.3/4096);///  1550

delay_us(36);//36->512

        }

 
allphase(testInput_V,testInput_C);

 
	status = arm_rfft_init_f32(&S,&S_CFFT, fftSize,  
	  								ifftFlag, doBitReverse); 
	 
	/* Process the data through the CFFT/CIFFT module */ 
	arm_rfft_f32(&S, testInput_V,testOutput); 

             testIndex=1;
	 angle[0]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//电压初始相位
	/* Process the data through the Complex Magnitude Module for  
	calculating the magnitude at each bin */ 

	/*******通过原始数据计算电压值***********/
//		arm_rfft_f32(&S, testInput_V_source,testOutput); 

	arm_cmplx_mag_f32(testOutput, reslut,  
	  				fftSize);  
	/* Calculates maxValue and returns corresponding BIN value */ 

	arm_max_f32(reslut, fftSize/2, &maxValue, &testIndex);
dianya_zhi_C=maxValue/1000;
dianya_zhi_C=dianya_zhi_C/4.35;
if(dianya_zhi_C<=100)dianya_zhi_C=0;

/*************************电压谐波率****************************************/

{
for(i=3;i<=21;i=i+2){HU_SUM_C=(reslut[i]*reslut[i])+HU_SUM_C;}
arm_sqrt_f32(HU_SUM_C,&HU_C);
C_HV=(HU_C/maxValue)*100;
}
/******************************************************************/


/******************************************************************/
	arm_rfft_f32(&S, testInput_C,testOutput); 
         
	angle[1]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//电流初始相位

	/*******通过原始数据计算电压值***********/

	//	arm_rfft_f32(&S, testInput_C_source,testOutput); 

	arm_cmplx_mag_f32(testOutput, reslut,  
	  				fftSize);  
	 
	/* Calculates maxValue and returns corresponding BIN value */ 
	arm_max_f32(reslut, fftSize/2, &maxValue_C, &testIndex);
/****************************************************************/
				angle[2]=((angle[1]-angle[0])*360)/PI2;

				  if(angle[2]>0){while(1){if(angle[2]>360){angle[2]=angle[2]-360;} else break;}}
				else if(angle[2]<0){while(1){if(angle[2]<-360){angle[2]=angle[2]+360;} else break;}}

					  
				if(angle[2]>0.0)
                               {
				if(angle[2]<90)L_C_flag_C=1;
				if(angle[2]>90&&angle[2]<180)L_C_flag_C=0;				
				if(angle[2]>180&&angle[2]<270)L_C_flag_C=1;
				if(angle[2]>270&&angle[2]<360)L_C_flag_C=0;


				}

				else if(angle[2]<=0.0)
				{
					if((angle[2]>-90.0&&angle[2]<=0.0))L_C_flag_C=0;
					if((angle[2]>-180.0&&angle[2]<-90.0))L_C_flag_C=1;
					if((angle[2]>-270.0&&angle[2]<-180.0))L_C_flag_C=0;
					if((angle[2]>-360.0&&angle[2]<-270.0))L_C_flag_C=1;

			     }

			if(angle[2]>0.0)
                               {
				if(angle[2]>90&&angle[2]<180)angle[2]=-(angle[2]-180);				
				if(angle[2]>180&&angle[2]<270)angle[2]=angle[2]-180;
				if(angle[2]>270&&angle[2]<360)angle[2]=-(angle[2]-360);


				}

				else if(angle[2]<=0.0)
				{
					if((angle[2]>-90.0&&angle[2]<=0.0))angle[2]=-angle[2];
					if((angle[2]>-180.0&&angle[2]<-90.0))angle[2]=(angle[2]+180);
					if((angle[2]>-270.0&&angle[2]<-180.0))angle[2]=-(angle[2]+180);
					if((angle[2]>-360.0&&angle[2]<-270.0))angle[2]=(angle[2]+360);

			     }


        
angle[2]=((angle[2])*PI2)/360;


/***************************************************************/
dianliuzhi_C=T*maxValue_C*cruccent_ratio/16.8;
 if(dianliuzhi_C<=zero_limit*T){dianliuzhi_C=0;gonglvshishu_C=100;L_C_flag_C=1;}
else
	{
	dianliuzhi_C=dianliuzhi_C/1000;
	gonglvshishu_C=arm_cos_f32(angle[2])*100;//功率因素
}
/*************************电流谐波率****************************************/
if((dianliuzhi_C==0)&&(gonglvshishu_C==100))C_HI=0;
else
{
for(i=3;i<=21;i=i+2){HI_SUM_C=(reslut[i]*reslut[i])+HI_SUM_C;}
arm_sqrt_f32(HI_SUM_C,&HI_C);
C_HI=(HI_C/maxValue_C)*1.03*100;

}
/******************************************************************/

arm_sqrt_f32(1-(arm_cos_f32(angle[2]))*(arm_cos_f32(angle[2])),&sine);
           c=dianya_zhi_C*dianliuzhi_C*sine;
	wugongkvar_C=dianya_zhi_C*dianliuzhi_C*sine/1000;
      wugongkvar_95C=dianya_zhi_C*dianliuzhi_C*0.3122/1000;
allkvar_C=dianya_zhi_C*dianliuzhi_C*(arm_cos_f32(angle[2]))/1000;

                               				

}



//tempshuzhi=phase_flag;








/****************************************************/
//computer_trans_rs485(0,33,0,0,0,CPT_C);

/***************************************************/
//inquiry_slave_status_dis(3,dis_list,comm_list);   

/*********************ALL***********************************/
dianya_zhi=1.732*(dianya_zhi_A+dianya_zhi_B+dianya_zhi_C)/3;
dianliuzhi=(dianliuzhi_A+dianliuzhi_B+dianliuzhi_C)/3;
gonglvshishu=(gonglvshishu_A+gonglvshishu_B+gonglvshishu_C)/3;
wugongkvar=(a+b+c)/1000;
//wugongkvar=wugongkvar_A+wugongkvar_B+wugongkvar_C;
allkvar=allkvar_A+allkvar_B+allkvar_C;//乘以3，是因为电流变量是一相的电流，应该变为三相的电流和
//HV=HVA+HVB+HVC;
//HI=HIA+HIB+HIC;
  wugongkvar_95=wugongkvar_95A+wugongkvar_95B+wugongkvar_95C;



}

}


/*********************变比判断*******************************/

if(KEY_3==1)dis_num=6;
else dis_num=0;

if(warning_flag==0&&(A_HV<HU_PROT_para&&B_HV<HU_PROT_para&&C_HV<HI_PROT_para)&&(A_HI<HI_PROT_para&&B_HI<HI_PROT_para&&C_HI<HI_PROT_para)&&(HI<HI_PROT_para)&&(HV<HU_PROT_para)&&((V_PROT_para_L+200)>dianya_zhi_A)&&((V_PROT_para_L+200)>dianya_zhi_B)&&((V_PROT_para_L+200)>dianya_zhi_C)&&((V_PROT_para_tri+400)>dianya_zhi))
{
{



if(gonglvshishu<COS_ON_para&&L_C_flag_B==1)
 {
      {
	  	
      	{
for(i=first;i<=32-dis_num;i++)
if(comm_list[i].work_status==0&&(wugongkvar>=comm_list[i].size)&&comm_list[i].size>0)
{
display_nothing_close_open_warn=1;//设置显示投入
{
set_74hc273(comm_list[i].id, ON);
 Light_pad_on(dis_com,comm_list[i].id,1,1,0);
comm_list[i].work_status=1;
//qeen( comm_list,i,32);
first=i+1;

}
return 0 ;
}
first=1;

      	}


  	
  
   
}
 }
}

if(((gonglvshishu>COS_OFF_para&&L_C_flag_B==1)||(L_C_flag_B==0)))
   
{
      {

{
for(i=end;i<=32-dis_num;i++)
if(comm_list[i].work_status==1&&comm_list[i].size>0)

{
display_nothing_close_open_warn=2;//设置显示切除
{
set_74hc273(comm_list[i].id, OFF);
 Light_pad_on(dis_com,comm_list[i].id,0,0,0);
comm_list[i].work_status=0;
end=i+1;
//qeen(i,32);
}
		{

return 0 ;

		}
}
end=1;

}
	  
       }
 }


}

if(KEY_3==1)
{
if(1)
  {
if(gonglvshishu_A<COS_ON_para&&L_C_flag_A==1)
{
      {
	  	
      	{
for(i=27;i<=30;i=i+3)
if(comm_list[i].work_status==0&&(wugongkvar_A>=comm_list[i].size)&&comm_list[i].size>0)
{
display_nothing_close_open_warn=1;//设置显示投入
{
set_74hc273(comm_list[i].id, ON);
 Light_pad_on(dis_com,comm_list[i].id,1,1,0);
comm_list[i].work_status=1;

}
return 0 ;
}

      	}
  
   
}
 }
if(gonglvshishu_B<COS_ON_para&&L_C_flag_B==1)
{
      {
	  	
      	{
for(i=28;i<=31;i=i+3)
if(comm_list[i].work_status==0&&(wugongkvar_B>=comm_list[i].size)&&comm_list[i].size>0)
{
display_nothing_close_open_warn=1;//设置显示投入
{
set_74hc273(comm_list[i].id, ON);
 Light_pad_on(dis_com,comm_list[i].id,1,1,0);
comm_list[i].work_status=1;

}
return 0 ;
}

      	}
  
   
}
 }

if(gonglvshishu_C<COS_ON_para&&L_C_flag_C==1)
{
      {
	  	
      	{
for(i=29;i<=32;i=i+3)
if(comm_list[i].work_status==0&&(wugongkvar_C>=comm_list[i].size)&&comm_list[i].size>0)
{
display_nothing_close_open_warn=1;//设置显示投入
{
set_74hc273(comm_list[i].id, ON);
 Light_pad_on(dis_com,comm_list[i].id,1,1,0);
comm_list[i].work_status=1;

}
return 0 ;
}

      	}
  
   
}
 }

	
  }







if(1)

{
if((gonglvshishu_A>COS_OFF_para&&L_C_flag_A==1)||L_C_flag_A==0)
{
      {

{
for(i=27;i<=30;i=i+3)
if(comm_list[i].work_status==1&&comm_list[i].size>0)

{
display_nothing_close_open_warn=2;//设置显示切除
{
set_74hc273(comm_list[i].id, OFF);
 Light_pad_on(dis_com,comm_list[i].id,0,0,0);
comm_list[i].work_status=0;
}
		{

return 0 ;

		}
}

}
	  
       }
 }
if((gonglvshishu_B>COS_OFF_para&&L_C_flag_B==1)||L_C_flag_B==0)
{
      {

{
for(i=28;i<=31;i=i+3)
if(comm_list[i].work_status==1&&comm_list[i].size>0)

{
display_nothing_close_open_warn=2;//设置显示切除
{
set_74hc273(comm_list[i].id, OFF);
 Light_pad_on(dis_com,comm_list[i].id,0,0,0);
comm_list[i].work_status=0;
}
		{

return 0 ;

		}
}

}
	  
       }
 }

if((gonglvshishu_C>COS_OFF_para&&L_C_flag_C==1)||L_C_flag_C==0)

{
      {

{
for(i=29;i<=32;i=i+3)
if(comm_list[i].work_status==1&&comm_list[i].size>0)

{
display_nothing_close_open_warn=2;//设置显示切除
{
set_74hc273(comm_list[i].id, OFF);
 Light_pad_on(dis_com,comm_list[i].id,0,0,0);
comm_list[i].work_status=0;
}
		{

return 0 ;

		}
}

}
	  
       }
 }
display_nothing_close_open_warn=0;//设置显示无
	
  }


}



//T=4;
/**************************end*************************/


if(1)
{
if(warning_flag==0&&(A_HV<HU_PROT_para&&B_HV<HU_PROT_para&&C_HV<HI_PROT_para)&&(A_HI<HI_PROT_para&&B_HI<HI_PROT_para&&C_HI<HI_PROT_para)&&(HI<HI_PROT_para)&&(HV<HU_PROT_para)&&((V_PROT_para_L+200)>dianya_zhi_A)&&((V_PROT_para_L+200)>dianya_zhi_B)&&((V_PROT_para_L+200)>dianya_zhi_C)&&((V_PROT_para_tri+400)>dianya_zhi))

{


{






display_nothing_close_open_warn=0;//设置显示无

  




}

}
else
	{

if(KEY_3==1)
	{
if(warning_flag==0&&(((V_PROT_para_L+200)<=dianya_zhi_A)||((V_PROT_para_L+200)<=dianya_zhi_B)||((V_PROT_para_L+200)<=dianya_zhi_C)))	
{
warning_flag=1;

}
if(warning_flag==1&&(((V_PROT_para_L+200-7)>dianya_zhi_A)||((V_PROT_para_L+200-7)>dianya_zhi_B)||((V_PROT_para_L+200-7)>dianya_zhi_C)))	
{
warning_flag=0;

}

}
if(KEY_3==0)
{
if(warning_flag==0&&(((V_PROT_para_tri+400)<=dianya_zhi)))
{
warning_flag=1;
}

if(warning_flag==1&&((V_PROT_para_tri+400-7)>dianya_zhi))
{
warning_flag=0;
}
}
display_nothing_close_open_warn=3;//设置显示报警

if(1)
{

{
for(i=end;i<=32-dis_num;i++)
if(comm_list[i].work_status==1&&comm_list[i].size>0)

{
display_nothing_close_open_warn=2;//设置显示切除
{
set_74hc273(comm_list[i].id, OFF);
comm_list[i].work_status=0;
end=i+1;
}
		{

return 0 ;

		}
}

end=1;
}
	  
       }

if(KEY_3==1)
{
{
      {

{
for(i=27;i<=30;i=i+3)
if(comm_list[i].work_status==1&&comm_list[i].size>0)

{
display_nothing_close_open_warn=2;//设置显示切除
{
set_74hc273(comm_list[i].id, OFF);
 Light_pad_on(dis_com,comm_list[i].id,0,0,0);
comm_list[i].work_status=0;
}
		{

return 0 ;

		}
}

}
	  
       }
 }
{
      {

{
for(i=29;i<=32;i=i+3)
if(comm_list[i].work_status==1&&comm_list[i].size>0)

{
display_nothing_close_open_warn=2;//设置显示切除
{
set_74hc273(comm_list[i].id, OFF);
 Light_pad_on(dis_com,comm_list[i].id,0,0,0);
comm_list[i].work_status=0;
}
		{

return 0 ;

		}
}

}
	  
       }
 }
{
      {

{
for(i=29;i<=32;i=i+3)
if(comm_list[i].work_status==1&&comm_list[i].size>0)

{
display_nothing_close_open_warn=2;//设置显示切除
{
set_74hc273(comm_list[i].id, OFF);
 Light_pad_on(dis_com,comm_list[i].id,0,0,0);
comm_list[i].work_status=0;
}
		{

return 0 ;

		}
}

}
	  
       }
 }


}


}

}



return 0;

}




/***********************************************************************
TIME_4

**********************************************************************/


/******************************************************/
void EXTI_Configuration(void)//初始化函数
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//打开时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

	 		
	//使能外部中断复用时钟
	
	//映射GPIOE的Pin0至EXTILine0
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource8);




EXTI_InitStructure.EXTI_Line = EXTI_Line8;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);         	//嵌套分组为组0
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;      	//中断通道为通道10
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   //抢断优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;    		//响应优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     		//开中断
	NVIC_Init(&NVIC_InitStructure);
	 EXTI_GenerateSWInterrupt(EXTI_Line8);
/*********************************************************************/
	

SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource12);
EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;      	//中断通道为通道10
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   //抢断优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;    		//响应优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     		//开中断
	NVIC_Init(&NVIC_InitStructure);
	 EXTI_GenerateSWInterrupt(EXTI_Line12);


SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource14);
EXTI_InitStructure.EXTI_Line = EXTI_Line14;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;      	//中断通道为通道10
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   //抢断优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;    		//响应优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     		//开中断
	NVIC_Init(&NVIC_InitStructure);
	 EXTI_GenerateSWInterrupt(EXTI_Line14);

}


void EXTI9_5_IRQHandler(void)
{
	OSIntEnter();   

  if(EXTI_GetITStatus(EXTI_Line8) != RESET)
	
	{


	}
      EXTI_ClearITPendingBit(EXTI_Line8);

  if(EXTI_GetITStatus(EXTI_Line9) != RESET)
	
	{
	delay_us(1000);//按键消抖
if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_9)==0)
{	
}

	}
      EXTI_ClearITPendingBit(EXTI_Line9);


	  

	   	OSIntExit();  

}

void EXTI15_10_IRQHandler(void)
{


	OSIntEnter();   

  if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	
	{
	if(COMMCAT_para==1)
{
if(dis_com==1)
{

/*快速控制器手动切 共补*/
set_74hc273(hand_id, OFF);
 Light_pad_on(dis_com,hand_id,1,1,0);
comm_list[hand_id].work_status=0;

}


}

	}
      EXTI_ClearITPendingBit(EXTI_Line12);

  if(EXTI_GetITStatus(EXTI_Line14) != RESET)
	
	{
	if(COMMCAT_para==1)
{
if(dis_com==1)
{



/*快速控制器手动投 共补*/

set_74hc273(hand_id, ON);
 Light_pad_on(dis_com,hand_id,1,1,0);
comm_list[hand_id].work_status=1;

}
	
}

	}
      EXTI_ClearITPendingBit(EXTI_Line14);


	  

	   	OSIntExit();  

}
/*************************************************/
void LIGHT_backligt_on()
{
GPIO_SetBits(GPIOD, GPIO_Pin_15);
light_time=100;

}


/*************************************************/
void LIGHT_backligt_off()
{
GPIO_ResetBits(GPIOD, GPIO_Pin_15);

}

/***********************************************************************
TIME_4
**********************************************************************/



 void TIM4_Int_Init(u16 arr,u16 psc)

{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //时钟使能
	
	//定时器TIM4初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //使能指定的TIM4中断,允许更新中断

	//中断优先级NVIC设置
			NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


	TIM_Cmd(TIM4, ENABLE);  //使能TIMx					 
}

 void TIM4_IRQHandler(void)   //TIM3中断
{	 
	OSIntEnter();   
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //检查TIM4更新中断发生与否
		{	  
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //清除TIMx更新中断标志
                    IWDG_Feed();
		}
	   	OSIntExit();  

 	}
/*************************************************/


 void TIM2_Int_Init(u16 arr,u16 psc)

{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能
	
	//定时器TIM4初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //使能指定的TIM4中断,允许更新中断

	//中断优先级NVIC设置
			NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


	TIM_Cmd(TIM2, ENABLE);  //使能TIMx					 
}

 void TIM2_IRQHandler(void)   //TIM3中断
{	 
	OSIntEnter();   
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //检查TIM4更新中断发生与否
		{	// IWDG_Feed();
		if(light_time_led>0)light_time_led--;
			if(light_time_led==0){Work_Flag=!Work_Flag;	light_time_led=50;}
			if(light_time>0)light_time--;
 if(light_time==0)LIGHT_backligt_off();
if(delay_on==1)
	{
	      if(delay_on_cont!=0)delay_on_cont--;
	}
if(delay_on==0){delay_on_cont=AT24CXX_ReadOneByte(0x0001);delay_on_cont=delay_on_cont*2;}

if(delay_off==1)
	{
	      if(delay_off_cont!=0)delay_off_cont--;
	}
if(delay_off==0){delay_off_cont=AT24CXX_ReadOneByte(0x0002);delay_off_cont=delay_off_cont*2;}

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //清除TIMx更新中断标志
		
		}
	   	OSIntExit();  

 	}

void set_bit(u8 b, u8 dis_com,light_status_node *light_status,u8 status_1,u8 status_2,u8 status_3 ,u8 status_4)
{
b=b-1;
if(dis_com==0){light_status->dis_comm=(~(0x00000001<<b))&light_status->dis_comm;}
if(dis_com==1){light_status->dis_comm=(0x00000001<<b)|light_status->dis_comm;}

{
if(status_1==0||status_1==3)light_status->work_status[0]=(~(0x00000001<<b))&light_status->work_status[0];
if(status_2==0||status_2==3)light_status->work_status[1]=(~(0x00000001<<b))&light_status->work_status[1];
if(status_3==0||status_3==3)light_status->work_status[2]=(~(0x00000001<<b))&light_status->work_status[2];
}

{
if(status_1==1)light_status->work_status[0]=(0x00000001<<b)|light_status->work_status[0];
if(status_2==1)light_status->work_status[1]=(0x00000001<<b)|light_status->work_status[1];
if(status_3==1)light_status->work_status[2]=(0x00000001<<b)|light_status->work_status[2];
}
{
	if(status_4==0)light_status->work_status[3]=(~(0x00000001<<b))&light_status->work_status[3];
       if(status_4==2)light_status->work_status[3]=(0x00000001<<b)|light_status->work_status[3];
}
}

u8 clear_bit(u8 b,u32 light_pad)
{
b=b-1;
if(((light_pad>>b)&0x00000001)==1)return 1;
if(((light_pad>>b)&0x00000001)==0) return 0;
else return 2;
}
void set_clear_existence(u8 true_false,u8 b,u32 *exist)
{
b=b-1;
if(true_false==0){(*exist)=(~(0x00000001<<b))&(*exist);}
if(true_false==1){(*exist)=(0x00000001<<b)|(*exist);}

}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/




