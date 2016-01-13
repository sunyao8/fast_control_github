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
#define  APP_TASK_START_STK_SIZE                         64u
static  OS_STK         App_TaskStartStk[APP_TASK_START_STK_SIZE];
#define  APP_TASK_START_PRIO                               10


#define  APP_TASK_LCD_STK_SIZE                          1024u
static  OS_STK         App_TaskLCDStk[APP_TASK_LCD_STK_SIZE];
#define  APP_TASK_LCD_PRIO                               3


#define  APP_TASK_COMPUTER_STK_SIZE                       10240u    
static  OS_STK         App_TaskComputerStk[APP_TASK_COMPUTER_STK_SIZE];
#define  APP_TASK_COMPUTER_PRIO                               2

#define  APP_TASK_Master_STK_SIZE                         64u
static  OS_STK         App_TaskMasterStk[APP_TASK_Master_STK_SIZE];
#define  APP_TASK_Master_PRIO                               1


#define urgent_TASK_PRIO       			0 
//设置任务堆栈大小
#define urgent_STK_SIZE  		    		64
//任务堆栈
OS_STK urgent_TASK_STK[urgent_STK_SIZE];


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
#if (OS_VIEW_MODULE == DEF_ENABLED)
extern void  App_OSViewTaskCreate   (void);
#endif

static  void  App_TaskCreate		(void);
static  void  App_TaskStart			(void		*p_arg);  
extern  void  App_UCGUI_TaskCreate  (void);
static  void  App_TaskLCD		(void		*p_arg); ;
static  void  App_Taskcomputer	 (void		*p_arg );
static  void  App_TaskMaster(void		*p_arg );
static  void urgent_task(void *pdata);

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
u8 computer_gonglu(status_dis_node *dis_list,status_comm_node *comm_list,u8 *slave_dis,u8 *slave_comm);


/*****************************485_start*********************************************************/

#define delay_time_base 1000                   //延时数量级
#define time_out 3000
#define LEN_control 14
#define EN_USART2_RX 	1			//0,不接收;1,接收.
#define RS485_TX_EN_1		GPIO_SetBits(GPIOC, GPIO_Pin_12)	// 485模式控制.0,接收;1,发送.本工程用PB15
#define RS485_TX_EN_0		GPIO_ResetBits(GPIOC, GPIO_Pin_12)	// 485模式控制.0,接收;1,发送.本工程用PB15
 OS_EVENT * RS485_STUTAS_MBOX_dis,* RS485_STUTAS_MBOX,* RS485_RT;			//	rs485邮箱信号量
 OS_EVENT *computer_sem,*urgent_sem;			 //

static u8 rs485buf[LEN_control];
;//发送控制信息


u32 hand_light_existence;

static u8 first_init=1;

//接收到的数据长度
u8 RS485_RX_CNT=0;  



 typedef struct  
{ u8 start;
  u8 myid;      //本电容箱ID号
  u8 source;
  u8 destination; //目的电容箱
  u8 send;      //是否是发送命令1为是，0为不是
  u8 relay;    //第几组电容器
  u8 message;     //开关信息
  u8 master;      //主机令牌
u8 end;
}box;
static box mybox;
u8 auto_on=1;
void RS485_Init(u32 bound);
void initmybox(void);//初始化自身信息

void UART4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
u16 comp_16(u16 a,u16 b);
int rs485_trans_order(u8 *tx_r485);//解析由主机发送过来的信号，并发送给下位机
 void order_trans_rs485(u8 source,u8 destination, u8 send,u8 relay,u8 message,u8 ctr);//主机程序，主机命令解析成RS485信息，发送给目的从机
 void computer_trans_rs485(u8 source,u8 destination, u8 send,u8 relay,u8 message,u8 ctr);//主机程序，主机计算出来数据解析成RS485信息，发送给目的从机
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

u8  subswitchABC_onoff	 (u8 relay,u8 message ,u8 flag);
void LIGHT_backligt_on(void);
void LIGHT_backligt_off(void);

/***********************************end*******************************************************/


/************************************TIME******************************************************/
void delay_time(u32 time);
 void heartbeat(u8 t);



/************************************TIME_end******************************************************/

void init_Queue_dis(status_dis_node *dis_list,u8 *slave_dis);
void change_Queue_dis(u8 abc,u8 Level, status_dis_node *dis_list,u8 *slave_dis);


/************************************MAster data structure*******************/
void scanf_slave_machine(status_dis_node *dis_list,status_comm_node *comm_list,u8 *slave_dis,u8 *slave_comm);
u8 inquiry_slave_status_comm(u8 id,u8 *slave_comm,status_comm_node *comm_list); 
void init_Queue(u8 id,u8 size_1,u8 size_2,u8 work_status_1,u8 work_status_2,u8 *slave_comm,status_comm_node *comm_list);
void del_comm_listnode(u8 id,u8 group,u8 *slave_comm,status_comm_node *comm_list);
void flash_comm_list(u8 id,u8 size ,u8 work_status ,u8 group,u8 *slave_comm,status_comm_node *comm_list);
void change_Queue(u8 *slave_comm,status_comm_node *comm_list,u8 size);
void init_listindex(u8 *slave_comm);




/********************************************************************************/
 void rs485_trans_status_dis(u8 count,u8 *tx_r485,status_dis_node *dis_list,status_comm_node *comm_list);//主机程序，主机命令解析成RS485信息，发送给目的从机
 u8 inquiry_slave_status_dis(u8 count,u8 id,status_dis_node *dis_list,status_comm_node *comm_list);   
void set_statuslist(u8 count,u8 id,u8 size,u8 work_status,u8 work_time,u8 dis_comm,u8 relay,status_dis_node *dis_list,status_comm_node *comm_list_1,status_comm_node *comm_list_2,u8 group);

void set_bit(u8 b, u8 dis_com,light_status_node *light_status,u8 status_1,u8 status_2,u8 status_3,u8 status_4);
u8 clear_bit(u8 b,u32 light_pad);
void set_clear_existence(u8 true_false,u8 b,u32 *exist);

/*************************************MAster data structure_end***************/





u8 L_C_flag_A=1;//感性容性标准变量
u8 L_C_flag_B=1;//感性容性标准变量
u8 L_C_flag_C=1;//感性容性标准变量
u16 hand_comm_onoff=0;//手动投切共补开关状态变量
u16 comm_number=0;//手动投切共补0为继电器1号 1为继电器2号

#define TEST_LENGTH_SAMPLES 512*2 
 
u8 phase_flag=0;
u16 scan_init=0;
u8 MASTER=1;
u8 light_time=100;

u8 delay_on=0,delay_off=0;
u8 delay_on_cont=1,delay_off_cont=1;
/********************控制器设置参数*************************/



extern u8 COMMCAT_para;
//extern u8 CT_para;
extern u8 capa1_array[32],capa2_array[32];

u8 hand_id=1;
u8 dis_com=1;
//u8 free_timeout_20=100;//轮休时间控制变量
//u8 free_timeout_10=100;//轮休时间控制变量
//u8 free_timeout_5=100;//轮休时间控制变量

/**********************************************/

INT32S main (void)
{
CPU_INT08U  os_err;
	

//CPU_IntDis();                   
/***************  Init hardware ***************/
//u8 i;

    OS_CPU_SysTickInit();/* Initialize the SysTick.                              */
	delay_init();
	delay_us(500000);
NVIC_Configuration();
GPIO_Configuration();

 //EXTI_Configuration();//初始化函数

initmybox();//初始化自身信息
 init_light_off();
 LIGHT_backligt_on();

os_err = os_err; 


   {
		OSInit();                        


	os_err = OSTaskCreateExt((void (*)(void *)) App_TaskStart,
                             (void          * ) 0,
                             (OS_STK        * )&App_TaskStartStk[APP_TASK_START_STK_SIZE - 1],
                             (INT8U           ) APP_TASK_START_PRIO,
                             (INT16U          ) APP_TASK_START_PRIO,
                             (OS_STK        * )&App_TaskStartStk[0],
                             (INT32U          ) APP_TASK_START_STK_SIZE,
                             (void          * )0,
                             (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));
                             

#if OS_TASK_NAME_EN > 0
    OSTaskNameSet(APP_TASK_START_PRIO, (CPU_INT08U *)"Start Task", &os_err);
#endif

	OSStart();                                               
	return (0);
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
static  void  App_TaskStart (void *p_arg)
{   
	(void)p_arg;
	


		OSStatInit();					//初始化统计任务.这里会延时1秒钟左右	
            App_TaskCreate();                                        /* Create application tasks.                            */
	OSTaskSuspend(APP_TASK_START_PRIO);	//挂起起始任务.




}

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

static  void  App_TaskCreate (void)
{
	
CPU_INT08U  os_err;

RS485_STUTAS_MBOX_dis=OSMboxCreate((void*)0);
RS485_STUTAS_MBOX=OSMboxCreate((void*)0);
computer_sem=OSSemCreate(0);
RS485_RT=OSMboxCreate((void*)0);
urgent_sem=OSSemCreate(0);
                             

#if OS_TASK_NAME_EN > 0
    OSTaskNameSet(APP_TASK_START_PRIO, (CPU_INT08U *)"Start Task", &os_err);
#endif
	 	OSTaskCreate(App_TaskLCD,(void *)0,(OS_STK*)&App_TaskLCDStk[APP_TASK_LCD_STK_SIZE-1],APP_TASK_LCD_PRIO);	 				   
		OSTaskCreate(App_Taskcomputer,(void *)0,(OS_STK*)&App_TaskComputerStk[APP_TASK_COMPUTER_STK_SIZE-1],APP_TASK_COMPUTER_PRIO);	 				   
//	 	OSTaskCreate(App_TaskMaster,(void *)0,(OS_STK*)&App_TaskMasterStk[APP_TASK_Master_STK_SIZE-1],APP_TASK_Master_PRIO);	 				   
//OSTaskCreate(urgent_task,(void *)0,(OS_STK*)&urgent_TASK_STK[urgent_STK_SIZE-1],urgent_TASK_PRIO);

     }

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
static  void  App_TaskMaster(void		*p_arg )
{
u8 i;
// static status_dis_node     dis_list[10];
 //static status_comm_node comm_list[10];
	for(;;)
		{ 


 if(MASTER==1)
 	{
 	

OSSemPost(computer_sem);
if(KEY_2==1)
	{
	}



	
 	}



delay_ms(100);
delay_ms(100);


delay_ms(100);

	        }
   	
}





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
static  void  App_TaskLCD	 (void		*p_arg )
{  

u8 status_1,status_2,status_3,status_4;
u8 exist;
u8 i;
	for(;;)
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
				hand_comm_onoff=0;
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
				hand_comm_onoff=0;
				break;
				}
	 
	  }
		//	if(i==0)hand_id=32;

	  }

	   if(exist==1)	{
           status_4=clear_bit(hand_id,light_status.work_status[3]);
	//	   dis_com= clear_bit(hand_id,light_status.dis_comm);
		if(status_4==0)   
			{
	status_1= clear_bit(hand_id,light_status.work_status[0]);
	status_2= clear_bit(hand_id,light_status.work_status[1]);
	status_3= clear_bit(hand_id,light_status.work_status[2]);
if(Work_Flag==1)Light_pad_on(dis_com,hand_id,status_1,status_2,status_3);
if(Work_Flag==0)Light_pad_off(dis_com,hand_id,status_1,status_2,status_3);
				}
			if(status_4==1)   
				{
if(Work_Flag==1)Light_pad_on(dis_com,hand_id,2,2,2);
if(Work_Flag==0)Light_pad_off(dis_com,hand_id,2,2,2);

			}
	   	}
		  
		  

}

                     delay_ms(200);//10

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
static  void  App_Taskcomputer	 (void		*p_arg )

{  
u8 i;
u8 err;
 static status_dis_node     dis_list[20];

static  u8 slave_dis[20];
static  u8 slave_comm[20];
static status_comm_node comm_list[78];
/*
 static status_dis_node     dis_list[20];

static  u8 slave_dis[10];
static  u8 slave_comm[10];
static status_comm_node comm_list[70];
*/
for(;;)
   	{
   //	OSSemPend(computer_sem,0,&err);
#if (FUNCTION_MODULE == DF_THREE)
if(first_init==1)
{
for(i=1;i<=32;i++)
{
comm_list[i].myid=i;
comm_list[i].work_status=0;
 Light_pad_on(dis_com,i,0,0,0);

}
rs485buf[1]=1;
rs485buf[2]=1;
first_init=0;
}
 //scanf_slave_machine(dis_list,comm_list,slave_dis,slave_comm);
  //init_Queue_dis(dis_list,slave_dis);
for(i=1;i<=32;i++)
{
comm_list[i].size=capa1_array[i-1];
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






if(COMMCAT_para==0) //自动模式
{

 computer_gonglu(dis_list,comm_list,slave_dis,slave_comm);

}
delay_ms(10);
#endif

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
void urgent_task(void *pdata)

{
u8 err;

          while(1)
          	{
       	OSSemPend(urgent_sem,0,&err);      	
//{order_trans_rs485(mybox.myid,0,1,1,0,CONTROL);order_trans_rs485(mybox.myid,0,1,2,0,CONTROL);}
	
 	


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
#if (FUNCTION_MODULE == DF_THREE)
//ADC2_CH8_DMA_Config_VEE();
//Init_ADC();
#endif

/********************485****************************************/	
/************************************************************/
//IWDG_Init(4,625); 


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
void RS485_Init(u32 bound)

{}

		void UART4_IRQHandler(void)
			{}

void RS485_Send_Data(u8 *buf,u8 len)
{}

int rs485_trans_order(u8 *tx_r485)//解析由主机发送过来的信号，并发送给下位机
{}

 void order_trans_rs485(u8 source,u8 destination, u8 send,u8 relay,u8 message,u8 ctr)//主机程序，主机命令解析成RS485信息，发送给目的从机
 	{}

 
 void computer_trans_rs485(u8 source,u8 destination, u8 send,u8 relay,u8 message,u8 ctr)//主机程序，主机计算出来数据解析成RS485信息，发送给目的从机

 	{}

 void heartbeat(u8 t)
{	/*u8 i;
for(i=0;i<=t;i++)
		{	
	       order_trans_rs485(mybox.myid,0,0,0,0);
		    delay_ms(1);
		}	
*/
}

 
void delay_time(u32 time)
{ heartbeat(time);
}    //本系统的延时函数，time*1ms




u16 comp_16(u16 a,u16 b)
{
u16 value=0;
value=((a&0x00FF)+((b<<8)&0xFF00));
return value;
}

void initmybox()//初始化自身信息
{  	 
  int i;
  mybox.master=0;
  mybox.myid=0;
 mybox.source=0;
 mybox.destination=0;
 mybox.send=0;
 mybox.relay=0;
 mybox.message=0;
for(i=1;i<=32;i++)
capa1_array[i-1] = AT24CXX_ReadOneByte(0x0010+(i-1)*2);  //存储DELAY_ON_para到eeprom
	

}

/*
void init_para(void)
{
 u8 DELAY_ON_para=10;
 u8 DELAY_OFF_para=10;
 u8 COS_ON_para=90;
 u8 COS_OFF_para=95;
 u8 V_PROT_para_L=40;
 u8 V_PROT_para_tri=40;
 u8 HU_PROT_para=100;
 u8 HI_PROT_para=100;

{



DELAY_ON_para=AT24CXX_ReadOneByte(0x1000);  //存储DELAY_ON_para到eeprom

		 DELAY_OFF_para=AT24CXX_ReadOneByte(0x2000);  //存储DELAY_OFF_para到eeprom

		 COS_ON_para=AT24CXX_ReadOneByte(0x3000);  //存储DELAY_OFF_para到eeprom
					 COS_OFF_para=AT24CXX_ReadOneByte(0x4000);  //存储DELAY_OFF_para到eeprom

		V_PROT_para_L=AT24CXX_ReadOneByte(0xb000);
		V_PROT_para_tri=AT24CXX_ReadOneByte(0xc000);

			HU_PROT_para=AT24CXX_ReadOneByte(0x7000); 
       HI_PROT_para=AT24CXX_ReadOneByte(0x8000); 
		
	  

 }	 

}
*/
void set_statuslist(u8 count,u8 id,u8 size,u8 work_status,u8 work_time,u8 dis_comm,u8 relay,status_dis_node *dis_list,status_comm_node *comm_list_1,status_comm_node *comm_list_2,u8 group)
{
if(dis_comm==0)
{
if(relay==1)
        {
       dis_list[count].myid[0]=id;
   	   dis_list[count].size[0]=size;
   	   dis_list[count].work_status[0]=work_status;
	     ///    Light_pad_onoff(1,id,work_status,3,3);

       }
if(relay==2)
        {
	dis_list[count].myid[1]=id;
   	   dis_list[count].size[1]=size;
   	   dis_list[count].work_status[1]=work_status;
	  ///       Light_pad_onoff(1,id,3,work_status,3);

       }
if(relay==3)
        {
       dis_list[count].myid[2]=id;
   	   dis_list[count].size[2]=size;
   	   dis_list[count].work_status[2]=work_status;
	  ///       Light_pad_onoff(1,id,3,3,work_status);

       }
}
if(dis_comm==1)
{
  if(relay==1)
  	{
	   comm_list_1[count].myid=id;
   	   comm_list_1[count].size=size;
   	   comm_list_1[count].work_status=work_status;
	   comm_list_1[count].group=group;
      // comm_list[count].work_time[0]=work_time;
  ///    Light_pad_onoff(2,id,work_status,3,3);
  	}
  if(relay==2)
  	{
	   comm_list_2[count].myid=id;
   	   comm_list_2[count].size=size;
   	   comm_list_2[count].work_status=work_status;
	   comm_list_2[count].group=group;
      // comm_list[count].work_time[1]=work_time;
    ///        Light_pad_onoff(2,id,3,work_status,3);

  	}  
}

}
/****************************************************************************************************/
void init_Queue(u8 id,u8 size_1,u8 size_2,u8 work_status_1,u8 work_status_2,u8 *slave_comm,status_comm_node *comm_list)
{
u8 i=0;
{
  if(size_1==2)
  	{
  	for(i=slave_comm[9];i>slave_comm[8];i--)//20的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
	slave_comm[9]++;

	  	for(i=slave_comm[7];i>slave_comm[6];i--)//10的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
		slave_comm[7]++;
			
	  	for(i=slave_comm[5];i>slave_comm[4];i--)//5的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}	
slave_comm[5]++;
		
	   comm_list[slave_comm[3]].myid=id;
   	   comm_list[slave_comm[3]].size=size_1;
   	   comm_list[slave_comm[3]].work_status=work_status_1;
	   comm_list[slave_comm[3]].group=1;

		 slave_comm[3]++;
		slave_comm[4]=slave_comm[3]+1;
		slave_comm[6]=slave_comm[5]+1;
		slave_comm[8]=slave_comm[7]+1;

		 slave_comm[0]++;

  }
  
    if(size_1==5)
  	{
  		for(i=slave_comm[9];i>slave_comm[8];i--)//20的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
	slave_comm[9]++;

	  	for(i=slave_comm[7];i>slave_comm[6];i--)//10的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
		slave_comm[7]++;
		
	   comm_list[slave_comm[5]].myid=id;
   	   comm_list[slave_comm[5]].size=size_1;
   	   comm_list[slave_comm[5]].work_status=work_status_1;
	   comm_list[slave_comm[5]].group=1;

	    slave_comm[5]++;
		slave_comm[6]=slave_comm[5]+1;
		slave_comm[8]=slave_comm[7]+1;
		 slave_comm[0]++;

  	} 
	if(size_1==10)
  	{
  		for(i=slave_comm[9];i>slave_comm[8];i--)//20的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
	slave_comm[9]++;
	
	   comm_list[slave_comm[7]].myid=id;
   	   comm_list[slave_comm[7]].size=size_1;
   	   comm_list[slave_comm[7]].work_status=work_status_1;
	   comm_list[slave_comm[7]].group=1;

	       slave_comm[7]++;
		slave_comm[8]=slave_comm[7]+1;
		 slave_comm[0]++;

  	} 
	if(size_1==20)
  	{
	   comm_list[slave_comm[9]].myid=id;
   	   comm_list[slave_comm[9]].size=size_1;
   	   comm_list[slave_comm[9]].work_status=work_status_1;
	   comm_list[slave_comm[9]].group=1;
	   	    slave_comm[9]++;
		 slave_comm[0]++;

  	}
}



{
  if(size_2==2)
  	{
  	for(i=slave_comm[9];i>slave_comm[8];i--)//20的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
	slave_comm[9]++;

	  	for(i=slave_comm[7];i>slave_comm[6];i--)//10的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
		slave_comm[7]++;
			
	  	for(i=slave_comm[5];i>slave_comm[4];i--)//5的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}	
slave_comm[5]++;
		
	   comm_list[slave_comm[3]].myid=id;
   	   comm_list[slave_comm[3]].size=size_2;
   	   comm_list[slave_comm[3]].work_status=work_status_2;
	   comm_list[slave_comm[3]].group=2;

		 slave_comm[3]++;
		slave_comm[4]=slave_comm[3]+1;
		slave_comm[6]=slave_comm[5]+1;
		slave_comm[8]=slave_comm[7]+1;

		 slave_comm[0]++;

  }
  
    if(size_2==5)
  	{
  		for(i=slave_comm[9];i>slave_comm[8];i--)//20的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
	slave_comm[9]++;

	  	for(i=slave_comm[7];i>slave_comm[6];i--)//10的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
		slave_comm[7]++;
		
	   comm_list[slave_comm[5]].myid=id;
   	   comm_list[slave_comm[5]].size=size_2;
   	   comm_list[slave_comm[5]].work_status=work_status_2;
	   comm_list[slave_comm[5]].group=2;

	    slave_comm[5]++;
		slave_comm[6]=slave_comm[5]+1;
		slave_comm[8]=slave_comm[7]+1;
		 slave_comm[0]++;

  	} 
	if(size_2==10)
  	{
  		for(i=slave_comm[9];i>slave_comm[8];i--)//20的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
	slave_comm[9]++;
	
	   comm_list[slave_comm[7]].myid=id;
   	   comm_list[slave_comm[7]].size=size_2;
   	   comm_list[slave_comm[7]].work_status=work_status_2;
	   comm_list[slave_comm[7]].group=2;

	       slave_comm[7]++;
		slave_comm[8]=slave_comm[7]+1;
		 slave_comm[0]++;

  	} 
	if(size_2==20)
  	{
	   comm_list[slave_comm[9]].myid=id;
   	   comm_list[slave_comm[9]].size=size_2;
   	   comm_list[slave_comm[9]].work_status=work_status_2;
	   comm_list[slave_comm[9]].group=2;
	   	    slave_comm[9]++;
		 slave_comm[0]++;

  	}
}

}

void init_listindex(u8 *slave_comm)
{
slave_comm[2]=0;//队列2v 标示初始化
slave_comm[3]=0;

slave_comm[4]=1;//队列5v 标示初始化
slave_comm[5]=1;

slave_comm[6]=2;//队列10v 标示初始化
slave_comm[7]=2;

slave_comm[8]=3;//队列20v 标示初始化
slave_comm[9]=3;

}

void change_Queue(u8 *slave_comm,status_comm_node *comm_list,u8 size)
{

u8 i=0;
u8 m,s,w,g;
{
  if(size==2)
  	{
  	m=comm_list[slave_comm[2]].myid;
	s=comm_list[slave_comm[2]].size;
	w=comm_list[slave_comm[2]].work_status;
	g=comm_list[slave_comm[2]].group;

	for(i=slave_comm[2];i<slave_comm[3]-1;i++)// 2 的队列移动
  		{
	   comm_list[i].myid=comm_list[i+1].myid;
   	   comm_list[i].size=comm_list[i+1].size;
   	   comm_list[i].work_status=comm_list[i+1].work_status;
	   comm_list[i].group=comm_list[i+1].group;
  		}

		
	   comm_list[slave_comm[3]-1].myid=m;
   	   comm_list[slave_comm[3]-1].size=s;
   	   comm_list[slave_comm[3]-1].work_status=w;
	   comm_list[slave_comm[3]-1].group=g;

  	}
  
    if(size==5)
    	{
  	m=comm_list[slave_comm[4]].myid;
	s=comm_list[slave_comm[4]].size;
	w=comm_list[slave_comm[4]].work_status;
	g=comm_list[slave_comm[4]].group;

	for(i=slave_comm[4];i<slave_comm[5]-1;i++)//20的队列移动
  		{
	   comm_list[i].myid=comm_list[i+1].myid;
   	   comm_list[i].size=comm_list[i+1].size;
   	   comm_list[i].work_status=comm_list[i+1].work_status;
	   comm_list[i].group=comm_list[i+1].group;
  		}

		
	   comm_list[slave_comm[5]-1].myid=m;
   	   comm_list[slave_comm[5]-1].size=s;
   	   comm_list[slave_comm[5]-1].work_status=w;
	   comm_list[slave_comm[5]-1].group=g;

  	}
	if(size==10)
  	{
  	m=comm_list[slave_comm[6]].myid;
	s=comm_list[slave_comm[6]].size;
	w=comm_list[slave_comm[6]].work_status;
	g=comm_list[slave_comm[6]].group;

	for(i=slave_comm[6];i<slave_comm[7]-1;i++)//20的队列移动
  		{
	   comm_list[i].myid=comm_list[i+1].myid;
   	   comm_list[i].size=comm_list[i+1].size;
   	   comm_list[i].work_status=comm_list[i+1].work_status;
	   comm_list[i].group=comm_list[i+1].group;
  		}

		
	   comm_list[slave_comm[7]-1].myid=m;
   	   comm_list[slave_comm[7]-1].size=s;
   	   comm_list[slave_comm[7]-1].work_status=w;
	   comm_list[slave_comm[7]-1].group=g;

  	} 
	if(size==20)
  	{
  	m=comm_list[slave_comm[8]].myid;
	s=comm_list[slave_comm[8]].size;
	w=comm_list[slave_comm[8]].work_status;
	g=comm_list[slave_comm[8]].group;

	for(i=slave_comm[8];i<slave_comm[9]-1;i++)//20的队列移动
  		{
	   comm_list[i].myid=comm_list[i+1].myid;
   	   comm_list[i].size=comm_list[i+1].size;
   	   comm_list[i].work_status=comm_list[i+1].work_status;
	   comm_list[i].group=comm_list[i+1].group;
  		}

		
	   comm_list[slave_comm[9]-1].myid=m;
   	   comm_list[slave_comm[9]-1].size=s;
   	   comm_list[slave_comm[9]-1].work_status=w;
	   comm_list[slave_comm[9]-1].group=g;

  	}
}
}
/***********************************************
void flash_comm_list(u8 id,u8 size ,u8 work_status ,u8 group,u8 *slave_comm,status_comm_node *comm_list)
节点状态更新函数


*********************************************/
void flash_comm_list(u8 id,u8 size ,u8 work_status ,u8 group,u8 *slave_comm,status_comm_node *comm_list)
{
u8 i;
if(size==2)
{
	for(i=slave_comm[2];i<slave_comm[3];i++)// 2 
	{
	if(id==comm_list[i].myid&&group==comm_list[i].group)//锁定节点进行更新
		{
	   comm_list[i].myid=id;
   	   comm_list[i].size=size;
   	   comm_list[i].work_status=work_status;
	   comm_list[i].group=group;
	   break;
		}
	}
}

if(size==5)
{
	for(i=slave_comm[4];i<slave_comm[5];i++)// 2 
	{
	if(id==comm_list[i].myid&&group==comm_list[i].group)//锁定节点进行更新
		{
	   comm_list[i].myid=id;
   	   comm_list[i].size=size;
   	   comm_list[i].work_status=work_status;
	   comm_list[i].group=group;
	   break;
		}
	}
}

if(size==10)
{
	for(i=slave_comm[6];i<slave_comm[7];i++)// 2 
	{
	if(id==comm_list[i].myid&&group==comm_list[i].group)//锁定节点进行更新
		{
	   comm_list[i].myid=id;
   	   comm_list[i].size=size;
   	   comm_list[i].work_status=work_status;
	   comm_list[i].group=group;
	   break;
		}
	}
}

if(size==20)
{
	for(i=slave_comm[8];i<slave_comm[9];i++)// 2 
	{
	if(id==comm_list[i].myid&&group==comm_list[i].group)//锁定节点进行更新
		{
	   comm_list[i].myid=id;
   	   comm_list[i].size=size;
   	   comm_list[i].work_status=work_status;
	   comm_list[i].group=group;
	   break;
		}
	}
}
}

void del_comm_listnode(u8 id,u8 group,u8 *slave_comm,status_comm_node *comm_list)
{
u8 i=0;
u8 j=0;
  	
{
  	  		for(i=slave_comm[2];i<=slave_comm[9]-1;i++)//2
	if(id==comm_list[i].myid&&group==comm_list[i].group)//锁定节点进行更新
		{ 

	
		for(j=i;j<slave_comm[9]-1;j++)
		      {
		         comm_list[j].myid=comm_list[j+1].myid;
   	   comm_list[j].size=comm_list[j+1].size;
   	   comm_list[j].work_status=comm_list[j+1].work_status;
	   comm_list[j].group=comm_list[j+1].group;
		      }
	       if(i<slave_comm[3])
		   	{
                         {
			slave_comm[3]--;
	             comm_list[slave_comm[3]].myid=0;
   	               comm_list[slave_comm[3]].size=0;
   	                comm_list[slave_comm[3]].work_status=0;
	              comm_list[slave_comm[3]].group=0;

		          }	
		   
                      {
			slave_comm[4]--;					  	
			slave_comm[5]--;
	             comm_list[slave_comm[5]].myid=0;
   	               comm_list[slave_comm[5]].size=0;
   	                comm_list[slave_comm[5]].work_status=0;
	              comm_list[slave_comm[5]].group=0;

		          }	
				  
                      {
			slave_comm[6]--;					  	
			slave_comm[7]--;
	             comm_list[slave_comm[7]].myid=0;
   	               comm_list[slave_comm[7]].size=0;
   	                comm_list[slave_comm[7]].work_status=0;
	              comm_list[slave_comm[7]].group=0;

		          }			
				  
	             {
			slave_comm[8]--;				 	
			slave_comm[9]--;
	             comm_list[slave_comm[9]].myid=0;
   	               comm_list[slave_comm[9]].size=0;
   	                comm_list[slave_comm[9]].work_status=0;
	              comm_list[slave_comm[9]].group=0;

		          }
		   	}
		else if(i<slave_comm[5]&&i>slave_comm[3])
			{
                         	   
                      {
			slave_comm[5]--;
	             comm_list[slave_comm[5]].myid=0;
   	               comm_list[slave_comm[5]].size=0;
   	                comm_list[slave_comm[5]].work_status=0;
	              comm_list[slave_comm[5]].group=0;

		          }	
				  
                      {
			slave_comm[6]--;					  	
			slave_comm[7]--;
	             comm_list[slave_comm[7]].myid=0;
   	               comm_list[slave_comm[7]].size=0;
   	                comm_list[slave_comm[7]].work_status=0;
	              comm_list[slave_comm[7]].group=0;

		          }			
				  
	             {
			slave_comm[8]--;				 	
			slave_comm[9]--;
	             comm_list[slave_comm[9]].myid=0;
   	               comm_list[slave_comm[9]].size=0;
   	                comm_list[slave_comm[9]].work_status=0;
	              comm_list[slave_comm[9]].group=0;

		          }
		   	}
		else if(i<slave_comm[7]&&i>slave_comm[5])
			{
                         		  
                      {
			slave_comm[7]--;
	             comm_list[slave_comm[7]].myid=0;
   	               comm_list[slave_comm[7]].size=0;
   	                comm_list[slave_comm[7]].work_status=0;
	              comm_list[slave_comm[7]].group=0;

		          }			
				  
	             {
			slave_comm[8]--;				 	
			slave_comm[9]--;
	             comm_list[slave_comm[9]].myid=0;
   	               comm_list[slave_comm[9]].size=0;
   	                comm_list[slave_comm[9]].work_status=0;
	              comm_list[slave_comm[9]].group=0;

		          }
		   	}
		else if(i<slave_comm[9]&&i>slave_comm[7])
			{
							  
	             {
			slave_comm[9]--;
	             comm_list[slave_comm[9]].myid=0;
   	               comm_list[slave_comm[9]].size=0;
   	                comm_list[slave_comm[9]].work_status=0;
	              comm_list[slave_comm[9]].group=0;

		          }
		   	}

break;
	}



		 
  	 
  	
}

}

/**********************/
u8 inquiry_slave_status_comm(u8 id,u8 *slave_comm,status_comm_node *comm_list)   
  {  u8 *msg;
        u8 err;
	

   order_trans_rs485(mybox.myid,id,3,0,0,CONTROL);
   msg=(u8 *)OSMboxPend(RS485_STUTAS_MBOX,OS_TICKS_PER_SEC/10,&err);
   if(err==OS_ERR_TIMEOUT)
   	{ return 0;}//(u8 id, u8 size, u8 work_status, u8 work_time) 
	else 
	{ 
if(msg[2]==id)
		{
 init_Queue(id,msg[3],msg[4],msg[5],msg[6],slave_comm,comm_list);
		
	return 1;
		}
else return 0;
	}

} //查询从机状态并保存到从机状态表中，参数id是要查询的从机号


/**********************************/
 u8 inquiry_slave_status_dis(u8 count,u8 id,status_dis_node *dis_list,status_comm_node *comm_list)   
  {  u8 *msg;
        u8 err;
/*		
			if(id==mybox.myid)
		{
set_statuslist(id,status_box.size[0],status_box.work_status[0],status_box.work_time[0],0,1,dis_list,comm_list);
set_statuslist(id,status_box.size[1],status_box.work_status[1],status_box.work_time[1],0,2,dis_list,comm_list);
set_statuslist(id,status_box.size[2],status_box.work_status[2],status_box.work_time[2],0,3,dis_list,comm_list);

return 1;
		}
	*/		
{
 computer_trans_rs485(mybox.myid,id,2,0,0,CONTROL);
  // order_trans_rs485(mybox.myid,id,2,0,0);

   msg=(u8 *)OSMboxPend(RS485_STUTAS_MBOX_dis,OS_TICKS_PER_SEC/10,&err);
   if(err==OS_ERR_TIMEOUT)
   	{
          return 0;
   }//(u8 id, u8 size, u8 work_status, u8 work_time) 
	else 
	{ 
	if(msg[2]==id)//检查传过来的从机的状态信息是否真是该从机的。如果不是就不录入
		{
	rs485_trans_status_dis(count,msg,dis_list,comm_list);//主机状态信息写入状态表
	return 1;
		}
	else return 0;
	}

}
} //查询从机状态并保存到从机状态表中，参数id是要查询的从机号
/**********************/

/**************/
 void rs485_trans_status_dis(u8 count,u8 *tx_r485,status_dis_node *dis_list,status_comm_node *comm_list)//主机程序，主机命令解析成RS485信息，发送给目的从机
 	{
 	 set_statuslist(count,tx_r485[2],tx_r485[3],tx_r485[6],0,0,1,dis_list,comm_list,0,0);//主机状态信息写入状态表
	 set_statuslist(count,tx_r485[2],tx_r485[4],tx_r485[7],0,0,2,dis_list,comm_list,0,0);//主机状态信息写入状态表
      	  set_statuslist(count,tx_r485[2],tx_r485[5],tx_r485[8],0,0,3,dis_list,comm_list,0,0);//主机状态信息写入状态表

   } 
 	


/*********************************/
u8 computer_gonglu(status_dis_node *dis_list,status_comm_node *comm_list,u8 *slave_dis,u8 *slave_comm)
{
int i=0;
arm_status status; 
arm_rfft_instance_f32 S;
arm_cfft_radix4_instance_f32  S_CFFT;
float32_t maxValue=0.0,maxValue_C=0.0; 
 float32_t testInput_V[TEST_LENGTH_SAMPLES]; 
 float32_t testInput_C[TEST_LENGTH_SAMPLES]; 

float32_t testOutput[TEST_LENGTH_SAMPLES*2/2]; 
float32_t reslut[TEST_LENGTH_SAMPLES/2]; 
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
dianya_zhi=maxValue/100;
dianya_zhi=dianya_zhi/2.57;
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

dianliuzhi=T*maxValue_C*cruccent_ratio;
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
   order_trans_rs485(mybox.myid,0,0,0,0,CPT_LL);

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
dianya_zhi_A=maxValue/100;
dianya_zhi_A=dianya_zhi_A/2.57;
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
 dianliuzhi_A=cruccent_ratio*maxValue_C*T;
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
dianya_zhi_B=maxValue/100;
dianya_zhi_B=dianya_zhi_B/2.57;
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
dianliuzhi_B=cruccent_ratio*maxValue_C*T;
 if(dianliuzhi_B<=zero_limit*T){dianliuzhi_B=0;gonglvshishu_B=100;L_C_flag_B=1;}
else {
        dianliuzhi_B=dianliuzhi_B/1000;
	gonglvshishu_B=arm_cos_f32(angle[2])*100;//功率因素
}
/*************************电流谐波率****************************************/
if((dianliuzhi_B==0)&&(gonglvshishu_B==100))B_HI=0;
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
dianya_zhi_C=maxValue/100;
dianya_zhi_C=dianya_zhi_C/2.57;
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
dianliuzhi_C=cruccent_ratio*maxValue_C*T;
 if(dianliuzhi_C<=zero_limit*T){dianliuzhi_C=0;gonglvshishu_C=100;L_C_flag_C=1;}
else
	{
	dianliuzhi_C=dianliuzhi_C/1000;
	gonglvshishu_C=arm_cos_f32(angle[2])*100;//功率因素
}
/*************************电流谐波率****************************************/
if((dianliuzhi_C==0)&&(gonglvshishu_C==100))C_HI=0;

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

   order_trans_rs485(mybox.myid,0,0,0,0,CPT_LL);


}

}


/*********************变比判断*******************************/



if(warning_flag==0&&(A_HV<HU_PROT_para&&B_HV<HU_PROT_para&&C_HV<HI_PROT_para)&&(A_HI<HI_PROT_para&&B_HI<HI_PROT_para&&C_HI<HI_PROT_para)&&(HI<HI_PROT_para)&&(HV<HU_PROT_para)&&((V_PROT_para_L+200)>dianya_zhi_A)&&((V_PROT_para_L+200)>dianya_zhi_B)&&((V_PROT_para_L+200)>dianya_zhi_C)&&((V_PROT_para_tri+400)>dianya_zhi))
{
{



if(gonglvshishu<COS_ON_para&&L_C_flag_B==1)
 {
      {
	  	
      	{
for(i=1;i<=32;i++)
if(comm_list[i].work_status==0&&(wugongkvar>=comm_list[i].size))
{
display_nothing_close_open_warn=1;//设置显示投入
{
set_74hc273(comm_list[i].myid, ON);
 Light_pad_on(dis_com,comm_list[i].myid,1,1,0);
comm_list[i].work_status=1;
		rs485buf[1]++;
}
if(rs485buf[1]==32)rs485buf[1]=1;
return 0 ;
}


      	}


  	
  
   
}
 }
}

if((gonglvshishu>COS_OFF_para&&L_C_flag_B==1)||(L_C_flag_B==0))
   
{
      {

{
for(i=1;i<=32;i++)
if(comm_list[i].work_status==1)

{
display_nothing_close_open_warn=2;//设置显示切除
{
set_74hc273(comm_list[i].myid, OFF);
 Light_pad_on(dis_com,comm_list[i].myid,0,0,0);
comm_list[i].work_status=0;

		rs485buf[2]++;
}
		{
if(rs485buf[2]==32)rs485buf[2]=1;

return 0 ;

		}
}


}
	  
       }
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
for(i=1;i<=32;i++)
if(comm_list[i].work_status==1)

{
display_nothing_close_open_warn=2;//设置显示切除
{
set_74hc273(comm_list[i].myid, OFF);
comm_list[i].work_status=0;
		rs485buf[2]++;
}
		{
if(rs485buf[2]==32)rs485buf[2]=1;

return 0 ;

		}
}


}
	  
       }		
}

}



return 0;

}


void scanf_slave_machine(status_dis_node *dis_list,status_comm_node *comm_list,u8 *slave_dis,u8 *slave_comm)
{
u8 i,j=0,g,flag_comm=0,flag_dis=0,s;
u8 g_1,g_2,g_3,f_1,f_2,f_3;
//u8 c;
u8 *msg;
  u8 err;
  static u8 dis_err[8];
    static u8 comm_err[32];
u8 count=0;

for(i=1;i<=8;i++)
{  

for(g=1;g<=slave_dis[0];g++)
{
if(i==dis_list[g].myid[0]||i==dis_list[g].myid[1]||i==dis_list[g].myid[2]){flag_dis=1;break;}
else flag_dis=0;
}
if(flag_dis==0)
		{
//	for(c=1;c<=2;c++)
		{
	j=inquiry_slave_status_dis(slave_dis[0]+1,i,dis_list,comm_list); 
	        if(j==1){ slave_dis[0]++;break;}
		}
			}
if(flag_dis==1)
{
 computer_trans_rs485(mybox.myid,i,2,0,0,CONTROL);
   msg=(u8 *)OSMboxPend(RS485_STUTAS_MBOX_dis,OS_TICKS_PER_SEC/10,&err);
     if(err==OS_ERR_TIMEOUT)
	 	{
	 	dis_err[i-1]++;
	 	if(dis_err[i-1]==3)//三次确认，如果三次都没有收到数据就认为是从机死了
	 	{
	 	capa1_array[i-1]=0;capa2_array[i-1]=0;//屏幕显示容量使用
	 	Light_pad_off(0,i,0,0,0);
		set_bit(i, 0, &light_status, 0,0, 0,2);
		dis_err[i-1]=0;
set_clear_existence(0,i,&hand_light_existence);

for(g_1=1;g_1<=slave_dis[0];g_1++)
{
if(i==dis_list[g_1].myid[0])
	{
	dis_list[g_1].work_status[0]=2;dis_list[g_1].myid[0]=0;
         break;
       }

}
for(g_2=1;g_2<=slave_dis[0];g_2++)
{
if(i==dis_list[g_2].myid[1])
	{
	dis_list[g_2].work_status[1]=2;dis_list[g_2].myid[1]=0;
         break;
       }

}


for(g_3=1;g_3<=slave_dis[0];g_3++)
{
if(i==dis_list[g_3].myid[2])
	{
	dis_list[g_3].work_status[2]=2;dis_list[g_3].myid[2]=0;
         break;
       }

}
for(f_1=g_1;f_1<slave_dis[0];f_1++)
{
dis_list[f_1].myid[0]=dis_list[f_1+1].myid[0];
dis_list[f_1].work_status[0]=dis_list[f_1+1].work_status[0];
dis_list[f_1].size[0]=dis_list[f_1+1].size[0];

}

for(f_2=g_2;f_2<slave_dis[0];f_2++)
{
dis_list[f_2].myid[1]=dis_list[f_2+1].myid[1];
dis_list[f_2].work_status[1]=dis_list[f_2+1].work_status[1];
dis_list[f_2].size[1]=dis_list[f_2+1].size[1];

}

for(f_3=g_3;f_3<slave_dis[0];f_3++)
{
dis_list[f_3].myid[2]=dis_list[f_3+1].myid[2];
dis_list[f_3].work_status[2]=dis_list[f_3+1].work_status[2];
dis_list[f_3].size[2]=dis_list[f_3+1].size[2];

}
slave_dis[0]--;

}
	     }
else if(msg[2]==i)//检查传过来的从机的状态信息是否真是该从机的。如果不是就不更新
{
if(msg[3]==1)capa1_array[msg[2]-1]=5;
if(msg[3]==3)capa1_array[msg[2]-1]=10;
if(msg[3]==6)capa1_array[msg[2]-1]=20;
       capa2_array[msg[2]-1]=0;
dis_err[i-1]=0;
		for(s=1;s<=slave_dis[0];s++)
			{
                   if(i==dis_list[s].myid[0])dis_list[s].work_status[0]=msg[6];
                   if(i==dis_list[s].myid[1])dis_list[s].work_status[1]=msg[7];
                   if(i==dis_list[s].myid[2])dis_list[s].work_status[2]=msg[8];
			}
Light_pad_on(0,i,msg[6],msg[7],msg[8]);
set_clear_existence(1,i,&hand_light_existence);
set_bit(i, 0,&light_status,msg[6],msg[7], msg[8],0);
}

}

	flag_dis=0;
       j=0;
    }

//delay_ms(1000);

j=0;
{
for(i=slave_dis[0]+1;i<=32;i++)
	{  

for(g=0;g<=slave_comm[9]-1;g++)
{
if(i==comm_list[g].myid){flag_comm=1;break;}

else flag_comm=0;
}
if(flag_comm==0)
		{		
 inquiry_slave_status_comm(i,slave_comm,comm_list);   
			
			}
//以下是为了更新从机的投切状态
if(flag_comm==1)

{
{order_trans_rs485(mybox.myid,i,3,0,0,CONTROL); 
  msg=(u8 *)OSMboxPend(RS485_STUTAS_MBOX,OS_TICKS_PER_SEC/10,&err);
     if(err==OS_ERR_TIMEOUT)
	 	{
	  	comm_err[i-1]++; 
if(comm_err[i-1]==3)
	  {
	  Light_pad_off(1,i,0,0,0);//指示灯使用
	  	 set_bit(i, 1, &light_status, 0,0, 0,2);//手动投切使用
	  	comm_err[i-1]=0; 
		set_clear_existence(0,i,&hand_light_existence);
			 	capa1_array[i-1]=0;capa2_array[i-1]=0;//屏幕显示容量使用

/*******************从已知队列中删除该节点***********************************/


 del_comm_listnode(i,1,slave_comm,comm_list);
 del_comm_listnode(i,2,slave_comm,comm_list);
slave_comm[0]=slave_comm[0]-2;
 /******************从已知队列中删除该节点end***********************************/

}


	 }
else  if(msg[2]==i)
	{
		  	comm_err[i-1]=0; 
	capa1_array[msg[2]-1]=msg[3];
       capa2_array[msg[2]-1]=msg[4];
	 Light_pad_on(1,i,msg[5],msg[6],0);
	 set_bit(i, 1, &light_status, msg[5],msg[6], 0,0);//手动投切使用
	 set_clear_existence(1,i,&hand_light_existence);

	if(flag_comm==1)
		{
		 flash_comm_list(i,msg[3] ,msg[5] ,1,slave_comm,comm_list);
		 flash_comm_list(i,msg[4] ,msg[6] ,2,slave_comm,comm_list);
		}
       }

}
	


}
	flag_comm=0;
       j=0;
    }

}

}

/**************************************************************/

/******************************************************************/




	


void init_Queue_dis(status_dis_node *dis_list,u8 *slave_dis)

{

u8 i,j;

u8 t=0;
u8 g=0;
u8 s=0;
{
for(i=2;i<=slave_dis[0];i++)
{
  
          t=dis_list[i].size[0];
	   g=dis_list[i].myid[0];
	   s=	dis_list[i].work_status[0];
	   for(j=i-1;j>=1;j--)
	   	{
	   	if(t<dis_list[j].size[0])
	   		{
	   	dis_list[j+1].myid[0]=dis_list[j].myid[0];
               dis_list[j+1].size[0]=dis_list[j].size[0];
		 	dis_list[j+1].work_status[0]=dis_list[j].work_status[0];

	   		}
		else break;
		}
	   dis_list[j+1].myid[0]=g;
	   dis_list[j+1].size[0]=t;
            dis_list[j+1].work_status[0]=s;
}
for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[0]==1)
{
slave_dis[1]=i;
break;
}
if(i>slave_dis[0]){slave_dis[1]=0;slave_dis[7]=0;}

if(slave_dis[1]!=0)
{
for(i=slave_dis[1];i<=slave_dis[0];i++)
if(dis_list[i].size[0]!=1)
{
slave_dis[7]=i;
break;
}
if(i>slave_dis[0]){slave_dis[7]=slave_dis[0]+1;}

}


for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[0]==3)
{
slave_dis[2]=i;
break;
}
if(i>slave_dis[0]){slave_dis[2]=0;slave_dis[8]=0;}

if(slave_dis[2]!=0)
{
for(i=slave_dis[2];i<=slave_dis[0];i++)
if(dis_list[i].size[0]!=3)
{
slave_dis[8]=i;
break;
}
if(i>slave_dis[0]){slave_dis[8]=slave_dis[0]+1;}

}



for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[0]==6)
{
slave_dis[3]=i;
break;
}
if(i>slave_dis[0]){slave_dis[3]=0;slave_dis[9]=0;}
if(slave_dis[3]!=0)
{
for(i=slave_dis[3];i<=slave_dis[0];i++)
if(dis_list[i].size[0]!=6)
{
slave_dis[9]=i;
break;
}
if(i>slave_dis[0]){slave_dis[9]=slave_dis[0]+1;}

}




}


/***************************************************/
{
for(i=2;i<=slave_dis[0];i++)
{
  
          t=dis_list[i].size[1];
	   g=dis_list[i].myid[1];//设置myid两个
	   s=	dis_list[i].work_status[1];

	   for(j=i-1;j>=1;j--)
	   	{
	   	if(t<dis_list[j].size[1])
	   		{
		dis_list[j+1].myid[1]=dis_list[j].myid[1];
               dis_list[j+1].size[1]=dis_list[j].size[1];
		 dis_list[j+1].work_status[1]=dis_list[j].work_status[1];

	   		}
		else break;
	       }
	   dis_list[j+1].myid[1]=g;
	   dis_list[j+1].size[1]=t;
            dis_list[j+1].work_status[1]=s;


}


for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[1]==1)
{
slave_dis[4]=i;
break;
}
if(i>slave_dis[0]){slave_dis[4]=0;slave_dis[10]=0;}

if(slave_dis[4]!=0)
{
for(i=slave_dis[4];i<=slave_dis[0];i++)
if(dis_list[i].size[1]!=1)
{
slave_dis[10]=i;
break;
}
if(i>slave_dis[0]){slave_dis[10]=slave_dis[0]+1;}

}



for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[1]==3)
{
slave_dis[5]=i;
break;
}
if(i>slave_dis[0]){slave_dis[5]=0;slave_dis[11]=0;}

if(slave_dis[5]!=0)
{
for(i=slave_dis[5];i<=slave_dis[0];i++)
if(dis_list[i].size[1]!=3)
{
slave_dis[11]=i;
break;
}
if(i>slave_dis[0]){slave_dis[11]=slave_dis[0]+1;}

}




for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[1]==6)
{
slave_dis[6]=i;
break;
}
//slave_comm[12]=slave_comm[0]+1;
if(i>slave_dis[0]){slave_dis[6]=0;slave_dis[12]=0;}
if(slave_dis[6]!=0)
{
for(i=slave_dis[6];i<=slave_dis[0];i++)
if(dis_list[i].size[1]!=6)
{
slave_dis[12]=i;
break;
}
if(i>slave_dis[0]){slave_dis[12]=slave_dis[0]+1;}

}
}
/***********************************/
{
for(i=2;i<=slave_dis[0];i++)
{
  
          t=dis_list[i].size[2];
	   g=dis_list[i].myid[2];//设置myid两个
	   s=	dis_list[i].work_status[2];

	   for(j=i-1;j>=1;j--)
	   	{
	   	if(t<dis_list[j].size[2])
	   		{
		dis_list[j+1].myid[2]=dis_list[j].myid[2];
               dis_list[j+1].size[2]=dis_list[j].size[2];
		 dis_list[j+1].work_status[2]=dis_list[j].work_status[2];

	   		}
		else break;
	       }
	   dis_list[j+1].myid[2]=g;
	   dis_list[j+1].size[2]=t;
            dis_list[j+1].work_status[2]=s;


}


for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[2]==1)
{
slave_dis[13]=i;
break;
}
if(i>slave_dis[0]){slave_dis[13]=0;slave_dis[14]=0;}

if(slave_dis[13]!=0)
{
for(i=slave_dis[13];i<=slave_dis[0];i++)
if(dis_list[i].size[2]!=1)
{
slave_dis[14]=i;
break;
}
if(i>slave_dis[0]){slave_dis[14]=slave_dis[0]+1;}

}



for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[2]==3)
{
slave_dis[15]=i;
break;
}
if(i>slave_dis[0]){slave_dis[15]=0;slave_dis[16]=0;}

if(slave_dis[15]!=0)
{
for(i=slave_dis[15];i<=slave_dis[0];i++)
if(dis_list[i].size[2]!=3)
{
slave_dis[16]=i;
break;
}
if(i>slave_dis[0]){slave_dis[16]=slave_dis[0]+1;}

}




for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[2]==6)
{
slave_dis[17]=i;
break;
}
//slave_comm[12]=slave_comm[0]+1;
if(i>slave_dis[0]){slave_dis[17]=0;slave_dis[18]=0;}
if(slave_dis[17]!=0)
{
for(i=slave_dis[17];i<=slave_dis[0];i++)
if(dis_list[i].size[2]!=6)
{
slave_dis[18]=i;
break;
}
if(i>slave_dis[0]){slave_dis[18]=slave_dis[0]+1;}

}
}




/********************************/
}




/*********************************************************************/
void change_Queue_dis(u8 abc,u8 Level, status_dis_node *dis_list,u8 *slave_dis)
{
u8 i;
u8 t=0, g=0,s=0;

{
if(Level==1)
	
{
if(abc==0)
{
if(slave_dis[1]!=0)

{
          t=dis_list[slave_dis[1]].size[abc];
	   g=dis_list[slave_dis[1]].myid[abc];
	   s=	dis_list[slave_dis[1]].work_status[abc];
for(i=slave_dis[1];i<slave_dis[7]-1;i++)
  {
          dis_list[i].size[abc]=dis_list[i+1].size[abc];
	   dis_list[i].myid[abc]=dis_list[i+1].myid[abc];
	   dis_list[i].work_status[abc]=dis_list[i+1].work_status[abc];

  }
        dis_list[slave_dis[7]-1].size[abc]=t;
	  dis_list[slave_dis[7]-1].myid[abc]=g;
	  dis_list[slave_dis[7]-1].work_status[abc]=s;
}
}
/***********************************************************************************/
if(abc==1)

{
if(slave_dis[4]!=0)

{
          t=dis_list[slave_dis[4]].size[abc];
	   g=dis_list[slave_dis[4]].myid[abc];
	   s=	dis_list[slave_dis[4]].work_status[abc];
for(i=slave_dis[4];i<slave_dis[10]-1;i++)
  {
          dis_list[i].size[abc]=dis_list[i+1].size[abc];
	   dis_list[i].myid[abc]=dis_list[i+1].myid[abc];
	   dis_list[i].work_status[abc]=dis_list[i+1].work_status[abc];

  }
        dis_list[slave_dis[10]-1].size[abc]=t;
	  dis_list[slave_dis[10]-1].myid[abc]=g;
	  dis_list[slave_dis[10]-1].work_status[abc]=s;
}
}

/***********************************************************************************/
if(abc==2)
{
if(slave_dis[13]!=0)

{
          t=dis_list[slave_dis[13]].size[abc];
	   g=dis_list[slave_dis[13]].myid[abc];
	   s=	dis_list[slave_dis[13]].work_status[abc];
for(i=slave_dis[13];i<slave_dis[14]-1;i++)
  {
          dis_list[i].size[abc]=dis_list[i+1].size[abc];
	   dis_list[i].myid[abc]=dis_list[i+1].myid[abc];
	   dis_list[i].work_status[abc]=dis_list[i+1].work_status[abc];

  }
        dis_list[slave_dis[14]-1].size[abc]=t;
	  dis_list[slave_dis[14]-1].myid[abc]=g;
	  dis_list[slave_dis[14]-1].work_status[abc]=s;
}
}
}
/***********************************************************************************/
/***********************************************************************************/
if(Level==3)

{
if(abc==0)
{
if(slave_dis[2]!=0)

{
          t=dis_list[slave_dis[2]].size[abc];
	   g=dis_list[slave_dis[2]].myid[abc];
	   s=	dis_list[slave_dis[2]].work_status[abc];
for(i=slave_dis[2];i<slave_dis[8]-1;i++)
  {
          dis_list[i].size[abc]=dis_list[i+1].size[abc];
	   dis_list[i].myid[abc]=dis_list[i+1].myid[abc];
	   dis_list[i].work_status[abc]=dis_list[i+1].work_status[abc];

  }
        dis_list[slave_dis[8]-1].size[abc]=t;
	  dis_list[slave_dis[8]-1].myid[abc]=g;
	  dis_list[slave_dis[8]-1].work_status[abc]=s;
}
}
/***********************************************************************************/
if(abc==1)

{
if(slave_dis[5]!=0)

{
          t=dis_list[slave_dis[5]].size[abc];
	   g=dis_list[slave_dis[5]].myid[abc];
	   s=	dis_list[slave_dis[5]].work_status[abc];
for(i=slave_dis[5];i<slave_dis[11]-1;i++)
  {
          dis_list[i].size[abc]=dis_list[i+1].size[abc];
	   dis_list[i].myid[abc]=dis_list[i+1].myid[abc];
	   dis_list[i].work_status[abc]=dis_list[i+1].work_status[abc];

  }
        dis_list[slave_dis[11]-1].size[abc]=t;
	  dis_list[slave_dis[11]-1].myid[abc]=g;
	  dis_list[slave_dis[11]-1].work_status[abc]=s;
}
}

/***********************************************************************************/
if(abc==2)
{
if(slave_dis[15]!=0)

{
          t=dis_list[slave_dis[15]].size[abc];
	   g=dis_list[slave_dis[15]].myid[abc];
	   s=	dis_list[slave_dis[15]].work_status[abc];
for(i=slave_dis[15];i<slave_dis[16]-1;i++)
  {
          dis_list[i].size[abc]=dis_list[i+1].size[abc];
	   dis_list[i].myid[abc]=dis_list[i+1].myid[abc];
	   dis_list[i].work_status[abc]=dis_list[i+1].work_status[abc];

  }
        dis_list[slave_dis[16]-1].size[abc]=t;
	  dis_list[slave_dis[16]-1].myid[abc]=g;
	  dis_list[slave_dis[16]-1].work_status[abc]=s;
}
}
}
/***********************************************************************************/
/***********************************************************************************/
if(Level==6)
{
if(abc==0)
{
if(slave_dis[3]!=0)

{
          t=dis_list[slave_dis[3]].size[abc];
	   g=dis_list[slave_dis[3]].myid[abc];
	   s=	dis_list[slave_dis[3]].work_status[abc];
for(i=slave_dis[3];i<slave_dis[9]-1;i++)
  {
          dis_list[i].size[abc]=dis_list[i+1].size[abc];
	   dis_list[i].myid[abc]=dis_list[i+1].myid[abc];
	   dis_list[i].work_status[abc]=dis_list[i+1].work_status[abc];

  }
        dis_list[slave_dis[9]-1].size[abc]=t;
	  dis_list[slave_dis[9]-1].myid[abc]=g;
	  dis_list[slave_dis[9]-1].work_status[abc]=s;
}
}
/***********************************************************************************/
if(abc==1)

{
if(slave_dis[6]!=0)

{
          t=dis_list[slave_dis[6]].size[abc];
	   g=dis_list[slave_dis[6]].myid[abc];
	   s=	dis_list[slave_dis[6]].work_status[abc];
for(i=slave_dis[6];i<slave_dis[12]-1;i++)
  {
          dis_list[i].size[abc]=dis_list[i+1].size[abc];
	   dis_list[i].myid[abc]=dis_list[i+1].myid[abc];
	   dis_list[i].work_status[abc]=dis_list[i+1].work_status[abc];

  }
        dis_list[slave_dis[12]-1].size[abc]=t;
	  dis_list[slave_dis[12]-1].myid[abc]=g;
	  dis_list[slave_dis[12]-1].work_status[abc]=s;
}
}

/***********************************************************************************/
if(abc==2)
{
if(slave_dis[17]!=0)

{
          t=dis_list[slave_dis[17]].size[abc];
	   g=dis_list[slave_dis[17]].myid[abc];
	   s=	dis_list[slave_dis[17]].work_status[abc];
for(i=slave_dis[17];i<slave_dis[18]-1;i++)
  {
          dis_list[i].size[abc]=dis_list[i+1].size[abc];
	   dis_list[i].myid[abc]=dis_list[i+1].myid[abc];
	   dis_list[i].work_status[abc]=dis_list[i+1].work_status[abc];

  }
        dis_list[slave_dis[18]-1].size[abc]=t;
	  dis_list[slave_dis[18]-1].myid[abc]=g;
	  dis_list[slave_dis[18]-1].work_status[abc]=s;
}
}
}


}

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

 scan_init=20;

	}
      EXTI_ClearITPendingBit(EXTI_Line8);

  if(EXTI_GetITStatus(EXTI_Line9) != RESET)
	
	{
	delay_us(1000);//按键消抖
if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_9)==0)
{	
order_trans_rs485(mybox.myid,0,1,1,0,CONTROL);
order_trans_rs485(mybox.myid,0,1,2,0,CONTROL);
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
order_trans_rs485(mybox.myid,hand_id,1,1,0,CONTROL);
order_trans_rs485(mybox.myid,hand_id,1,2,0,CONTROL);
hand_comm_onoff=0;

/*快速控制器手动切 共补*/
set_74hc273(hand_id, OFF);
 Light_pad_on(dis_com,hand_id,1,1,0);
}

if(dis_com==0)
{
computer_trans_rs485(mybox.myid,hand_id,1,1,23,CONTROL);//三相一起切命令
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
if(comm_number==0&&hand_comm_onoff==0)
{
order_trans_rs485(mybox.myid,hand_id,1,1,1,CONTROL);
comm_number=1;
hand_comm_onoff=1;

}

if(comm_number==1&&hand_comm_onoff==0)
{
order_trans_rs485(mybox.myid,hand_id,1,2,1,CONTROL);
comm_number=0;
hand_comm_onoff=1;

}
/*快速控制器手动投 共补*/

set_74hc273(hand_id, ON);
 Light_pad_on(dis_com,hand_id,1,1,0);
//comm_list[i].work_status=1;

}
if(dis_com==0)
{
computer_trans_rs485(mybox.myid,hand_id,1,1,32,CONTROL);//三相一起投命令
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
	//if(free_timeout>1)free_timeout--;
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
		{	
			Work_Flag=!Work_Flag;	
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




