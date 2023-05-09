/**
  ******************************************************************************
  * 文件名程: main.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-8-14
  * 功    能: 伺服驱动器速度位置模式
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F1Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "string.h"
#include "usart/bsp_debug_usart.h"
#include "ASDA_B2/bsp_ASDA_B2.h"
#include "AdvancedTIM/bsp_AdvancedTIM.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
#define  MOTOR_MOVING    0x01          // 电机正在运动
#define  MOTOR_STOP      0x00          // 电机停止运动
/* 私有变量 ------------------------------------------------------------------*/
__IO uint8_t Rx_Buf[100];               // 接收数据缓存
__IO uint16_t Togle_Speed = 100;       // 定时器输出翻转速度 
__IO uint8_t  Motor_STA = MOTOR_STOP;  // 电机状态
static __IO uint32_t Pulse = 0; 
__IO uint8_t tmpRxBuf_485 = 0;     // 接收缓存
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
void ServoMotorMov(int32_t Step,uint32_t Speed);
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 系统时钟配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     //使能PWR时钟

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  //设置调压器输出电压级别1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // 外部晶振，8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        //打开HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    //打开PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            //PLL时钟源选择HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 //8分频MHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               //336倍频
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     //2分频，得到168MHz主时钟
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 //USB/SDIO/随机数产生器等的主PLL分频系数
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // 系统时钟：168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHB时钟： 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1时钟：42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2时钟：84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // 使能CSS功能，优先使用外部晶振，内部时钟源为备用
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms中断一次
	// HAL_RCC_GetHCLKFreq()/100000	 10us中断一次
	// HAL_RCC_GetHCLKFreq()/1000000 1us中断一次
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                // 配置并启动系统滴答定时器
  /* 系统滴答定时器时钟源 */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* 系统滴答定时器中断优先级配置 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


int Pos1;
int Pos2;
int Pos1_Speed;
int Pos2_Speed;
int Torque;
int Accelerate_Time;
int Moderate_Time;


/**
  * 函数功能: 主函数.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
int main(void)
{  
  int8_t ServoMotorDir = 1;       // 电机方向控制
  /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  HAL_Init();
  /* 配置系统时钟 */
  SystemClock_Config();

  /* 初始化串口并配置串口中断优先级 */
  MX_DEBUG_USART_Init();
  RS232_USARTx_Init();
  HAL_UART_Receive_IT(&husartx_rs485 ,(uint8_t*)&tmpRxBuf_485,1);
  
  SERVOMOTOR_TIMx_Init();
  /* 确定定时器 */
  HAL_TIM_Base_Start(&htimx_SERVOMOTOR);
  /* 启动比较输出并使能中断 */
  HAL_TIM_OC_Stop_IT(&htimx_SERVOMOTOR,TIM_CHANNEL_1);
  
  /* 初始化ASDA-B2参数 */
  ASDAB2_Init(); 
  /* 设定速度模式的速度为 30r/min */
  SetSpeed(REG_SP1,300);      
  /* 启动伺服 */
  StartServo();

  UsartState = WaitRx;

  /* 无限循环 */
  while (1)
  {
    /* 控制伺服电机以速度模式(30 r/min)走4s */
    S_P_MODE_S();
    HAL_Delay(4000);
    /* 控制伺服电机以位置模式走一圈 */
    S_P_MODE_PT();
    ServoMotorDir = CCW;                          // 方向设定,逆时针
    ServoMotorMov( 3*ServoMotorDir*PULSE_REV,10); // 伺服电机旋转三圈,速度为 1 rev/s
    /* 启动比较输出并使能中断 */
    HAL_TIM_OC_Start_IT(&htimx_SERVOMOTOR,TIM_CHANNEL_1);
    HAL_Delay(4000);


    if(UsartState == Ready)
    {
        if((Rx_Buf[0] == 0x52) && (Rx_Buf[1] == 0x45))  //协议头
        {
            /* 使能关、顺时针、逆时针 */
            if(Rx_Buf[5] == 0x30)           //关
            {
                StopServo();
            }
            else if(Rx_Buf[5] == 0x31)       //顺时针
            {
                StartServo();
                ServoMotorDir = CW;                          // 方向设定,顺时针
            }
            else if(Rx_Buf[5] == 0x32)       //逆时针
            {
                StartServo();
                ServoMotorDir = CCW;                          // 方向设定,逆时针
            }

            /* 位置1、位置2 选择 */
            if(Rx_Buf[56] == 0x30)  //位置1
            {
                //位置1 值
                Pos1 = (Rx_Buf[8] - 0x30) * 10000 + (Rx_Buf[9] - 0x30) * 1000 + (Rx_Buf[10] - 0x30) * 100 +
                        (Rx_Buf[11] - 0x30) * 10 + (Rx_Buf[12] - 0x30);

                //位置1 速度值
                if(Rx_Buf[15] == 0x2D)    //负数
                {
                    Pos1_Speed = -((Rx_Buf[16] - 0x30) * 1000 + (Rx_Buf[17] - 0x30) * 100 + (Rx_Buf[18] - 0x30) * 10 +
                                    (Rx_Buf[19] - 0x30));
                }
                else
                {
                    Pos1_Speed = (Rx_Buf[16] - 0x30) * 1000 + (Rx_Buf[17] - 0x30) * 100 + (Rx_Buf[18] - 0x30) * 10 +
                                    (Rx_Buf[19] - 0x30);
                }
            }
            else if(Rx_Buf[56] == 0x31)  //位置2
            {
                //位置2 值
                Pos2 = (Rx_Buf[22] - 0x30) * 10000 + (Rx_Buf[23] - 0x30) * 1000 + (Rx_Buf[24] - 0x30) * 100 +
                        (Rx_Buf[25] - 0x30) * 10 + (Rx_Buf[26] - 0x30);

                //位置2 速度值
                if(Rx_Buf[29] == 0x2D)    //负数
                {
                    Pos1_Speed = -((Rx_Buf[30] - 0x30) * 1000 + (Rx_Buf[31] - 0x30) * 100 + (Rx_Buf[32] - 0x30) * 10 +
                                    (Rx_Buf[33] - 0x30));
                }
                else
                {
                    Pos1_Speed = (Rx_Buf[30] - 0x30) * 1000 + (Rx_Buf[31] - 0x30) * 100 + (Rx_Buf[32] - 0x30) * 10 +
                                    (Rx_Buf[33] - 0x30);
                }        
            }

            //扭矩
            if(Rx_Buf[36] == 0x2D)    //负数
            {
                Torque = -((Rx_Buf[37] - 0x30) * 100 + (Rx_Buf[38] - 0x30) * 10 + (Rx_Buf[39] - 0x30));
            }
            else
            {
                Torque = (Rx_Buf[37] - 0x30) * 100 + (Rx_Buf[38] - 0x30) * 10 + (Rx_Buf[39] - 0x30);
            }

            //加速时间
            Accelerate_Time = (Rx_Buf[42] - 0x30) * 10000 + (Rx_Buf[43] - 0x30) * 1000 + (Rx_Buf[44] - 0x30) * 100 +
                                (Rx_Buf[11] - 0x30) * 10 + (Rx_Buf[12] - 0x30);

            //减速时间
            Moderate_Time = (Rx_Buf[49] - 0x30) * 10000 + (Rx_Buf[50] - 0x30) * 1000 + (Rx_Buf[51] - 0x30) * 100 +
                                (Rx_Buf[52] - 0x30) * 10 + (Rx_Buf[53] - 0x30);
        }

        UsartState = WaitRx;
    }

  }
}

/**
  * 函数功能: 伺服电机运动控制函数
  * 输入参数: Step: TIM输出脉冲数  Speed: 伺服电机运动速度,单位0.1 rev/s
  * 返 回 值: 无
  * 说    明: 台达官方给出的驱动器分辨数:17-bit (160,000 p/rev)
  *           电机走一圈需要的脉冲数是160000,需要给的脉冲数是:
  *           Pulse = 160000/齿轮比,本例程设置齿轮比为 50/10,所以一圈的脉冲数是32000
  */
void ServoMotorMov(int32_t Step,uint32_t Speed)
{
  if(Motor_STA != MOTOR_MOVING)   //在运动过程中不接受指令
  {
    if(Step != 0)  
    {
      /* 方向控制 */
      if(Step > 0)
        SERVOMOTOR_SETDIR_CW();
      else 
      {
        Step = -Step;
        SERVOMOTOR_SETDIR_CCW();
      }
      Pulse = Step;
      /* 速度(r/min) 转换成定时器计数值
       * v =  Pulse/T = 1/t;(单脉冲/单周期)
       * t = 1/Ft;          (单周期 = 1/频率)
       * c = Ft/v;          (单周期对应定时器计数值)
       * Togle_Speed = c/2; (定时器翻转模式,半周期翻转一次)
       */
      Togle_Speed = (T1_FREQ/(PPR*Speed))/2;
      HAL_TIM_OC_Start_IT(&htimx_SERVOMOTOR,TIM_CHANNEL_1);
      Motor_STA = MOTOR_MOVING;
    }
    else return ;
  }
  else return ;
}
/**
  * 函数功能: TIM1,CH1比较翻转回调函数
  * 输入参数: UARTHandle:TIM句柄
  * 返 回 值: 无
  * 说    明: 可以通过设置比较值改变输出脉冲的频率/占空比.
  *           同时记录每个总的脉冲数.
  */
void SERVOMOTOR_TIMx_IRQHandler(void)
{
  uint32_t count;
  uint32_t tmp;
  static uint8_t i = 0; 
  __HAL_TIM_CLEAR_IT(&htimx_SERVOMOTOR, TIM_IT_CC1);
  count = __HAL_TIM_GET_COUNTER(&htimx_SERVOMOTOR);
  tmp = SERVOMOTOR_TIM_PERIOD & (count+Togle_Speed);
  __HAL_TIM_SET_COMPARE(&htimx_SERVOMOTOR,TIM_CHANNEL_1,tmp);
  i++;
  Pulse = Pulse - (i>=2?--i,i--:0);   // 每个脉冲Pulse计数一次,Pulse--
  if(Pulse == 0)
  {
    HAL_TIM_OC_Stop_IT(&htimx_SERVOMOTOR,TIM_CHANNEL_1);
    Motor_STA = MOTOR_STOP;
  }
}

/**
  * 函数功能: 串口接收回调函数
  * 输入参数: UARTHandle:串口句柄
  * 返 回 值: 无
  * 说    明: 接收到从机的反馈数据之后,分包并存放到Rx_Buf里面,一次只能接收一帧数据
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  static uint8_t i = 0;
  if(UartHandle->Instance == RS232_USARTx)
  {
    Rx_Buf[i] = tmpRxBuf_485;
    if(Rx_Buf[0] == ':')
    { 
      i++;
      if(Rx_Buf[i-1] == 0x0A)
      {  
        if(Rx_Buf[i-2] ==0x0D)
        {
          Rx_Buf[i] = '\0';         // 手动添加结束符
          UsartState = Ready;       // 接收完成,通讯待机
          i = 0;  
        }
      }
    }
    else i = 0;    
    HAL_UART_Receive_IT(&husartx_rs485 ,(uint8_t*)&tmpRxBuf_485 ,1);
  }
}
