/**
  ******************************************************************************
  * �ļ�����: main.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-8-14
  * ��    ��: �ŷ��������ٶ�λ��ģʽ
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F1Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "string.h"
#include "usart/bsp_debug_usart.h"
#include "ASDA_B2/bsp_ASDA_B2.h"
#include "AdvancedTIM/bsp_AdvancedTIM.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
#define  MOTOR_MOVING    0x01          // ��������˶�
#define  MOTOR_STOP      0x00          // ���ֹͣ�˶�
/* ˽�б��� ------------------------------------------------------------------*/
__IO uint8_t Rx_Buf[100];               // �������ݻ���
__IO uint16_t Togle_Speed = 100;       // ��ʱ�������ת�ٶ� 
__IO uint8_t  Motor_STA = MOTOR_STOP;  // ���״̬
static __IO uint32_t Pulse = 0; 
__IO uint8_t tmpRxBuf_485 = 0;     // ���ջ���
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
void ServoMotorMov(int32_t Step,uint32_t Speed);
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ϵͳʱ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     //ʹ��PWRʱ��

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  //���õ�ѹ�������ѹ����1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // �ⲿ����8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        //��HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    //��PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            //PLLʱ��Դѡ��HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 //8��ƵMHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               //336��Ƶ
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     //2��Ƶ���õ�168MHz��ʱ��
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 //USB/SDIO/������������ȵ���PLL��Ƶϵ��
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // ϵͳʱ�ӣ�168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHBʱ�ӣ� 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1ʱ�ӣ�42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2ʱ�ӣ�84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // ʹ��CSS���ܣ�����ʹ���ⲿ�����ڲ�ʱ��ԴΪ����
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms�ж�һ��
	// HAL_RCC_GetHCLKFreq()/100000	 10us�ж�һ��
	// HAL_RCC_GetHCLKFreq()/1000000 1us�ж�һ��
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                // ���ò�����ϵͳ�δ�ʱ��
  /* ϵͳ�δ�ʱ��ʱ��Դ */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* ϵͳ�δ�ʱ���ж����ȼ����� */
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
  * ��������: ������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
int main(void)
{  
  int8_t ServoMotorDir = 1;       // ����������
  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();
  /* ����ϵͳʱ�� */
  SystemClock_Config();

  /* ��ʼ�����ڲ����ô����ж����ȼ� */
  MX_DEBUG_USART_Init();
  RS232_USARTx_Init();
  HAL_UART_Receive_IT(&husartx_rs485 ,(uint8_t*)&tmpRxBuf_485,1);
  
  SERVOMOTOR_TIMx_Init();
  /* ȷ����ʱ�� */
  HAL_TIM_Base_Start(&htimx_SERVOMOTOR);
  /* �����Ƚ������ʹ���ж� */
  HAL_TIM_OC_Stop_IT(&htimx_SERVOMOTOR,TIM_CHANNEL_1);
  
  /* ��ʼ��ASDA-B2���� */
  ASDAB2_Init(); 
  /* �趨�ٶ�ģʽ���ٶ�Ϊ 30r/min */
  SetSpeed(REG_SP1,300);      
  /* �����ŷ� */
  StartServo();

  UsartState = WaitRx;

  /* ����ѭ�� */
  while (1)
  {
    /* �����ŷ�������ٶ�ģʽ(30 r/min)��4s */
    S_P_MODE_S();
    HAL_Delay(4000);
    /* �����ŷ������λ��ģʽ��һȦ */
    S_P_MODE_PT();
    ServoMotorDir = CCW;                          // �����趨,��ʱ��
    ServoMotorMov( 3*ServoMotorDir*PULSE_REV,10); // �ŷ������ת��Ȧ,�ٶ�Ϊ 1 rev/s
    /* �����Ƚ������ʹ���ж� */
    HAL_TIM_OC_Start_IT(&htimx_SERVOMOTOR,TIM_CHANNEL_1);
    HAL_Delay(4000);


    if(UsartState == Ready)
    {
        if((Rx_Buf[0] == 0x52) && (Rx_Buf[1] == 0x45))  //Э��ͷ
        {
            /* ʹ�ܹء�˳ʱ�롢��ʱ�� */
            if(Rx_Buf[5] == 0x30)           //��
            {
                StopServo();
            }
            else if(Rx_Buf[5] == 0x31)       //˳ʱ��
            {
                StartServo();
                ServoMotorDir = CW;                          // �����趨,˳ʱ��
            }
            else if(Rx_Buf[5] == 0x32)       //��ʱ��
            {
                StartServo();
                ServoMotorDir = CCW;                          // �����趨,��ʱ��
            }

            /* λ��1��λ��2 ѡ�� */
            if(Rx_Buf[56] == 0x30)  //λ��1
            {
                //λ��1 ֵ
                Pos1 = (Rx_Buf[8] - 0x30) * 10000 + (Rx_Buf[9] - 0x30) * 1000 + (Rx_Buf[10] - 0x30) * 100 +
                        (Rx_Buf[11] - 0x30) * 10 + (Rx_Buf[12] - 0x30);

                //λ��1 �ٶ�ֵ
                if(Rx_Buf[15] == 0x2D)    //����
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
            else if(Rx_Buf[56] == 0x31)  //λ��2
            {
                //λ��2 ֵ
                Pos2 = (Rx_Buf[22] - 0x30) * 10000 + (Rx_Buf[23] - 0x30) * 1000 + (Rx_Buf[24] - 0x30) * 100 +
                        (Rx_Buf[25] - 0x30) * 10 + (Rx_Buf[26] - 0x30);

                //λ��2 �ٶ�ֵ
                if(Rx_Buf[29] == 0x2D)    //����
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

            //Ť��
            if(Rx_Buf[36] == 0x2D)    //����
            {
                Torque = -((Rx_Buf[37] - 0x30) * 100 + (Rx_Buf[38] - 0x30) * 10 + (Rx_Buf[39] - 0x30));
            }
            else
            {
                Torque = (Rx_Buf[37] - 0x30) * 100 + (Rx_Buf[38] - 0x30) * 10 + (Rx_Buf[39] - 0x30);
            }

            //����ʱ��
            Accelerate_Time = (Rx_Buf[42] - 0x30) * 10000 + (Rx_Buf[43] - 0x30) * 1000 + (Rx_Buf[44] - 0x30) * 100 +
                                (Rx_Buf[11] - 0x30) * 10 + (Rx_Buf[12] - 0x30);

            //����ʱ��
            Moderate_Time = (Rx_Buf[49] - 0x30) * 10000 + (Rx_Buf[50] - 0x30) * 1000 + (Rx_Buf[51] - 0x30) * 100 +
                                (Rx_Buf[52] - 0x30) * 10 + (Rx_Buf[53] - 0x30);
        }

        UsartState = WaitRx;
    }

  }
}

/**
  * ��������: �ŷ�����˶����ƺ���
  * �������: Step: TIM���������  Speed: �ŷ�����˶��ٶ�,��λ0.1 rev/s
  * �� �� ֵ: ��
  * ˵    ��: ̨��ٷ��������������ֱ���:17-bit (160,000 p/rev)
  *           �����һȦ��Ҫ����������160000,��Ҫ������������:
  *           Pulse = 160000/���ֱ�,���������ó��ֱ�Ϊ 50/10,����һȦ����������32000
  */
void ServoMotorMov(int32_t Step,uint32_t Speed)
{
  if(Motor_STA != MOTOR_MOVING)   //���˶������в�����ָ��
  {
    if(Step != 0)  
    {
      /* ������� */
      if(Step > 0)
        SERVOMOTOR_SETDIR_CW();
      else 
      {
        Step = -Step;
        SERVOMOTOR_SETDIR_CCW();
      }
      Pulse = Step;
      /* �ٶ�(r/min) ת���ɶ�ʱ������ֵ
       * v =  Pulse/T = 1/t;(������/������)
       * t = 1/Ft;          (������ = 1/Ƶ��)
       * c = Ft/v;          (�����ڶ�Ӧ��ʱ������ֵ)
       * Togle_Speed = c/2; (��ʱ����תģʽ,�����ڷ�תһ��)
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
  * ��������: TIM1,CH1�ȽϷ�ת�ص�����
  * �������: UARTHandle:TIM���
  * �� �� ֵ: ��
  * ˵    ��: ����ͨ�����ñȽ�ֵ�ı���������Ƶ��/ռ�ձ�.
  *           ͬʱ��¼ÿ���ܵ�������.
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
  Pulse = Pulse - (i>=2?--i,i--:0);   // ÿ������Pulse����һ��,Pulse--
  if(Pulse == 0)
  {
    HAL_TIM_OC_Stop_IT(&htimx_SERVOMOTOR,TIM_CHANNEL_1);
    Motor_STA = MOTOR_STOP;
  }
}

/**
  * ��������: ���ڽ��ջص�����
  * �������: UARTHandle:���ھ��
  * �� �� ֵ: ��
  * ˵    ��: ���յ��ӻ��ķ�������֮��,�ְ�����ŵ�Rx_Buf����,һ��ֻ�ܽ���һ֡����
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
          Rx_Buf[i] = '\0';         // �ֶ���ӽ�����
          UsartState = Ready;       // �������,ͨѶ����
          i = 0;  
        }
      }
    }
    else i = 0;    
    HAL_UART_Receive_IT(&husartx_rs485 ,(uint8_t*)&tmpRxBuf_485 ,1);
  }
}
