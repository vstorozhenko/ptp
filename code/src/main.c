#include "cmsis_os.h"
#include "stm32f4x7_eth.h"
#include "netconf.h"
#include "main.h"
#include "tcpip.h"
//#include "telnet.h"
//#include "ptpd.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/*--------------- LCD Messages ---------------*/
#define MESSAGE1   "     STM32F4x7      "
#define MESSAGE2   " STM32F-4 Discovery "
#define MESSAGE3   "  PTPD Client Demo  "
#define MESSAGE4   "                    "

/*--------------- Tasks Priority -------------*/
#define DHCP_TASK_PRIO   ( osPriorityIdle + 2 )      
//#define LED_TASK_PRIO    ( osPriorityIdle + 1 )

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void UART_Init(void);
//void LCD_LED_Init(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured to 
       168 MHz, this is done through SystemInit() function which is called from
       startup file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */  

 	/* Initialize UART */
	//UART_Init();

	/* Initialize LCD and Leds */
  //LCD_LED_Init();
  
  /* Configure ethernet (GPIOs, clocks, MAC, DMA) */ 
  //ETH_BSP_Config();
    
  /* Initilaize the LwIP stack */
  //LwIP_Init();
  
	/* Initialize the PTP daemon. */
	//ptpd_init();

  /* Initialize telnet shell server. */
  //telnet_shell_init();

#ifdef USE_DHCP
  /* Start DHCP Client */
	sys_thread_new("DHCP", LwIP_DHCP_task, NULL, DEFAULT_THREAD_STACKSIZE, DHCP_TASK_PRIO);
#endif
    
  for ( ;; )
	{
    /* Toggle LED4 each 250ms */
    STM_EVAL_LEDToggle(LED4);
    sys_msleep(250);
  }
}

/**
  * @brief  Initializes the UART resources.
  * @param  None
  * @retval None
  */
void UART_Init(void)
{
	USART_InitTypeDef USART_InitStructure;

  // USARTx configured as follows: 
	//   115200 baud, 8 Bits, 1 Stop, No Parity, 
	//   No Flow Control, Receive and Transmit Enabled.
	/*USART_StructInit(&USART_InitStructure);
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  STM_EVAL_COMInit(COM1, &USART_InitStructure);

  // Output a message using printf function.
  printf("\nUSART Initialized\n");*/
}

/**
  * @brief  Initializes the LCD and LEDs resources.
  * @param  None
  * @retval None
  */
	//void LCD_LED_Init(void)
	//{
	//  STM_EVAL_LEDInit(LED4); 

	//#ifdef USE_LCD
  /* Initialize the STM324xG-EVAL's LCD */
  //STM32f4_Discovery_LCD_Init();

  /* Clear the LCD */
  //LCD_Clear(Black);

  /* Set the LCD Back Color */
  //LCD_SetBackColor(Black);

  /* Set the LCD Text Color */
  //LCD_SetTextColor(White);

  /* Display message on the LCD*/
  //LCD_DisplayStringLine(Line0, (uint8_t*)MESSAGE1);
  ////LCD_DisplayStringLine(Line1, (uint8_t*)MESSAGE2);
  //LCD_DisplayStringLine(Line2, (uint8_t*)MESSAGE3);
  //LCD_DisplayStringLine(Line3, (uint8_t*)MESSAGE4); 
	//#endif
	//}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
/*int fputc(int ch, FILE *f)
{
	if (ch == '\n') fputc('\r', f);
  USART_SendData(EVAL_COM1, (uint8_t) ch);
  while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET);
  return ch;
}*/

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{
  while (1)
  {}
}
#endif

/*********** Portions COPYRIGHT 2012 Embest Tech. Co., Ltd.*****END OF FILE****/
