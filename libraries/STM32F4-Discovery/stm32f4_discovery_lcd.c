/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_lcd.h"
#include "fonts.c"

#define LCD_RST_PIN                  (GPIO_Pin_3)
#define LCD_RST_PORT                 (GPIOD)

#define LCD_PWM_PIN                  (GPIO_Pin_13)
#define LCD_PWM_PORT                 (GPIOD)

/* Note: LCD /CS is NE1 - Bank 1 of NOR/SRAM Bank 1~4 */
#define  LCD_BASE_Data               ((u32)(0x60000000|0x00100000))
#define  LCD_BASE_Addr               ((u32)(0x60000000|0x00000000))
#define  LCD_CMD                     (*(vu16 *)LCD_BASE_Addr)
#define  LCD_Data                    (*(vu16 *)LCD_BASE_Data)

#define MAX_POLY_CORNERS             200
#define POLY_Y(Z)                    ((int32_t)((Points + Z)->X))
#define POLY_X(Z)                    ((int32_t)((Points + Z)->Y))

#define ABS(X)  ((X) > 0 ? (X) : -(X))     

static sFONT *LCD_Currentfonts;
static __IO uint16_t TextColor = 0x0000, BackColor = 0xFFFF;
  

#ifndef USE_Delay
static void delay(__IO uint32_t nCount);
#endif /* USE_Delay*/
static void PutPixel(int16_t x, int16_t y);
static void LCD_PolyLineRelativeClosed(pPoint Points, uint16_t PointCount, uint16_t Closed);

/*void LCD_DeInit(void)
{ 
  GPIO_InitTypeDef GPIO_InitStructure;
  LCD_DisplayOff();
  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM3, ENABLE);
  FSMC_NORSRAMDeInit(FSMC_Bank1_NORSRAM3);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 |
                                GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
 
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_MCO);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
                                GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                                GPIO_Pin_15;

  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11 , GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource12 , GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13 , GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource14 , GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource15 , GPIO_AF_MCO);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | 
                                GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 | 
                                GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource0, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource1, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource2, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource3, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource4, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource5, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource12, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource13, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource14, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource15, GPIO_AF_MCO);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | 
                                GPIO_Pin_4 | GPIO_Pin_5;

  GPIO_Init(GPIOG, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOG,GPIO_PinSource0, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOG,GPIO_PinSource1, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOG,GPIO_PinSource2, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOG,GPIO_PinSource3, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOG,GPIO_PinSource4, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOG,GPIO_PinSource5, GPIO_AF_MCO);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13; 

  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOD,GPIO_PinSource11, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource12, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource13, GPIO_AF_MCO);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 |GPIO_Pin_5;

  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource4, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource5, GPIO_AF_MCO);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 

  GPIO_Init(GPIOG, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource12, GPIO_AF_MCO);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; 
  GPIO_Init(GPIOE, &GPIO_InitStructure); 

  GPIO_PinAFConfig(GPIOE,GPIO_PinSource0, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource1, GPIO_AF_MCO);
}*/


void LCD_CtrlLinesConfig(void)
{
  /*GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE |
                         RCC_AHB1Periph_GPIOF, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 |
                                GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15 |
                                GPIO_Pin_4 |GPIO_Pin_5;;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
                                GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                                GPIO_Pin_15;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource12 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource14 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource15 , GPIO_AF_FSMC);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOE, &GPIO_InitStructure);  
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource3, GPIO_AF_FSMC);	   

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);

  GPIO_InitStructure.GPIO_Pin = LCD_RST_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

  GPIO_Init(LCD_RST_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LCD_PWM_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

  GPIO_Init(LCD_PWM_PORT, &GPIO_InitStructure);
  GPIO_SetBits(LCD_PWM_PORT, LCD_PWM_PIN);*/
}

void LCD_FSMCConfig(void)
{
  /*FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef  p;
   
  RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);

  p.FSMC_AddressSetupTime = 1;
  p.FSMC_AddressHoldTime = 0;
  p.FSMC_DataSetupTime = 9;
  p.FSMC_BusTurnAroundDuration = 0;
  p.FSMC_CLKDivision = 0;
  p.FSMC_DataLatency = 0;
  p.FSMC_AccessMode = FSMC_AccessMode_A;

  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;

  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);   

  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);*/
}

void STM32f4_Discovery_LCD_Init(void){ 
  /*unsigned long ulCount;
  LCD_CtrlLinesConfig();
  LCD_FSMCConfig();	
  _delay_(5); 
  GPIO_ResetBits(LCD_RST_PORT, LCD_RST_PIN);	
  _delay_(10);	
  GPIO_SetBits(LCD_RST_PORT, LCD_RST_PIN);
  LCD_WriteReg(SSD2119_SLEEP_MODE_1_REG, 0x0001);
  LCD_WriteReg(SSD2119_PWR_CTRL_5_REG, 0x00B2);
  LCD_WriteReg(SSD2119_VCOM_OTP_1_REG, 0x0006);
  LCD_WriteReg(SSD2119_OSC_START_REG, 0x0001);
  LCD_WriteReg(SSD2119_OUTPUT_CTRL_REG, 0x30EF);
  LCD_WriteReg(SSD2119_SLEEP_MODE_1_REG, 0x0000);
  _delay_(5);
  LCD_WriteReg(SSD2119_ENTRY_MODE_REG, ENTRY_MODE_DEFAULT);
  LCD_WriteReg(SSD2119_SLEEP_MODE_2_REG, 0x0999);
  LCD_WriteReg(SSD2119_ANALOG_SET_REG, 0x3800);
  LCD_WriteReg(SSD2119_DISPLAY_CTRL_REG, 0x0033);
  LCD_WriteReg(SSD2119_PWR_CTRL_2_REG, 0x0005);
  LCD_WriteReg(SSD2119_GAMMA_CTRL_1_REG, 0x0000);
  LCD_WriteReg(SSD2119_GAMMA_CTRL_2_REG, 0x0303);
  LCD_WriteReg(SSD2119_GAMMA_CTRL_3_REG, 0x0407);
  LCD_WriteReg(SSD2119_GAMMA_CTRL_4_REG, 0x0301);
  LCD_WriteReg(SSD2119_GAMMA_CTRL_5_REG, 0x0301);
  LCD_WriteReg(SSD2119_GAMMA_CTRL_6_REG, 0x0403);
  LCD_WriteReg(SSD2119_GAMMA_CTRL_7_REG, 0x0707);
  LCD_WriteReg(SSD2119_GAMMA_CTRL_8_REG, 0x0400);
  LCD_WriteReg(SSD2119_GAMMA_CTRL_9_REG, 0x0a00);
  LCD_WriteReg(SSD2119_GAMMA_CTRL_10_REG, 0x1000);
  LCD_WriteReg(SSD2119_PWR_CTRL_3_REG, 0x000A);
  LCD_WriteReg(SSD2119_PWR_CTRL_4_REG, 0x2E00);
  LCD_WriteReg(SSD2119_V_RAM_POS_REG, (LCD_PIXEL_HEIGHT-1) << 8);
  LCD_WriteReg(SSD2119_H_RAM_START_REG, 0x0000);
  LCD_WriteReg(SSD2119_H_RAM_END_REG, LCD_PIXEL_WIDTH-1);
  LCD_WriteReg(SSD2119_X_RAM_ADDR_REG, 0x00);
  LCD_WriteReg(SSD2119_Y_RAM_ADDR_REG, 0x00);
  LCD_WriteReg(SSD2119_RAM_DATA_REG, 0x0000);
  for(ulCount = 0; ulCount < (LCD_PIXEL_WIDTH * LCD_PIXEL_HEIGHT); ulCount++){
    LCD_WriteRAM(0x0000);
  }
  LCD_SetFont(&LCD_DEFAULT_FONT);
	*/
}


void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos){
  /*LCD_WriteReg(SSD2119_X_RAM_ADDR_REG, Xpos);
  LCD_WriteReg(SSD2119_Y_RAM_ADDR_REG, Ypos);*/
}

void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue){
  /*LCD_CMD = LCD_Reg;
  LCD_Data = LCD_RegValue;*/
}


uint16_t LCD_ReadReg(uint8_t LCD_Reg){
  /*LCD_CMD = LCD_Reg;*/
  return (LCD_Data);
}

void LCD_WriteRAM_Prepare(void)
{
	//LCD_CMD = SSD2119_RAM_DATA_REG;
}

/**
  * @brief  Writes to the LCD RAM.
  * @param  RGB_Code: the pixel color in RGB mode (5-6-5).
  * @retval None
  */
void LCD_WriteRAM(uint16_t RGB_Code)
{
  //LCD_Data = RGB_Code;
}

uint16_t LCD_ReadRAM(void)
{
  /* Write 16-bit Index (then Read Reg) */
//  LCD_CMD = SSD2119_RAM_DATA_REG; /* Select GRAM Reg */
  /* Read 16-bit Reg */
  return LCD_Data;
}

/**
  * @brief  Test LCD Display
  * @retval None
  */
void LCD_RGB_Test(void)
{
  /*uint32_t index;
  LCD_SetCursor(0x00, 0x00); 
  LCD_WriteRAM_Prepare();

  for(index = 0; index < (LCD_PIXEL_HEIGHT*LCD_PIXEL_WIDTH)/3; index++){
    LCD_Data = LCD_COLOR_RED;
  }
	  
  for(;index < 2*(LCD_PIXEL_HEIGHT*LCD_PIXEL_WIDTH)/3; index++){
    LCD_Data = LCD_COLOR_GREEN;
  }
	  
  for(; index < LCD_PIXEL_HEIGHT*LCD_PIXEL_WIDTH; index++){
    LCD_Data = LCD_COLOR_BLUE;
  }*/
}


void LCD_SetColors(__IO uint16_t _TextColor, __IO uint16_t _BackColor)
{
  //TextColor = _TextColor; 
  //BackColor = _BackColor;
}


void LCD_GetColors(__IO uint16_t *_TextColor, __IO uint16_t *_BackColor)
{
  *_TextColor = TextColor; *_BackColor = BackColor;
}

/**
  * @brief  Sets the Text color.
  * @param  Color: specifies the Text color code RGB(5-6-5).
  * @retval None
  */
void LCD_SetTextColor(__IO uint16_t Color)
{
  //TextColor = Color;
}



void LCD_SetBackColor(__IO uint16_t Color)
{
  //BackColor = Color;
}

/**
LCD_DisplayOff
  */
void LCD_DisplayOff(void)
{

}

/**
LCD_DisplayOn
  */
void LCD_DisplayOn(void)
{

}


void LCD_SetFont(sFONT *fonts)
{
  LCD_Currentfonts = fonts;
}


sFONT *LCD_GetFont(void)
{
  return LCD_Currentfonts;
}


void LCD_ClearLine(uint16_t Line)
{
  uint16_t refcolumn = 0;

  do {
    LCD_DisplayChar(Line, refcolumn, ' ');
    refcolumn += LCD_Currentfonts->Width;
  } while (refcolumn < LCD_PIXEL_WIDTH);	
}


void LCD_Clear(uint16_t Color)
{
  uint32_t index = 0;
  
  LCD_SetCursor(0x00, 0x00); 
  LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
  for(index = 0; index < LCD_PIXEL_HEIGHT*LCD_PIXEL_WIDTH; index++)
  {
    LCD_Data = Color;
  }  
}


static void PutPixel(int16_t x, int16_t y)
{ 
  if(x < 0 || x > LCD_PIXEL_WIDTH-1 || y < 0 || y > LCD_PIXEL_HEIGHT-1)
  {
    return;  
  }
  LCD_DrawLine(x, y, 1, LCD_DIR_HORIZONTAL);
}


void LCD_DrawChar(uint16_t Xpos, uint16_t Ypos, const uint16_t *c)
{
 /* uint32_t index = 0, i = 0;
  uint16_t  Xaddress = 0;
  Xaddress = Xpos;
  
  LCD_SetCursor(Ypos, Xaddress);
  
  for(index = 0; index < LCD_Currentfonts->Height; index++)
  {
    LCD_WriteRAM_Prepare();
    for(i = 0; i < LCD_Currentfonts->Width; i++)
    {
      if((((c[index] & ((0x80 << ((LCD_Currentfonts->Width / 12 ) * 8 ) ) >> i)) == 0x00) &&(LCD_Currentfonts->Width <= 12))||
        (((c[index] & (0x1 << i)) == 0x00)&&(LCD_Currentfonts->Width > 12 )))

      {
        LCD_WriteRAM(BackColor);
      }
      else
      {
        LCD_WriteRAM(TextColor);
      } 
    }
    Xaddress++;
    LCD_SetCursor(Ypos, Xaddress);
  }*/
}


void LCD_DisplayChar(uint16_t Line, uint16_t Column, uint8_t Ascii)
{
  //Ascii -= 32;
 // LCD_DrawChar(Line, Column, &LCD_Currentfonts->table[Ascii * LCD_Currentfonts->Height]);
}


void LCD_DisplayStringLine(uint16_t Line, uint8_t *ptr)
{
  /*uint16_t refcolumn = 0;
  while (*ptr != 0)
  {
    LCD_DisplayChar(Line, refcolumn, *ptr);
    refcolumn += LCD_Currentfonts->Width;
	if (refcolumn >= LCD_PIXEL_WIDTH) {
		break;
	}
    ptr++;
  }*/
}


void LCD_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t width, uint16_t Height)
{
 /* uint32_t value = 0;	

  LCD_WriteReg(SSD2119_H_RAM_START_REG, Xpos);
	
  if ((Xpos+width) >= LCD_PIXEL_WIDTH) {
    LCD_WriteReg(SSD2119_H_RAM_END_REG, LCD_PIXEL_WIDTH-1);	
  } else {
    LCD_WriteReg(SSD2119_H_RAM_END_REG, Xpos+width);		
  }
  
  if ((Ypos+Height) >= LCD_PIXEL_HEIGHT) {
    value = (LCD_PIXEL_HEIGHT-1) << 8;	
  } else {
    value = (Ypos+Height) << 8;	
  }
  value |= Xpos;
  LCD_WriteReg(SSD2119_V_RAM_POS_REG, value);
  LCD_SetCursor(Xpos, Ypos);*/
}

void LCD_WindowModeDisable(void)
{
/*#if 0
  LCD_SetDisplayWindow(239, 0x13F, 240, 320);
  LCD_WriteReg(LCD_REG_3, 0x1018);    
#endif*/
}


void LCD_DrawLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction)
{
  /*uint32_t i = 0;
  
  LCD_SetCursor(Xpos, Ypos);
  if(Direction == LCD_DIR_HORIZONTAL)
  {
    LCD_WriteRAM_Prepare();
    for(i = 0; i < Length; i++)
    {
      LCD_WriteRAM(TextColor);
    }
  }
  else
  {
    for(i = 0; i < Length; i++)
    {
      LCD_WriteRAM_Prepare(); 
      LCD_WriteRAM(TextColor);
      Ypos++;
      LCD_SetCursor(Xpos, Ypos);
    }
  }*/
}

void LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width)
{
  /*LCD_DrawLine(Xpos, Ypos, Width, LCD_DIR_VERTICAL);
  LCD_DrawLine((Xpos + Height-1), Ypos, Width, LCD_DIR_VERTICAL);
  
  LCD_DrawLine(Xpos, Ypos, Height, LCD_DIR_HORIZONTAL);
  LCD_DrawLine(Xpos, (Ypos + Width), Height, LCD_DIR_HORIZONTAL);*/
}


void LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  /*int32_t  D;
  uint32_t  CurX;
  uint32_t  CurY;
  
  D = 3 - (Radius << 1);
  CurX = 0;
  CurY = Radius;
  
  while (CurX <= CurY)
  {
    LCD_SetCursor(Xpos + CurX, Ypos + CurY);
    LCD_WriteRAM_Prepare();
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos + CurX, Ypos - CurY);
    LCD_WriteRAM_Prepare(); 
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos - CurX, Ypos + CurY);
    LCD_WriteRAM_Prepare();
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos - CurX, Ypos - CurY);
    LCD_WriteRAM_Prepare();
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos + CurY, Ypos + CurX);
    LCD_WriteRAM_Prepare(); 
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos + CurY, Ypos - CurX);
    LCD_WriteRAM_Prepare();
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos - CurY, Ypos + CurX);
    LCD_WriteRAM_Prepare();
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos - CurY, Ypos - CurX);
    LCD_WriteRAM_Prepare(); 
    LCD_WriteRAM(TextColor);
    if (D < 0)
    { 
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }*/
}


void LCD_DrawMonoPict(const uint32_t *Pict)
{
  uint32_t index = 0, i = 0;
  LCD_SetCursor(0, (LCD_PIXEL_WIDTH - 1)); 
  LCD_WriteRAM_Prepare();
  for(index = 0; index < 2400; index++)
  {
    for(i = 0; i < 32; i++)
    {
      if((Pict[index] & (1 << i)) == 0x00)
      {
        LCD_WriteRAM(BackColor);
      }
      else
      {
        LCD_WriteRAM(TextColor);
      }
    }
  }
}


void LCD_WriteBMP(uint32_t BmpAddress)
{
/*#if 0
  uint32_t index = 0, size = 0;
  size = *(__IO uint16_t *) (BmpAddress + 2);
  size |= (*(__IO uint16_t *) (BmpAddress + 4)) << 16;
  index = *(__IO uint16_t *) (BmpAddress + 10);
  index |= (*(__IO uint16_t *) (BmpAddress + 12)) << 16;
  size = (size - index)/2;
  BmpAddress += index;
  LCD_WriteReg(LCD_REG_3, 0x1008);
 
  LCD_WriteRAM_Prepare();
 
  for(index = 0; index < size; index++)
  {
    LCD_WriteRAM(*(__IO uint16_t *)BmpAddress);
    BmpAddress += 2;
  }

  LCD_WriteReg(LCD_REG_3, 0x1018);
#endif*/
}


void LCD_DrawFullRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  /*LCD_SetTextColor(TextColor);

  LCD_DrawLine(Xpos, Ypos, Width, LCD_DIR_HORIZONTAL);
  LCD_DrawLine(Xpos, (Ypos+Height), Width, LCD_DIR_HORIZONTAL);
  
  LCD_DrawLine(Xpos, Ypos, Height, LCD_DIR_VERTICAL);
  LCD_DrawLine((Xpos+Width-1), Ypos, Height, LCD_DIR_VERTICAL);

  Height--;
  Ypos++;

  LCD_SetTextColor(BackColor);

  while(Height--)
  {
    LCD_DrawLine(Xpos, Ypos++, Width, LCD_DIR_HORIZONTAL);    
  }

  LCD_SetTextColor(TextColor);*/
}


void LCD_DrawFullCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  /*int32_t  D;  
  uint32_t  CurX;
  uint32_t  CurY;
  
  D = 3 - (Radius << 1);

  CurX = 0;
  CurY = Radius;
  
  LCD_SetTextColor(BackColor);

  while (CurX <= CurY)
  {
    if(CurY > 0) 
    {
      LCD_DrawLine(Xpos - CurX, Ypos + CurY, 2*CurY, LCD_DIR_HORIZONTAL);
      LCD_DrawLine(Xpos + CurX, Ypos + CurY, 2*CurY, LCD_DIR_HORIZONTAL);
    }

    if(CurX > 0) 
    {
      LCD_DrawLine(Xpos - CurY, Ypos + CurX, 2*CurX, LCD_DIR_HORIZONTAL);
      LCD_DrawLine(Xpos + CurY, Ypos + CurX, 2*CurX, LCD_DIR_HORIZONTAL);
    }
    if (D < 0)
    { 
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }

  LCD_SetTextColor(TextColor);
  LCD_DrawCircle(Xpos, Ypos, Radius);*/
}

void LCD_DrawUniLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  /*int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, 
  curpixel = 0;
  
  deltax = ABS(x2 - x1);        
  deltay = ABS(y2 - y1);  
  x = x1;                      
  y = y1;                      
  
  if (x2 >= x1)                
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                         
  {
    xinc1 = -1;
    xinc2 = -1;
  }
  
  if (y2 >= y1)                 
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                        
  {
    yinc1 = -1;
    yinc2 = -1;
  }
  
  if (deltax >= deltay)         
  {
    xinc1 = 0;                  
    yinc2 = 0;                 
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;        
  }
  else                          
  {
    xinc2 = 0;                 
    yinc1 = 0;                 
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay; 
		
  }
  
  for (curpixel = 0; curpixel <= numpixels; curpixel++)
  {
    PutPixel(x, y);             
    num += numadd;              
    if (num >= den)            
    {
      num -= den;              
      x += xinc1;              
      y += yinc1;               
    }
    x += xinc2;                 
    y += yinc2;                
  }*/
}


void LCD_PolyLine(pPoint Points, uint16_t PointCount)
{
  /*int16_t X = 0, Y = 0;

  if(PointCount < 2)
  {
    return;
  }

  while(--PointCount)
  {
    X = Points->X;
    Y = Points->Y;
    Points++;
    LCD_DrawUniLine(X, Y, Points->X, Points->Y);
  }*/
}


static void LCD_PolyLineRelativeClosed(pPoint Points, uint16_t PointCount, uint16_t Closed)
{
  /*int16_t X = 0, Y = 0;
  pPoint First = Points;

  if(PointCount < 2)
  {
    return;
  }  
  X = Points->X;
  Y = Points->Y;
  while(--PointCount)
  {
    Points++;
    LCD_DrawUniLine(X, Y, X + Points->X, Y + Points->Y);
    X = X + Points->X;
    Y = Y + Points->Y;
  }
  if(Closed)
  {
    LCD_DrawUniLine(First->X, First->Y, X, Y);
  }  */
}

void LCD_ClosedPolyLine(pPoint Points, uint16_t PointCount)
{
  //LCD_PolyLine(Points, PointCount);
  //LCD_DrawUniLine(Points->X, Points->Y, (Points+PointCount-1)->X, (Points+PointCount-1)->Y);
}


void LCD_PolyLineRelative(pPoint Points, uint16_t PointCount)
{
  //LCD_PolyLineRelativeClosed(Points, PointCount, 0);
}


void LCD_ClosedPolyLineRelative(pPoint Points, uint16_t PointCount)
{
  //LCD_PolyLineRelativeClosed(Points, PointCount, 1);
}



void LCD_FillPolyLine(pPoint Points, uint16_t PointCount)
{
 
  /*uint16_t  nodes = 0, nodeX[MAX_POLY_CORNERS], pixelX = 0, pixelY = 0, i = 0,
  j = 0, swap = 0;
  uint16_t  IMAGE_LEFT = 0, IMAGE_RIGHT = 0, IMAGE_TOP = 0, IMAGE_BOTTOM = 0;

  IMAGE_LEFT = IMAGE_RIGHT = Points->X;
  IMAGE_TOP= IMAGE_BOTTOM = Points->Y;

  for(i = 1; i < PointCount; i++)
  {
    pixelX = POLY_X(i);
    if(pixelX < IMAGE_LEFT)
    {
      IMAGE_LEFT = pixelX;
    }
    if(pixelX > IMAGE_RIGHT)
    {
      IMAGE_RIGHT = pixelX;
    }
    
    pixelY = POLY_Y(i);
    if(pixelY < IMAGE_TOP)
    { 
      IMAGE_TOP = pixelY;
    }
    if(pixelY > IMAGE_BOTTOM)
    {
      IMAGE_BOTTOM = pixelY;
    }
  }
  
  LCD_SetTextColor(BackColor);  

  for (pixelY = IMAGE_TOP; pixelY < IMAGE_BOTTOM; pixelY++) 
  {  

    nodes = 0; j = PointCount-1;

    for (i = 0; i < PointCount; i++) 
    {
      if (((POLY_Y(i)<(double) pixelY) && (POLY_Y(j)>=(double) pixelY)) || \
          ((POLY_Y(j)<(double) pixelY) && (POLY_Y(i)>=(double) pixelY)))
      {
        nodeX[nodes++]=(int) (POLY_X(i)+((pixelY-POLY_Y(i))*(POLY_X(j)-POLY_X(i)))/(POLY_Y(j)-POLY_Y(i))); 
      }
      j = i; 
    }
  
    i = 0;
    while (i < nodes-1) 
    {
      if (nodeX[i]>nodeX[i+1]) 
      {
        swap = nodeX[i]; 
        nodeX[i] = nodeX[i+1]; 
        nodeX[i+1] = swap; 
        if(i)
        {
          i--; 
        }
      }
      else 
      {
        i++;
      }
    }
  

    for (i = 0; i < nodes; i+=2) 
    {
      if(nodeX[i] >= IMAGE_RIGHT) 
      {
        break;
      }
      if(nodeX[i+1] > IMAGE_LEFT) 
      {
        if (nodeX[i] < IMAGE_LEFT)
        {
          nodeX[i]=IMAGE_LEFT;
        }
        if(nodeX[i+1] > IMAGE_RIGHT)
        {
          nodeX[i+1] = IMAGE_RIGHT;
        }
        LCD_SetTextColor(BackColor);
        LCD_DrawLine(pixelY, nodeX[i+1], nodeX[i+1] - nodeX[i], LCD_DIR_HORIZONTAL);
        LCD_SetTextColor(TextColor);
        PutPixel(pixelY, nodeX[i+1]);
        PutPixel(pixelY, nodeX[i]);
      }
    }
  } 
  LCD_SetTextColor(TextColor);*/
}


#ifndef USE_Delay

static void delay(__IO uint32_t nCount)
{
  __IO uint32_t index = 0; 
  for(index = (10000 * nCount); index != 0; index--)
  {
  }
}
#endif 


/*********** Portions COPYRIGHT 2012 Embest Tech. Co., Ltd.*****END OF FILE****/
