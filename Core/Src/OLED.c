#include "oled.h"
#include "OLED_Font.h"

// ===================== 1:1复制标准库的I2C时序 =====================
static void I2C_Delay(void)
{
  // 延时 ~5us @72MHz，给 I2C 足够的数据建立/保持时间
  volatile uint32_t delay = 50;
  while (delay--);
}

static void OLED_W_SCL(uint8_t x)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, (GPIO_PinState)x);
  I2C_Delay();
}

static void OLED_W_SDA(uint8_t x)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, (GPIO_PinState)x);
  I2C_Delay();
}

// 1:1复制标准库的I2C起始
void OLED_I2C_Start(void)
{
  OLED_W_SDA(1);
  OLED_W_SCL(1);
  OLED_W_SDA(0);
  OLED_W_SCL(0);
}

// 1:1复制标准库的I2C停止
void OLED_I2C_Stop(void)
{
  OLED_W_SDA(0);
  OLED_W_SCL(1);
  OLED_W_SDA(1);
}

// 1:1复制标准库的I2C发送字节
void OLED_I2C_SendByte(uint8_t Byte)
{
  uint8_t i;
  for (i = 0; i < 8; i++)
  {
    OLED_W_SDA(!!(Byte & (0x80 >> i)));
    OLED_W_SCL(1);
    OLED_W_SCL(0);
  }
  OLED_W_SCL(1); // 额外的一个时钟，不处理应答
  OLED_W_SCL(0);
}

// 1:1复制标准库的写命令
void OLED_WriteCommand(uint8_t Command)
{
  OLED_I2C_Start();
  OLED_I2C_SendByte(0x78); // 你原来的地址，绝对正确
  OLED_I2C_SendByte(0x00); // 写命令
  OLED_I2C_SendByte(Command); 
  OLED_I2C_Stop();
}

// 1:1复制标准库的写数据
void OLED_WriteData(uint8_t Data)
{
  OLED_I2C_Start();
  OLED_I2C_SendByte(0x78);
  OLED_I2C_SendByte(0x40); // 写数据
  OLED_I2C_SendByte(Data);
  OLED_I2C_Stop();
}

// 1:1复制标准库的设置光标
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
  OLED_WriteCommand(0xB0 | Y);
  OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));
  OLED_WriteCommand(0x00 | (X & 0x0F));
}

// 1:1复制标准库的清屏
void OLED_Clear(void)
{  
  uint8_t i, j;
  for (j = 0; j < 8; j++)
  {
    OLED_SetCursor(j, 0);
    for(i = 0; i < 128; i++)
    {
      OLED_WriteData(0x00);
    }
  }
}

// 1:1复制标准库的显示字符
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{      	
  uint8_t i;
  OLED_SetCursor((Line - 1) * 2, Column * 8);
  for (i = 0; i < 8; i++)
  {
    OLED_WriteData(OLED_F8x16[Char - ' '][i]);
  }
  OLED_SetCursor((Line - 1) * 2 + 1, Column * 8);
  for (i = 0; i < 8; i++)
  {
    OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
  }
}

// 1:1复制标准库的显示字符串
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
  uint8_t i;
  for (i = 0; String[i] != '\0'; i++)
  {
    OLED_ShowChar(Line, Column + i, String[i]);
  }
}

// 1:1复制标准库的次方函数
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
  uint32_t Result = 1;
  while (Y--)
  {
    Result *= X;
  }
  return Result;
}

// 1:1复制标准库的显示数字
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
  uint8_t i;
  for (i = 0; i < Length; i++)							
  {
    OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
  }
}

// 1:1复制标准库的初始化
void OLED_Init(void)
{

  // 上电等待 100ms，确保 OLED 稳定
  HAL_Delay(100);

  // 引脚初始化，和标准库完全一样
  OLED_W_SCL(1);
  OLED_W_SDA(1);
  
  // 1:1复制标准库的初始化命令，一个都不差
  OLED_WriteCommand(0xAE);
  OLED_WriteCommand(0xD5);
  OLED_WriteCommand(0x80);
  OLED_WriteCommand(0xA8);
  OLED_WriteCommand(0x3F);
  OLED_WriteCommand(0xD3);
  OLED_WriteCommand(0x00);
  OLED_WriteCommand(0x40);
  OLED_WriteCommand(0xA1);
  OLED_WriteCommand(0xC8);
  OLED_WriteCommand(0xDA);
  OLED_WriteCommand(0x12);
  OLED_WriteCommand(0x81);
  OLED_WriteCommand(0xCF);
  OLED_WriteCommand(0xD9);
  OLED_WriteCommand(0xF1);
  OLED_WriteCommand(0xDB);
  OLED_WriteCommand(0x30);
  OLED_WriteCommand(0xA4);
  OLED_WriteCommand(0xA6);
  OLED_WriteCommand(0x8D);
  OLED_WriteCommand(0x14);
  OLED_WriteCommand(0xAF);
		
  OLED_Clear();
}