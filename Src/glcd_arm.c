/*****************************************************************************************
*****************************************************************************************/
#include "glcd_arm.h"

extern void Error_Handler(void);

static void _GLCD_HW_Init(void);
static void _GLCD_TransmitByte(uint8_t data);
static void _GLCD_TransmitWord(uint16_t data);
static void _GLCD_WriteCommand(uint8_t cmd);
static void _GLCD_WriteData(uint8_t data);
static void _GLCD_SetAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1,uint8_t y1);

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;

/**
 * @brief  Инициализация IO, SPI3 (4-wire mode) для работы с ST7735
 *         PA15  ------> SPI3_NSS    OLED_CS
 *         PC10  ------> SPI3_SCK    OLED_SCLK
 *         PC12  ------> SPI3_MOSI   OLED_SDIN
 *         PD0   ------> GPIO_Output OLED_RES
 *         PC11  ------> GPIO_Output OLED_DC
 */
static void _GLCD_HW_Init(void){
  GPIO_InitTypeDef GPIO_InitStruct;

  OLED_RES_GPIO_CLK_ENABLE();
  OLED_DC_GPIO_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_SET);

  /* Configure GPIO pin : OLED_RES_Pin */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pin = OLED_RES_Pin;
  HAL_GPIO_Init(OLED_RES_GPIO_Port, &GPIO_InitStruct);
  /* Configure GPIO pin : OLED_DC_Pin */
  GPIO_InitStruct.Pin = OLED_DC_Pin;
  HAL_GPIO_Init(OLED_DC_GPIO_Port, &GPIO_InitStruct);

  /* SPI init */
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLED;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_SPI_ENABLE(&hspi3);
}


/**
 * @brief вызывается из HAL_SPI_Init()
 * Init the low level hardware : GPIO, CLOCK, NVIC...
 *
 * @param hspi: SPI handle
 */
void GLCD_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  assert_param(hspi->Instance == SPI3);

  /* Peripheral clock enable */
  __SPI3_CLK_ENABLE();

  /**SPI3 GPIO Configuration
  PA15     ------> SPI3_NSS
  PC10     ------> SPI3_SCK
  PC12     ------> SPI3_MOSI
  */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  /* Configure GPIO pin : SPI3_NSS */
  GPIO_InitStruct.Pin = OLED_CS_Pin;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(OLED_CS_GPIO_Port, &GPIO_InitStruct);
  /* Configure GPIO pin : SPI3_SCK & SPI3_MOSI */
  GPIO_InitStruct.Pin = OLED_SCLK_Pin|OLED_SDIN_Pin;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(OLED_SCLK_GPIO_Port, &GPIO_InitStruct);

  /* SPI interrupt init*/
  HAL_NVIC_SetPriority(SPI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(SPI3_IRQn);
}


/**
 * @brief Вывод байта в SPI.
 *        Software Out - 10uS/byte (8 Imp SCLK. CPHA=0,CPOL=0)
 *        Hardware - 3uS/byte (8 Imp SCLK. CPHA=0,CPOL=0. Tscyc=0.44uS)
 *        Interrupt - 3uS/byte (8 Imp SCLK. CPHA=0,CPOL=0. Tscyc=0.44uS) Интервал между байтами 12uS
 *        DMA -  3uS/byte (8 Imp SCLK. CPHA=0,CPOL=0. Tscyc=0.44uS)  Интервал между байтами 12uS
 * @param  data 8-bit data
 */
static void _GLCD_TransmitByte(uint8_t data)
{
  while (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_TXE) != SET);

	*(__IO uint8_t*)&(hspi3.Instance->DR) = data;
}


static void _GLCD_TransmitWord(uint16_t data)
{
  // В режиме передачи 8бит и записи в DR 16 бит меняетя порядок выхода
  // младший байт вызодит вторым (см. Packing data in FIFO for transmission and reception)
  data = (data << 8) | (data >>8);
  while (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_TXE) != SET);
  *(__IO uint16_t*)&(hspi3.Instance->DR) = data;
}


/**
 * @brief  Запись cmd в command register (DC Low)
 * @param  cmd 8-bit data
 */
static void _GLCD_WriteCommand(uint8_t cmd)
{
  if (GPIOC->ODR & GPIO_PIN_11)
  { // DC hi
    while (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_TXE) != SET);
    while (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_BSY) != RESET);
    SPI_DC_LOW();
  }
  _GLCD_TransmitByte(cmd);
}


/**
 * @brief  Запись data в command register (DC High)
 * @param  data 8-bit data
 */
static void _GLCD_WriteData(uint8_t data) {
  if (!(GPIOC->ODR & GPIO_PIN_11))
  { // DC low
    while (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_TXE) != SET);
    while (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_BSY) != RESET);
    SPI_DC_HI();
  }
  _GLCD_TransmitByte(data);
}


static void _GLCD_SetAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
  uint8_t colstart = 0,
          rowstart = 0;

  _GLCD_WriteCommand(ST7735_CASET); // Column addr set
  _GLCD_WriteData(0x00);
  _GLCD_WriteData(x0 + colstart);   // XSTART
  _GLCD_WriteData(0x00);
  _GLCD_WriteData(x1 + colstart);   // XEND

  _GLCD_WriteCommand(ST7735_RASET); // Row addr set
  _GLCD_WriteData(0x00);
  _GLCD_WriteData(y0 + rowstart);   // YSTART
  _GLCD_WriteData(0x00);
  _GLCD_WriteData(y1 + rowstart);   // YEND

  _GLCD_WriteCommand(ST7735_RAMWR); // write to RAM
}




void ST7735_Init()
{
  // Hardware initialization
  _GLCD_HW_Init();

  // Screen initialization
  // подсмотрел в Arduino library for the Adafruit 1.8" SPI display (Adafruit_ST7735.cpp)
  SPI_RES_HI();
  HAL_Delay(100); // arduino 500 ms, Power On
  SPI_RES_LOW();
  HAL_Delay(1);   // arduino 500 ms, datasheet: Tresw = 10 us (min)
  SPI_RES_HI();
  HAL_Delay(120); // arduino 500 ms, datasheet: Trest = 120 ms (max)

//  ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
//    150,                    //     150 ms delay
  _GLCD_WriteCommand(ST7735_SWRESET);
  HAL_Delay(120); // arduino 150 ms, pdf: 120 ms

//  ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
//    255,                    //     500 ms delay
  _GLCD_WriteCommand(ST7735_SLPOUT);
  HAL_Delay(120); // arduino 500 ms, pdf: 120 ms

//  ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
//    0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
  _GLCD_WriteCommand(ST7735_FRMCTR1);
  _GLCD_WriteData(0x01);
  _GLCD_WriteData(0x2C);
  _GLCD_WriteData(0x2D);

//  ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
//    0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
  _GLCD_WriteCommand(ST7735_FRMCTR2);
  _GLCD_WriteData(0x01);
  _GLCD_WriteData(0x2C);
  _GLCD_WriteData(0x2D);

//  ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
//    0x01, 0x2C, 0x2D,       //     Dot inversion mode
//    0x01, 0x2C, 0x2D,       //     Line inversion mode
  _GLCD_WriteCommand(ST7735_FRMCTR3);
  _GLCD_WriteData(0x01);
  _GLCD_WriteData(0x2C);
  _GLCD_WriteData(0x2D);
  _GLCD_WriteData(0x01);
  _GLCD_WriteData(0x2C);
  _GLCD_WriteData(0x2D);

//  ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
//    0x07,                   //     No inversion
  _GLCD_WriteCommand(ST7735_INVCTR);
  _GLCD_WriteData(0x07);

//  ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
//    0xA2,
//    0x02,                   //     -4.6V
//    0x84,                   //     AUTO mode
  _GLCD_WriteCommand(ST7735_PWCTR1);
  _GLCD_WriteData(0xA2);
  _GLCD_WriteData(0x02);
  _GLCD_WriteData(0x84);

//  ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
//    0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
  _GLCD_WriteCommand(ST7735_PWCTR2);
  _GLCD_WriteData(0xC5);

//  ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
//    0x0A,                   //     Opamp current small
//    0x00,                   //     Boost frequency
  _GLCD_WriteCommand(ST7735_PWCTR3);
  _GLCD_WriteData(0x0A);
  _GLCD_WriteData(0x00);

//  ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
//    0x8A,                   //     BCLK/2, Opamp current small & Medium low
//    0x2A,
  _GLCD_WriteCommand(ST7735_PWCTR4);
  _GLCD_WriteData(0x8A);
  _GLCD_WriteData(0x2A);

//  ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
//    0x8A, 0xEE,
  _GLCD_WriteCommand(ST7735_PWCTR5);
  _GLCD_WriteData(0x8A);
  _GLCD_WriteData(0xEE);

//  ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
//    0x0E,
  _GLCD_WriteCommand(ST7735_VMCTR1);
  _GLCD_WriteData(0x0E);

  //  ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
  _GLCD_WriteCommand(ST7735_INVOFF);

//  ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
//    0xC8,                   //     row addr/col addr, bottom to top refresh
  _GLCD_WriteCommand(ST7735_MADCTL);
  #ifdef ST7735_BGR_ORDER
    _GLCD_WriteData(0xC8);
  #else
    _GLCD_WriteData(0xC0);
  #endif

//  ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
//    0x05 },                 //     16-bit color
  _GLCD_WriteCommand(ST7735_COLMOD);
  _GLCD_WriteData(0x05);

//Rcmd2red[] = {              // Init for 7735R, part 2 (red tab only)
//  2,                        //  2 commands in list:
//  ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
//    0x00, 0x00,             //     XSTART = 0
//    0x00, 0x7F,             //     XEND = 127
  _GLCD_WriteCommand(ST7735_CASET);
  _GLCD_WriteData(0x00);
  _GLCD_WriteData(0x00);
  _GLCD_WriteData(0x00);
  _GLCD_WriteData(0x7F);

//  ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
//    0x00, 0x00,             //     XSTART = 0
//    0x00, 0x9F },           //     XEND = 159
  _GLCD_WriteCommand(ST7735_RASET);
  _GLCD_WriteData(0x00);
  _GLCD_WriteData(0x00);
  _GLCD_WriteData(0x00);
  _GLCD_WriteData(0x9F);

//Rcmd3[] = {                 // Init for 7735R, part 3 (red or green tab)
//  4,                        //  4 commands in list:
//  ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
//    0x02, 0x1c, 0x07, 0x12,
  _GLCD_WriteCommand(ST7735_GMCTRP1);
  _GLCD_WriteData(0x02);
  _GLCD_WriteData(0x1C);
  _GLCD_WriteData(0x07);
  _GLCD_WriteData(0x12);
//    0x37, 0x32, 0x29, 0x2d,
  _GLCD_WriteData(0x37);
  _GLCD_WriteData(0x32);
  _GLCD_WriteData(0x29);
  _GLCD_WriteData(0x2D);
//    0x29, 0x25, 0x2B, 0x39,
  _GLCD_WriteData(0x29);
  _GLCD_WriteData(0x25);
  _GLCD_WriteData(0x2B);
  _GLCD_WriteData(0x39);
//    0x00, 0x01, 0x03, 0x10,
  _GLCD_WriteData(0x00);
  _GLCD_WriteData(0x01);
  _GLCD_WriteData(0x03);
  _GLCD_WriteData(0x10);

//  ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
//    0x03, 0x1d, 0x07, 0x06,
  _GLCD_WriteCommand(ST7735_GMCTRN1);
  _GLCD_WriteData(0x03);
  _GLCD_WriteData(0x1D);
  _GLCD_WriteData(0x07);
  _GLCD_WriteData(0x06);
//    0x2E, 0x2C, 0x29, 0x2D,
  _GLCD_WriteData(0x2E);
  _GLCD_WriteData(0x2C);
  _GLCD_WriteData(0x29);
  _GLCD_WriteData(0x2D);
//    0x2E, 0x2E, 0x37, 0x3F,
  _GLCD_WriteData(0x2E);
  _GLCD_WriteData(0x2E);
  _GLCD_WriteData(0x37);
  _GLCD_WriteData(0x3F);
//    0x00, 0x00, 0x02, 0x10,
  _GLCD_WriteData(0x00);
  _GLCD_WriteData(0x00);
  _GLCD_WriteData(0x02);
  _GLCD_WriteData(0x10);

//  ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
//    10,                     //     10 ms delay
  _GLCD_WriteCommand(ST7735_NORON);
  // HAL_Delay(10); // pdf: w/o delay

//  ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
//    100 },                  //     100 ms delay
  _GLCD_WriteCommand(ST7735_DISPON);
  //HAL_Delay(100); // pdf: w/o delay


}


/**
 * @brief Передача по SPI определённого числа WORD.
 * В SPI отключаем аппаратное управление CS(NSS). Экономим 2 такта на передаче байта (10clc -> 8ckc).
 * Данные передаются без разрывов.
 * Перед передачей CS опускается в 0.
 * После передачи восстанавливается аппаратное управление CS(NSS).
 *
 * @param data слово (16ит) для передачи
 * @param count количество передаваемых слов
 */
void GLCD_Transmit_Sequence(uint16_t data, uint32_t count)
{
  uint32_t copy_PURDR, copy_CR2;

  assert_param(count == 0);

  // В режиме передачи 8бит и записи в DR 16 бит меняетя порядок выхода
  // младший байт вызодит вторым (см. Packing data in FIFO for transmission and reception)
  data = (data << 8) | (data >> 8);

  //***************************
  // disable NSS pulse mode (speedup - 2 clc)
  // NSS pin LOW during SPI activ
  while (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_TXE) != SET);
  while (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_BSY) != RESET);
  // CS pin (PA15) Pull-up activation
  copy_PURDR = GPIOA->PUPDR;
  GPIOA->PUPDR = copy_PURDR | (GPIO_PULLUP << (15 * 2));
  // No NSS pulse
  __HAL_SPI_DISABLE(&hspi3);
  copy_CR2 = hspi3.Instance->CR2;
  hspi3.Instance->CR2 = copy_CR2 & ~SPI_CR2_NSSP;
  __HAL_SPI_ENABLE(&hspi3);
  // CS pin No Pull-up or Pull-down activation
  GPIOA->PUPDR = copy_PURDR;
  //***************************

  while (count--)
  {
    while (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_TXE) != SET);
    hspi3.Instance->DR = data;
  }
  //***************************
  // restore NSS pulse mode
  while (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_TXE) != SET);
  while (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_BSY) != RESET);
  // CS pin Pull-up activation
  GPIOA->PUPDR |= GPIO_PULLUP << (15 * 2);
  // restore NSS pulse management
  __HAL_SPI_DISABLE(&hspi3);
  hspi3.Instance->CR2 |=  copy_CR2 & SPI_CR2_NSSP;
  __HAL_SPI_ENABLE(&hspi3);
  // CS pin No Pull-up or Pull-down activation
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (15 * 2));
  //***************************
}


int8_t GLCD_DRIVER_DRAW_LINE(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint16_t color)
{

  /* horizontal line */
  if ( y0 == y1 )
  {
    GLCD_DRIVER_FILL_FRAME(x0, y0, x1, y1, color);
  }
  /* vertical line */
  else if ( x0 == x1 )
  {
    GLCD_DRIVER_FILL_FRAME(x0, y0, x1, y1, color);
  }
  else
  {
    return -1;
  }

  return 0;
}


int8_t GLCD_DRIVER_FILL_FRAME(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint16_t color)
{
  int32_t tmp;

  if (x0 > x1)
  {
    tmp = x0;
    x0 = x1;
    x1 = tmp;
  }
  if (y0 > y1)
  {
    tmp = y0;
    y0 = y1;
    y1 = tmp;
  }

  // rudimentary clipping (drawChar w/big text requires this)
  if ((x0 >= 128) || (y0 >= 160))
    return 0;
  if (x1 >= 128)
    x1 = 127;
  if (y1 >= 160)
    y1 = 159;

  _GLCD_SetAddrWindow(x0, y0, x1, y1);

  while (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_BSY) != RESET);
  SPI_DC_HI();

  GLCD_Transmit_Sequence(color, (x1 - x0 + 1) * (y1 - y0 + 1));

  return 0;
}


void* GLCD_DRIVER_FILL_AREA(int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
  _GLCD_SetAddrWindow(x0, y0, x1, y1);

  while (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_BSY) != RESET);
  SPI_DC_HI();

  return (&_GLCD_TransmitWord);
}


/**
 * @brief Установка пикселя
 *
 * @param x Х
 * @param y Y
 * @param color цвет RGB565 (16bit)
 */
void GLSD_drawPixel(int16_t x, int16_t y, uint16_t color)
{
  if ((uint16_t)x >= 128 || (uint16_t)y >= 160)
    return;

  _GLCD_SetAddrWindow(x, y, x + 1, y + 1);

  //*rsport |=  rspinmask;
  //*csport &= ~cspinmask;
  _GLCD_WriteData(color >> 8);
  _GLCD_WriteData(color);
  //*csport |= cspinmask;
}


uint16_t RGBto565 (uint8_t R, uint8_t G, uint8_t B)
{
  return ((R & 0xF8) << 8 | (G & 0xFC) << 3 | (B & 0xF8) >> 3);
}


/**
 * @brief Смена R<>B в формате RGB(565)
 *
 * @param x цвет в формате RGB565
 *
 * @return uint16_t BGR565
 *

inline uint16_t swapcolor(uint16_t x) {
  return (x << 11) | (x & 0x07E0) | (x >> 11);
}
*/


static void _FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  // rudimentary clipping (drawChar w/big text requires this)
  if ((x >= 128) || (y >= 160))
    return;
  if ((x + w - 1) >= 128)
    w = 128  - x;
  if ((y + h - 1) >= 160)
    h = 160 - y;

  _GLCD_SetAddrWindow(x, y, x + w - 1, y + h - 1);

  while (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_BSY) != RESET);
  SPI_DC_HI();

  GLCD_Transmit_Sequence(color, (uint32_t)(h * w));
}


void ST7735_Clear(uint16_t color)
{
  _FillRect(0, 0,  128, 160, color);
}


void ST7735_InvDisp(uint8_t i)
{
  _GLCD_WriteCommand(i ? ST7735_INVON : ST7735_INVOFF);
}

