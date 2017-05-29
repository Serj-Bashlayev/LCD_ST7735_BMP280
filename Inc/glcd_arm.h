/*

*/
#ifndef _GLCD_ARM_H
#define _GLCD_ARM_H

#include "stm32f3xx_hal.h"
/**
  * @brief  Definition for OLED Interface software used pins (SPI3 used) 
  */
#define OLED_RES_Pin                    GPIO_PIN_0
#define OLED_RES_GPIO_Port              GPIOD
#define OLED_RES_GPIO_CLK_ENABLE()      __GPIOD_CLK_ENABLE()
#define OLED_RES_GPIO_CLK_DISABLE()     __GPIOD_CLK_DISABLE()

/* Reset Pin macro definition */
#define SPI_RES_LOW()   HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_RESET)
#define SPI_RES_HI()    HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_SET)

#define OLED_DC_Pin                     GPIO_PIN_11
#define OLED_DC_GPIO_Port               GPIOC
#define OLED_DC_GPIO_CLK_ENABLE()       __GPIOC_CLK_ENABLE()
#define OLED_DC_GPIO_CLK_DISABLE()      __GPIOC_CLK_DISABLE()

/* Data/Command Pin macro definition */
// use HAL
//#define SPI_DC_LOW()    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET)
//#define SPI_DC_HI()	    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET)
// fast HW access
#define SPI_DC_LOW()    OLED_DC_GPIO_Port->BSRRH = OLED_DC_Pin
#define SPI_DC_HI()     OLED_DC_GPIO_Port->BSRRL = OLED_DC_Pin

#define OLED_CS_Pin         GPIO_PIN_15
#define OLED_CS_GPIO_Port   GPIOA

#define OLED_SCLK_Pin       GPIO_PIN_10
#define OLED_SCLK_GPIO_Port GPIOC

#define OLED_SDIN_Pin       GPIO_PIN_12
#define OLED_SDIN_GPIO_Port GPIOC

void ST7735_Init(void);
void GLSD_drawPixel(int16_t x, int16_t y, uint16_t color);
void GLCD_SPI_MspInit(SPI_HandleTypeDef *hspi);
int8_t GLCD_DRIVER_DRAW_LINE(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint16_t color);
int8_t GLCD_DRIVER_FILL_FRAME(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint16_t color);
void* GLCD_DRIVER_FILL_AREA(int32_t x0, int32_t y0, int32_t x1, int32_t y1);
void ST7735_Clear(uint16_t color);
uint16_t RGBto565(uint8_t R, uint8_t G, uint8_t B);
void ST7735_InvDisp(uint8_t i);

// BGR color filter panel (see MADCTL (36h): Memory Data Access Control)
//#define ST7735_BGR_ORDER
#define ST7735_WIDTH   128
#define ST7735_HEIGHT  160


// some ST7735 command
#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

#endif //_GLCD_ARM_H


