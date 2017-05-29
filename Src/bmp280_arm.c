/***********************************************************************
 File Name    : 'bmp280.c'
 Title        :
 Description  : Free to use, free to change, free to delete
 
 CPU STM32F303
 F = 72MHz
 bmp280_compensate_temperature_int32  2 uSec
 bmp280_compensate_temperature_float  3 uSec
 bmp280_compensate_pressure_int32     4 uSec
 bmp280_compensate_pressure_int64     62 uSec
 bmp280_compensate_pressure_float     8 uSec
 
 Author       : Serj Balabay (phreak_ua@yahoo.com)
 Created      : 18/05/2017
 Revised      :
 Version      : 1.0
 Target MCU   : STM32
 Compiler     : ARM Compiler v5.04 for µVision armcc
 Editor Tabs  : 2
***********************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "bmp280.h"
#include "profiling.h"

/* Private variables ---------------------------------------------------------*/
struct bmp280_t bmp280;

#define	SPI_READ	0x80
#define SPI_WRITE	0x7F
#define SPI_BUFFER_LEN 5

/* Private function prototypes -----------------------------------------------*/
static int8_t BMP280_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
static int8_t BMP280_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
extern SPI_HandleTypeDef hspi1;
extern struct {
  int32_t v_actual_temp_s32;
  int32_t v_actual_press_u32;
  int32_t v_actual_temp_combined_s32;
  int32_t v_actual_press_combined_u32;
  uint32_t actual_press_u64;
  float v_actual_temp_float;
  float v_actual_press_float;
  double v_actual_temp_double;
  double v_actual_press_double;
  int32_t err_cnt;
} bmp_data;


int8_t BMP280_Begin(void)
{
  int8_t com_rslt;

  bmp280.bus_write = BMP280_SPI_bus_write;
  bmp280.bus_read = BMP280_SPI_bus_read;
  bmp280.delay_msec = HAL_Delay;

  com_rslt = bmp280_init(&bmp280);

  if (com_rslt != 0)
  {
    return -1;
  }

  /*	For reading the pressure and temperature data it is required to
   *	set the work mode
   *	The measurement period in the Normal mode is depends on the setting of
   *	over sampling setting of pressure, temperature and standby time
   *
   *    OSS                        pressure OSS    temperature OSS
   *    ultra low power            x1            x1
   *    low power                  x2            x1
   *    standard resolution        x4            x1
   *    high resolution            x8            x2
   *    ultra high resolution     x16            x2
   */
  com_rslt += bmp280_set_work_mode(BMP280_ULTRA_HIGH_RESOLUTION_MODE);

  com_rslt += bmp280_set_filter(BMP280_FILTER_COEFF_16);

  // Set CTRL_MEAS REG [0xF4] 0000 0011
  com_rslt += bmp280_set_power_mode(BMP280_NORMAL_MODE);

  com_rslt += bmp280_set_standby_durn(BMP280_STANDBY_TIME_500_MS);


  return com_rslt;
}


float BMP280_ReadTemperature(void)
{
  int32_t uncomp_temp;
  uint8_t array[3]= {0x55,0x55,0x55};

  BMP280_SPI_bus_read(0, BMP280_TEMPERATURE_MSB_REG, array, 3);

  uncomp_temp = (int32_t)((array[0] << 12) | (array[1] << 4) | (array[2] >> 4));

  return bmp280_compensate_temperature_float(uncomp_temp);
}


float BMP280_ReadPressure(void)
{
  int32_t uncomp_press;
  int32_t uncomp_temp;
  uint8_t array[6] = {0x55,0x55,0x55,0x55,0x55,0x55};
  float   pressure;

  BMP280_SPI_bus_read(0, BMP280_PRESSURE_MSB_REG, array, 6);

  uncomp_temp = (int32_t)((array[3] << 12) | (array[4] << 4) | (array[5] >> 4));

profiling_start("float");
  bmp280_compensate_temperature_float(uncomp_temp);
profiling_event("compensate_temperature_float");
  uncomp_press = (int32_t)((array[0] << 12) | (array[1] << 4) | (array[2] >> 4));

profiling_event("---");
   pressure = bmp280_compensate_pressure_float(uncomp_press);
profiling_event("compensate_pressure_float");
profiling_stop();
  return pressure;
}


/**
 * @brief The function is used as SPI bus read
 *
 * @param dev_addr The device address of the sensor
 * @param reg_addr Address of the first register, where data is going to be read
 * @param reg_data This is the data read from the sensor, which is held in an array
 * @param cnt The no of data to be read
 *
 * @return Status of the SPI read
 */
static int8_t BMP280_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
  int8_t iError = 0;
  HAL_StatusTypeDef ret;

  reg_addr |= SPI_READ;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  ret = HAL_SPI_Transmit(&hspi1, &reg_addr, 1, 2);
  if (ret != HAL_OK)
    iError = -1;
  ret = HAL_SPI_Receive(&hspi1, reg_data, cnt, 2);
  if (ret != HAL_OK)
    iError = -1;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return iError;
}


/**
 * @brief The function is used as SPI bus write
 *
 * @param dev_addr The device address of the sensor
 * @param reg_addr Address of the first register, where data is to be written
 * @param reg_data It is a value held in the array, which is written in the register
 * @param cnt The no of bytes of data to be written
 *
 * @return Status of the SPI write
 */
static int8_t BMP280_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
  HAL_StatusTypeDef ret;
 	int8_t iError = 0;
  uint8_t array[SPI_BUFFER_LEN * 2];
  uint8_t stringpos;
  uint8_t index = 0;

  if (cnt * 2 > sizeof(array))
    return -1;

  for (stringpos = 0; stringpos < cnt; stringpos++)
  {
    array[index++] = (reg_addr++) & SPI_WRITE;
    array[index++] = *(reg_data + stringpos);
  }

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  ret = HAL_SPI_Transmit(&hspi1, array, cnt * 2, 100);
  if (ret != HAL_OK)
    iError = -1;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return iError;
}


float bmp280_compensate_temperature_float(s32 v_uncomp_temperature_s32)
{
  float v_x1_u32r;
  float v_x2_u32r;
  float temperature;

  /* calculate x1*/
  v_x1_u32r = (((float)v_uncomp_temperature_s32) / 16384.0f - ((float)bmp280.calib_param.dig_T1) / 1024.0f) * ((float)bmp280.calib_param.dig_T2);
  /* calculate x2*/
  v_x2_u32r = ((((float)v_uncomp_temperature_s32) / 131072.0f - ((float)bmp280.calib_param.dig_T1) / 8192.0f) *
               (((float)v_uncomp_temperature_s32) / 131072.0f - ((float)bmp280.calib_param.dig_T1) / 8192.0f)) *
                ((float)bmp280.calib_param.dig_T3);
  /* calculate t_fine*/
  bmp280.calib_param.t_fine = (s32)(v_x1_u32r + v_x2_u32r);
  /* calculate true pressure*/
  temperature = (v_x1_u32r + v_x2_u32r) / 5120.0f;

  return temperature;
}


float bmp280_compensate_pressure_float(s32 v_uncomp_pressure_s32)
{
  float v_x1_u32r;
  float v_x2_u32r;
  float pressure;

  v_x1_u32r = ((float)bmp280.calib_param.t_fine / 2.0f) - 64000.0f;
  v_x2_u32r = v_x1_u32r * v_x1_u32r *
              ((float)bmp280.calib_param.dig_P6) / 32768.0f;
  v_x2_u32r = v_x2_u32r + v_x1_u32r *
              ((float)bmp280.calib_param.dig_P5) * 2.0f;
  v_x2_u32r = (v_x2_u32r / 4.0f) +
              (((float)bmp280.calib_param.dig_P4) * 65536.0f);
  v_x1_u32r = (((float)bmp280.calib_param.dig_P3) *
               v_x1_u32r * v_x1_u32r / 524288.0f +
               ((float)bmp280.calib_param.dig_P2) * v_x1_u32r) / 524288.0f;
  v_x1_u32r = (1.0f + v_x1_u32r / 32768.0f) *
              ((float)bmp280.calib_param.dig_P1);
  pressure = 1048576.0f - (float)v_uncomp_pressure_s32;
  /* Avoid exception caused by division by zero */
  if ((v_x1_u32r > 0) || (v_x1_u32r < 0))
    pressure = (pressure - (v_x2_u32r / 4096.0f)) * 6250.0f / v_x1_u32r;
  else
    return BMP280_INVALID_DATA;

  v_x1_u32r = ((float)bmp280.calib_param.dig_P9) * pressure * pressure / 2147483648.0f;
  v_x2_u32r = pressure * ((float)bmp280.calib_param.dig_P8) / 32768.0f;
  pressure = pressure + (v_x1_u32r + v_x2_u32r + ((float)bmp280.calib_param.dig_P7)) / 16.0f;

  return pressure;
}


uint32_t BMP280_ReadPressure_int64(void)
{
  s32 uncomp_temperature_s32;
  s32 uncomp_pressure_s32;
  s32 comp_temperature_s32;
  u32 comp_pressure_u32;

  bmp280_read_uncomp_temperature(&uncomp_temperature_s32);

profiling_start("int32 vs int64");
  comp_temperature_s32 = bmp280_compensate_temperature_int32(uncomp_temperature_s32);
profiling_event("compensate_temperature_int32");
  bmp_data.v_actual_temp_s32 = comp_temperature_s32;


  bmp280_read_uncomp_pressure(&uncomp_pressure_s32);

profiling_event("---");
  comp_pressure_u32 = bmp280_compensate_pressure_int64(uncomp_pressure_s32);
profiling_event("compensate_pressure_int64");
  bmp_data.v_actual_press_u32 = bmp280_compensate_pressure_int32(uncomp_pressure_s32);
profiling_event("compensate_pressure_int32");
profiling_stop();

  return comp_pressure_u32;
}

