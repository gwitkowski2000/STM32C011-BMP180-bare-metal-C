/* BMP180 STM32C011F6 example / I2C1 test
 * G. Witkowski 2023
 * gwitkowski2000@gmail.com
 * Based on Adafruit BMP085 C++ library
 *
 * For more precise altitude and sea level pressure enter your data
 * in readSealevelPressure() and readAltitude() function definitions 
 *
 * Pinout:
 * SDA: GPIO C14
 * SCL: GPIO B7
 */

#include "bmp180.h"
#include <stm32c011xx.h>
#include <math.h>

int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;
uint8_t oversampling;

volatile uint64_t tick = 0;

void delay_ms(uint64_t ms){
	tick = 0;
	while(tick < ms){
		__NOP();
	}
}

void bmp180_init(void){
	RCC->IOPENR |= RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOCEN;
	RCC->APBENR1 |= RCC_APBENR1_I2C1EN;
	
	// Select alternate function mode
	GPIOB->MODER &= ~GPIO_MODER_MODE7_0;
	GPIOC->MODER &= ~GPIO_MODER_MODE14_0;
	
	// Select AF14 for B7 and C14
	GPIOB->AFR[0] |= 0xE0000000;
	GPIOC->AFR[1] |= 0x0E000000;
	
	// Set output type as open drain for B7 and C14
	GPIOB->OTYPER |= GPIO_OTYPER_OT7;
	GPIOC->OTYPER |= GPIO_OTYPER_OT14;
	
	// Enable pull-ups on B7 and C14
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD7_0;
	GPIOC->PUPDR |= GPIO_PUPDR_PUPD14_0;
	
	I2C1->TIMINGR = 0x04;
	
	I2C1->CR1 |= I2C_CR1_PE;
		
  //if (read8(0xD0) != 0x55);
  /* read calibration data */
  ac1 = read16(BMP180_CAL_AC1);
  ac2 = read16(BMP180_CAL_AC2);
  ac3 = read16(BMP180_CAL_AC3);
  ac4 = read16(BMP180_CAL_AC4);
  ac5 = read16(BMP180_CAL_AC5);
  ac6 = read16(BMP180_CAL_AC6);

  b1 = read16(BMP180_CAL_B1);
  b2 = read16(BMP180_CAL_B2);

  mb = read16(BMP180_CAL_MB);
  mc = read16(BMP180_CAL_MC);
  md = read16(BMP180_CAL_MD);

}

int32_t computeB5(int32_t UT) {
  int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
  int32_t X2 = ((int32_t)mc << 11) / (X1 + (int32_t)md);
  return X1 + X2;
}

uint16_t readRawTemperature(void) {
  write8(BMP180_CONTROL, BMP180_READTEMPCMD);
  delay_ms(5);
  return read16(BMP180_TEMPDATA);
}

uint32_t readRawPressure(void) {
  uint32_t raw;

  write8(BMP180_CONTROL, BMP180_READPRESSURECMD + (oversampling << 6));

  if (oversampling == BMP180_ULTRALOWPOWER)
    delay_ms(5);
  else if (oversampling == BMP180_STANDARD)
    delay_ms(8);
  else if (oversampling == BMP180_HIGHRES)
    delay_ms(14);
  else
    delay_ms(26);

  raw = read16(BMP180_PRESSUREDATA);

  raw <<= 8;
  raw |= read8(BMP180_PRESSUREDATA + 2);
  raw >>= (8 - oversampling);

  return raw;
}

int32_t readPressure(void) {
  int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
  uint32_t B4, B7;

  UT = readRawTemperature();
  UP = readRawPressure();

  B5 = computeB5(UT);


  // do pressure calcs
  B6 = B5 - 4000;
  X1 = ((int32_t)b2 * ((B6 * B6) >> 12)) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)ac1 * 4 + X3) << oversampling) + 2) / 4;


  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
  B7 = ((uint32_t)UP - B3) * (uint32_t)(50000UL >> oversampling);


  if (B7 < 0x80000000) {
    p = (B7 * 2) / B4;
  } else {
    p = (B7 / B4) * 2;
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;

  p = p + ((X1 + X2 + (int32_t)3791) >> 4);

  return p;
}

int32_t readSealevelPressure(void){
	// altitude in my case
	float altitude_meters = 129;
  float pressure = readPressure();
  return (int32_t)(pressure / pow(1.0 - altitude_meters / 44330, 5.255));
}

float readTemperature(void) {
  int32_t UT, B5;
  float temp;

  UT = readRawTemperature();

  B5 = computeB5(UT);
  temp = (B5 + 8) >> 4;
  temp /= 10;

  return temp;
}

float readAltitude(void){
	// sea level pressure in my case
	float sealevelPressure = 100454;
  float altitude;

  float pressure = readPressure();

  altitude = 44330 * (1.0 - pow(pressure / sealevelPressure, 0.1903));

  return altitude;
}


uint8_t read8(uint8_t a) {
  uint8_t ret;
	I2C1->CR2 = (1 << I2C_CR2_NBYTES_Pos) | (BMP180_I2CADDR << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TXIS) == 0){
		__NOP();
	}
	
	I2C1->TXDR = a;
	
	while((I2C1->ISR & I2C_ISR_TC) == 0){
		__NOP();
	}
	
	I2C1->CR2 = (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | (BMP180_I2CADDR << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_RD_WRN | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_RXNE) == 0){
		__NOP();
	}
	ret = I2C1->RXDR;
	
  return ret;
}

uint16_t read16(uint8_t a) {
  uint8_t retbuf[2];
  uint16_t ret;


	I2C1->CR2 = (1 << I2C_CR2_NBYTES_Pos) | (BMP180_I2CADDR << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TXIS) == 0){
		__NOP();
	}
	
	I2C1->TXDR = a;
	
	while((I2C1->ISR & I2C_ISR_TC) == 0){
		__NOP();
	}
	
	I2C1->CR2 = (2 << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | (BMP180_I2CADDR << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_RD_WRN | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_RXNE) == 0){
	
	}
	retbuf[0] = I2C1->RXDR;
	

	while((I2C1->ISR & I2C_ISR_RXNE) == 0){
	
	}
	retbuf[1] = I2C1->RXDR;
		

  ret = retbuf[1] | (retbuf[0] << 8);
	I2C1->ICR |= I2C_ICR_STOPCF;
  return ret;
}



void write8(uint8_t addr, uint8_t data){
	I2C1->CR2 = (2 << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | (BMP180_I2CADDR << (I2C_CR2_SADD_Pos + 1)) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TXIS) == 0){
		__NOP();
	}
	I2C1->TXDR = addr;
	while((I2C1->ISR & I2C_ISR_TXIS) == 0){
		__NOP();
	}
	I2C1->TXDR = data;
	
}

void SysTick_Handler(void){
	tick++;
}