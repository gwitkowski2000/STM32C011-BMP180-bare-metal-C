/* BMP180 STM32C011F6 example / I2C1 test
 * G. Witkowski 2023
 * gwitkowski2000@gmail.com
 * Based on Adafruit BMP085 C++ library
 
 * Pinout:
 * SDA: GPIO C14
 * SCL: GPIO B7
 */

#include "stm32c0xx.h"
#include "bmp180.h"

uint8_t read;
uint16_t read2bytes;
float temperature;
float pressure;
float altitude;
float seaLevelPressure;
uint64_t main_counter = 0;

int main() {
  SysTick_Config(SystemCoreClock / 1000);
  bmp180_init();

  temperature = readTemperature();
  while (1) {	
    // below command should return 0x55 (BMP180 datasheet) and can be used to check if communcation works properly
    //read = read8(0xD0);
    temperature = readTemperature();
    delay_ms(50);
    pressure = readPressure() / 100.0;
    delay_ms(50);
    altitude = readAltitude();
    delay_ms(50);
    seaLevelPressure = readSealevelPressure() / 100.0;
    main_counter++;
  }
}
