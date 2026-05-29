/** @file BMI323_SPI.ino 
*
*
* @author Jonathan Mejorado Lopez
*
* @bug No known bugs.
*/

#include <SPI.h>
#include "DevLab_BMI323.h"



/// *************Setup SPI Config
#define MOSI_PIN D11
#define MISO_PIN D12
#define SCK_PIN  D13
#define CS_PIN   D10  
#define SPI_FAST_SPEED 4000000

SPIClass spi_bus(SPI);                    //Inicializacion de bus SPI

DevLab_BMI323 imuSpi(spi_bus,CS_PIN,MISO_PIN,MOSI_PIN,SCK_PIN,SPI_FAST_SPEED);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);

  Serial.println("Iniciando puerto");

  imuSpi.begin();
  delay(10);

  imuSpi.test_chip_id(BMI323_CHIP_ID, REG_CHIP_ID);

}

void loop() {
  // put your main code here, to run repeatedly:

}
