/** @file BMI323_AdvancedRead.ino 
*
*
* @author Jonathan Mejorado Lopez
*
* @bug No known bugs.
*/

#include <Wire.h>
#include "DevLab_BMI323.h"

// ------------- I2C Cfg
#define PIN_INT1 D7
#define PIN_INT2 D6
#define SDA_PIN 6
#define SCL_PIN 7
#define FAST_SPEED  400000



// ── Variables compartidas con las ISR ─────────────────────────
volatile bool eventoNuevo   = false;
volatile int  estadoActual  = HIGH;
volatile unsigned long tEvento = 0;
volatile int  cntRising  = 0;
volatile int  cntFalling = 0;
// ----------- Registros principales ------------------------

#define BMI323_ADDR 0x69

#define REG_CHIP_ID       0x00

#define REG_ACC_DATA_X    0x03
#define REG_ACC_DATA_Y    0x04
#define REG_ACC_DATA_Z    0x05
#define REG_GYR_DATA_X    0x06
#define REG_GYR_DATA_Y    0x07
#define REG_GYR_DATA_Z    0x08
#define REG_TEMP_DATA     0x09
#define REG_ACC_CONF      0x20
#define REG_GYR_CONF      0x21
#define REG_CMD           0x7E

uint16_t regBuilded = 0;
// Tiempos de asentamiento seguros (Periodo Base de cada ODR multiplicado por 2.5)
// Incluye el nuevo valor inicial para 0.78 Hz en el índice 0.
const uint16_t tiemposAsentamientoODR[15] = {
  3200, // Índice 0:  0.78 Hz (1280ms * 2.5) -> Filtro totalmente asentado
  1600, // Índice 1:  1.56 Hz (640ms * 2.5)
  800,  // Índice 2:  3.12 Hz (320ms * 2.5)
  400,  // Índice 3:  6.25 Hz (160ms * 2.5)
  200,  // Índice 4:  12.5 Hz (80ms * 2.5)
  100,  // Índice 5:  25 Hz   (40ms * 2.5)
  50,   // Índice 6:  50 Hz   (20ms * 2.5)
  25,   // Índice 7:  100 Hz  (10ms * 2.5)
  13,   // Índice 8:  200 Hz  (5ms * 2.5)
  10,   // Índice 9:  400 Hz  (Margen mínimo de seguridad I2C)
  10,   // Índice 10: 800 Hz  (Margen mínimo de seguridad I2C)
  10,   // Índice 11: 1600 Hz (Margen mínimo de seguridad I2C)
  10,   // Índice 12: 3200 Hz ...
  10,   // Índice 13: 6400 Hz
  10    // Índice 14: 12800 Hz
};


const char* etiquetasAccOdr[] = { 
    "0.78",
    "1.56",
    "3.12",
    "6.25",
    "12.5",
    "25",
    "50",
    "100",
    "200",
    "400",
    "800",
    "1K6",
    "3K2",
    "6K4"
};

const char* etiquetasAccRange[] = {
  "2G",
  "4G",
  "8G",
  "16G"
};

const char* etiquetasAccBw[] = {
  "ODR/2",
  "ODR/4"
};

const char* etiquetasAccAVGNUM[] = {
  "NOAVG",
  "AVG2",
  "AVG4",
  "AVG8",
  "AVG16",
  "AVG32",
  "AVG64"
};

const char* etiquetasMode[] = {
  "Disabled",
  "Cycling",
  "Continous",
  "HPM",
};

//Valores de configuracion acelerometro 
const uint8_t acc_odr[14] = {
    0x1, 
    0x2,
    0x3,
    0x4,
    0x5,
    0x6,
    0x7,
    0x8,
    0x9,
    0xA,
    0xB,
    0xC,
    0xD,
    0xE
};

const uint8_t acc_range[4] = {
    0x0,
    0x1,
    0x2,
    0x3
};

const uint8_t acc_bw[2] = {
    0x0,
    0x1
};

const uint8_t acc_avgnum[7] = {
    0x0,
    0x1,
    0x2,
    0x3,
    0x4,
    0x5,
    0x6
};

const uint8_t acc_mode[4] = {
    0x0,  
    0x3,
    0x4,
    0x7
};



// ----------------- Giroscopio Config --------------------------

//Valores de configuracion acelerometro 
const uint8_t gyr_odr[14] = {
    0x1, 
    0x2,
    0x3,
    0x4,
    0x5,
    0x6,
    0x7,
    0x8,
    0x9,
    0xA,
    0xB,
    0xC,
    0xD,
    0xE
};


const uint8_t gyr_avgnum[7] = {
    0x0,
    0x1,
    0x2,
    0x3,
    0x4,
    0x5,
    0x6
};

const uint8_t gyr_mode[5] = {
    0x0,
    0x1,
    0x3,
    0x4,
    0x7
};

const char* etiquetasGyrOdr[] = { 
    "0.78",
    "1.56",
    "3.12",
    "6.25",
    "12.5",
    "25",
    "50",
    "100",
    "200",
    "400",
    "800",
    "1K6",
    "3K2",
    "6K4"
};


const char* etiquetasGyrAVGNUM[] = {
  "NOAVG",
  "AVG2",
  "AVG4",
  "AVG8",
  "AVG16",
  "AVG32",
  "AVG64"
};

const char* etiquetasGyrMode[] = {
  "Disabled",
  "DDriveEnabled",
  "Cycling",
  "Continous",
  "HPM",
};

// ── Valores esperados / constantes ──────3──────────────────
#define BMI323_CHIP_ID    0x43
#define BMI323_SOFT_RST   0xDEAF  // Comando soft reset

//Configuracion Constructor IMU
DevLab_BMI323 imu(Wire,BMI323_ADDR);

//Struct to store the data
DevLab_BMI323::SensorData data;

DevLab_BMI323::acc_cfg        acc_Config;
DevLab_BMI323::gyr_cfg        gyr_Config;
DevLab_BMI323::int_ctrl       int_Config;
DevLab_BMI323::BMI323_INT1_SRC src1 = DevLab_BMI323::BMI323_SRC1_ANY_MOTION_OUT;
DevLab_BMI323::BMI323_INT2_SRC src2 = DevLab_BMI323::BMI323_SRC2_ACC_DRDY_INT;
DevLab_BMI323::BMI323_INT_DEST  dst1 = DevLab_BMI323::BMI323_INT1;
DevLab_BMI323::BMI323_INT_DEST  dst2 = DevLab_BMI323::BMI323_INT2;

void configSens(){
  acc_Config.acc_mode = 0x4;
  acc_Config.acc_odr = 0x8;
  acc_Config.acc_range = 0x2;
  acc_Config.acc_bw = 0x0;
  acc_Config.acc_avgnum = 0x0;

  Serial.print("acc_odr valor: 0x");
  Serial.println(acc_Config.acc_odr, HEX);  // debe ser 0x8 para 100Hz
  imu.configAccBMI323(acc_Config);

  delay(160);
}

void configInterrupt(){
   imu.softReset();
  int_Config.int1_level = 1;                        //1 Active High | 0 Active Low
  int_Config.int1_od = 0;                           //1 Open Drain  0 Push Pull
  int_Config.int1_en = 1;                           //Enabled or disabled
  int_Config.int2_level = 1;
  int_Config.int2_od = 0;
  int_Config.int2_en = 1;

  imu.configINT(int_Config);                        //Configuramos la interrupción

  imu.configIntLatch(0);

  delay(5);

  imu.enableFeatEngine(true);
  imu.setINTMap2(src2, dst1);

  imu.setINTMap1(src1, dst2);

  Serial.print(F("INT_MAP1: 0x")); Serial.println(imu.readINTStatus1(), HEX);
  Serial.print(F("INT_MAP2: 0x")); Serial.println(imu.readINTStatus2(), HEX);
}


// ── Configuración de pines e ISR ──────────────────────────────
void setupINTPins() {
  // Push-pull activo-alto → INPUT puro, sin pull-up ni pull-down
 Serial.println(F("Diagnóstico polling manual (500ms):"));
  pinMode(PIN_INT1, INPUT);
  pinMode(PIN_INT2, INPUT);

  uint32_t t0 = millis();
  uint8_t flancos = 0;
  uint8_t estadoAnterior = digitalRead(22);

  while (millis() - t0 < 500) {
    uint8_t estadoActual = digitalRead(22);
    if (estadoActual != estadoAnterior) {
      flancos++;
      Serial.print(F("  Flanco GPIO22: "));
      Serial.println(estadoActual ? F("SUBIDA") : F("BAJADA"));
      estadoAnterior = estadoActual;
    }
    // También leer STATUS para ver si el chip genera eventos
    uint16_t s2 = imu.readINTStatus2();
    if (s2 & 0x400) {
      Serial.print(F("  drdy_acc en STATUS2: 0x"));
      Serial.println(s2, HEX);
    }
    delayMicroseconds(100);
  }

  Serial.print(F("Flancos detectados en GPIO22: "));
  Serial.println(flancos);
  Serial.print(F("GPIO22 estado final: "));
  Serial.println(digitalRead(22));
  Serial.print(F("GPIO23 estado final: "));
  Serial.println(digitalRead(23));
}
/*
// ── Lectura de estado en tiempo real (polling sin ISR) ─────────
void readINTPinsPolling() {
  Serial.print(F("INT1: ")); Serial.print(digitalRead(INT1_PIN));
  Serial.print(F("  INT2: ")); Serial.println(digitalRead(INT2_PIN));
}



bool validateINT1_drdy() {
  Serial.println(F("\n[VAL-INT1] drdy_acc — conteo de pulsos 2s"));

  int1Count = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 2000) delay(1);

  uint32_t n = int1Count;
  // A 100 Hz esperamos ~200 pulsos en 2s (tolerancia ±20%)
  bool pass = (n >= 80 && n <= 120);   // ±20% de 100

  Serial.print(F("  Pulsos contados : ")); Serial.println(n);
  Serial.print(F("  Esperado        : 160-240 (100Hz x 2s)\n"));
  Serial.print(F("  INT_STATUS2     : 0x"));
  Serial.println(imu.readINTStatus2(), HEX);
  Serial.println(pass ? F("  [PASS]") : F("  [FAIL]"));
  return pass;
}

bool validateINT2_anyMotion(uint32_t timeoutMs = 10000) {
  Serial.println(F("\n[VAL-INT2] any_motion — mueve el sensor"));

  int2Flag = false;
  uint32_t t0 = millis();

  while (!int2Flag && millis() - t0 < timeoutMs) {
    // Punto de diagnóstico: estado crudo del pin cada 1s
    if ((millis() - t0) % 1000 < 10) {
      Serial.print(F(".  INT2 pin raw: "));
      Serial.println(digitalRead(INT2_PIN));
    }
    delay(10);
  }

  bool pass = int2Flag;
  Serial.println();
  Serial.print(F("  INT_STATUS1     : 0x"));
  Serial.println(imu.readINTStatus1(), HEX);
  Serial.println(pass ? F("  [PASS]") : F("  [FAIL] timeout sin flanco"));
  return pass;
}

*/


// IRAM_ATTR: función en RAM interna, no en flash
void IRAM_ATTR manejadorISR1() {
  estadoActual = digitalRead(PIN_INT1);
  tEvento      = millis();
  eventoNuevo  = true;

  if (estadoActual == HIGH) cntRising++;
  else                      cntFalling++;
}

// IRAM_ATTR: función en RAM interna, no en flash
void IRAM_ATTR manejadorISR2() {
  estadoActual = digitalRead(PIN_INT2);
  tEvento      = millis();
  eventoNuevo  = true;

  if (estadoActual == HIGH) cntRising++;
  else                      cntFalling++;
}
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
  while (!Serial && millis() < 3000);
  delay(500);

  Serial.println(F("\n============================================"));
  Serial.println(F("  BMI323 — UNIT ELECTRONICS"));
  Serial.println(F("  Datasheet: BST-BMI323-DS000-13"));
  Serial.println(F("============================================\n"));

  // Inicializar I2C a 400kHz (modo Fm)
   // Initialize BMI323
  if (!imu.begin(SDA_PIN, SCL_PIN, FAST_SPEED)) {

    Serial.println("ERROR: BMI323 initialization failed.");

    while (1) {
      delay(1000);
    }
  }
  delay(5);
  //Verificacion ID del dispositivo
  imu.test_chip_id(BMI323_CHIP_ID,REG_CHIP_ID);




   attachInterrupt(
    digitalPinToInterrupt(PIN_INT1),
    manejadorISR1,
    CHANGE
  );

   attachInterrupt(
    digitalPinToInterrupt(PIN_INT2),
    manejadorISR2,
    CHANGE
  );
  //Test Interruptions
  //Setting Up accelerometer
  configSens();

  //Setting Up interruption
  configInterrupt();

  // Configurar pines del MCU y attachInterrupt
  setupINTPins();

  // Validar
  //validateINT1_drdy();
  //validateINT2_anyMotion(10000);
  
  Serial.println("BMI323 initialized successfully.");
  Serial.println();
}

void sweepMeasurementsGyr(){
    for (int m = 2; m < 5; m++) {
      gyr_Config.gyr_mode = gyr_mode[m];
    for(int i = 0; i < 14; i++){
       gyr_Config.gyr_odr = gyr_odr[i];

      imu.softReset();
      delay(15);
          for(int l = 0; l < 7; l++){
             gyr_Config.gyr_avgnum = gyr_avgnum[l];
          

            //imu.softReset();


            regBuilded = imu.configGyrBMI323(gyr_Config);

            
            delay(tiemposAsentamientoODR[i] + 20);
            if (imu.readData(data)) {
              
              Serial.print(regBuilded, HEX);
              Serial.print(";");
              Serial.print(etiquetasGyrMode[m]);
              Serial.print(";");
              Serial.print(etiquetasGyrOdr[i]);
              Serial.print(";");
              Serial.print(etiquetasGyrAVGNUM[l]);
              Serial.print(";");
              // Accelerometer data
              Serial.print(data.gyrX);
              Serial.print(";");
              Serial.print(data.gyrY);
              Serial.print(";");
              Serial.println(data.gyrZ);

            } else {

              Serial.println("ERROR: Failed to read BMI323 data.");
            }

          }
        }
      }
    
}


void sweepMeasurements(){
  for (int m = 1; m < 4; m++) {
   acc_Config.acc_mode = acc_mode[m];
    for(int i = 0; i < 14; i++){
      acc_Config.acc_odr = acc_odr[i];

      imu.softReset();
      delay(15);
      for(int j = 0; j < 4; j++){
        acc_Config.acc_range = acc_range[j];
        for(int k = 0; k < 2; k++){
          acc_Config.acc_bw = acc_bw[k];
          for(int l = 0; l < 7; l++){
            acc_Config.acc_avgnum = acc_avgnum[l];
          

            //imu.softReset();


            regBuilded = imu.configAccBMI323(acc_Config);

            
            delay(tiemposAsentamientoODR[i] + 20);
            if (imu.readData(data)) {
              
              Serial.print(regBuilded, HEX);
              Serial.print(";");
              Serial.print(etiquetasMode[m]);
              Serial.print(";");
              Serial.print(etiquetasAccOdr[i]);
              Serial.print(";");
              Serial.print(etiquetasAccRange[j]);
              Serial.print(";");
              Serial.print(etiquetasAccBw[k]);
              Serial.print(";");
              Serial.print(etiquetasAccAVGNUM[l]);
              Serial.print(";");
              // Accelerometer data
              Serial.print(data.accX);
              Serial.print(";");
              Serial.print(data.accY);
              Serial.print(";");
              Serial.println(data.accZ);


              /*

              // Temperature data
              Serial.print("Temperature   [C]");
              Serial.print("    ");
              Serial.println(data.temperatureC, 2);
              */

            } else {

              Serial.println("ERROR: Failed to read BMI323 data.");
            }

          }
        }
      }
    }
  }
}

void sweepTemp(){
          
            if (imu.readData(data)) {
              
              
              // Temperature data
              Serial.print("Temperature   [C]");
              Serial.print(";");
              Serial.print("    ");
              Serial.println(data.temperatureC, 2);

            } else {

              Serial.println("ERROR: Failed to read BMI323 data.");
            }

          
       
}
void loop() {
  // put your main code here, to run repeatedly:
  //sweepMeasurements();
  //sweepMeasurementsGyr();
  //sweepTemp();
  //imu.test_chip_id(BMI323_CHIP_ID, REG_CHIP_ID);

  if (eventoNuevo) {
    eventoNuevo = false;

    // Parpadeo LED como confirmación visual
    // Imprimir qué flanco fue
    if (estadoActual == HIGH) {
      Serial.print("[RISING  ↑] subida  #");
      Serial.print(cntRising);
    } else {
      Serial.print("[FALLING ↓] bajada  #");
      Serial.print(cntFalling);
    }

    Serial.print("  |  pin=");
    Serial.print(estadoActual == HIGH ? "HIGH (1)" : "LOW  (0)");
    Serial.print("  |  t=");
    Serial.print(tEvento);
    Serial.println(" ms");
  }else{
     //Serial.println("Sin evento");
  }
}
