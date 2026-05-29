/** @file BMI323_interrupts.ino 
*
*
* @author Jonathan Mejorado Lopez
*
* @bug No known bugs.
*/
#include <Wire.h>
#include "DevLab_BMI323.h"

// ------------- I2C Cfg
#define SDA_PIN 6
#define SCL_PIN 7
#define FAST_SPEED  400000

// FIX: D5 y D7 
#define INT1_PIN  D7   // D5 → ANY_MOTION
#define INT2_PIN  D5   // D7 → ACC_DRDY

// ── Variables compartidas con las ISR ─────────────────────────
volatile uint32_t int1Count = 0;
volatile bool     int2Flag  = false;

// ----------- Registros principales ------------------------
#define BMI323_ADDR 0x69

#define REG_CHIP_ID       0x00
#define REG_ERR_REG       0x01
#define REG_STATUS        0x02
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

const uint16_t tiemposAsentamientoODR[15] = {
  3200, 1600, 800, 400, 200, 100, 50, 25, 13, 10, 10, 10, 10, 10, 10
};

const char* etiquetasAccOdr[] = {
    "0.78","1.56","3.12","6.25","12.5","25","50","100","200","400","800","1K6","3K2","6K4"
};
const char* etiquetasAccRange[] = { "2G","4G","8G","16G" };
const char* etiquetasAccBw[]    = { "ODR/2","ODR/4" };
const char* etiquetasAccAVGNUM[]= { "NOAVG","AVG2","AVG4","AVG8","AVG16","AVG32","AVG64" };
const char* etiquetasMode[]     = { "Disabled","Cycling","Continous","HPM" };

const uint8_t acc_odr[14]    = { 0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8,0x9,0xA,0xB,0xC,0xD,0xE };
const uint8_t acc_range[4]   = { 0x0,0x1,0x2,0x3 };
const uint8_t acc_bw[2]      = { 0x0,0x1 };
const uint8_t acc_avgnum[7]  = { 0x0,0x1,0x2,0x3,0x4,0x5,0x6 };
const uint8_t acc_mode[4]    = { 0x0,0x3,0x4,0x7 };

// ----------- Giroscopio Config ----------------------------
const uint8_t gyr_odr[14]    = { 0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8,0x9,0xA,0xB,0xC,0xD,0xE };
const uint8_t gyr_avgnum[7]  = { 0x0,0x1,0x2,0x3,0x4,0x5,0x6 };
const uint8_t gyr_mode[5]    = { 0x0,0x1,0x3,0x4,0x7 };

const char* etiquetasGyrOdr[]    = { "0.78","1.56","3.12","6.25","12.5","25","50","100","200","400","800","1K6","3K2","6K4" };
const char* etiquetasGyrAVGNUM[] = { "NOAVG","AVG2","AVG4","AVG8","AVG16","AVG32","AVG64" };
const char* etiquetasGyrMode[]   = { "Disabled","DDriveEnabled","Cycling","Continous","HPM" };

#define BMI323_CHIP_ID  0x43
#define BMI323_SOFT_RST 0xDEAF

// ── ISR ───────────────────────────────────────────────────────
void IRAM_ATTR isr_INT1() { int1Count++; }
void IRAM_ATTR isr_INT2() { int2Flag = true; }

// ── Objetos y structs ─────────────────────────────────────────
DevLab_BMI323 imu(Wire, BMI323_ADDR);
DevLab_BMI323::SensorData   data;
DevLab_BMI323::acc_cfg      acc_Config;
DevLab_BMI323::gyr_cfg      gyr_Config;
DevLab_BMI323::int_ctrl     int_Config;

DevLab_BMI323::BMI323_INT1_SRC src1 = DevLab_BMI323::BMI323_SRC1_ANY_MOTION_OUT;
DevLab_BMI323::BMI323_INT2_SRC src2 = DevLab_BMI323::BMI323_SRC2_ACC_DRDY_INT;
DevLab_BMI323::BMI323_INT_DEST  dst1 = DevLab_BMI323::BMI323_INT1;
DevLab_BMI323::BMI323_INT_DEST  dst2 = DevLab_BMI323::BMI323_INT2;

// ── Configuración del acelerómetro ────────────────────────────
void configSens() {
  acc_Config.acc_mode   = 0x4;
  acc_Config.acc_odr    = 0x8;   // 100 Hz
  acc_Config.acc_range  = 0x2;
  acc_Config.acc_bw     = 0x0;
  acc_Config.acc_avgnum = 0x0;

  Serial.print(F("acc_odr valor: 0x"));
  Serial.println(acc_Config.acc_odr, HEX);
  imu.configAccBMI323(acc_Config);
  delay(160);
}

// ── Configuración de interrupciones ──────────────────────────
// FIX: softReset() removido de aquí — se hace UNA sola vez en setup()
//      antes de cualquier configuración, para no borrar configSens()
void configInterrupt() {
  int_Config.int1_level = 1;   // 1 Active High | 0 Active Low
  int_Config.int1_od    = 0;   // 1 Open Drain  | 0 Push Pull
  int_Config.int1_en    = 1;
  int_Config.int2_level = 1;
  int_Config.int2_od    = 0;
  int_Config.int2_en    = 1;

  imu.configINT(int_Config);
  imu.configIntLatch(0);
  delay(5);


  imu.testAddresses(0x14);
  
  imu.setINTMap2(src2, dst1);
  imu.setINTMap1(src1, dst2);

  Serial.print(F("INT_MAP1: 0x")); Serial.println(imu.getINT1Map(), HEX);
  Serial.print(F("INT_MAP2: 0x")); Serial.println(imu.getINT2Map(), HEX);
}

// ── Pines e ISR del MCU ───────────────────────────────────────
void setupINTPins() {
  // Push-pull activo-alto → INPUT puro, el BMI323 conduce la línea
  pinMode(INT1_PIN, INPUT);
  pinMode(INT2_PIN, INPUT);

  Serial.print(F("INT1 (D5) estado inicial: "));
  Serial.println(digitalRead(INT1_PIN) ? F("HIGH (activo)") : F("LOW (reposo OK)"));
  Serial.print(F("INT2 (D7) estado inicial: "));
  Serial.println(digitalRead(INT2_PIN) ? F("HIGH (activo)") : F("LOW (reposo OK)"));

  // RISING: flanco LOW→HIGH, coincide con activo-alto del BMI323
  attachInterrupt(digitalPinToInterrupt(INT1_PIN), isr_INT1, RISING);
  attachInterrupt(digitalPinToInterrupt(INT2_PIN), isr_INT2, RISING);
}

// ── Validaciones ──────────────────────────────────────────────
bool validateINT1_drdy() {
  Serial.println(F("\n[VAL-INT1] drdy_acc — conteo de pulsos 2s"));

  int1Count = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 2000) delay(1);

  uint32_t n = int1Count;
  // FIX: a 100Hz en 2s = 200 pulsos esperados, tolerancia ±20% = 160-240
  bool pass = (n >= 160 && n <= 240);

  Serial.print(F("  Pulsos contados : ")); Serial.println(n);
  Serial.println(F("  Esperado        : 160-240 (100Hz x 2s)"));
  Serial.print(F("  INT_STATUS1     : 0x"));
  Serial.println(imu.readINTStatus1(), HEX);
  Serial.println(pass ? F("  [PASS]") : F("  [FAIL]"));
  return pass;
}

bool validateINT2_anyMotion(uint32_t timeoutMs = 10000) {
  Serial.println(F("\n[VAL-INT2] any_motion — mueve el sensor"));

  int2Flag = false;
  uint32_t t0 = millis();

  while (!int2Flag && millis() - t0 < timeoutMs) {
    if ((millis() - t0) % 1000 < 10) {
      Serial.print(F(".  INT2 pin raw: "));
      Serial.println(digitalRead(INT2_PIN));
    }
    delay(10);
  }

  bool pass = int2Flag;
  Serial.println();
  Serial.print(F("  INT_STATUS2     : 0x"));
  Serial.println(imu.readINTStatus2(), HEX);
  Serial.println(pass ? F("  [PASS]") : F("  [FAIL] timeout sin flanco"));
  return pass;
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  delay(500);

  Serial.println(F("\n============================================"));
  Serial.println(F("  BMI323 — UNIT ELECTRONICS"));
  Serial.println(F("  INT1=D5 (any_motion)  INT2=D7 (acc_drdy)"));
  Serial.println(F("============================================\n"));

  if (!imu.begin(SDA_PIN, SCL_PIN, FAST_SPEED)) {
    Serial.println(F("ERROR: BMI323 initialization failed."));
    while (1) delay(1000);
  }
  delay(5);

  imu.test_chip_id(BMI323_CHIP_ID, REG_CHIP_ID);

  // FIX: softReset UNA sola vez aquí, antes de cualquier configuración
  imu.softReset();
  delay(15);


bool featOk = imu.enableFeatEngine();
Serial.print(F("Feature engine: "));
Serial.println(featOk ? F("LISTO") : F("TIMEOUT"));

// Configurar any_motion en FEATURE_IO0
imu.configAnyMotion();
  // Orden correcto: sensor → interrupciones → pines MCU
  configSens();
  configInterrupt();
  setupINTPins();

  validateINT1_drdy();
  validateINT2_anyMotion(10000);

  Serial.println(F("\nBMI323 initialized successfully."));
}

// ── Sweeps ────────────────────────────────────────────────────
void sweepMeasurementsGyr() {
  for (int m = 2; m < 5; m++) {
    gyr_Config.gyr_mode = gyr_mode[m];
    for (int i = 0; i < 14; i++) {
      gyr_Config.gyr_odr = gyr_odr[i];
      imu.softReset();
      delay(15);
      for (int l = 0; l < 7; l++) {
        gyr_Config.gyr_avgnum = gyr_avgnum[l];
        regBuilded = imu.configGyrBMI323(gyr_Config);
        delay(tiemposAsentamientoODR[i] + 20);
        if (imu.readData(data)) {
          Serial.print(regBuilded, HEX); Serial.print(";");
          Serial.print(etiquetasGyrMode[m]); Serial.print(";");
          Serial.print(etiquetasGyrOdr[i]); Serial.print(";");
          Serial.print(etiquetasGyrAVGNUM[l]); Serial.print(";");
          Serial.print(data.gyrX); Serial.print(";");
          Serial.print(data.gyrY); Serial.print(";");
          Serial.println(data.gyrZ);
        } else {
          Serial.println(F("ERROR: Failed to read BMI323 data."));
        }
      }
    }
  }
}

void sweepMeasurements() {
  for (int m = 1; m < 4; m++) {
    acc_Config.acc_mode = acc_mode[m];
    for (int i = 0; i < 14; i++) {
      acc_Config.acc_odr = acc_odr[i];
      imu.softReset();
      delay(15);
      for (int j = 0; j < 4; j++) {
        acc_Config.acc_range = acc_range[j];
        for (int k = 0; k < 2; k++) {
          acc_Config.acc_bw = acc_bw[k];
          for (int l = 0; l < 7; l++) {
            acc_Config.acc_avgnum = acc_avgnum[l];
            regBuilded = imu.configAccBMI323(acc_Config);
            delay(tiemposAsentamientoODR[i] + 20);
            if (imu.readData(data)) {
              Serial.print(regBuilded, HEX); Serial.print(";");
              Serial.print(etiquetasMode[m]); Serial.print(";");
              Serial.print(etiquetasAccOdr[i]); Serial.print(";");
              Serial.print(etiquetasAccRange[j]); Serial.print(";");
              Serial.print(etiquetasAccBw[k]); Serial.print(";");
              Serial.print(etiquetasAccAVGNUM[l]); Serial.print(";");
              Serial.print(data.accX); Serial.print(";");
              Serial.print(data.accY); Serial.print(";");
              Serial.println(data.accZ);
            } else {
              Serial.println(F("ERROR: Failed to read BMI323 data."));
            }
          }
        }
      }
    }
  }
}

void sweepTemp() {
  if (imu.readData(data)) {
    Serial.print(F("Temperature   [C];")); 
    Serial.println(data.temperatureC, 2);
  } else {
    Serial.println(F("ERROR: Failed to read BMI323 data."));
  }
}

void loop() {
  // sweepMeasurements();
  // sweepMeasurementsGyr();
  // sweepTemp();

    uint16_t s1 = imu.readINTStatus1();
    uint16_t s2 = imu.readINTStatus2();
  
  if (s1 || s2) {
    Serial.print(F("STATUS1: 0x")); Serial.print(s1, HEX);
    Serial.print(F("  STATUS2: 0x")); Serial.println(s2, HEX);
  }

}
