/*
 * =====================================================================
 *  CiGo E-Bike Controller v7.6 - SERGI MODU
 * =====================================================================
 *  v7.5'e gore farklar:
 *
 *   [1] Hiz hesabi motor capi uzerinden:
 *       WHEEL_DIAMETER_M (0.66m teker) -> MOTOR_DIAMETER_M (0.141m motor)
 *       v = (rpm_mech/60) * pi * 0.141 * 3.6  -> GPS ile uyumlu km/h
 *
 *   [2] IMU tamamen sessize alindi (sergi sirasinda zirt-pirt reset yok):
 *       - setup: imuInit/verify cagrilari devre disi, sadece "off" mesaji
 *       - loop:  imuPoll() cagrisi yorum satiri
 *       - emergencyReboot() bos -> kazara cagrilsa bile bir sey yapmaz
 *       Kod hala derlenebilir, IMU degiskenleri duruyor; sadece "uyandirilmiyor"
 *
 *   [3] Regen voltaj kilidi:
 *       Batarya voltaji 41.5V'tan yuksekse regen FREN UYGULAMAZ.
 *       Bunun yerine vescSetCurrent(0) (coast) gonderir.
 *       Sebep: tam dolu bataryada regen -> BMS overvoltage / hasar.
 *
 *   DEGISMEYEN: VESC protokol, RemoteXY config, mod sistemi (A/B),
 *   BLE kopma guvenligi, ON/OFF, slider, voltaj bucket.
 * =====================================================================
 */

#include <Arduino.h>
#include <Wire.h>

// ================= REMOTEXY BLE =================
#define REMOTEXY_MODE__ESP32CORE_BLE
#include <BLEDevice.h>
#define REMOTEXY_BLUETOOTH_NAME   "CiGo"
#define REMOTEXY_ACCESS_PASSWORD  "6868"
#include <RemoteXY.h>

#pragma pack(push, 1)
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] = {
  255,3,0,5,0,124,0,19,0,0,0,67,105,71,111,0,8,1,106,200,
  1,1,8,0,73,80,7,18,138,4,131,0,4,26,0,0,0,0,0,0,
  200,66,0,0,0,0,66,18,156,72,26,131,36,120,2,36,66,34,21,0,
  36,26,31,31,79,78,0,79,70,70,0,3,35,12,34,19,130,30,16,129,
  35,185,38,10,64,35,86,101,108,111,99,105,116,121,32,0,4,6,1,19,
  154,32,30,26,129,42,33,21,8,64,30,109,111,100,101,0,129,72,146,30,
  7,64,120,66,65,84,84,69,82,89,0
};

struct {
  uint8_t onoff;
  uint8_t mode;
  int8_t  slider_01;
  float   batterybar;
  int8_t  Velocity;
  uint8_t connect_flag;
} RemoteXY;
#pragma pack(pop)

// ================= KONFIG =================
#define VESC_SERIAL        Serial2
#define VESC_RX_PIN        16
#define VESC_TX_PIN        17
#define VESC_BAUD          115200

#define MOTOR_POLE_PAIRS   15            // 30 mıknatıs / 2 = 15 kutup cifti
#define MOTOR_DIAMETER_M   0.141f        // [DEGISTI] 141mm motor cap (eski: 0.66 teker)

#define MODE_A_CURRENT_A   13.5f
#define MODE_A_REGEN_A     10.0f
#define MODE_B_CURRENT_A   55.0f
#define MODE_B_REGEN_A     35.0f

#define BATTERY_V_MAX      42.0f
#define BATTERY_V_MIN      30.0f

// [YENI] Bu voltajin uzerinde regen UYGULANMAZ (BMS koruma)
#define REGEN_BLOCK_VOLT   41.5f

#define VESC_POLL_MS       50
#define CMD_POLL_MS        50

// === IMU (kullanilmiyor ama kod derlensin diye define'lar duruyor) ===
#define IMU_SDA_PIN          21
#define IMU_SCL_PIN          22
#define IMU_I2C_FREQ         400000
#define MPU6050_ADDR         0x68
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define IMU_POLL_MS          20

#define VBUCKET_HYSTERESIS   0.4f

// ================= VESC PROTOKOL =================
enum COMM_PACKET_ID {
  COMM_GET_VALUES        = 4,
  COMM_SET_CURRENT       = 6,
  COMM_SET_CURRENT_BRAKE = 7,
};

const unsigned short crc16_tab[256] = {
  0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
  0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
  0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
  0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
  0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
  0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
  0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
  0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
  0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
  0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
  0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
  0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
  0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
  0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
  0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
  0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
  0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
  0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
  0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
  0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
  0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
  0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
  0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
  0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
  0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
  0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
  0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
  0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
  0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
  0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
  0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
  0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

uint16_t crc16(const uint8_t *buf, uint32_t len) {
  uint16_t c = 0;
  for (uint32_t i = 0; i < len; i++)
    c = crc16_tab[((c >> 8) ^ *buf++) & 0xFF] ^ (c << 8);
  return c;
}

void vescSend(uint8_t *payload, int len) {
  uint8_t buf[32]; int idx = 0;
  buf[idx++] = 0x02;
  buf[idx++] = (uint8_t)len;
  memcpy(&buf[idx], payload, len); idx += len;
  uint16_t c = crc16(payload, len);
  buf[idx++] = c >> 8;
  buf[idx++] = c & 0xFF;
  buf[idx++] = 0x03;
  VESC_SERIAL.write(buf, idx);
}

void vescGetValues() {
  static const uint8_t pkt[] = { 0x02, 0x01, 0x04, 0x40, 0x84, 0x03 };
  VESC_SERIAL.write(pkt, 6);
}

void vescSetCurrent(float A) {
  int32_t v = (int32_t)(A * 1000.0f);
  uint8_t p[5] = { COMM_SET_CURRENT,
    (uint8_t)(v>>24),(uint8_t)(v>>16),(uint8_t)(v>>8),(uint8_t)v };
  vescSend(p, 5);
}

void vescSetCurrentBrake(float A) {
  int32_t v = (int32_t)(A * 1000.0f);
  uint8_t p[5] = { COMM_SET_CURRENT_BRAKE,
    (uint8_t)(v>>24),(uint8_t)(v>>16),(uint8_t)(v>>8),(uint8_t)v };
  vescSend(p, 5);
}

// ================= VESC RX =================
int16_t bget_i16(const uint8_t *b, int *i) {
  int16_t r = ((uint16_t)b[*i] << 8) | b[*i+1]; *i += 2; return r;
}
int32_t bget_i32(const uint8_t *b, int *i) {
  int32_t r = ((uint32_t)b[*i]<<24)|((uint32_t)b[*i+1]<<16)|
              ((uint32_t)b[*i+2]<<8)|b[*i+3];
  *i += 4; return r;
}

struct {
  float    v_in;
  float    current_motor;
  float    current_in;
  float    duty;
  int32_t  erpm;
  int32_t  rpm_mech;
  uint8_t  fault_code;
  uint32_t last_update_ms;
} vesc;

bool parseGetValues(const uint8_t *p, int len) {
  if (len < 60) return false;
  int i = 1;
  i += 2; i += 2;
  vesc.current_motor = bget_i32(p,&i) / 100.0f;
  vesc.current_in    = bget_i32(p,&i) / 100.0f;
  i += 4; i += 4;
  vesc.duty          = bget_i16(p,&i) / 1000.0f;
  vesc.erpm          = bget_i32(p,&i);
  vesc.rpm_mech      = vesc.erpm / MOTOR_POLE_PAIRS;
  vesc.v_in          = bget_i16(p,&i) / 10.0f;
  i += 4*6;
  if (i < len) vesc.fault_code = p[i];
  vesc.last_update_ms = millis();
  return true;
}

enum { ST_START, ST_LEN, ST_PAY, ST_CRC1, ST_CRC2, ST_END };
uint8_t  rxBuf[128];
int      rxLen=0, rxIdx=0;
uint16_t rxCrc=0;
uint8_t  rxState=ST_START;
uint32_t rxLastByte=0;

void vescRxPoll() {
  if (rxState != ST_START && millis() - rxLastByte > 100) rxState = ST_START;
  while (VESC_SERIAL.available()) {
    uint8_t b = VESC_SERIAL.read();
    rxLastByte = millis();
    switch (rxState) {
      case ST_START: if (b==0x02) rxState=ST_LEN; break;
      case ST_LEN:   rxLen=b; rxIdx=0; rxState=(rxLen?ST_PAY:ST_CRC1); break;
      case ST_PAY:
        if (rxIdx < (int)sizeof(rxBuf)) {
          rxBuf[rxIdx++]=b;
          if (rxIdx>=rxLen) rxState=ST_CRC1;
        } else rxState=ST_START;
        break;
      case ST_CRC1:  rxCrc=((uint16_t)b)<<8; rxState=ST_CRC2; break;
      case ST_CRC2:  rxCrc|=b; rxState=ST_END; break;
      case ST_END:
        if (b==0x03 && crc16(rxBuf,rxLen)==rxCrc && rxLen>0) {
          if (rxBuf[0]==COMM_GET_VALUES) parseGetValues(rxBuf,rxLen);
        }
        rxState=ST_START;
        break;
    }
  }
}

// ================= IMU (SERGI MODUNDA DEVRE DISI) =================
// Kod derlensin diye fonksiyonlar ve degiskenler duruyor ama hicbiri cagrilmiyor.
bool     imu_ok = false;
float    last_ax = 0, last_ay = 0, last_az = 0, last_mag = 0;
uint32_t fall_violation_start_ms = 0;
bool     fall_violation_pending  = false;

bool mpuReadAccelG(float *ax, float *ay, float *az) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t got = Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)6);
  if (got < 6) return false;
  int16_t rx = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t ry = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t rz = ((int16_t)Wire.read() << 8) | Wire.read();
  *ax = rx / 16384.0f;
  *ay = ry / 16384.0f;
  *az = rz / 16384.0f;
  return true;
}

bool imuInit() {
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  Wire.setClock(IMU_I2C_FREQ);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;
  delay(50);
  float ax, ay, az;
  return mpuReadAccelG(&ax, &ay, &az);
}

// [SERGI MODU] Bos birakildi - kazara cagrilsa bile sistem reset atmaz
void emergencyReboot() {
  // intentionally empty
}

// [SERGI MODU] imuPoll cagrilmiyor - loop'tan yorum satiri yapildi
void imuPoll() {
  if (!imu_ok) return;
  float ax, ay, az;
  if (!mpuReadAccelG(&ax, &ay, &az)) return;
  last_ax = ax; last_ay = ay; last_az = az;
  last_mag = sqrtf(ax*ax + ay*ay + az*az);
}

// ================= VOLTAJ BUCKETING =================
float bucketedVoltage(float v_raw) {
  static float current = -1.0f;
  if (current < 0.0f) {
    if      (v_raw >= 39.5f) current = 41.0f;
    else if (v_raw >= 36.5f) current = 38.0f;
    else if (v_raw >= 33.5f) current = 35.0f;
    else                     current = 32.0f;
    return current;
  }
  if      (current == 32.0f && v_raw > 33.5f + VBUCKET_HYSTERESIS) current = 35.0f;
  else if (current == 35.0f && v_raw > 36.5f + VBUCKET_HYSTERESIS) current = 38.0f;
  else if (current == 38.0f && v_raw > 39.5f + VBUCKET_HYSTERESIS) current = 41.0f;
  else if (current == 41.0f && v_raw < 39.5f - VBUCKET_HYSTERESIS) current = 38.0f;
  else if (current == 38.0f && v_raw < 36.5f - VBUCKET_HYSTERESIS) current = 35.0f;
  else if (current == 35.0f && v_raw < 33.5f - VBUCKET_HYSTERESIS) current = 32.0f;
  return current;
}

// ================= KONTROL =================
enum LastStop { STOP_NONE, STOP_ONOFF, STOP_BLE_LOST, STOP_STARTUP } last_stop_reason = STOP_STARTUP;

bool isSportMode() { return RemoteXY.mode != 0; }

void getModeLimits(float *fwd_max, float *regen_max) {
  if (isSportMode()) { *fwd_max=MODE_B_CURRENT_A; *regen_max=MODE_B_REGEN_A; }
  else               { *fwd_max=MODE_A_CURRENT_A; *regen_max=MODE_A_REGEN_A; }
}

const char* modeName() { return isSportMode() ? "B-SPORT" : "A-YURUYUS"; }

void computeAndSendCommand() {
  if (RemoteXY.connect_flag == 0) {
    vescSetCurrent(0.0f);
    if (last_stop_reason != STOP_BLE_LOST) {
      last_stop_reason = STOP_BLE_LOST;
      Serial.println("!! MOTOR OFF: BLE baglanti yok");
    }
    return;
  }
  if (RemoteXY.onoff == 0) {
    vescSetCurrent(0.0f);
    if (last_stop_reason != STOP_ONOFF) {
      last_stop_reason = STOP_ONOFF;
      Serial.println("!! MOTOR OFF: ON/OFF switch OFF");
    }
    return;
  }
  if (last_stop_reason != STOP_NONE) {
    last_stop_reason = STOP_NONE;
    Serial.printf(">> MOTOR HAZIR (mod: %s)\n", modeName());
  }

  int sl = RemoteXY.slider_01;
  float fwd_max, regen_max;
  getModeLimits(&fwd_max, &regen_max);

  if (sl > 0) {
    vescSetCurrent((sl / 100.0f) * fwd_max);
  }
  else if (sl < 0) {
    // [YENI] Batarya dolu ise regen YAPMA - coast (sifir tork)
    if (vesc.v_in > REGEN_BLOCK_VOLT) {
      vescSetCurrent(0.0f);
      static uint32_t last_warn = 0;
      if (millis() - last_warn > 2000) {
        last_warn = millis();
        Serial.printf("!! REGEN BLOK (V=%.1f > %.1f) - BMS koruma, motor coast\n",
                      vesc.v_in, REGEN_BLOCK_VOLT);
      }
    } else {
      vescSetCurrentBrake((-sl / 100.0f) * regen_max);
    }
  }
  else {
    vescSetCurrent(0.0f);
  }
}

// ================= REMOTEXY CIKTI =================
void updateRemoteXYOutputs() {
  // Voltaj kademeleri (32/35/38/41V)
  float v_bucket = bucketedVoltage(vesc.v_in);
  float p = (v_bucket - BATTERY_V_MIN) / (BATTERY_V_MAX - BATTERY_V_MIN) * 100.0f;
  if (p < 0) p = 0; if (p > 100) p = 100;
  RemoteXY.batterybar = p;

  // [DEGISTI] Hiz = (rpm_mech/60) * pi * D_motor * 3.6  (D=141mm)
  float kmh = (vesc.rpm_mech / 60.0f) * 3.14159f * MOTOR_DIAMETER_M * 3.6f;
  if (kmh < 0) kmh = -kmh;
  float vel_pct = (kmh / 25.0f) * 100.0f;
  if (vel_pct > 100) vel_pct = 100;
  if (vel_pct < 0) vel_pct = 0;
  RemoteXY.Velocity = (int8_t)vel_pct;
}

// ================= DEBUG =================
void printStatus() {
  bool con = (RemoteXY.connect_flag == 1);
  bool on  = (RemoteXY.onoff == 1);
  bool active = con && on;

  float fwd_max, regen_max;
  getModeLimits(&fwd_max, &regen_max);

  // Anlik km/h hesabini debug icin de bas
  float kmh = (vesc.rpm_mech / 60.0f) * 3.14159f * MOTOR_DIAMETER_M * 3.6f;
  if (kmh < 0) kmh = -kmh;

  Serial.printf("[%s] CON=%d ON=%d MOD=%s (+%.1f/-%.1f) | "
                "V=%.1f(B%.0f)%s | ERPM=%ld (%ld rpm, %.1f km/h) "
                "Im=%.1f Ib=%.1f D=%.2f flt=%u | sl=%d\n",
                active ? "RUN " : "STOP",
                con, on, modeName(), fwd_max, regen_max,
                vesc.v_in, bucketedVoltage(vesc.v_in),
                (vesc.v_in > REGEN_BLOCK_VOLT) ? " [REGEN-BLOK]" : "",
                vesc.erpm, vesc.rpm_mech, kmh,
                vesc.current_motor, vesc.current_in, vesc.duty,
                vesc.fault_code, RemoteXY.slider_01);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("\n========================================");
  Serial.println(" CiGo E-Bike v7.6 - SERGI MODU");
  Serial.println(" Batarya: 10S Li-ion (36V / 42V full / 30V empty)");
  Serial.println(" Modlar:");
  Serial.printf("   A-YURUYUS: +%.1fA / -%.1fA regen\n",
                MODE_A_CURRENT_A, MODE_A_REGEN_A);
  Serial.printf("   B-SPORT  : +%.1fA / -%.1fA regen\n",
                MODE_B_CURRENT_A, MODE_B_REGEN_A);
  Serial.printf(" Hiz: motor capi %.0f mm uzerinden (rpm_mech * pi * D * 3.6)\n",
                MOTOR_DIAMETER_M * 1000.0f);
  Serial.printf(" Regen blok voltaji: %.1fV (uzerinde fren -> coast)\n",
                REGEN_BLOCK_VOLT);
  Serial.printf(" Voltaj kademeleri: 32/35/38/41 V (hys %.1fV)\n", VBUCKET_HYSTERESIS);
  Serial.println("========================================");

  VESC_SERIAL.begin(VESC_BAUD, SERIAL_8N1, VESC_RX_PIN, VESC_TX_PIN);
  Serial.printf(" VESC UART @ %d baud, RX=%d TX=%d\n",
                VESC_BAUD, VESC_RX_PIN, VESC_TX_PIN);

  RemoteXY_Init();
  Serial.printf(" BLE: %s / %s\n", REMOTEXY_BLUETOOTH_NAME, REMOTEXY_ACCESS_PASSWORD);

  memset(&vesc, 0, sizeof(vesc));

  // === IMU SERGI MODUNDA DEVRE DISI ===
  imu_ok = false;
  Serial.println(" !! IMU ve dusme tespiti SERGI MODUNDA devre disi.");
  Serial.println(" !! BLE/ON-OFF guvenligi aktif - bisiklet kacmaz.");
  /*
  // ORIJINAL IMU INIT (gerekirse acilir):
  Serial.printf(" IMU init (SDA=%d, SCL=%d) ... ", IMU_SDA_PIN, IMU_SCL_PIN);
  imu_ok = imuInit();
  if (imu_ok) Serial.println("OK");
  else        Serial.println("BASARISIZ");
  */

  Serial.println(" Hazir.\n");
}

// ================= LOOP =================
uint32_t t_vesc = 0, t_cmd = 0, t_dbg = 0, t_xy = 0, t_imu = 0;

void loop() {
  RemoteXY_Handler();
  vescRxPoll();

  uint32_t now = millis();

  // [SERGI MODU] IMU polling devre disi
  /*
  if (now - t_imu >= IMU_POLL_MS) {
    t_imu = now;
    imuPoll();
  }
  */
  (void)t_imu; // unused warning susturma

  if (now - t_vesc >= VESC_POLL_MS) {
    t_vesc = now;
    vescGetValues();
  }
  if (now - t_cmd >= CMD_POLL_MS) {
    t_cmd = now;
    computeAndSendCommand();
  }
  if (now - t_xy >= 200) {
    t_xy = now;
    updateRemoteXYOutputs();
  }
  if (now - t_dbg >= 500) {
    t_dbg = now;
    printStatus();
  }
}




