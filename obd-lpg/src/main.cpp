/*
  Teensy 4.1 – LPG ECU + OBD-II (with lambda + full fuel system decode)
*/

#include <Arduino.h>
#include <USBHost_t36.h>
#include <FlexCAN_T4.h>

// ============================================================
// USB HOST (LPG)
// ============================================================

USBHost myusb;
USBHub hub1(myusb);
USBSerial lpgSerial(myusb);

// ============================================================
// CAN (OBD)
// ============================================================

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

#define PID_FUEL_SYS   0x03
#define PID_STFT_B1    0x06
#define PID_LTFT_B1    0x07
#define PID_LAMBDA_CMD 0x44
#define PID_O2_S1AB    0x24

#define OBD2_REQUEST_ID  0x7DF
#define OBD2_RESPONSE_ID 0x7E8
#define OBD_TIMEOUT      20

struct OBDFrame {
  float stft;
  float ltft;
  uint8_t fuel_cl;
  float lambda_cmd;
  float o2_lambda;
  float o2_voltage;
};

// ============================================================
// LPG PROTOCOL
// ============================================================

#define HANDSHAKE_PREAMBLE 0xAC

const uint8_t POLL_CMD[] = {
  0x14,0x11,0xfe,0xef,0x0f,0x0f,0x0f,
  0x00,0x20,0x04,0xc1,0x3e,
  0x00,0x3c,0x00,0x10,0x00,0x00,0x02
};

#define POLL_LEN sizeof(POLL_CMD)

uint8_t token;

// ============================================================
// LPG HELPERS
// ============================================================

uint8_t checksum(const uint8_t *data, size_t len) {
  uint32_t sum = 0;
  for (size_t i = 0; i < len; i++) sum += data[i];
  return sum & 0xFF;
}

size_t buildFrame(uint8_t token, const uint8_t *payload, size_t plen, uint8_t *out) {
  uint8_t len = plen + 1;

  out[0] = token;
  out[1] = 0x00;
  out[2] = 0x00;
  out[3] = len;

  memcpy(out + 4, payload, plen);
  out[4 + plen] = checksum(out, 4 + plen);

  return 5 + plen;
}

bool readFrame(uint8_t token, uint8_t *buffer, size_t &outLen, uint32_t timeout = 2000) {
  uint32_t start = millis();

  while (millis() - start < timeout) {
    myusb.Task();
    while (lpgSerial.available()) {
      if (lpgSerial.read() == token) goto found;
    }
  }
  return false;

found:
  uint8_t hdr[3];
  if (lpgSerial.readBytes(hdr, 3) != 3) return false;

  uint8_t len = hdr[2];

  buffer[0] = token;
  memcpy(buffer + 1, hdr, 3);

  if (lpgSerial.readBytes(buffer + 4, len) != len) return false;

  outLen = 4 + len;
  return true;
}

// ============================================================
// LPG DECODING
// ============================================================

uint16_t be16(const uint8_t *f, int o) {
  return (f[o] << 8) | f[o + 1];
}

float decodeMAP(uint8_t *f) { return be16(f, 0x0A) * 0.01f; }
uint16_t decodeRPM(uint8_t *f) { return be16(f, 0x14); }

void decodePB(uint8_t *f, float &a, float &b, float &c, float &d) {
  a = be16(f, 0x30) * 0.005f;
  b = be16(f, 0x34) * 0.005f;
  c = be16(f, 0x38) * 0.005f;
  d = be16(f, 0x3C) * 0.005f;
}

// ============================================================
// CAN / OBD
// ============================================================

void initCAN() {
  can1.begin();
  can1.setBaudRate(500000);
  can1.setMaxMB(16);
  can1.enableFIFO();

  can1.setFIFOFilter(REJECT_ALL);
  for (int i = 0; i < 8; i++)
    can1.setFIFOFilter(i, OBD2_RESPONSE_ID + i, STD);
}

bool obd_query(uint8_t pid, CAN_message_t &msg) {
  CAN_message_t req = {};
  req.id = OBD2_REQUEST_ID;
  req.len = 8;
  req.buf[0] = 0x02;
  req.buf[1] = 0x01;
  req.buf[2] = pid;

  if (!can1.write(req)) return false;

  uint32_t t = millis() + OBD_TIMEOUT;

  while (millis() < t) {
    if (can1.read(msg)) {
      if (msg.buf[1] == 0x41 && msg.buf[2] == pid)
        return true;
    }
  }
  return false;
}

// ============================================================
// OBD DECODERS
// ============================================================

float obd_trim(const CAN_message_t &m) {
  return ((float)m.buf[3] - 128.0f) * 100.0f / 128.0f;
}

// FULL fuel system decode → still outputs 1/0
uint8_t obd_fuel_cl(const CAN_message_t &m) {
  uint8_t v = m.buf[3];

  if (v == 0x02 || v == 0x04) return 1;  // closed loop
  return 0;
}

// λ commanded
float obd_lambda_cmd(const CAN_message_t &m) {
  uint16_t ab = (m.buf[3] << 8) | m.buf[4];
  return 2.0f * ab / 65536.0f;
}

// wideband O2
void obd_o2(const CAN_message_t &m, float &lambda, float &volt) {
  uint16_t ab = (m.buf[3] << 8) | m.buf[4];
  uint16_t cd = (m.buf[5] << 8) | m.buf[6];

  lambda = 2.0f * ab / 65536.0f;
  volt   = 8.0f * cd / 65536.0f;
}

// ============================================================
// POLL
// ============================================================

bool obd_poll(OBDFrame &f) {
  CAN_message_t msg;
  bool ok = true;

  if (obd_query(PID_STFT_B1, msg)) f.stft = obd_trim(msg); else ok = false;
  if (obd_query(PID_LTFT_B1, msg)) f.ltft = obd_trim(msg); else ok = false;
  if (obd_query(PID_FUEL_SYS, msg)) f.fuel_cl = obd_fuel_cl(msg); else ok = false;
  if (obd_query(PID_LAMBDA_CMD, msg)) f.lambda_cmd = obd_lambda_cmd(msg); else ok = false;
  if (obd_query(PID_O2_S1AB, msg)) obd_o2(msg, f.o2_lambda, f.o2_voltage); else ok = false;

  return ok;
}

// ============================================================
// HANDSHAKE
// ============================================================

bool handshake() {
  uint8_t payload[3] = {0x00,0x00,token};
  uint8_t frame[16];

  size_t len = buildFrame(HANDSHAKE_PREAMBLE, payload, 3, frame);
  lpgSerial.write(frame, len);

  uint8_t rx[128];
  size_t rlen;

  return readFrame(token, rx, rlen, 3000);
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  myusb.begin();
  while (!lpgSerial) myusb.Task();

  initCAN();
  lpgSerial.begin(57600);

  while(!Serial.available()){
    Serial.println("Waiting for serial\r\n");delay(100);
  }

  randomSeed(analogRead(A0));
  token = random(1,255);

  handshake();


  // CSV header
  Serial.print("RPM,MAP,PB1,PB2,PB3,PB4,STFT,LTFT,FuelCL,lambda_cmd,O2_lambda,O2_voltage\r\n");
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  myusb.Task();

  static uint8_t pollFrame[128];
  static size_t pollLen = 0;

  if (pollLen == 0)
    pollLen = buildFrame(token, POLL_CMD, POLL_LEN, pollFrame);

  lpgSerial.write(pollFrame, pollLen);

  uint8_t rx[256];
  size_t len;

  static OBDFrame obd = {};
  static uint32_t obdTimer = 0;

  if (millis() - obdTimer > 200) {
    obd_poll(obd);
    obdTimer = millis();
  }

  if (readFrame(token, rx, len, 50)) {
    if (len > 0x3E && rx[4] == 0x94) {

      float pb1,pb2,pb3,pb4;
      decodePB(rx,pb1,pb2,pb3,pb4);

      uint16_t rpm = decodeRPM(rx);
      float map = decodeMAP(rx);

      Serial.printf(
        "%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%.4f,%.4f,%.4f\r\n",
        rpm,
        map,
        pb1,pb2,pb3,pb4,
        obd.stft,
        obd.ltft,
        obd.fuel_cl,
        obd.lambda_cmd,
        obd.o2_lambda,
        obd.o2_voltage
      );
    }
  }

  delay(10);
}