// Peer for the fixture-service HIL over the 8-wire P4 link (E154: 33,32,26..31 pin to pin).
// UART echo: peer RX = GPIO26 (probe TX), peer TX = GPIO27 (probe RX), 115200.
// GPIO mirror: peer copies GPIO28 (probe drives) onto GPIO29 (probe samples).
// I2C controller (new driver, E147-E150) on SDA=GPIO32 / SCL=GPIO33 driven by text commands on USB:
//   WRITE <hz> <hex>            one write transaction with the given bytes
//   FRAME <hz> <hex>            1-byte length header transaction, then the payload transaction
//   READ <hz> <length>          one read transaction; prints the bytes
#include <Arduino.h>
#include <driver/i2c_master.h>

static i2c_master_bus_handle_t bus = nullptr;
static i2c_master_dev_handle_t dev = nullptr;
static uint32_t devHz = 0;
static const uint8_t kAddress = 0x42;

static bool ensureDevice(uint32_t hz) {
  if (dev && devHz == hz) return true;
  if (dev) { i2c_master_bus_rm_device(dev); dev = nullptr; }
  if (!bus) {
    i2c_master_bus_config_t cfg = {};
    cfg.i2c_port = I2C_NUM_1; cfg.sda_io_num = GPIO_NUM_32; cfg.scl_io_num = GPIO_NUM_33;
    cfg.clk_source = I2C_CLK_SRC_DEFAULT; cfg.glitch_ignore_cnt = 7; cfg.flags.enable_internal_pullup = 1;
    if (i2c_new_master_bus(&cfg, &bus) != ESP_OK) return false;
  }
  i2c_device_config_t dcfg = {};
  dcfg.dev_addr_length = I2C_ADDR_BIT_LEN_7; dcfg.device_address = kAddress; dcfg.scl_speed_hz = hz;
  if (i2c_master_bus_add_device(bus, &dcfg, &dev) != ESP_OK) { dev = nullptr; return false; }
  devHz = hz;
  return true;
}

static size_t parseHex(const char *s, uint8_t *out, size_t cap) {
  size_t n = 0;
  while (s[0] && s[1] && n < cap) {
    char pair[3] = {s[0], s[1], 0};
    out[n++] = (uint8_t)strtoul(pair, nullptr, 16);
    s += 2;
  }
  return n;
}

void setup() {
  // A FRAME command for 128 bytes is 269 characters; the HWCDC RX ring defaults to
  // 256 bytes and drops the excess (E155), which showed up as 121-123 byte payloads.
  Serial.setRxBufferSize(4096);
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 26, 27);
  pinMode(28, INPUT);
  pinMode(29, OUTPUT);
}

void loop() {
  digitalWrite(29, digitalRead(28));
  while (Serial1.available()) Serial1.write(Serial1.read());
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n'); line.trim();
  if (line == "?") { Serial.println("# HIL peer_p4b uart26/27 mirror28->29 i2c32/33"); return; }
  char cmd[8] = {0}; unsigned long hz = 0; char arg[300] = {0};
  if (sscanf(line.c_str(), "%7s %lu %299s", cmd, &hz, arg) < 2) { Serial.println("ERR usage"); return; }
  if (!ensureDevice(hz)) { Serial.println("ERR device"); return; }
  uint8_t buf[256];
  if (!strcmp(cmd, "WRITE")) {
    const size_t n = parseHex(arg, buf, sizeof buf);
    const esp_err_t r = i2c_master_transmit(dev, buf, n, 100);
    Serial.printf("WRITE result=0x%x bytes=%u\n", (unsigned)r, (unsigned)n);
  } else if (!strcmp(cmd, "FRAME")) {
    const size_t n = parseHex(arg, buf, sizeof buf);
    const uint8_t header = (uint8_t)n;
    const esp_err_t r1 = i2c_master_transmit(dev, &header, 1, 100);
    const esp_err_t r2 = i2c_master_transmit(dev, buf, n, 100);
    Serial.printf("FRAME header=0x%x payload=0x%x bytes=%u\n", (unsigned)r1, (unsigned)r2, (unsigned)n);
  } else if (!strcmp(cmd, "READ")) {
    const size_t n = strtoul(arg, nullptr, 10);
    if (!n || n > sizeof buf) { Serial.println("ERR length"); return; }
    const esp_err_t r = i2c_master_receive(dev, buf, n, 100);
    Serial.printf("READ result=0x%x data=", (unsigned)r);
    for (size_t i = 0; i < n; ++i) Serial.printf("%02x", buf[i]);
    Serial.println();
  } else Serial.println("ERR cmd");
}
