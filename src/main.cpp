// ESP32-CAM (GC2145) -> UDP JPEG streamer, pipelined (encode on core 1, send on core 0)
// Packet: [camID 1B][frameID 2B][chunkIdx 2B][chunkCount 2B][payload]

#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "img_converters.h"

// ---------------- USER CONFIG ----------------
const char* WIFI_SSID = "Mainak_777";
const char* WIFI_PASS = "1234567890";

#define CAMERA_ID 1

#define USE_STATIC_IP true
IPAddress local_IP(192, 168, 0, 101);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(8, 8, 8, 8);
IPAddress dest_ip(192, 168, 0, 112);     // your PC
const uint16_t dest_port = 2222;

// Resolution: FRAMESIZE_QQVGA (160x120) / FRAMESIZE_QVGA (320x240) / FRAMESIZE_VGA (640x480)
#define FRAME_SIZE    FRAMESIZE_VGA

// Test switch: 0 = grayscale + software JPEG (works), 1 = ask sensor for hardware JPEG.
// If the sensor doesn't support it, camera init fails and prints an error.
#define USE_HW_JPEG   0

#define JPEG_QUALITY  50      // software mode: 1-100, higher = better/bigger
#define HW_JPEG_QUAL  12      // hardware mode: 0-63, LOWER = better
#define TARGET_FPS    60      // upper limit; real FPS is whatever the board can do

#define CHUNK_SIZE    1400
#define HDR_SIZE      7
// ------------------------------------------------

#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM   0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM     5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

struct Jpg { uint8_t *buf; size_t len; };

WiFiUDP udp;
QueueHandle_t jpgQueue;
static uint8_t pkt[HDR_SIZE + CHUNK_SIZE];
static uint16_t frameId = 0;

volatile uint32_t encodedFrames = 0, sentFrames = 0, droppedFrames = 0;
volatile uint32_t encMsAcc = 0, lastSize = 0;

void setupCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.sccb_i2c_port = 0;

#if USE_HW_JPEG
  c.pixel_format = PIXFORMAT_JPEG;
  c.jpeg_quality = HW_JPEG_QUAL;
#else
  c.pixel_format = PIXFORMAT_GRAYSCALE;
  c.jpeg_quality = 12;
#endif

  if (psramFound()) {
    c.frame_size  = FRAME_SIZE;
    c.fb_count    = 2;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode   = CAMERA_GRAB_LATEST;
  } else {
    c.frame_size  = FRAMESIZE_QQVGA;
    c.fb_count    = 1;
    c.fb_location = CAMERA_FB_IN_DRAM;
    c.grab_mode   = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x %s\n", err,
                  USE_HW_JPEG ? "(sensor probably has no hardware JPEG - set USE_HW_JPEG 0)" : "");
    while (true) delay(1000);
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
#if USE_STATIC_IP
  WiFi.config(local_IP, gateway, subnet, dns1);
#endif
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.setSleep(false);
  Serial.print("Connecting to WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
    if (millis() - t0 > 20000) ESP.restart();
  }
  Serial.print("\nConnected, IP: ");
  Serial.println(WiFi.localIP());
}

void sendJpeg(const uint8_t *jpg, size_t len) {
  uint16_t chunkCount = (len + CHUNK_SIZE - 1) / CHUNK_SIZE;
  pkt[0] = CAMERA_ID;
  memcpy(&pkt[1], &frameId, 2);
  memcpy(&pkt[5], &chunkCount, 2);

  for (uint16_t i = 0; i < chunkCount; i++) {
    size_t off = (size_t)i * CHUNK_SIZE;
    size_t n = (len - off < (size_t)CHUNK_SIZE) ? (len - off) : (size_t)CHUNK_SIZE;
    memcpy(&pkt[3], &i, 2);
    memcpy(&pkt[HDR_SIZE], jpg + off, n);

    udp.beginPacket(dest_ip, dest_port);
    udp.write(pkt, HDR_SIZE + n);
    udp.endPacket();

    if ((i & 7) == 7) delay(1);
  }
  frameId++;
}

// Core 1: grab frame + compress to JPEG
void captureTask(void *) {
  const uint32_t interval = 1000 / TARGET_FPS;
  for (;;) {
    uint32_t t0 = millis();

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

    Jpg j = { nullptr, 0 };
#if USE_HW_JPEG
    j.buf = (uint8_t *)malloc(fb->len);
    if (j.buf) { memcpy(j.buf, fb->buf, fb->len); j.len = fb->len; }
    esp_camera_fb_return(fb);
#else
    bool ok = frame2jpg(fb, JPEG_QUALITY, &j.buf, &j.len);
    esp_camera_fb_return(fb);
    if (!ok) j.buf = nullptr;
#endif
    if (!j.buf) continue;

    encMsAcc += millis() - t0;
    encodedFrames++;

    // If the sender is still busy, drop this frame instead of building latency
    if (xQueueSend(jpgQueue, &j, 0) != pdTRUE) {
      free(j.buf);
      droppedFrames++;
    }

    uint32_t el = millis() - t0;
    if (el < interval) vTaskDelay(pdMS_TO_TICKS(interval - el));
    else vTaskDelay(1);   // always yield so the watchdog stays happy
  }
}

// Core 0: send over UDP
void sendTask(void *) {
  Jpg j;
  for (;;) {
    if (xQueueReceive(jpgQueue, &j, portMAX_DELAY) == pdTRUE) {
      sendJpeg(j.buf, j.len);
      lastSize = j.len;
      free(j.buf);
      sentFrames++;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  setupCamera();
  connectWiFi();
  udp.begin(12345);

  jpgQueue = xQueueCreate(2, sizeof(Jpg));
  xTaskCreatePinnedToCore(captureTask, "capture", 8192, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(sendTask,    "send",    4096, NULL, 2, NULL, 0);

  Serial.printf("Streaming cam %d to %s:%u (%s)\n", CAMERA_ID,
                dest_ip.toString().c_str(), dest_port,
                USE_HW_JPEG ? "hardware JPEG" : "software JPEG");
}

void loop() {
  static uint32_t last = millis();
  if (WiFi.status() != WL_CONNECTED) connectWiFi();

  if (millis() - last >= 1000) {
    uint32_t enc = encodedFrames, sent = sentFrames, drop = droppedFrames;
    uint32_t avg = enc ? encMsAcc / enc : 0;
    Serial.printf("encode %u fps | sent %u fps | dropped %u | avg grab+encode %u ms | %u B/frame | RSSI %d\n",
                  enc, sent, drop, avg, (unsigned)lastSize, WiFi.RSSI());
    encodedFrames = sentFrames = droppedFrames = 0;
    encMsAcc = 0;
    last = millis();
  }
  delay(20);
}