// ESP32-CAM (GC2145) -> UDP JPEG streamer
// Each frame is split into STRIPS horizontal strips, JPEG-encoded in parallel on both cores.
// Packet: [camID 1B][frameID 2B][stripIdx 1B][stripCount 1B][chunkIdx 2B][chunkCount 2B][payload]

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

#define FRAME_SIZE    FRAMESIZE_HVGA      // or FRAMESIZE_SVGA
#define STRIPS        1                  // 2 = parallel encode on both cores, 1 = single encoder
#define JPEG_QUALITY  20                 // 1-100; lower = faster encode + smaller packets
#define TARGET_FPS    60                 // upper limit only

#define CHUNK_SIZE    1400
#define HDR_SIZE      9
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
struct FrameOut { Jpg strip[2]; };

WiFiUDP udp;
QueueHandle_t outQueue;
SemaphoreHandle_t helperStart, helperDone;
static camera_fb_t *sharedFb = nullptr;
static Jpg helperOut = { nullptr, 0 };

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
  c.pixel_format = PIXFORMAT_GRAYSCALE;   // 1 byte/pixel = cheapest to encode
  c.jpeg_quality = 12;

  if (psramFound()) {
    c.frame_size  = FRAME_SIZE;
    c.fb_count    = 2;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode   = CAMERA_GRAB_LATEST;
  } else {
    Serial.println("No PSRAM found - VGA/SVGA need PSRAM, falling back to QQVGA");
    c.frame_size  = FRAMESIZE_QQVGA;
    c.fb_count    = 1;
    c.fb_location = CAMERA_FB_IN_DRAM;
    c.grab_mode   = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
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

// Encode one horizontal strip of the grayscale frame
static bool encodeStrip(camera_fb_t *fb, int idx, Jpg &out) {
  int h = fb->height / STRIPS;
  camera_fb_t t = *fb;
  t.buf    = fb->buf + (size_t)idx * h * fb->width;
  t.height = h;
  t.len    = (size_t)h * fb->width;
  out.buf = nullptr;
  out.len = 0;
  return frame2jpg(&t, JPEG_QUALITY, &out.buf, &out.len) && out.buf;
}

static void sendStrip(const Jpg &j, uint8_t stripIdx) {
  uint16_t chunkCount = (j.len + CHUNK_SIZE - 1) / CHUNK_SIZE;
  pkt[0] = CAMERA_ID;
  memcpy(&pkt[1], &frameId, 2);
  pkt[3] = stripIdx;
  pkt[4] = STRIPS;
  memcpy(&pkt[7], &chunkCount, 2);

  for (uint16_t i = 0; i < chunkCount; i++) {
    size_t off = (size_t)i * CHUNK_SIZE;
    size_t n = (j.len - off < (size_t)CHUNK_SIZE) ? (j.len - off) : (size_t)CHUNK_SIZE;
    memcpy(&pkt[5], &i, 2);
    memcpy(&pkt[HDR_SIZE], j.buf + off, n);

    udp.beginPacket(dest_ip, dest_port);
    udp.write(pkt, HDR_SIZE + n);
    udp.endPacket();

    if ((i & 7) == 7) delay(1);
  }
}

// Core 0: encodes the bottom strip while core 1 encodes the top strip
void helperTask(void *) {
  for (;;) {
    xSemaphoreTake(helperStart, portMAX_DELAY);
    encodeStrip(sharedFb, 1, helperOut);
    xSemaphoreGive(helperDone);
  }
}

// Core 1: grab frame, encode top strip (+ trigger helper), queue result
void captureTask(void *) {
  const uint32_t interval = 1000 / TARGET_FPS;
  for (;;) {
    uint32_t t0 = millis();

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

    FrameOut o = {};
#if STRIPS == 2
    sharedFb = fb;
    xSemaphoreGive(helperStart);
#endif
    bool ok = encodeStrip(fb, 0, o.strip[0]);
#if STRIPS == 2
    xSemaphoreTake(helperDone, portMAX_DELAY);
    o.strip[1] = helperOut;
    ok = ok && o.strip[1].buf;
#endif
    esp_camera_fb_return(fb);

    if (!ok) {
      for (int i = 0; i < STRIPS; i++) if (o.strip[i].buf) free(o.strip[i].buf);
      vTaskDelay(1);
      continue;
    }

    encMsAcc += millis() - t0;
    encodedFrames++;

    if (xQueueSend(outQueue, &o, 0) != pdTRUE) {   // sender busy -> drop, don't add latency
      for (int i = 0; i < STRIPS; i++) free(o.strip[i].buf);
      droppedFrames++;
    }

    uint32_t el = millis() - t0;
    if (el < interval) vTaskDelay(pdMS_TO_TICKS(interval - el));
    else vTaskDelay(1);
  }
}

// Core 0: send all strips of a frame over UDP
void sendTask(void *) {
  FrameOut o;
  for (;;) {
    if (xQueueReceive(outQueue, &o, portMAX_DELAY) == pdTRUE) {
      size_t total = 0;
      for (int i = 0; i < STRIPS; i++) {
        sendStrip(o.strip[i], i);
        total += o.strip[i].len;
        free(o.strip[i].buf);
      }
      frameId++;
      lastSize = total;
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

  outQueue    = xQueueCreate(2, sizeof(FrameOut));
  helperStart = xSemaphoreCreateBinary();
  helperDone  = xSemaphoreCreateBinary();

  xTaskCreatePinnedToCore(captureTask, "capture", 10240, NULL, 2, NULL, 1);
#if STRIPS == 2
  xTaskCreatePinnedToCore(helperTask,  "helper",  10240, NULL, 2, NULL, 0);
#endif
  xTaskCreatePinnedToCore(sendTask,    "send",     4096, NULL, 2, NULL, 0);

  Serial.printf("Streaming cam %d to %s:%u  strips=%d\n", CAMERA_ID,
                dest_ip.toString().c_str(), dest_port, STRIPS);
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