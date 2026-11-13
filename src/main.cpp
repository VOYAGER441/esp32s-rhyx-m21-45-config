// ======= ESP32-CAM (GC2145) Multi-Camera UDP JPEG Streamer + HTTP Flashlight =======
// ======= + SD Card Footage Storage + HC-SR501 PIR Motion-Triggered Flash    =======
// Board:  AI-Thinker ESP32-CAM (RHYX-M21)
// Sensor: GC2145 (Grayscale capture -> Software JPEG compression -> UDP Stream)
// HTTP:   Web server hosting:
//           /flash?state=on|off              - manual flashlight control
//           /status                          - camera + SD + motion status
//           /footage                         - HTML listing of stored footage
//           /footage/file?path=<full path>   - serve one stored JPEG
//           /active-motion-detector?state=on|off - arm/disarm PIR-triggered flash + capture
//
// ARCHITECTURE (relay setup):
//   ESP32-CAM --(UDP, same home WiFi)--> Phone running Termux relay --(Cloudflare
//   Tunnel, outbound)--> Public URL --> React Native app / any viewer, any network
//
// IMPORTANT: dest_ip below must be the RELAY PHONE's LOCAL IP on your home WiFi
// (NOT a public/internet address). The ESP32 and the phone are on the SAME
// network, so this is a normal LAN UDP send - no NAT/port-forwarding involved
// here. The phone's Cloudflare Tunnel is what makes the stream reachable from
// outside afterwards; that happens entirely on the phone/relay side, not here.
//
// SD CARD NOTES (AI-Thinker board):
//   - Uses the onboard SD_MMC slot in 1-BIT mode (CLK=GPIO14, CMD=GPIO15, D0=GPIO2).
//     1-bit mode deliberately avoids D1/D2/D3 (GPIO4/12/13), because GPIO4 is
//     shared with the onboard flash LED and GPIO12 is a boot-strapping pin.
//   - "Continuous recording" here means periodic JPEG snapshots (not true video) -
//     the GC2145 has no hardware JPEG encoder and SD write speed + CPU budget on
//     this board can't sustain full-motion video encoding alongside UDP streaming.
//     Tune CONTINUOUS_SNAPSHOT_INTERVAL_MS to taste.
//   - There's no onboard RTC, so timestamps in filenames rely on NTP sync at boot.
//     If NTP fails (no internet route), filenames fall back to boot-relative time.
//
// PIR NOTES:
//   - HC-SR501 OUT pin -> GPIO13 (a normally-free, non-strapping pin on this board).
//     Power the PIR from 5V/GND. Give it ~30-60s to settle after power-up before
//     trusting readings (that's a PIR hardware property, not something this code
//     can shortcut).
//   - Motion detection is DISARMED by default. Hit /active-motion-detector?state=on
//     to arm it.
//
// Give the relay phone a FIXED local IP so this never has to change:
//   - Easiest: Android WiFi settings -> your network -> Advanced -> IP settings
//     -> Static, and set an IP outside your router's DHCP range (e.g. .120)
//   - Or: DHCP reservation for the phone's MAC address in your router admin page

#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "img_converters.h"
#include "esp_http_server.h"
#include "FS.h"
#include "SD_MMC.h"
#include <time.h>
#include <vector>
#include <algorithm>

// ----------------- USER CONFIGURATION -----------------
const char* ssid     = "";
const char* password = "INDINET#2.4G";

// --- CAMERA SPECIFIC SETTINGS (CHANGE THESE FOR EACH BOARD) ---
#define CAMERA_ID 1                                  // Unique ID: 1, 2, 3, etc.
const char* HOST_NAME = "esp32-cam-01";              // Unique hostname for local network
// IPAddress local_IP(192, 168, 0, 101);                // Unique Static IP (e.g., .101, .102, .103)
IPAddress local_IP(192, 168, 9, 51);                // Unique Static IP (e.g., .101, .102, .103)

// --- ROUTER CONFIGURATION ---
IPAddress gateway(192, 168, 9, 1);                   // Your Router Gateway IP
IPAddress subnet(255, 255, 255, 0);                  // Subnet Mask
IPAddress primaryDNS(8, 8, 8, 8);                    // Primary DNS (Google)
IPAddress secondaryDNS(8, 8, 4, 4);                  // Secondary DNS (Google)

// --- UDP TARGET SETTINGS (relay phone, LOCAL network address) ---
// IPAddress dest_ip(192, 168, 0, 100);                 // <-- SET TO YOUR RELAY PHONE'S LOCAL IP
IPAddress dest_ip(192, 168, 9, 36);                 // <-- SET TO YOUR RELAY PHONE'S LOCAL IP

const uint16_t dest_port      = 2222;                // Port the Termux relay listens on
const uint16_t LOCAL_UDP_PORT = 12345;

#define CHUNK_SIZE   1400
#define JPEG_QUALITY 60      // 10 (best) - 63 (worst)
#define STREAM_SKIP  1       // Subsampling (1 = full res)

// Live UDP stream is capped to this rate on purpose - running flat-out (no
// gap between frames) keeps the CPU busy encoding/transmitting non-stop,
// which is the main source of heat on this board. Lowering this trades
// stream smoothness for a cooler, lower-power board. Raise it back up if
// you need smoother motion and don't mind the extra heat.
#define STREAM_TARGET_FPS         9
#define STREAM_FRAME_INTERVAL_MS  (1000 / STREAM_TARGET_FPS)

// --- NTP (for readable timestamps on stored footage) ---
#define NTP_SERVER_1   "pool.ntp.org"
#define NTP_SERVER_2   "time.nist.gov"
#define GMT_OFFSET_SEC (5 * 3600 + 30 * 60)   // IST (UTC+5:30) - change if needed
#define DST_OFFSET_SEC 0

// --- SD Card / Footage storage ---
#define RECORD_ROOT                          "/footage"
#define CONTINUOUS_RECORDING_ENABLED         false   // motion-only capture mode; set true (and uncomment the task in setup()) to bring back periodic snapshots
#define CONTINUOUS_SNAPSHOT_INTERVAL_MIN_MS  5000    // fastest snapshot rate (card has room)
#define CONTINUOUS_SNAPSHOT_INTERVAL_MAX_MS  60000   // slowest snapshot rate (card nearly full)
#define SD_USAGE_SCALE_START_PCT             70      // usage% where interval starts stretching out
#define SD_PRUNE_CHECK_INTERVAL_MS           60000   // re-check storage usage every 60s
#define SD_USAGE_PRUNE_THRESHOLD_PCT         90      // still delete oldest continuous shots above this
#define SD_PRUNE_BATCH_COUNT                 20

// --- PIR Motion Detector (HC-SR501) ---
#define PIR_GPIO_NUM            16   // free, non-strapping GPIO on AI-Thinker board
#define MOTION_POLL_INTERVAL_MS 150
#define MOTION_HOLD_MS          15000  // keep flash on this long after last motion
#define MOTION_BURST_SHOTS      5
#define MOTION_BURST_INTERVAL_MS 350

// Pins Configuration
#define LED_FLASH_GPIO_NUM 4

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

WiFiUDP udp;
httpd_handle_t httpd_server = NULL;
static uint8_t *subBuf = nullptr;
static size_t   subBufCap = 0;

// --- Shared state ---
SemaphoreHandle_t camMutex = NULL;   // guards esp_camera_fb_get/return + frame2jpg
SemaphoreHandle_t sdMutex  = NULL;   // guards SD_MMC file operations
volatile bool sdCardReady           = false;
volatile bool motionDetectionActive = false;
volatile bool flashManualOn         = false; // set via /flash or serial
volatile bool flashMotionOn         = false; // set via PIR trigger
volatile uint32_t currentSnapshotIntervalMs = CONTINUOUS_SNAPSHOT_INTERVAL_MIN_MS; // adapts to SD usage
char camFolder[40];                          // e.g. "/footage/cam1"

// ----------------- CRC16 -----------------
uint16_t crc16(const uint8_t *data, uint32_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= *data++;
    for (int i = 0; i < 8; i++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc = (crc >> 1);
    }
  }
  return crc;
}

// ----------------- Grayscale Subsampling -----------------
uint8_t* subsampleGray(camera_fb_t *fb, int skip, uint16_t &outW, uint16_t &outH, size_t &outLen) {
  if (skip <= 1) {
    outW = fb->width; outH = fb->height; outLen = fb->len;
    return fb->buf;
  }
  outW = fb->width  / skip;
  outH = fb->height / skip;
  outLen = (size_t)outW * outH;

  if (subBufCap < outLen) {
    if (subBuf) free(subBuf);
    subBuf = (uint8_t*)malloc(outLen);
    subBufCap = outLen;
  }
  if (!subBuf) return nullptr;

  uint8_t *dst = subBuf;
  for (uint16_t y = 0; y < outH; y++) {
    const uint8_t *rowSrc = fb->buf + (size_t)y * skip * fb->width;
    for (uint16_t x = 0; x < outW; x++) {
      *dst++ = rowSrc[x * skip];
    }
  }
  return subBuf;
}

// ----------------- Shared frame capture (mutex-protected) -----------------
// Used by both the live UDP streamer and the SD recording/motion tasks so they
// never touch the camera driver at the same time. Caller must free(*outBuf).
bool captureJpegFrame(uint8_t **outBuf, size_t *outLen, uint16_t *outW, uint16_t *outH) {
  if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(2000)) != pdTRUE) return false;

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { xSemaphoreGive(camMutex); return false; }

  uint16_t subW, subH;
  size_t   subLen;
  uint8_t *subPixels = subsampleGray(fb, STREAM_SKIP, subW, subH, subLen);
  if (!subPixels) {
    esp_camera_fb_return(fb);
    xSemaphoreGive(camMutex);
    return false;
  }

  camera_fb_t tempFb = *fb;
  tempFb.buf    = subPixels;
  tempFb.width  = subW;
  tempFb.height = subH;
  tempFb.len    = subLen;

  bool ok = frame2jpg(&tempFb, JPEG_QUALITY, outBuf, outLen);
  esp_camera_fb_return(fb);
  xSemaphoreGive(camMutex);

  if (ok) { *outW = subW; *outH = subH; }
  return ok;
}

// ----------------- Flash helper (merges manual + motion sources) -----------------
void applyFlashState() {
  digitalWrite(LED_FLASH_GPIO_NUM, (flashManualOn || flashMotionOn) ? HIGH : LOW);
}

// ==================================================================
//                          SD CARD / FOOTAGE
// ==================================================================

bool initSDCard() {
  // true = force 1-bit mode (CLK14/CMD15/D0-2 only) so GPIO4 stays free for flash
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("[SD] Mount failed (or no card inserted) - continuing without local storage");
    return false;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("[SD] No SD card detected");
    return false;
  }
  Serial.printf("[SD] Card mounted. Total: %lluMB, Used: %lluMB\n",
                SD_MMC.totalBytes() / (1024ULL * 1024ULL),
                SD_MMC.usedBytes()  / (1024ULL * 1024ULL));
  return true;
}

// Creates every missing folder along a path, e.g. "/footage/cam1/motion"
bool makeDirRecursive(const String &path) {
  if (path.length() == 0 || path == "/") return true;
  String cur = "";
  int start = (path.charAt(0) == '/') ? 1 : 0;
  while (true) {
    int slash = path.indexOf('/', start);
    String segment = (slash == -1) ? path.substring(start) : path.substring(start, slash);
    if (segment.length() > 0) {
      cur += "/" + segment;
      if (!SD_MMC.exists(cur)) {
        if (!SD_MMC.mkdir(cur)) return false;
      }
    }
    if (slash == -1) break;
    start = slash + 1;
  }
  return true;
}

// Returns "YYYYMMDD_HHMMSS" if NTP has synced, else "boot_<millis>" as a
// monotonically-increasing fallback so filenames stay unique and sortable.
String getTimestampString() {
  time_t now = time(nullptr);
  if (now > 1700000000) { // sane epoch => NTP has synced
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    char buf[24];
    strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &timeinfo);
    return String(buf);
  }
  char buf[24];
  snprintf(buf, sizeof(buf), "boot_%010lu", (unsigned long)millis());
  return String(buf);
}

bool saveJpegToSD(const String &folder, const String &filename, uint8_t *buf, size_t len) {
  if (!sdCardReady) return false;
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(3000)) != pdTRUE) return false;

  makeDirRecursive(folder);
  String fullPath = folder + "/" + filename;
  File f = SD_MMC.open(fullPath, FILE_WRITE);
  bool ok = false;
  if (f) {
    ok = (f.write(buf, len) == len);
    f.close();
  } else {
    Serial.printf("[SD] Failed to open %s for writing\n", fullPath.c_str());
  }
  xSemaphoreGive(sdMutex);
  return ok;
}

// Deletes the oldest N files in a folder (filenames are timestamp-prefixed,
// so lexical sort == chronological sort). Used for basic storage pruning.
void pruneOldestInFolder(const String &folder, int deleteCount) {
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(3000)) != pdTRUE) return;

  File dir = SD_MMC.open(folder);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    xSemaphoreGive(sdMutex);
    return;
  }

  std::vector<String> names;
  File entry = dir.openNextFile();
  while (entry) {
    if (!entry.isDirectory()) {
      String raw = String(entry.name());
      int slashIdx = raw.lastIndexOf('/');
      names.push_back((slashIdx >= 0) ? raw.substring(slashIdx + 1) : raw);
    }
    entry.close();
    entry = dir.openNextFile();
  }
  dir.close();
  std::sort(names.begin(), names.end());

  for (int i = 0; i < deleteCount && i < (int)names.size(); i++) {
    String path = folder + "/" + names[i];
    SD_MMC.remove(path);
  }
  xSemaphoreGive(sdMutex);
}

// Stretches the continuous-snapshot interval out as the card fills up, so
// retention degrades gracefully (fewer, further-apart shots) instead of
// staying at full rate right up until pruning has to start deleting things.
// Below SD_USAGE_SCALE_START_PCT: full speed (MIN interval).
// Between SCALE_START and PRUNE_THRESHOLD: linearly ramps toward MAX interval.
// At/above PRUNE_THRESHOLD: capped at MAX interval, and pruning kicks in too.
void updateAdaptiveInterval() {
  if (!sdCardReady) {
    currentSnapshotIntervalMs = CONTINUOUS_SNAPSHOT_INTERVAL_MIN_MS;
    return;
  }
  uint64_t total = SD_MMC.totalBytes();
  uint64_t used  = SD_MMC.usedBytes();
  if (total == 0) return;

  int usedPct = (int)((used * 100) / total);
  uint32_t newInterval;

  if (usedPct <= SD_USAGE_SCALE_START_PCT) {
    newInterval = CONTINUOUS_SNAPSHOT_INTERVAL_MIN_MS;
  } else if (usedPct >= SD_USAGE_PRUNE_THRESHOLD_PCT) {
    newInterval = CONTINUOUS_SNAPSHOT_INTERVAL_MAX_MS;
  } else {
    float t = (float)(usedPct - SD_USAGE_SCALE_START_PCT) /
              (float)(SD_USAGE_PRUNE_THRESHOLD_PCT - SD_USAGE_SCALE_START_PCT);
    newInterval = CONTINUOUS_SNAPSHOT_INTERVAL_MIN_MS +
                  (uint32_t)(t * (CONTINUOUS_SNAPSHOT_INTERVAL_MAX_MS - CONTINUOUS_SNAPSHOT_INTERVAL_MIN_MS));
  }

  if (newInterval != currentSnapshotIntervalMs) {
    Serial.printf("[SD] Usage %d%% - snapshot interval now %lums\n", usedPct, (unsigned long)newInterval);
    currentSnapshotIntervalMs = newInterval;
  }
}

void checkAndPruneStorage() {
  if (!sdCardReady) return;
  uint64_t total = SD_MMC.totalBytes();
  uint64_t used  = SD_MMC.usedBytes();
  if (total == 0) return;

  int usedPct = (int)((used * 100) / total);
  if (usedPct >= SD_USAGE_PRUNE_THRESHOLD_PCT) {
    Serial.printf("[SD] Usage at %d%% - pruning oldest continuous footage\n", usedPct);
    pruneOldestInFolder(String(camFolder) + "/continuous", SD_PRUNE_BATCH_COUNT);
  }
}

// Recursively builds an HTML <li> listing for the /footage page (depth-limited).
// Opens each directory entry exactly once via openNextFile() - avoids the
// earlier approach of reopening every file a second time just to check its
// size/type, which roughly doubled SD access for no benefit.
struct FootageItem { String name; bool isDir; size_t size; };

void listDirHTML(const String &path, String &html, int depth) {
  if (depth > 4) return;
  File dir = SD_MMC.open(path);
  if (!dir || !dir.isDirectory()) { if (dir) dir.close(); return; }

  std::vector<FootageItem> items;
  File entry = dir.openNextFile();
  while (entry) {
    String raw = String(entry.name());
    int slashIdx = raw.lastIndexOf('/');
    String baseName = (slashIdx >= 0) ? raw.substring(slashIdx + 1) : raw;
    items.push_back({ baseName, entry.isDirectory(), entry.isDirectory() ? 0 : entry.size() });
    entry.close();
    entry = dir.openNextFile();
  }
  dir.close();

  std::sort(items.begin(), items.end(), [](const FootageItem &a, const FootageItem &b) {
    return a.name < b.name;
  });

  for (auto &it : items) {
    String childPath = path + "/" + it.name;
    if (it.isDir) {
      html += "<li><strong>" + childPath + "/</strong><ul>";
      listDirHTML(childPath, html, depth + 1);
      html += "</ul></li>";
    } else {
      html += "<li><a href=\"/footage/file?path=" + childPath + "\">" + it.name +
              "</a> (" + String(it.size) + " bytes)</li>";
    }
  }
}

// ==================================================================
//                     BACKGROUND TASKS (SD + PIR)
// ==================================================================

// Periodic snapshot recorder - runs independently of live UDP streaming
void continuousRecordTask(void *pv) {
  for (;;) {
    if (sdCardReady && CONTINUOUS_RECORDING_ENABLED) {
      uint8_t *buf; size_t len; uint16_t w, h;
      if (captureJpegFrame(&buf, &len, &w, &h)) {
        saveJpegToSD(String(camFolder) + "/continuous", getTimestampString() + ".jpg", buf, len);
        free(buf);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(currentSnapshotIntervalMs));
  }
}

// PIR polling + motion-triggered flash/capture + storage pruning
void motionTask(void *pv) {
  pinMode(PIR_GPIO_NUM, INPUT);
  unsigned long lastMotionTime = 0;
  unsigned long lastPruneCheck = 0;

  for (;;) {
    if (motionDetectionActive) {
      if (digitalRead(PIR_GPIO_NUM) == HIGH) {
        lastMotionTime = millis();

        if (!flashMotionOn) {
          flashMotionOn = true;
          applyFlashState();
          Serial.println("[MOTION] Detected - flashlight ON, capturing burst");

          if (sdCardReady) {
            String burstFolder = String(camFolder) + "/motion/" + getTimestampString();
            for (int i = 0; i < MOTION_BURST_SHOTS; i++) {
              uint8_t *buf; size_t len; uint16_t w, h;
              if (captureJpegFrame(&buf, &len, &w, &h)) {
                saveJpegToSD(burstFolder, "shot_" + String(i) + ".jpg", buf, len);
                free(buf);
              }
              vTaskDelay(pdMS_TO_TICKS(MOTION_BURST_INTERVAL_MS));
            }
          }
        }
      }

      // Turn the motion-driven flash off after a quiet period (manual flash unaffected)
      if (flashMotionOn && (millis() - lastMotionTime > MOTION_HOLD_MS)) {
        flashMotionOn = false;
        applyFlashState();
        Serial.println("[MOTION] Timeout - flashlight OFF (motion-triggered only)");
      }
    } else if (flashMotionOn) {
      // Detector was disarmed mid-trigger - release the motion-held flash
      flashMotionOn = false;
      applyFlashState();
    }

    if (millis() - lastPruneCheck > SD_PRUNE_CHECK_INTERVAL_MS) {
      updateAdaptiveInterval();
      checkAndPruneStorage();
      lastPruneCheck = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(MOTION_POLL_INTERVAL_MS));
  }
}

// ==================================================================
//                          CAMERA + STREAMING
// ==================================================================

void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;

  if (psramFound()) {
    config.frame_size  = FRAMESIZE_HVGA;   // 640x480
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size  = FRAMESIZE_QQVGA;
    config.fb_count     = 1;
    config.fb_location  = CAMERA_FB_IN_DRAM;
    config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init FAILED: 0x%x\n", err);
    while (true) delay(1000);
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
  s->set_brightness(s, 1);
  s->set_contrast(s, 1);
  s->set_saturation(s, -1);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_aec2(s, 1);
  s->set_gain_ctrl(s, 1);
  s->set_exposure_ctrl(s, 1);
}

// UDP Streaming Task (Runs on Core 1)
void udpStreamerTask(void *pv) {
  uint32_t frameCount = 0;
  uint32_t fpsWindowStart = millis();

  for (;;) {
    uint32_t frameStart = millis();

    uint8_t *jpg_buf = NULL;
    size_t jpg_len = 0;
    uint16_t subW, subH;

    if (!captureJpegFrame(&jpg_buf, &jpg_len, &subW, &subH)) {
      vTaskDelay(pdMS_TO_TICKS(20)); // camera busy/failed - back off briefly, don't spin hot
      continue;
    }

    uint16_t crc = crc16(jpg_buf, jpg_len);
    uint32_t len32 = (uint32_t)jpg_len;
    uint8_t  camID = (uint8_t)CAMERA_ID;

    // Send Header Packet: [1 Byte Camera ID] + [4 Bytes Frame Length]
    udp.beginPacket(dest_ip, dest_port);
    udp.write(&camID, 1);
    udp.write((uint8_t*)&len32, 4);
    udp.endPacket();

    // Stream JPEG Data in Chunks
    for (uint32_t sent = 0; sent < jpg_len; sent += CHUNK_SIZE) {
      size_t remaining = jpg_len - sent;
      size_t n = (remaining < (size_t)CHUNK_SIZE) ? remaining : (size_t)CHUNK_SIZE;
      udp.beginPacket(dest_ip, dest_port);
      udp.write(jpg_buf + sent, n);
      udp.endPacket();
    }

    // Send Footer Packet: [1 Byte Camera ID] + [2 Bytes CRC16]
    udp.beginPacket(dest_ip, dest_port);
    udp.write(&camID, 1);
    udp.write((uint8_t*)&crc, 2);
    udp.endPacket();

    free(jpg_buf);

    frameCount++;
    uint32_t now = millis();
    if (now - fpsWindowStart >= 1000) {
      Serial.printf("[CAM %u] FPS: %u  (%ux%u -> JPEG %u bytes)  RSSI: %d\n",
                     camID, frameCount, subW, subH, (unsigned)jpg_len, WiFi.RSSI());
      frameCount = 0;
      fpsWindowStart = now;
    }

    // Pace to the target FPS instead of looping flat-out - this is the main
    // lever for reducing CPU load and board heat. If encoding+sending this
    // frame already took longer than the target interval, just yield once.
    uint32_t frameElapsed = millis() - frameStart;
    if (frameElapsed < STREAM_FRAME_INTERVAL_MS) {
      vTaskDelay(pdMS_TO_TICKS(STREAM_FRAME_INTERVAL_MS - frameElapsed));
    } else {
      vTaskDelay(1);
    }
  }
}

// ==================================================================
//                           HTTP HANDLERS
// ==================================================================

esp_err_t flash_handler(httpd_req_t *req) {
  char query[32];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    if (strstr(query, "state=on")) {
      flashManualOn = true;
      applyFlashState();
      httpd_resp_sendstr(req, "Flashlight turned ON");
      return ESP_OK;
    } else if (strstr(query, "state=off")) {
      flashManualOn = false;
      applyFlashState();
      httpd_resp_sendstr(req, "Flashlight turned OFF");
      return ESP_OK;
    }
  }
  httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Use ?state=on or ?state=off");
  return ESP_FAIL;
}

esp_err_t motion_handler(httpd_req_t *req) {
  char query[32];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    if (strstr(query, "state=on")) {
      motionDetectionActive = true;
      httpd_resp_sendstr(req, "Motion detector ARMED - flashlight + capture will trigger on motion");
      return ESP_OK;
    } else if (strstr(query, "state=off")) {
      motionDetectionActive = false;
      httpd_resp_sendstr(req, "Motion detector DISARMED");
      return ESP_OK;
    }
  }
  httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Use ?state=on or ?state=off");
  return ESP_FAIL;
}

esp_err_t footage_list_handler(httpd_req_t *req) {
  String html = "<html><head><title>Footage - Cam " + String(CAMERA_ID) + "</title></head><body>";
  html += "<h2>Camera " + String(CAMERA_ID) + " Footage</h2>";
  if (!sdCardReady) {
    html += "<p>No SD card detected on this unit.</p>";
  } else {
    html += "<p>Motion detector: " + String(motionDetectionActive ? "ARMED" : "disarmed") + "</p><ul>";
    // Same lock the background recording/motion/prune tasks use - without this,
    // the listing can collide with an in-progress SD write and stall badly.
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(3000)) == pdTRUE) {
      listDirHTML(String(camFolder), html, 0);
      xSemaphoreGive(sdMutex);
    } else {
      html += "<li>SD card busy right now - refresh in a moment</li>";
    }
    html += "</ul>";
  }
  html += "</body></html>";
  httpd_resp_set_type(req, "text/html");
  httpd_resp_sendstr(req, html.c_str());
  return ESP_OK;
}

esp_err_t footage_file_handler(httpd_req_t *req) {
  if (!sdCardReady) {
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No SD card");
    return ESP_FAIL;
  }

  char query[160];
  char pathBuf[128];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "path", pathBuf, sizeof(pathBuf)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Use ?path=<full footage path>");
    return ESP_FAIL;
  }

  String path = String(pathBuf);
  // Basic traversal protection: only serve files under our own footage root
  if (!path.startsWith(RECORD_ROOT)) {
    httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Invalid path");
    return ESP_FAIL;
  }

  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(3000)) != pdTRUE) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "SD busy");
    return ESP_FAIL;
  }

  File f = SD_MMC.open(path, FILE_READ);
  if (!f || f.isDirectory()) {
    if (f) f.close();
    xSemaphoreGive(sdMutex);
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
    return ESP_FAIL;
  }

  // These snapshots are only a few KB (grayscale JPEGs), so read the whole
  // file into RAM and send it in one shot - simpler and more robust than a
  // chunked read/send loop, and avoids holding the SD card open/mutexed for
  // the duration of a slow network write.
  size_t fileSize = f.size();
  const size_t MAX_SERVE_SIZE = 2 * 1024 * 1024; // 2MB safety cap
  if (fileSize == 0 || fileSize > MAX_SERVE_SIZE) {
    f.close();
    xSemaphoreGive(sdMutex);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "File empty or too large");
    return ESP_FAIL;
  }

  uint8_t *fileBuf = (uint8_t*)malloc(fileSize);
  if (!fileBuf) {
    f.close();
    xSemaphoreGive(sdMutex);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
    return ESP_FAIL;
  }

  size_t readBytes = f.read(fileBuf, fileSize);
  f.close();
  xSemaphoreGive(sdMutex);

  if (readBytes != fileSize) {
    free(fileBuf);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "SD read failed");
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_send(req, (const char*)fileBuf, readBytes);
  free(fileBuf);
  return ESP_OK;
}

esp_err_t status_handler(httpd_req_t *req) {
  char buf[300];
  snprintf(buf, sizeof(buf),
           "CAMERA_ID=%d IP=%s RSSI=%d dest=%s:%u sd=%s motion_armed=%s flash=%s snapshot_interval_ms=%lu",
           CAMERA_ID, WiFi.localIP().toString().c_str(), WiFi.RSSI(),
           dest_ip.toString().c_str(), dest_port,
           sdCardReady ? "ready" : "absent",
           motionDetectionActive ? "yes" : "no",
           (flashManualOn || flashMotionOn) ? "on" : "off",
           (unsigned long)currentSnapshotIntervalMs);
  httpd_resp_sendstr(req, buf);
  return ESP_OK;
}

void startWebServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 10;
  config.stack_size = 8192; // extra headroom for the footage-serving handlers

  httpd_uri_t flash_uri        = { "/flash",                  HTTP_GET, flash_handler,        NULL };
  httpd_uri_t status_uri       = { "/status",                 HTTP_GET, status_handler,       NULL };
  httpd_uri_t motion_uri       = { "/active-motion-detector", HTTP_GET, motion_handler,       NULL };
  httpd_uri_t footage_uri      = { "/footage",                HTTP_GET, footage_list_handler, NULL };
  httpd_uri_t footage_file_uri = { "/footage/file",            HTTP_GET, footage_file_handler, NULL };

  if (httpd_start(&httpd_server, &config) == ESP_OK) {
    httpd_register_uri_handler(httpd_server, &flash_uri);
    httpd_register_uri_handler(httpd_server, &status_uri);
    httpd_register_uri_handler(httpd_server, &motion_uri);
    httpd_register_uri_handler(httpd_server, &footage_uri);
    httpd_register_uri_handler(httpd_server, &footage_file_uri);
    Serial.println("HTTP Server started successfully!");
  }
}

// Check Serial Monitor Input
void checkSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); command.toUpperCase();

    if (command == "1" || command == "ON") {
      flashManualOn = true;
      applyFlashState();
      Serial.println("Flashlight turned ON via Serial");
    } else if (command == "0" || command == "OFF") {
      flashManualOn = false;
      applyFlashState();
      Serial.println("Flashlight turned OFF via Serial");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n--- Starting ESP32-CAM [ID: %d] ---\n", CAMERA_ID);

  pinMode(LED_FLASH_GPIO_NUM, OUTPUT);
  digitalWrite(LED_FLASH_GPIO_NUM, LOW);

  setupCamera();

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOST_NAME);

  // Apply Static IP Configuration
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Failed to configure Static IP!");
  }

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("Connecting to WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
    if (millis() - start > 20000) {
      ESP.restart();
    }
  }

  Serial.println("\nWiFi Connected!");
  Serial.print("Camera ID: "); Serial.println(CAMERA_ID);
  Serial.print("Fixed IP Address: "); Serial.println(WiFi.localIP());
  Serial.printf("Streaming to relay %s:%u\n", dest_ip.toString().c_str(), dest_port);

  // NTP sync (best-effort, short wait) so footage filenames get real timestamps
  configTime(GMT_OFFSET_SEC, DST_OFFSET_SEC, NTP_SERVER_1, NTP_SERVER_2);
  time_t nowCheck = time(nullptr);
  unsigned long ntpStart = millis();
  while (nowCheck < 1700000000 && millis() - ntpStart < 5000) {
    delay(200);
    nowCheck = time(nullptr);
  }
  Serial.println(nowCheck > 1700000000 ? "[NTP] Time synced" : "[NTP] Sync failed - using boot-relative filenames");

  // SD card + folder layout
  sdCardReady = initSDCard();
  snprintf(camFolder, sizeof(camFolder), "%s/cam%d", RECORD_ROOT, CAMERA_ID);
  if (sdCardReady) {
    makeDirRecursive(String(camFolder) + "/continuous");
    makeDirRecursive(String(camFolder) + "/motion");
  }

  camMutex = xSemaphoreCreateMutex();
  sdMutex  = xSemaphoreCreateMutex();

  udp.begin(LOCAL_UDP_PORT);
  startWebServer();

  // Live UDP streaming - Core 1
  xTaskCreatePinnedToCore(udpStreamerTask, "UDPStream", 8192, NULL, 1, NULL, 1);

  // Continuous periodic snapshots - DISABLED. Only motion-triggered captures
  // run now. Uncomment this line to bring back the always-on snapshot timer.
  // xTaskCreatePinnedToCore(continuousRecordTask, "SDRecord", 8192, NULL, 1, NULL, 0);

  // SD card housekeeping (storage pruning) + PIR motion watch - Core 0
  xTaskCreatePinnedToCore(motionTask, "MotionPIR", 8192, NULL, 1, NULL, 0);
}

void loop() {
  checkSerialCommands();
  vTaskDelay(50 / portTICK_PERIOD_MS);
}
