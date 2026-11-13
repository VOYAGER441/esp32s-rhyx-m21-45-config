#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include "img_converters.h"
#include "esp_http_server.h"

// ================ WIFI CONFIG =================

const char* WIFI_SSID = "Mainak_777";
const char* WIFI_PASS = "1234567890";

#define USE_STATIC_IP true

IPAddress local_IP(192, 168, 0, 101);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(8, 8, 8, 8);

// ================ CAMERA CONFIG =================

#define FRAME_SIZE FRAMESIZE_HVGA
#define JPEG_QUALITY 90

// GC2145 pin configuration
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

// ================================================

static httpd_handle_t camera_httpd = NULL;

volatile uint32_t streamedFrames = 0;

// ================ CAMERA INITIALIZATION ================

void setupCamera() {

    camera_config_t c = {};

    c.ledc_channel = LEDC_CHANNEL_0;
    c.ledc_timer   = LEDC_TIMER_0;

    c.pin_d0 = Y2_GPIO_NUM;
    c.pin_d1 = Y3_GPIO_NUM;
    c.pin_d2 = Y4_GPIO_NUM;
    c.pin_d3 = Y5_GPIO_NUM;
    c.pin_d4 = Y6_GPIO_NUM;
    c.pin_d5 = Y7_GPIO_NUM;
    c.pin_d6 = Y8_GPIO_NUM;
    c.pin_d7 = Y9_GPIO_NUM;

    c.pin_xclk = XCLK_GPIO_NUM;
    c.pin_pclk = PCLK_GPIO_NUM;
    c.pin_vsync = VSYNC_GPIO_NUM;
    c.pin_href = HREF_GPIO_NUM;

    c.pin_sccb_sda = SIOD_GPIO_NUM;
    c.pin_sccb_scl = SIOC_GPIO_NUM;

    c.pin_pwdn  = PWDN_GPIO_NUM;
    c.pin_reset = RESET_GPIO_NUM;

    c.xclk_freq_hz = 20000000;

    // GC2145 supported format
    c.pixel_format = PIXFORMAT_GRAYSCALE;

    if (psramFound()) {

        Serial.println("PSRAM detected");

        c.frame_size = FRAME_SIZE;
        c.fb_count = 2;

        c.fb_location = CAMERA_FB_IN_PSRAM;
        c.grab_mode = CAMERA_GRAB_LATEST;

    } else {

        Serial.println("PSRAM not found");

        c.frame_size = FRAMESIZE_QQVGA;
        c.fb_count = 1;

        c.fb_location = CAMERA_FB_IN_DRAM;
        c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    }

    esp_err_t err = esp_camera_init(&c);

    if (err != ESP_OK) {

        Serial.printf(
            "Camera initialization failed: 0x%x\n",
            err
        );

        while (true) {
            delay(1000);
        }
    }

    sensor_t *s = esp_camera_sensor_get();

    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);

    Serial.println("GC2145 initialized successfully");
}

// ================ WIFI CONNECTION =================

void connectWiFi() {

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

#if USE_STATIC_IP
    if (!WiFi.config(local_IP, gateway, subnet, dns1)) {
        Serial.println("Static IP configuration failed");
    }
#endif

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.print("Connecting to WiFi");

    uint32_t start = millis();

    while (WiFi.status() != WL_CONNECTED) {

        delay(300);
        Serial.print(".");

        if (millis() - start > 20000) {
            ESP.restart();
        }
    }

    Serial.println("\nWiFi connected");

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    Serial.print("RSSI: ");
    Serial.println(WiFi.RSSI());
}

// ================ WEB PAGE =================

static esp_err_t indexHandler(httpd_req_t *req) {

    static const char html[] PROGMEM = R"rawliteral(

<!DOCTYPE html>
<html>

<head>

<meta name="viewport" content="width=device-width, initial-scale=1">

<title>ESP32 Camera</title>

<style>

body {
    background: #101010;
    color: white;
    font-family: Arial;
    text-align: center;
    margin: 0;
    padding: 20px;
}

h2 {
    margin-bottom: 20px;
}

img {
    width: 100%;
    max-width: 900px;
    border-radius: 10px;
    background: black;
}

.status {
    margin-top: 15px;
    color: #00ff99;
}

</style>

</head>

<body>

<h2>GC2145 Live Camera</h2>

<img id="camera" src="/stream">

<div class="status">
    ESP32 Local Camera Server
</div>

</body>

</html>

)rawliteral";

    httpd_resp_set_type(req, "text/html");

    return httpd_resp_send(
        req,
        html,
        HTTPD_RESP_USE_STRLEN
    );
}

// ================ MJPEG STREAM HANDLER =================

static esp_err_t streamHandler(httpd_req_t *req) {

    esp_err_t res = ESP_OK;

    httpd_resp_set_type(
        req,
        "multipart/x-mixed-replace;boundary=frame"
    );

    httpd_resp_set_hdr(
        req,
        "Access-Control-Allow-Origin",
        "*"
    );

    httpd_resp_set_hdr(
        req,
        "Cache-Control",
        "no-cache"
    );

    httpd_resp_set_hdr(
        req,
        "Pragma",
        "no-cache"
    );

    camera_fb_t *fb = NULL;

    uint8_t *jpg_buf = NULL;
    size_t jpg_len = 0;

    char part_header[96];

    Serial.println("Browser connected to camera stream");

    while (true) {

        // Capture frame
        fb = esp_camera_fb_get();

        if (!fb) {

            Serial.println("Camera capture failed");

            res = ESP_FAIL;
            break;
        }

        // Convert RGB565 to JPEG
        bool converted = frame2jpg(
            fb,
            JPEG_QUALITY,
            &jpg_buf,
            &jpg_len
        );

        // Camera buffer can be returned after conversion
        esp_camera_fb_return(fb);
        fb = NULL;

        if (!converted || !jpg_buf) {

            if (jpg_buf) {
                free(jpg_buf);
                jpg_buf = NULL;
            }

            Serial.println("JPEG conversion failed");

            res = ESP_FAIL;
            break;
        }

        // Multipart boundary
        static const char boundary[] = "--frame\r\n";

        res = httpd_resp_send_chunk(
            req,
            boundary,
            sizeof(boundary) - 1
        );

        if (res != ESP_OK) {
            free(jpg_buf);
            jpg_buf = NULL;
            break;
        }

        // JPEG frame header
        int header_len = snprintf(
            part_header,
            sizeof(part_header),
            "Content-Type: image/jpeg\r\n"
            "Content-Length: %u\r\n\r\n",
            (unsigned int)jpg_len
        );

        res = httpd_resp_send_chunk(
            req,
            part_header,
            header_len
        );

        if (res != ESP_OK) {
            free(jpg_buf);
            jpg_buf = NULL;
            break;
        }

        // Send JPEG data
        res = httpd_resp_send_chunk(
            req,
            (const char *)jpg_buf,
            jpg_len
        );

        free(jpg_buf);
        jpg_buf = NULL;

        if (res != ESP_OK) {
            break;
        }

        // End of frame
        res = httpd_resp_send_chunk(
            req,
            "\r\n",
            2
        );

        if (res != ESP_OK) {
            break;
        }

        streamedFrames++;
    }

    if (fb) {
        esp_camera_fb_return(fb);
    }

    if (jpg_buf) {
        free(jpg_buf);
    }

    Serial.println("Browser disconnected");

    return res;
}

// ================ START HTTP SERVER =================

void startCameraServer() {

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.server_port = 80;
    config.ctrl_port = 32768;

    config.max_uri_handlers = 4;
    config.stack_size = 8192;

    config.lru_purge_enable = true;

    if (httpd_start(&camera_httpd, &config) == ESP_OK) {

        httpd_uri_t index_uri = {};

        index_uri.uri = "/";
        index_uri.method = HTTP_GET;
        index_uri.handler = indexHandler;

        httpd_register_uri_handler(
            camera_httpd,
            &index_uri
        );

        httpd_uri_t stream_uri = {};

        stream_uri.uri = "/stream";
        stream_uri.method = HTTP_GET;
        stream_uri.handler = streamHandler;

        httpd_register_uri_handler(
            camera_httpd,
            &stream_uri
        );

        Serial.println("HTTP server started");

    } else {

        Serial.println("HTTP server failed to start");
    }
}

// ================ SETUP =================

void setup() {

    Serial.begin(115200);

    delay(500);

    Serial.println();
    Serial.println("==============================");
    Serial.println(" ESP32 GC2145 CAMERA SERVER");
    Serial.println("==============================");

    setupCamera();

    connectWiFi();

    startCameraServer();

    Serial.println();
    Serial.println("Camera server ready!");

    Serial.print("Open: http://");
    Serial.println(WiFi.localIP());

    Serial.println("==============================");
}

// ================ LOOP =================

void loop() {

    static uint32_t last = millis();

    if (WiFi.status() != WL_CONNECTED) {

        Serial.println("WiFi disconnected");

        WiFi.reconnect();
    }

    if (millis() - last >= 1000) {

        uint32_t fps = streamedFrames;

        streamedFrames = 0;

        Serial.printf(
            "Stream FPS: %lu | RSSI: %d dBm | Free Heap: %u | PSRAM: %u\n",
            (unsigned long)fps,
            WiFi.RSSI(),
            ESP.getFreeHeap(),
            ESP.getFreePsram()
        );

        last = millis();
    }

    delay(10);
}