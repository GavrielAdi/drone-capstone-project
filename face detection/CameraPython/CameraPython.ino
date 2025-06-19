// —— Konfigurasi pin kamera untuk ESP32-CAM AI Thinker ——
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

#include <WiFi.h>
#include <HTTPClient.h>
#include "esp_camera.h"
#include <WebServer.h>

const char* ssid     = "YOUR_SSID";
const char* password = "PASSWORD";
String serverUrl     = "http://IP_ADDRESS:5000/upload"; //cek ipconfig di cmd

WebServer server(80);
unsigned long lastUpload = 0;
const unsigned long uploadInterval = 50; // ms = 1 detik

bool initCamera() {
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 10;
  config.fb_count     = 1;
  if (psramFound()) {
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }
  return true;
}

// Kirim gambar ke Flask server
void sendImageToServer() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Auto capture failed");
    return;
  }

  HTTPClient http;
  http.begin(serverUrl);
  http.addHeader("Content-Type", "image/jpeg");
  int code = http.POST(fb->buf, fb->len);
  Serial.printf("Auto upload response: %d\n", code);
  http.end();

  esp_camera_fb_return(fb);
}

// Endpoint manual: /capture
void handleCapture() {
  Serial.println("Manual capture...");
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Capture failed");
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }

  Serial.printf("Captured %u bytes\n", fb->len);

  HTTPClient http;
  http.begin(serverUrl);
  http.addHeader("Content-Type", "image/jpeg");
  int code = http.POST(fb->buf, fb->len);
  Serial.printf("Upload response: %d\n", code);

  if (code > 0) {
    server.send(200, "text/plain", "Image uploaded");
  } else {
    server.send(500, "text/plain", "Upload failed");
  }

  http.end();
  esp_camera_fb_return(fb);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Inisialisasi kamera
  if (!initCamera()) {
    Serial.println("Camera init failed");
    ESP.restart();
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_brightness(s, 1);    // dari 2 → 1
  s->set_contrast(s, 1);      // dari 2 → 1
  s->set_saturation(s, 0);    // dari 1 → 0
  s->set_gainceiling(s, (gainceiling_t)4); // dari 6 → 4
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_exposure_ctrl(s, 1);



  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // Mulai server
  server.on("/capture", HTTP_GET, handleCapture);
  server.begin();
  Serial.println("HTTP server started");
}


void loop() {
  server.handleClient();

  // auto-upload tiap 1 detik
  if (millis() - lastUpload > uploadInterval) {
    lastUpload = millis();
    sendImageToServer();
  }

  delay(0);
}
