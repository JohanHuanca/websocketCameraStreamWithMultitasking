#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_camera.h"
#include <WiFi.h>
#include <WebSocketsClient.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h" //disable brownout problems
#include "driver/gpio.h"


// configuration for AI Thinker Camera board
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

IPAddress ip(192, 168, 1, 200);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

// Datos WiFi
const char* ssid = "Wonderland";
const char* password = "christian7795";

// Configuración del servidor WebSocket
const char* ws_host = "192.168.1.47";
//const char* ws_host = "robothumanoidbackend-production.up.railway.app";
const uint16_t ws_port = 8080;
//const uint16_t ws_port = 443;
const char* ws_handler = "/api/v1/images/receive";

WebSocketsClient webSocket; // Objeto WebSocket

camera_fb_t * fb = NULL;
size_t _jpg_buf_len = 0;
uint8_t * _jpg_buf = NULL;

// Estructura para los datos de la imagen
struct ImageData {
    camera_fb_t *fb;
};


// Cola para transferir datos de imagen entre tareas
QueueHandle_t imageQueue;

esp_err_t init_camera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // parameters for image quality and size
  config.frame_size = FRAMESIZE_SVGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
  config.jpeg_quality = 20; //10-63 lower number means higher quality
  config.fb_count = 2;
  
  
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("camera init FAIL: 0x%x", err);
    return err;
  }
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA);
  Serial.println("camera init OK");
  return ESP_OK;
};

// Nuevo manejador de eventos WebSocket
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("[WSc] Desconectado!");
      break;
    case WStype_CONNECTED:
      Serial.print("[WSc] Conectado a URL: ");
      Serial.println((char *)payload);
      break;
  }
}

// Inicialización de WiFi con WebSockets
void init_wifi() {
  // Conexión WiFi
  //WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.config(ip, gateway, subnet);
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Iniciado STA:\t");
  Serial.println(ssid);
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());

  // Inicializar y conectar WebSocket (usar beginSSL para una conexion wss//)
  webSocket.begin(ws_host, ws_port, ws_handler); // Cambiar URL si es necesario
  webSocket.onEvent(webSocketEvent);
}

void cameraTask(void *pvParameters) {
  // Inicialización de la cámara
  init_camera();

  for (;;) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      ImageData imgData;
      imgData.fb = fb;
      if (!xQueueSend(imageQueue, &imgData, portMAX_DELAY)) {
        esp_camera_fb_return(fb);
      }
    } else {
      Serial.println("Error en captura de imagen");
    }
    delay(100); // Ajustar según sea necesario
  }
}

void wifiTask(void *pvParameters) {
  // Inicialización de WiFi y WebSocket
  init_wifi();

  for (;;) {
    webSocket.loop();
    ImageData imgData;
    if (xQueueReceive(imageQueue, &imgData, portMAX_DELAY)) {
      if (webSocket.isConnected() && WiFi.status() == WL_CONNECTED) {
        webSocket.sendBIN((const uint8_t*) imgData.fb->buf, imgData.fb->len);
        Serial.println("image sent, size: " + String(imgData.fb->len) + " bytes");
      }
      esp_camera_fb_return(imgData.fb);
    }
    delay(100); // Ajustar según sea necesario
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);

  // Crear la cola para imágenes
  imageQueue = xQueueCreate(2, sizeof(ImageData));

 // Creación de tareas asignadas a núcleos específicos
  xTaskCreatePinnedToCore(cameraTask, "CameraTask", 10000, NULL, 1, NULL, 0); // Núcleo 0
  xTaskCreatePinnedToCore(wifiTask, "WifiTask", 10000, NULL, 1, NULL, 1); // Núcleo 1
}

void loop() {}