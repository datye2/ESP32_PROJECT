#include <ESP32Servo.h>
#include <WiFi.h>
#include <AsyncWebSocket.h>
#include "WebSocketsServer.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "index_test.h"  

#define SERVO_PIN1 16  // ESP32 pin GPIO26 connected to servo motor
#define SERVO_PIN2 18
Servo servo1;
Servo servo2;
//
xQueueHandle message_queue;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
//
// const char* ssid = "TP-LINK_B2BE";     
// const char* password = "34737521";
// const char* ssid = "Hcmut-c3";     
// const char* password = "Khongduocvao123";  
const char* ssid = "Datsieucap";
const char* password = "123123123";
// const char* ssid = "Datmen";
// const char* password = "mldz2981";
// const char* ssid = "TP-Link_5632";
// const char* password = "55365963";

TaskHandle_t Task1;
TaskHandle_t Task2; // task 2 sẽ tạo timer và lấy giá trị  theo chu kỳ 20 hz

AsyncWebServer server(80);
bool timer_flag = false ,  receive_flag = false ,queue_read_feedback = false , AB_flag = false;;
WebSocketsServer webSocket = WebSocketsServer(81);  // WebSocket server on port 81
int angle1, angle2, sum_angle1,sum_angle2, counter = 0;
unsigned long start_time = 0 ;
void  IRAM_ATTR onTimer() 
{
  portENTER_CRITICAL_ISR(&timerMux);
  timer_flag = true; 
  portEXIT_CRITICAL_ISR(&timerMux);
}



struct Servo_goc
{
  int sum_goc_lai;
  int sum_goc_ga;
  int count;
};

struct Servo_goc Servo_message_TX;
struct Servo_goc Servo_message_RX;

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num); 
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      }
      break;
    case WStype_TEXT: 
      start_time = millis();
      String message = String((char*)payload);
      sscanf(message.c_str(), "Servo1: %d, Servo2: %d", &angle1, &angle2);
      sum_angle1 += angle1;
      sum_angle2 += angle2;
      // Đóng gói tin nhắn truyền

      // kiểm tra cờ và chuẩn bị gửi:
      while(AB_flag  == true)
      {
        if(millis() -  start_time >= 150)
        {
          break;
        }
      }
      if(AB_flag == false)
      {
        AB_flag = true;
        Servo_message_TX.sum_goc_ga = sum_angle1;
        Servo_message_TX.sum_goc_lai = sum_angle2;
        Servo_message_TX.count = counter;
        xQueueSendFromISR(message_queue,(void *) &Servo_message_TX,( TickType_t ) 0 );
        sum_angle1 = 0;
        sum_angle2 = 0;
        counter = 0;
        AB_flag = false;
      }
      counter++;
      break;
  }
}

void setup() {
  Serial.begin(115200);
  servo1.attach(SERVO_PIN1);  // attaches the servo on ESP32 pin
  servo2.attach(SERVO_PIN2);
  //
  timer = timerBegin(0,80,true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 50000, true);
  timerAlarmEnable(timer);
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  message_queue = xQueueCreate(1, sizeof(Servo_goc));
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialize WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Serve a basic HTML page with JavaScript to create the WebSocket connection
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    Serial.println("Web Server: received a web page request");
    String html = HTML_CONTENT;  // Use the HTML content from the index.h file
    request->send(200, "text/html", html);
  });
  server.begin();
  Serial.print("ESP32 Web Server's IP address: ");
  Serial.println(WiFi.localIP());

  // Create a task to monitor WiFi connection
  xTaskCreatePinnedToCore(
    Task1code,     // Function to implement the task
    "Task1",       // Name of the task
    10000,         // Stack size in words
    NULL,          // Task input parameter
    1,             // Priority of the task
    &Task1,        // Task handle
    0              // Core where the task should run
  );
  xTaskCreatePinnedToCore(
    Task2code,     // Function to implement the task
    "Task_control",       // Name of the task
    10000,         // Stack size in words
    NULL,          // Task input parameter
    1,             // Priority of the task
    &Task2,        // Task handle
    1              // Core where the task should run
  );
}

void loop() 
{
  
}


void Task1code(void * pvParameters) 
{
  while(1) 
  {
    webSocket.loop();
    vTaskDelay(10);
    if (WiFi.status() != WL_CONNECTED) 
    {
      Serial.println("WiFi lost connection. Setting servos to 180 degrees...");
      servo1.write(180);
      servo2.write(180);
      while (WiFi.status() != WL_CONNECTED) 
      {
        delay(1000);
        Serial.println("Reconnecting to WiFi...");
        WiFi.begin(ssid, password);
      }
      Serial.println("Reconnected to WiFi");
    }
  }
}

int goc1, goc2, temp_sumangel1 = 0 , temp_sumangle2 = 0, temp_counter = 0;
void Task2code(void * pvParameters) 
{
  while(1)
  {
    if(timer_flag == true)
    {
      while(AB_flag  == true)
      {
        if(millis() -  start_time >= 150)
        {
          break;
        }
      }
      if(AB_flag == false)
      {
        AB_flag = true;
        xQueueReceive( message_queue, &(Servo_message_RX), ( TickType_t ) 0 );
        // queue_read_feedback = true;
        while(Servo_message_RX.count > 0)
        {
          goc1 = Servo_message_RX.sum_goc_ga/Servo_message_RX.count;
          goc2 = Servo_message_RX.sum_goc_lai/Servo_message_RX.count;
          Serial.print(Servo_message_RX.sum_goc_ga);
          Serial.print('\t');
          Serial.print(Servo_message_RX.sum_goc_lai);
          Serial.print('\t');
          Serial.print(Servo_message_RX.count);
          Serial.print('\t');
          Serial.print(goc1);
          Serial.print('\t');
          Serial.println(goc2); 
          servo1.write(goc1);
          servo2.write(goc2);
          break;
        }
        AB_flag = false;
      }
      portENTER_CRITICAL_ISR(&timerMux);
      timer_flag = false; 
      portEXIT_CRITICAL_ISR(&timerMux);
    }

    vTaskDelay(10);
  } 
}