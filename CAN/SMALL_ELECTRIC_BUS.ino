#include "driver/gpio.h"
#include "driver/twai.h"
#include <string>
#include <stdlib.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
// Thông tin mạng WiFi
const char* ssid = "Dat";
const char* password = "123123123";

String send_frame;
uint8_t send_frame_size;
twai_message_t message;
//-------------------------------------
#define _Base_Freq 10000    // Hz
#define _Working_Freq 1000  // Hz
#define _Request_Freq 1     // Hz
#define _Control_Freq 2     // Hz

#define _Sys_CLK 80000000       // Hz
#define _BaseTimer_CLK 2000000  // Hz
#define _BaseTimer_PRE (_Sys_CLK / _BaseTimer_CLK)
#define _BaseTimer_TOP (_BaseTimer_CLK / _Base_Freq)
#define _WrkCycle_MAX (_Base_Freq / _Working_Freq)
#define _ReqCycle_MAX (_Base_Freq / _Request_Freq)
#define _ControlCycle_MAX (_Base_Freq / _Control_Freq)
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t* BaseTimer = NULL;
unsigned int WrkCycle_Counter = _WrkCycle_MAX, RequestCycle_Counter = _ReqCycle_MAX, ControlCycle_Counter = _ControlCycle_MAX;
bool F_Working_Cycle = false, F_Control_Cycle = false, F_Request_Cycle = false;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// variable calcutlate and send to GUI

#define Relay_Pin 13
#define SOC_Cut_off 40
#define diff_Vol_Cut_off 150  //mV

//-------------------------------------

float Bat_Cell[23];
uint16_t Temper_1, Temper_2, Temper_3, Temper_4, T_MOS, T_balance;
uint8_t Cell_max_Pos, Cell_min_Pos;
uint8_t SOC;
float Cap_total, Cap_remain, Total_Cycle, Power, Bat_Voltage, Bat_Current;
float Cell_max_Vol, Cell_min_Vol;
double Cell_diff;
float Cell_avg_Vol, DSV, DV, CV, Total_vol, Total_Current;
bool receive_complete = false; // sau khi nhận đủ frame kia sẽ xét ID 610 để tính BatVol và Bat Current

void tinh_toan();
// Timer configuation
void IRAM_ATTR onBaseTimer() 
{
  if (!RequestCycle_Counter--)  // 10Hz
  {
    F_Request_Cycle = true;
    RequestCycle_Counter = _ReqCycle_MAX;
  }
  if (!ControlCycle_Counter--)  // 5Hz
  {
    F_Control_Cycle = true;
    ControlCycle_Counter = _ControlCycle_MAX;
  }
}

int a = 0, b = 0;
twai_message_t Open_port_message[4];
bool Request_complete = false, header_wrong, accepted = false, debug = true;
twai_message_t Receive_message;
twai_message_t Respone_message[23];
uint16_t counter = 0;  // counter2 dùng reset buffer
uint8_t Respone_buffer_index = 0;
int temp = 0;
// RTOS task 1 : Receiva data form controller
void Re_data(void* parameters) {
  // Loop forever
  while (1) 
  {
    if (F_Request_Cycle && !Request_complete) 
    {
      portENTER_CRITICAL_ISR(&timerMux);
      F_Request_Cycle = false;
      portEXIT_CRITICAL_ISR(&timerMux);
      twai_transmit(&Open_port_message[0], pdMS_TO_TICKS(1));
      twai_transmit(&Open_port_message[1], pdMS_TO_TICKS(1));
      twai_transmit(&Open_port_message[2], pdMS_TO_TICKS(1));
      twai_transmit(&Open_port_message[3], pdMS_TO_TICKS(1));
      twai_transmit(&Open_port_message[4], pdMS_TO_TICKS(1));
      // báo cờ xong request, clear index.
      Request_complete = true;
      counter = 0;
    }
    counter++;
    if (counter > 50)  // while send but not respone then this will avoid pending
    {
      Request_complete = false;
    }
    if (twai_receive(&Receive_message, pdMS_TO_TICKS(1)) == ESP_OK) 
    {
      if (Receive_message.identifier > 0x7FF && !receive_complete)  // used to check 2 frame header
      {
        if (temp == 0) {
          if (Receive_message.data[0] == 0x7E) {
            temp++;
          }
        } else if (temp == 1) {
          if (Receive_message.data[0] == 0x04 && Receive_message.data[1] == 0x17) {
            accepted = true;
            counter = 0;
            Respone_buffer_index++;
          }
        } else {
          Serial.println("Sai Header");
        }

        if (accepted)  // right header
        {
          Respone_message[Respone_buffer_index].identifier = Receive_message.identifier;
          Respone_message[Respone_buffer_index].data[0] = Receive_message.data[0];
          Respone_message[Respone_buffer_index].data[1] = Receive_message.data[1];
          Respone_message[Respone_buffer_index].data[2] = Receive_message.data[2];
          Respone_message[Respone_buffer_index].data[3] = Receive_message.data[3];
          Respone_message[Respone_buffer_index].data[4] = Receive_message.data[4];
          Respone_message[Respone_buffer_index].data[5] = Receive_message.data[5];
          Respone_message[Respone_buffer_index].data[6] = Receive_message.data[6];
          Respone_message[Respone_buffer_index].data[7] = Receive_message.data[7];
          Respone_buffer_index++;
          if (Respone_buffer_index > 21) {
            accepted = false;
            temp = 0;
            Respone_buffer_index = 0;
            tinh_toan();
          }
          Request_complete = false;
        }
      }
      else if(Receive_message.identifier == 0x610 && receive_complete)
      {
        receive_complete = false;
        Bat_Voltage = (Receive_message.data[1] * 256 + Receive_message.data[0])*0.1;
        Bat_Current = ((Receive_message.data[3] * 256 + Receive_message.data[2]) - 1000)*0.1;
      }
    }
  }
}
// RTOS task 2 : Send data to GUI
void sendGUI(void* parameters) {
  // Loop forever
  int count = 0;
  while (1) {
    if (F_Control_Cycle) {
      F_Control_Cycle = false;
      ArduinoOTA.handle();
      if ((SOC < SOC_Cut_off) || (Cell_diff > diff_Vol_Cut_off)) {
        digitalWrite(Relay_Pin, HIGH);
      }
      displayData();
    }
    vTaskDelay(10);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(Relay_Pin, OUTPUT);
  digitalWrite(Relay_Pin, LOW);
  // initialize LCD
  lcd.init();
  // turn on LCD backlight
  lcd.backlight();
  BaseTimer = timerBegin(0, _BaseTimer_PRE, true);
  timerAttachInterrupt(BaseTimer, &onBaseTimer, true);
  timerAlarmWrite(BaseTimer, _BaseTimer_TOP, true);
  timerAlarmEnable(BaseTimer);
  disableCore0WDT();
  disableCore1WDT();
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4, TWAI_MODE_NORMAL);
  // TX CAN Controller D5 - TX pin module CAN Transceiver CAN transmit data input
  // RX CAN Controller D4 - RX pin module CAN Transceiver CAN receive data output
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.print("Driver installed\n");
  } else {
    Serial.print("Failed to install driver\n");
    return;
  }

  if (twai_start() == ESP_OK) {
    Serial.print("Driver started\n");
  } else {
    Serial.print("Failed to start driver\n");
    return;
  }
  Open_port_message[0].identifier = 0x18FFFAFF;
  Open_port_message[0].extd = 1;
  Open_port_message[0].data_length_code = 8;
  Open_port_message[0].data[0] = 0x7E;
  Open_port_message[0].data[1] = 0xA1;
  Open_port_message[0].data[2] = 0x01;
  Open_port_message[0].data[3] = 0x00;
  Open_port_message[0].data[4] = 0x00;
  Open_port_message[0].data[5] = 0x00;
  Open_port_message[0].data[6] = 0x00;
  Open_port_message[0].data[7] = 0x00;

  Open_port_message[1].identifier = 0x18FFFAFF;
  Open_port_message[1].extd = 1;
  Open_port_message[1].data_length_code = 8;
  Open_port_message[1].data[0] = 0x00;
  Open_port_message[1].data[1] = 0x00;
  Open_port_message[1].data[2] = 0x00;
  Open_port_message[1].data[3] = 0x00;
  Open_port_message[1].data[4] = 0x00;
  Open_port_message[1].data[5] = 0x00;
  Open_port_message[1].data[6] = 0x00;
  Open_port_message[1].data[7] = 0x00;

  Open_port_message[2].identifier = 0x18FFFAFF;
  Open_port_message[2].extd = 1;
  Open_port_message[2].data_length_code = 8;
  Open_port_message[2].data[0] = 0XAA;
  Open_port_message[2].data[1] = 0X55;
  Open_port_message[2].data[2] = 0x00;
  Open_port_message[2].data[3] = 0x00;
  Open_port_message[2].data[4] = 0x00;
  Open_port_message[2].data[5] = 0x00;
  Open_port_message[2].data[6] = 0x00;
  Open_port_message[2].data[7] = 0x00;

  Open_port_message[3].identifier = 0x18FFFAFF;
  Open_port_message[3].extd = 1;
  Open_port_message[3].data_length_code = 8;
  Open_port_message[3].data[0] = 0x7E;
  Open_port_message[3].data[1] = 0xA1;
  Open_port_message[3].data[2] = 0x01;
  Open_port_message[3].data[3] = 0x00;
  Open_port_message[3].data[4] = 0x00;
  Open_port_message[3].data[5] = 0xBE;
  Open_port_message[3].data[6] = 0x18;
  Open_port_message[3].data[7] = 0x55;


  Open_port_message[4].identifier = 0x18FFFAFF;
  Open_port_message[4].extd = 1;
  Open_port_message[4].data_length_code = 8;
  Open_port_message[4].data[0] = 0xAA;
  Open_port_message[4].data[1] = 0x55;
  Open_port_message[4].data[2] = 0x00;
  Open_port_message[4].data[3] = 0x00;
  Open_port_message[4].data[4] = 0x00;
  Open_port_message[4].data[5] = 0x00;
  Open_port_message[4].data[6] = 0x00;
  Open_port_message[4].data[7] = 0x00;
  xTaskCreatePinnedToCore(Re_data,
                          "Re_data",
                          10000,
                          NULL,
                          1,
                          NULL,
                          0);

  // Start blink task
  xTaskCreatePinnedToCore(sendGUI,
                          "sendGUI",
                          2048,
                          NULL,
                          1,
                          NULL,
                          1);
  // Kết nối WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_SPIFFS
      type = "filesystem";
    }
    // Lưu ý: Nếu dùng SPIFFS, cần mount SPIFFS trước
    Serial.println("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress * 100) / total);
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

  ArduinoOTA.begin();
  Serial.println("OTA ready");

  vTaskDelete(NULL);
}

void loop() {
}

float Bat_Cell_Vol(uint8_t a, uint8_t b) {
  float result = ((float)b * 256 + a) / 1000;
  return result;
}



void tinh_toan() 
{
  uint8_t index = 4;
  uint8_t byte_pos = 2;
  for (int i = 0; i < 23; i++) 
  {
    if (byte_pos >= 7) 
    {
      index++;
      byte_pos = 0;
    }
    Bat_Cell[i] = Bat_Cell_Vol(Respone_message[index].data[byte_pos], Respone_message[index].data[byte_pos + 1]);
    byte_pos += 2;
  }
  index++;
  switch (index) 
  {
    case 10:
      Temper_1 = Respone_message[index].data[1] * 256 + Respone_message[index].data[0];
      Temper_2 = Respone_message[index].data[3] * 256 + Respone_message[index].data[2];
      Temper_3 = Respone_message[index].data[5] * 256 + Respone_message[index].data[4];
      Temper_4 = Respone_message[index].data[7] * 256 + Respone_message[index].data[6];
      index++;
    case 11:
      T_MOS = Respone_message[index].data[1] * 256 + Respone_message[index].data[0];
      T_balance = Respone_message[index].data[3] * 256 + Respone_message[index].data[2];
      index++;
    case 12:
      SOC = Respone_message[index].data[0];
      index++;
    case 13:
      Cap_total = (Respone_message[index].data[3] << 24 | Respone_message[index].data[2] << 16 | Respone_message[index].data[1] << 8 | Respone_message[index].data[0]) / 1000000;
      Cap_remain = (Respone_message[index].data[7] << 24 | Respone_message[index].data[6] << 16 | Respone_message[index].data[5] << 8 | Respone_message[index].data[4]) / 1000000;
      index++;
    case 14:
      Total_Cycle = (Respone_message[index].data[3] << 24 | Respone_message[index].data[2] << 16 | Respone_message[index].data[1] << 8 | Respone_message[index].data[0]) / 1000;
      Power = (Respone_message[index].data[7] << 24 | Respone_message[index].data[6] << 16 | Respone_message[index].data[5] << 8 | Respone_message[index].data[4]);
      index++;
    case 15:
      index++;
    case 16:
      Cell_max_Vol = Bat_Cell_Vol(Respone_message[index].data[0], Respone_message[index].data[1]);
      Cell_min_Vol = Bat_Cell_Vol(Respone_message[index].data[4], Respone_message[index].data[5]);
      Cell_max_Pos = Respone_message[index].data[3] * 256 + Respone_message[index].data[2];
      Cell_min_Pos = Respone_message[index].data[7] * 256 + Respone_message[index].data[6];
      index++;
    case 17:
      Cell_diff = (Respone_message[index].data[1] * 256 + Respone_message[index].data[0]);
      Cell_avg_Vol = (Respone_message[index].data[3] * 256 + Respone_message[index].data[2]) / 1000;
      DSV = (Respone_message[index].data[5] * 256 + Respone_message[index].data[4]);
      DV = (Respone_message[index].data[7] * 256 + Respone_message[index].data[6]);
      index++;
    case 18:
      CV = 0;
      index++;
      receive_complete = true;
      break;
  }
  if(debug)
  {
    Serial.println("------------------------");
      for (int i = 0; i < 23; i++) {
        Serial.print("Batcell ");
        Serial.print(i);
        Serial.print(": \t");
        Serial.println(Bat_Cell[i], 3);
      }
      Serial.print("BatVol: ");
      Serial.print(Bat_Voltage);
      Serial.println(" V");
      Serial.print("BatCur: ");
      Serial.print(Bat_Current);
      Serial.println(" A");
      Serial.print("T1: ");
      Serial.print(Temper_1);
      Serial.println(" Deg C");
      Serial.print("T2: ");
      Serial.print(Temper_2);
      Serial.println(" Deg C");
      Serial.print("T3: ");
      Serial.print(Temper_3);
      Serial.println(" Deg C");
      Serial.print("T4: ");
      Serial.print(Temper_4);
      Serial.println(" Deg C");
      Serial.print("T_MOS: ");
      Serial.print(T_MOS);
      Serial.println(" Deg C");
      Serial.print("T_balance: ");
      Serial.print(T_balance);
      Serial.println(" Deg C");
      Serial.print("SOC: ");
      Serial.print(SOC);
      Serial.println(" %");
      Serial.print("Cap_total: ");
      Serial.print(Cap_total);
      Serial.println(" Ah");
      Serial.print("Cap_remain: ");
      Serial.print(Cap_remain);
      Serial.println(" Ah");
      Serial.print("Total_Cycle: ");
      Serial.println(Total_Cycle);
      Serial.print("Power: ");
      Serial.print(Power);
      Serial.println(" W");
      Serial.print("Cell_max_Vol: ");
      Serial.println(Cell_max_Vol);
      Serial.print("Cell_min_Vol: ");
      Serial.println(Cell_min_Vol);
      Serial.print("Cell_max_Pos: ");
      Serial.println(Cell_max_Pos);
      Serial.print("Cell_min_Pos: ");
      Serial.println(Cell_min_Pos);
      Serial.print("Cell_diff: ");
      Serial.print(Cell_diff, 3);
      Serial.println(" mV");
      Serial.print("Cell_avg_Vol: ");
      Serial.println(Cell_avg_Vol);
      Serial.print("DSV: ");
      Serial.println(DSV);
      Serial.print("DV: ");
      Serial.println(DV);
      Serial.print("CV: ");
      Serial.println(CV);
  }
}

void displayData() 
{
  // clear the display to print new message
  lcd.clear();

  // display SOC on the first row
  lcd.setCursor(0, 0);  // first column, first row
  lcd.print("SOC : ");
  lcd.print((float)SOC, 2);  // display SOC value with 2 decimal places

  // display V_diff on the second row
  lcd.setCursor(0, 1);  // first column, second row
  lcd.print("V_diff: ");
  lcd.print(Cell_diff, 3);  // display V_diff value with 2 decimal places
}