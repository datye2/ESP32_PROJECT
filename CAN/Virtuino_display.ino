#include "driver/gpio.h"
#include "driver/twai.h"
#include "BluetoothSerial.h"
#include <string>
#include <stdlib.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

#include "VirtuinoCM.h"
VirtuinoCM virtuino;               
#define V_memory_count 32          // the size of V memory. You can change it to a number <=255)
float V[V_memory_count];           // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.

boolean debug = true;              // set this variable to false on the finale code to decrease the request time.

String send_frame;
uint8_t send_frame_size;
twai_message_t message;
//-------------------------------------
#define _Base_Freq          200   // Hz
//-------------------------------------
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
#define _Sys_CLK            80000000  // Hz
#define _BaseTimer_CLK      1000000  // Hz
#define _BaseTimer_PRE      (_Sys_CLK / _BaseTimer_CLK) 
#define _BaseTimer_TOP      (_BaseTimer_CLK / _Base_Freq)
hw_timer_t * BaseTimer = NULL;
bool timer_flag = false, timer_flag1 = false , temp = false;

// variable calcutlate and send to GUI

float Bat_Vol = 75,Bat_Current = 11, HST,VHS, MTS,TPS;
//
bool rec_flag = false, Gui_flag = false;
uint8_t counter = 0;
float temp_speed_scale = 1, temp_current_scale = 1; // scale này để thay đổi vì đang nghi ngờ
char read_buffer[6];

//-------------------------------------
SemaphoreHandle_t xSemaphore = xSemaphoreCreateBinary(); // Semaphore đồng bộ 2 Task

//-------------------------------------
uint16_t calculate_uint16(uint8_t index)
{
  uint16_t value = 0;
  value = (message.data[index+1] * 256 + message.data[index]);
  return value;
}

int16_t calculate_int16(uint8_t index)
{
  int16_t value = 0;
  value = (message.data[index+1] * 256 + message.data[index]);
  if(value > 32768)
  {
    value = (-65536 + value);
  }
  else
  {
    value = value ;
  }
  return value;
}
//-------------------------------------

int temp1 = 0;
// Timer configuation
void IRAM_ATTR onBaseTimer() 
{
  portENTER_CRITICAL_ISR(&timerMux);
  rec_flag = true;
  counter++;
  if(counter >= 10)
  {
    Gui_flag = true;
    counter = 0;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}
int a = 0, b = 0;
// RTOS task 1 : Receiva data form controller
void Re_data(void *parameters) 
{
  // Loop forever
  while (1) 
  {
    //if(rec_flag)
    {
      portENTER_CRITICAL_ISR(&timerMux);
      rec_flag = false;
      portEXIT_CRITICAL_ISR(&timerMux);
        if(twai_receive(&message, pdMS_TO_TICKS(1)) == ESP_OK)
        {
          // Serial.println(1);
          switch (message.identifier)
          {
            case 0x455 :
            Bat_Vol = calculate_uint16(0)*0.0625;
            HST = message.data[2];
            Bat_Current = calculate_int16(3)*0.0625;
            break;
            case 0x269 :
              VHS = calculate_int16(2)*0.0625*3.6;
              break;
            case 0x401 :
              MTS = calculate_int16(4);
              break;
            case 0x393 :
              TPS = (message.data[7] * 256 + message.data[6])*100/1024;
              // // Serial.print(temp_current_scale);
              //Serial.println(TPS);
              break;
          break;
        }
      //Serial.println(1);
      }
       xSemaphoreGive(xSemaphore);
    }
    //vTaskDelay(10);
  }
}
// RTOS task 2 : Send data to GUI
void sendGUI(void *parameters) 
{
  // Loop forever
  int count = 0;
  while (1) 
  {
    if(xSemaphoreTake(xSemaphore, (TickType_t) 1) == pdTRUE)
    {
      //if(Gui_flag) // 5 hz
      {
        virtuinoRun();
        V[1] = Bat_Vol;
        V[2] = Bat_Current;
        V[3] = HST;
        V[4] = abs(VHS);
        V[5] = abs(MTS);
        V[6] = TPS;
        V[7] = 100;
        portENTER_CRITICAL_ISR(&timerMux);
        Gui_flag = false;
        portEXIT_CRITICAL_ISR(&timerMux);
      }
    }
    //vTaskDelay(10);
  }
}

void setup() 
{
  Serial.begin(115200);
  SerialBT.begin("datmen"); //Bluetooth device name   // enable bluetooth
  virtuino.begin(onReceived,onRequested,256);  // enable virtuino

  BaseTimer = timerBegin(0, _BaseTimer_PRE, true); 
  timerAttachInterrupt(BaseTimer, &onBaseTimer, true);
  timerAlarmWrite(BaseTimer, _BaseTimer_TOP, true);
  timerAlarmEnable(BaseTimer);  
  disableCore0WDT(); 
  disableCore1WDT(); 
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4, TWAI_MODE_NORMAL);
  //twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_32, GPIO_NUM_33, TWAI_MODE_NORMAL);
  // TX CAN Controller D5 - TX pin module CAN Transceiver CAN transmit data input
  // RX CAN Controller D4 - RX pin module CAN Transceiver CAN receive data output
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  {
    Serial.print("Driver installed\n");
  } 
  else 
  {
    Serial.print("Failed to install driver\n");
    return;
  }
  
  if (twai_start() == ESP_OK) 
  {
    Serial.print("Driver started\n");
  } 
  else 
  {
    Serial.print("Failed to start driver\n");
    return;
  }
    xTaskCreatePinnedToCore(Re_data,
                          "Re_data",
                          2048,
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
  vTaskDelete(NULL);
}
//
void virtuinoRun(){
  while (SerialBT.available()) {
      char tempChar=SerialBT.read();
      if (tempChar==CM_START_CHAR) {               // a new command is starting...
            virtuino.readBuffer=CM_START_CHAR;     // copy the new command to the virtuino readBuffer
            virtuino.readBuffer+=SerialBT.readStringUntil(CM_END_CHAR);
            virtuino.readBuffer+=CM_END_CHAR;
            //if (debug) Serial.println("\nCommand= "+virtuino.readBuffer);
            String* response= virtuino.getResponse();    // get the text that has to be sent to Virtuino as reply. The library will check the inptuBuffer and it will create the response text
            //if (debug) Serial.println("Response : "+*response);
            SerialBT.print(*response);
            break; 
        }
  }
}
 

//
void onReceived(char variableType, uint8_t variableIndex, String valueAsText){     
  if (variableType=='V'){
      float value = valueAsText.toFloat();        // convert the value to float. The valueAsText have to be numerical
      if (variableIndex<V_memory_count) V[variableIndex]=value;              // copy the received value to arduino V memory array
  }
}

//==============================================================
/* This function is called every time Virtuino app requests to read a pin value*/
String onRequested(char variableType, uint8_t variableIndex){     
    if (variableType=='V') {
    if (variableIndex<V_memory_count) return  String(V[variableIndex]);   // return the value of the arduino V memory array
  }
  return "";
}

//

void loop() 
{
}
