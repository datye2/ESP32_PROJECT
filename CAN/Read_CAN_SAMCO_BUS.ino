#include "driver/gpio.h"
#include "driver/twai.h"
#include <string>
#include <stdlib.h>


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


int16_t Motor_Speed = 0,TPS = 0, TPS2 = 0, Motor_Current, actual_shift = 0, Bat_Current; // FWD : 1 , NEUTRAL = 0, RVD = -1
//Nghi ngờ
uint8_t Heatsink_temp;
int16_t speed_1 , speed_2;  
int16_t current_1,current_2; 
uint16_t  FWD_shift, REV_shift,Bat_Vol;
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


char *p_num;
void send_num_uint8_t(uint8_t num_uint8_t, char id[2])
{
  Serial.write(id[0]);
  Serial.write(id[1]);
  p_num = (char*)&num_uint8_t; 
  Serial.write((char)*p_num);
  Serial.write(';');
}

void send_num_uint16_t(uint16_t num_uint16_t, char id[2])
{
  Serial.write(id[0]);
  Serial.write(id[1]);
  p_num = (char*)&num_uint16_t; 
  Serial.write((char)*(p_num + 1));
  Serial.write((char)*p_num);
  Serial.write(';');
}

void send_num_float_t(float num_float_t, char id[2])
{
  Serial.write(id[0]);
  Serial.write(id[1]);
  p_num = (char*)&num_float_t;
  Serial.write((char)*p_num +3);
  Serial.write((char)*p_num + 2);
  Serial.write((char)*p_num + 1);
  Serial.write((char)*p_num);
  Serial.write(';');
}

void send_num_int16_t(int16_t num_int16_t, char id[2])
{
  Serial.write(id[0]);
  Serial.write(id[1]);
  p_num = (char*)&num_int16_t; 
  Serial.write((char)*(p_num + 1));
  Serial.write((char)*p_num);
  Serial.write(';');
}

byte *p_number;

float split_num_float()
{
  float num_float;
  p_number = (byte*)&num_float;
  *(p_number + 3) = read_buffer[0];
  *(p_number + 2) = read_buffer[1];
  *(p_number + 1) = read_buffer[2];
  *p_number = read_buffer[3];
  return num_float;
}
int temp1 = 0;
// Timer configuation
void IRAM_ATTR onBaseTimer() 
{
  portENTER_CRITICAL_ISR(&timerMux);
  rec_flag = true;
  counter++;
  if(counter >= 20)
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
          switch (message.identifier)
          {
            case 0x455 :
            Bat_Vol = calculate_uint16(0);
            Heatsink_temp = message.data[2];
            Bat_Current = calculate_int16(3);
            current_1 = calculate_int16(6);
            break;
          case 0x259 :
            current_2 = calculate_int16(0);
            Motor_Speed = calculate_int16(2);
            break;
          case 0x269 :
            Motor_Current = calculate_int16(0);
            speed_1 = calculate_int16(2);
            TPS2 = calculate_int16(4);
            break;
          case 0x401 :
            FWD_shift = calculate_uint16(2);
            speed_2 = calculate_int16(4);
            REV_shift =  calculate_uint16(6);
            if(abs(FWD_shift - REV_shift) < 150)
            {
              if(REV_shift > 150)
                actual_shift = -1;
              else
                actual_shift = 1;
            }
            else
              actual_shift = 0;
            //actual_shift = FWD_shift - REV_shift;  
            break;
          case 0x393 :
            TPS = (message.data[7] * 256 + message.data[6]);
            // // Serial.print(temp_current_scale);
            break;
          
          break;
        }
       xSemaphoreGive(xSemaphore);
      }
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
      //if(Gui_flag)
        send_num_uint8_t(Heatsink_temp,"HT");
        send_num_uint16_t(Bat_Vol,"BV");
        send_num_int16_t(Bat_Current,"BC");
        send_num_int16_t(Motor_Speed,"MS");
        send_num_int16_t(Motor_Current,"MC");
        send_num_int16_t(actual_shift,"AS");
        send_num_int16_t(TPS,"T1");
        send_num_int16_t(TPS2,"T2");
        send_num_int16_t(speed_1,"S1");
        send_num_int16_t(speed_2,"S2");
        send_num_int16_t(current_1,"C1");
        send_num_int16_t(current_2,"C2");
        Serial.println();
    }
    //vTaskDelay(10);
  }
}

void setup() 
{
  Serial.begin(115200);
  BaseTimer = timerBegin(0, _BaseTimer_PRE, true); 
  timerAttachInterrupt(BaseTimer, &onBaseTimer, true);
  timerAlarmWrite(BaseTimer, _BaseTimer_TOP, true);
  timerAlarmEnable(BaseTimer);  
  disableCore0WDT(); 
  disableCore1WDT(); 
   twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_23, GPIO_NUM_22, TWAI_MODE_NORMAL);
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

void loop() 
{
}
