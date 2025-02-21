#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include <stdio.h>
#include <stdlib.h>
//-------------------------------------
#define _uint32_t_MAX       4294967296
//-------------------------------------
#define _Base_Freq          160000   // Hz
#define _Working_Freq       40       // Hz    
#define gain_out            0.2
//-------------------------------------
#define _Sys_CLK            80000000  // Hz
#define _BaseTimer_CLK      16000000  // Hz
#define _BaseTimer_PRE      (_Sys_CLK / _BaseTimer_CLK)
#define _BaseTimer_TOP      (_BaseTimer_CLK / _Base_Freq)
#define _WrkCycle_MAX       (_Base_Freq / _Working_Freq)
hw_timer_t * BaseTimer = NULL;
uint32_t WrkCycle_Counter = _WrkCycle_MAX;
bool F_Working_Cycle = false;
//-------------------------------------
// User-defined: input capture pins
#define _CAP00_minFreq      0.2             // Hz
#define _CAP00_wchdgMAX     ((_Base_Freq/_CAP00_minFreq) + 1)
#define _CAP00_IN           GPIO_NUM_21   // GPIO21 as input pin for Unit 0 & Capture input 0
#define _CAP0_INT_EN        BIT(27)       // Capture 0 interrupt bit
#define array_size          7
#define median_pos          3//((array_size - 1)/2)

uint32_t cap_wchdgCount = 0;   
volatile uint32_t mcpwm_intr_status, new_cap_value, old_cap_value;
uint32_t cap_value_temp = 0;
uint32_t cap_value = 0;
uint32_t cap_value_raw[array_size], cap_value_sort[array_size], temp_speed;
uint8_t cap_value_index;
uint8_t F_detect_phase = 0, F_clear_encoder_counter = 1;
bool F_cap_newValue = false;
//-------------------------------------
mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1}; 
//-------------------------------------
TaskHandle_t taskBackGround;    // Core-0
//-------------------------------------
#define enc_cnA             22
#define enc_cnB             23
#define enc_cnZ             21    // proximity sensor
#define Injector_1          GPIO_NUM_25
#define Injector_2          GPIO_NUM_26
#define MAF_sensor          34
#define MAF_trigger_pos1    800
#define MAF_trigger_pos2    (MAF_trigger_pos1 + 4000)
//-------------------------------------
volatile uint16_t encoder_counter, latch_encoder_counter;
volatile uint16_t encoder_counter_INJT1 = 0, encoder_counter_INJT2 = 0;
volatile float INJT1_angle, INJT2_angle;
volatile uint16_t inj1_count, inj2_count, inj_time = 1000, old_inj_time;
//-------------------------------------
volatile float kp, ki, kd;
volatile float kp_read, ki_read, kd_read;
volatile uint8_t f_PID_speed = 0;
volatile float engine_speed, req_engine_speed;
float e, e_old, old_out, out, out_P, out_I, out_D;
const float e_max = 1000.0;
const float dt = 0.025;
const uint16_t inj_time_MAX = (_Base_Freq/40);
uint16_t MAF_adc_new, MAF_adc_old;
uint8_t ig_on_count = 10;
//-------------------------------------
char read_buffer[6];
byte *p_num;
float num_float;
//-------------------------------------
// base timer ISR
//-------------------------------------
void IRAM_ATTR onBaseTimer()
{     
  //---------------------------------
  if (!(--WrkCycle_Counter)) 
  {
    WrkCycle_Counter = _WrkCycle_MAX;
    F_Working_Cycle = true;
  }
  //---------------------------------
  if (!(inj1_count--))
    GPIO.out_w1tc = (1 << Injector_1);
  if (!(inj2_count--))
    GPIO.out_w1tc = (1 << Injector_2);
  //---------------------------------
  if (cap_wchdgCount) 
  {
    cap_wchdgCount--;
    if (!cap_wchdgCount)
    {
      engine_speed = 0;
      F_detect_phase = 0;
      ig_on_count = 10;
    }
  }
  //---------------------------------
}
//-------------------------------------
// End of base timer ISR
//-------------------------------------
void encoder_count()
{
  if (encoder_counter == encoder_counter_INJT1)
  {
    if (inj_time)
    {
      inj1_count = inj_time;
      GPIO.out_w1ts = (1 << Injector_1);
    }
  }
  if (encoder_counter == encoder_counter_INJT2)
  {
    if (inj_time)
    {
      inj2_count = inj_time;
      GPIO.out_w1ts = (1 << Injector_2);
    }
  }   
  encoder_counter++;
  if (F_detect_phase == 1)
  {
    if (encoder_counter == MAF_trigger_pos1)
      MAF_adc_old = analogRead(MAF_sensor);
    else if (encoder_counter == MAF_trigger_pos2)
    {
      MAF_adc_new = analogRead(MAF_sensor);
      if (MAF_adc_old > MAF_adc_new)
        F_clear_encoder_counter = 1;
      else
        F_clear_encoder_counter = 2;
      F_detect_phase = 2;
    }
  }
}
//-------------------------------------
// ISR: MCPWM0 input capture
//-------------------------------------
void IRAM_ATTR mcpwm0_IC_ISR(void *arg)
{     
  mcpwm_intr_status = MCPWM0.int_st.val;
  if (mcpwm_intr_status & _CAP0_INT_EN) 
  {
    new_cap_value = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);  
    if (cap_wchdgCount < (_CAP00_wchdgMAX - 160))
    {
      cap_wchdgCount = _CAP00_wchdgMAX;
      F_cap_newValue = true;
      if (F_clear_encoder_counter)
      {
        F_clear_encoder_counter--;
        if (F_clear_encoder_counter == 0)
        {
          F_clear_encoder_counter = 2;
          encoder_counter = 0;
        }
      }
      if (F_detect_phase == 0)
      {
        ig_on_count--;
        encoder_counter = 0;
        F_clear_encoder_counter = 20;
        if (!ig_on_count)
          F_detect_phase = 1;
      }
    }
  }  
  //---------------------------------
  MCPWM[MCPWM_UNIT_0]->int_clr.val = mcpwm_intr_status; 
}
//-------------------------------------
// End of ISR: MCPWM0 input capture
//-------------------------------------
//-------------------------------------
void taskBckGrdCode( void * pvParameters )
{
  while (1)
  {
    
  }
}
//-------------------------------------
int cmpfunc (const void * a, const void * b)
{
  return ( *(uint32_t*)a - *(uint32_t*)b );
}
//-------------------------------------
void setup()
{
  disableCore0WDT();    // Core0/watchdog-timer should be disabled to avoid hard reset.
  xTaskCreatePinnedToCore(  taskBckGrdCode,     // Task function
                            "taskBackGround",   // Name of task
                            10000,              // Stack size of task
                            NULL,               // Parameter of the task
                            1,                  // Priority of the task
                            &taskBackGround,    // Task handle to keep track of created task
                            0);                 // Pin task to core 0
  //-------------------------------------
  // base timer configuration
  BaseTimer = timerBegin(0, _BaseTimer_PRE, true); 
  timerAttachInterrupt(BaseTimer, &onBaseTimer, true);
  timerAlarmWrite(BaseTimer, _BaseTimer_TOP, true);
  timerAlarmEnable(BaseTimer);  
  //-------------------------------------
  pinMode(MAF_sensor, INPUT);
  pinMode(2, OUTPUT);
  pinMode(Injector_1, OUTPUT);
  pinMode(Injector_2, OUTPUT);
  GPIO.out_w1tc = (1 << Injector_1);
  GPIO.out_w1tc = (1 << Injector_2);
  //-------------------------------------
  // Configurate capture input pin:
  // . Select MCPWM unit: 0, 1
  // . Select input capture module: 0
  // . Set GPIOx pin as capture input
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, _CAP00_IN);  
  // Configurate input capture
  // . Select MCPWM unit: 0, 1
  // . Select input capture module: 0, 1, 2
  // . Select edge detection: negative, positive
  // . Select pulse-prescaler  
  mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);
  gpio_pullup_en(_CAP00_IN);
  // Configurate input capture interrupt
  // . Select MCPWM unit: 0, 1
  // . Enable input capture module: 0, 1, 2 
  // . Set ISR handler
  MCPWM0.int_ena.val = (_CAP0_INT_EN);
  mcpwm_isr_register(MCPWM_UNIT_0, mcpwm0_IC_ISR, NULL, ESP_INTR_FLAG_IRAM, NULL);
  //-------------------------------------    
  Serial.begin(115200);
  attachInterrupt(enc_cnA, encoder_count, CHANGE);
  attachInterrupt(enc_cnB, encoder_count, CHANGE);
}
//-------------------------------------
void loop()
{
  if (F_Working_Cycle)
  {
    //---------------------------------
    F_Working_Cycle = false;   
    kp = kp_read;
    ki = ki_read;
    kd = kd_read;
    if (f_PID_speed)
    {
      old_inj_time = inj_time;
      inj_time = PID_cal(engine_speed, req_engine_speed)*inj_time_MAX;
      inj_time = (inj_time > (old_inj_time + 200))?(old_inj_time + 200):inj_time;
    }
    else
      inj_time = 0;
    send_data();
    digitalWrite(2, !digitalRead(2));
  }
  //-----------------------------------------   
  // Get input capture 0
  //-----------------------------------------   
  if (F_cap_newValue)
  {
    F_cap_newValue = false;
    if (new_cap_value < old_cap_value)
      cap_value = _uint32_t_MAX - old_cap_value + new_cap_value;
    else
      cap_value = new_cap_value - old_cap_value;
    old_cap_value = new_cap_value;
    
    // without meadian filter
//    if (cap_value)
//      engine_speed = 4800000000.0/cap_value;
//    else
//      engine_speed = cap_value; 
    
    // with meadian filter
    cap_value_raw[cap_value_index++] = cap_value;
    cap_value_index = cap_value_index%array_size;
    for (uint8_t i = 0; i < array_size; i++)
      cap_value_sort[i] = cap_value_raw[i];
        
//    qsort(cap_value_sort, array_size, sizeof(uint32_t), cmpfunc);  

    for (int i = 0; i < (array_size - 1); i++)  
    { 
      for (int j = (i + 1); j < array_size; j++)
      {
        if (cap_value_sort[i] >= cap_value_sort[j])
        {
          temp_speed = cap_value_sort[j];
          cap_value_sort[j] = cap_value_sort[i];
          cap_value_sort[i] = temp_speed;
        }
      }
    }
    
    if (cap_value)
      engine_speed = 4800000000.0/cap_value_sort[median_pos];
    else
      engine_speed = cap_value;   
  } 
  //-----------------------------------------   
  // End of Get input capture 0
  //-----------------------------------------   
  if (Serial.available() >= 7)
  {
    switch (Serial.read())
    {
      case 'k': switch (Serial.read())
                {
                  case 'p': Serial.readBytes(read_buffer, 5);
                            if (read_buffer[4] == ';')
                              kp_read = split_num_float();
                            break;
                  case 'i': Serial.readBytes(read_buffer, 5);
                            if (read_buffer[4] == ';')
                              ki_read = split_num_float();
                            break;
                  case 'd': Serial.readBytes(read_buffer, 5);
                            if (read_buffer[4] == ';')
                              kd_read = split_num_float();
                            break;
                }
                break;
      case 'n': if (Serial.read() == 'e')
                {
                  Serial.readBytes(read_buffer, 5);
                  if (read_buffer[4] == ';')
                    req_engine_speed = split_num_float();
                }
                break;
      case 's': if (Serial.read() == 't')
                {
                  Serial.readBytes(read_buffer, 5);
                  if (read_buffer[4] == ';')
                    f_PID_speed = split_num_float();
                }
                break;
      case 't': switch (Serial.read())
                {
                  case '1': Serial.readBytes(read_buffer, 5);
                            if (read_buffer[4] == ';')
                            {
                              INJT1_angle = split_num_float();
                              if (INJT1_angle > 38.7)
                                encoder_counter_INJT1 = (uint16_t)(INJT1_angle*100.0/9.0 - 430);
                              else
                                encoder_counter_INJT1 = (uint16_t)(7560 + INJT1_angle*100.0/9.0);
                            }
                            break;
                  case '2': Serial.readBytes(read_buffer, 5);
                            if (read_buffer[4] == ';')
                            {
                              INJT2_angle = split_num_float();
                              if (INJT2_angle > 38.7)
                                encoder_counter_INJT2 = (uint16_t)(INJT2_angle*100.0/9.0 - 430);
                              else
                                encoder_counter_INJT2 = (uint16_t)(7560 + INJT2_angle*100.0/9.0);
                            }
                            break;
                }
                break;
    }
  }
}
//-------------------------------------------
float PID_cal(float cur, float req)
{  
  e = (req - cur)/e_max;
  out_P = kp*e;
  out_I += ki*e*dt;
  out_I = (out_I<-0.1)?-0.1:((out_I>1.0)?1.0:out_I);
  out_D = kd*((e-e_old)/dt);
  e_old = e;
  out = out_P + out_I + out_D;      //tinh toan output cho bo dieu khien PID
  out = (out<0.0)?0.0:((out>1.0)?1.0:out);
  return out;
}
//-------------------------------------------
void send_data()
{  
  Serial.print("NE");
  send_num_float(engine_speed);
  Serial.print("KP");
  send_num_float(kp);
  Serial.print("KI");
  send_num_float(ki);
  Serial.print("KD");
  send_num_float(kd);
  Serial.print("IT");
  send_num_float(inj_time);
  Serial.println();
}
//-------------------------------------------
void send_num_float(float num_float)
{  
  p_num = (byte*)&num_float; 
  Serial.print((char)*(p_num + 3));
  Serial.print((char)*(p_num + 2));
  Serial.print((char)*(p_num + 1));
  Serial.print((char)*p_num);
  Serial.print(';');
}
//-------------------------------------------
float split_num_float()
{
  p_num = (byte*)&num_float;
  *(p_num + 3) = read_buffer[0];
  *(p_num + 2) = read_buffer[1];
  *(p_num + 1) = read_buffer[2];
  *p_num = read_buffer[3];
  return num_float;
}
