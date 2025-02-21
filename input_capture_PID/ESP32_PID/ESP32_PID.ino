#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include <stdio.h>
#include <stdlib.h>
//-------------------------------------
#define _uint32_t_MAX       4294967296
//-------------------------------------
#define _Base_Freq          160000   // Hz
#define _PID_Freq           100       // Hz    
#define _Working_Freq       10       // Hz    
#define gain_out            0.2
#define dt                  (1.0/_PID_Freq)
//-------------------------------------
#define _Sys_CLK            80000000  // Hz
#define _BaseTimer_CLK      16000000  // Hz
#define _BaseTimer_PRE      (_Sys_CLK / _BaseTimer_CLK)
#define _BaseTimer_TOP      (_BaseTimer_CLK / _Base_Freq)
#define _PIDCycle_MAX       (_Base_Freq / _PID_Freq)
#define _WrkCycle_MAX       (_Base_Freq / _Working_Freq)
hw_timer_t * BaseTimer = NULL;
uint32_t PIDCycle_Counter = _PIDCycle_MAX;
bool F_PID_Cycle = false;
uint32_t WrkCycle_Counter = _WrkCycle_MAX;
bool F_Working_Cycle = false;
//-------------------------------------
// User-defined: input capture pins
#define _CAP00_minFreq      1             // Hz
#define _CAP00_wchdgMAX     ((_Working_Freq/_CAP00_minFreq) + 1)
#define _CAP00_IN           GPIO_NUM_21   // GPIO21 as input pin for Unit 0 & Capture input 0
#define _CAP0_INT_EN        BIT(27)       // Capture 0 interrupt bit
#define array_size          7
#define median_pos          3//((array_size - 1)/2)

#define RC_signal_pin       GPIO_NUM_12
#define Rotate_dir_pin      GPIO_NUM_19
#define Throttle_pin        GPIO_NUM_34
#define Shift_FWD_pin       GPIO_NUM_22
#define Shift_REV_pin       GPIO_NUM_23

#define PWM0_bit            12
#define PWM0_res            4096  
#define PWM0_freq           400
#define channel0            0

#define MAX_set_speed       2800.0  // rpm
#define speed_PulsePerRev   3.0    // ppr
#define diff_ratio          28.0 
#define wheel_radius        150.0   // mm
#define calculate_speed     (_Sys_CLK*60.0/speed_PulsePerRev)


float Acc_FWD_time = 5.0, Dec_FWD_time = 10.0, Acc_REV_time = 5.0, Dec_REV_time = 10.0;
float Acc_FWD_rate, Dec_FWD_rate, Acc_REV_rate, Dec_REV_rate, _shift = 0;

uint16_t duty = 2458;
float duty_float;
uint8_t _FWD, _REV;
volatile float throttle_set_speed, actual_set_speed, motor_speed;
uint32_t cap_wchdgCount = 0;   
volatile uint32_t mcpwm_intr_status, new_cap_value, old_cap_value;
uint32_t cap_value_temp = 0;
uint32_t cap_value = 0;
uint32_t cap_value_raw[array_size], cap_value_sort[array_size], temp_speed;
uint8_t cap_value_index;
uint8_t F_detect_phase = 0, F_clear_encoder_counter = 1;
bool F_cap_newValue = false;
//-------------------------------------
mcpwm_dev_t *MCPWM = &MCPWM0; 
//-------------------------------------
TaskHandle_t taskBackGround;    // Core-0
//-------------------------------------
volatile float kp = 1, ki = 4, kd = 0.001;
volatile float kp_read = 1, ki_read = 4, kd_read = 0.001;
float e, e_old, old_out, out, out_P, out_I, out_D;
const float e_max = 2800.0;
//-------------------------------------
char read_buffer[6];
byte *p_num;
float num_float;
//-------------------------------------
// base timer ISR
//-------------------------------------
void IRAM_ATTR onBaseTimer()
{     
  if (!(--WrkCycle_Counter)) 
  {
    WrkCycle_Counter = _WrkCycle_MAX;
    F_Working_Cycle = true;
  }
  
  if (!(--PIDCycle_Counter)) 
  {
    PIDCycle_Counter = _PIDCycle_MAX;
    F_PID_Cycle = true;
  }
}
//-------------------------------------
// End of base timer ISR
//-------------------------------------
//-------------------------------------
// ISR: MCPWM0 input capture
//-------------------------------------
void IRAM_ATTR mcpwm0_IC_ISR(void *arg)
{     
  mcpwm_intr_status = MCPWM0.int_st.val;
  if (mcpwm_intr_status & _CAP0_INT_EN) 
  {
    new_cap_value = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);
    {
      cap_wchdgCount = _CAP00_wchdgMAX;
      F_cap_newValue = true;      
    }
  }  
  //---------------------------------
  MCPWM->int_clr.val = mcpwm_intr_status; 
}
//-------------------------------------
// End of ISR: MCPWM0 input capture
//-------------------------------------
float temp_LPF_speed, temp_motor_speed;
const float LPF_gain = 0.1;
void LPF_motor_speed(void)
{
  temp_LPF_speed = temp_motor_speed - motor_speed;
  temp_LPF_speed *= LPF_gain;
  motor_speed += temp_LPF_speed;
}
//-------------------------------------
void taskBckGrdCode( void * pvParameters )
{
  while (1)
  {
    
  }
}
//-------------------------------------
void setup()
{
  disableCore0WDT();    // Core0/watchdog-timer should be disabled to avoid hard reset.
  disableCore1WDT();    // Core0/watchdog-timer should be disabled to avoid hard reset.
  xTaskCreatePinnedToCore(  taskBckGrdCode,     // Task function
                            "taskBackGround",   // Name of task
                            10000,              // Stack size of task
                            NULL,               // Parameter of the task
                            1,                  // Priority of the task
                            &taskBackGround,    // Task handle to keep track of created task
                            0);                 // Pin task to core 0
  // control rate -----------------------
  Acc_FWD_rate = MAX_set_speed/Acc_FWD_time/_Working_Freq;
  Dec_FWD_rate = MAX_set_speed/Dec_FWD_time/_Working_Freq;
  Acc_REV_rate = MAX_set_speed/Acc_REV_time/_Working_Freq;
  Dec_REV_rate = MAX_set_speed/Dec_REV_time/_Working_Freq;
  // base timer configuration
  BaseTimer = timerBegin(0, _BaseTimer_PRE, true); 
  timerAttachInterrupt(BaseTimer, &onBaseTimer, true);
  timerAlarmWrite(BaseTimer, _BaseTimer_TOP, true);
  timerAlarmEnable(BaseTimer);  
  // pin setup ---------------------------  
  pinMode(RC_signal_pin, OUTPUT);
  pinMode(_CAP00_IN, INPUT);
  pinMode(Rotate_dir_pin, INPUT);
  pinMode(Throttle_pin, INPUT);
  pinMode(Shift_FWD_pin, INPUT);
  pinMode(Shift_REV_pin, INPUT);

  ledcSetup(channel0, PWM0_freq, PWM0_bit);
  ledcAttachPin(RC_signal_pin, channel0);
  ledcWrite(channel0, duty); 
  //-------------------------------------
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, _CAP00_IN);  
  mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);
//  gpio_pullup_en(_CAP00_IN);
  MCPWM0.int_ena.val = (_CAP0_INT_EN);
  mcpwm_isr_register(MCPWM_UNIT_0, mcpwm0_IC_ISR, NULL, ESP_INTR_FLAG_IRAM, NULL);
  //-------------------------------------    
  Serial.begin(115200);
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
    throttle_set_speed = analogRead(Throttle_pin)/4095.0*2800.0;    
    _FWD = digitalRead(Shift_FWD_pin);  
    _REV = digitalRead(Shift_REV_pin);  
    if (cap_wchdgCount) 
    {
      cap_wchdgCount--; 
      if (!cap_wchdgCount)
      {
        motor_speed = 0;   
        temp_LPF_speed = 0;
        temp_motor_speed = 0;
      }
    }
    if ((!_FWD)&&(_REV))    // FWD
    {
      _shift = 1;
      if (actual_set_speed <= throttle_set_speed)
      {      
        if (actual_set_speed < (throttle_set_speed - Acc_FWD_rate))
          actual_set_speed = actual_set_speed + Acc_FWD_rate;
        else
          actual_set_speed = throttle_set_speed;
      }
      else
      {  
        if (actual_set_speed > (throttle_set_speed + Dec_FWD_rate))
          actual_set_speed = actual_set_speed - Dec_FWD_rate;
        else
          actual_set_speed = throttle_set_speed;
      }
    }
    else if ((_FWD)&&(!_REV))     // REV
    {
      _shift = -1;
      if (actual_set_speed <= throttle_set_speed)
      {      
        if (actual_set_speed < (throttle_set_speed - Acc_REV_rate))
          actual_set_speed = actual_set_speed + Acc_REV_rate;
        else
          actual_set_speed = throttle_set_speed;
      }
      else
      {  
        if (actual_set_speed > (throttle_set_speed + Dec_REV_rate))
          actual_set_speed = actual_set_speed - Dec_REV_rate;
        else
          actual_set_speed = throttle_set_speed;
      }
    }
    else      // NEUTRAL
    {
      _shift = 0;
      actual_set_speed = 0;
    }
    send_data();
  }
  //-----------------------------------------   
  if (F_PID_Cycle)
  {
    //---------------------------------
    F_PID_Cycle = false;   
    if (_shift == 1)    // FWD      
      duty = 2458.0 + PID_cal(motor_speed, actual_set_speed)*820.0;
    else if (_shift == -1)     // REV
      duty = 2458.0 - PID_cal(motor_speed, actual_set_speed)*820.0;
    else      // NEUTRAL
    {
      duty = 2458.0;    
      out_I = 0.0;
    }
    ledcWrite(channel0, duty); 
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
    if (cap_value > 0)
    {
      cap_value_raw[cap_value_index++] = calculate_speed/cap_value;
      cap_value_index = cap_value_index%array_size;
    }
    for (uint8_t i = 0; i < array_size; i++)
      cap_value_sort[i] = cap_value_raw[i];
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
    {
      if (cap_value_sort[median_pos])
//        motor_speed = calculate_speed/cap_value_sort[median_pos];
        temp_motor_speed = cap_value_sort[median_pos];        
      LPF_motor_speed();
    }
    else
    {
      motor_speed = cap_value; 
      for (uint8_t i = 0; i < array_size; i++)
        cap_value_raw[i] = 0;  
    }
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
  duty_float = duty;
  Serial.print("NE");
  send_num_float(motor_speed);
  Serial.print("SH");
  send_num_float(_shift);
  Serial.print("TS");
  send_num_float(throttle_set_speed);
  Serial.print("AS");
  send_num_float(actual_set_speed);
  Serial.print("DT");
  send_num_float(duty_float);
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
