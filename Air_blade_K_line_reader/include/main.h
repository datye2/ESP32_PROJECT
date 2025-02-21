#include <Arduino.h>
#include "driver/twai.h"

/*------------------------------------
---------TiMER CONFIGURATION----------
------------------------------------*/

#define _Base_Freq          10000   // Hz
#define _Working_Freq       1000     // Hz
#define _Request_Freq       50      // Hz
#define _GUI_Freq           5     // Hz

#define _Sys_CLK            80000000  // Hz
#define _BaseTimer_CLK      2000000   // Hz
#define _BaseTimer_PRE      (_Sys_CLK / _BaseTimer_CLK)
#define _BaseTimer_TOP      (_BaseTimer_CLK / _Base_Freq)
#define _WrkCycle_MAX       (_Base_Freq / _Working_Freq)
#define _ReqCycle_MAX       (_Base_Freq / _Request_Freq)
#define _GUICycle_MAX       (_Base_Freq / _GUI_Freq)
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t * BaseTimer = NULL;
unsigned int WrkCycle_Counter = _WrkCycle_MAX, RequestCycle_Counter= _ReqCycle_MAX, GUICycle_Counter= _GUICycle_MAX;
bool F_Working_Cycle = false, F_GUI_Cycle = false, F_Request_Cycle = false;

void IRAM_ATTR onBaseTimer(); //timer vector interrupt
void timer_int()
{
    BaseTimer = timerBegin(0, _BaseTimer_PRE, true); 
    timerAttachInterrupt(BaseTimer, &onBaseTimer, true);
    timerAlarmWrite(BaseTimer, _BaseTimer_TOP, true);
    timerAlarmEnable(BaseTimer);  
}


/*------------------------------------
---------LEDC CONFIGURATION----------
------------------------------------*/

#define LEDC_PIN            25
#define LEDC_FREQUENCY      1000  // Select frequency of PWM.
#define LEDC_RESOLUTION     10  // Range is 1-20 bits for ESP32.

class esp_ledc 
{
    public:
        esp_ledc();  // Constructor declaration
        void pwm_create(float duty);// creat pwm with set freq and duty
        void pwm_change_freq(uint8_t pin, uint32_t freq, uint8_t resolution);
    private:
        uint8_t pin = LEDC_PIN;
        uint32_t freq = LEDC_FREQUENCY;
        uint8_t resolution = LEDC_RESOLUTION;
        uint8_t channel = 0;
};

// Constructor definition
esp_ledc::esp_ledc() {
    pinMode(pin,OUTPUT);
    ledcSetup(channel, freq, resolution);
    ledcAttachPin(pin, channel);
}

void esp_ledc::pwm_create(float duty)
{
    ledcWrite(channel,duty);
}

void esp_ledc::pwm_change_freq(uint8_t pin, uint32_t freq, uint8_t resolution)
{
    ledcChangeFrequency(pin,freq, resolution);
}


/*------------------------------------
---------ADC CONFIGURATION----------
------------------------------------*/
#define ADC_NUM_OF_SAMPLE   5
#define ADC_NUM_OF_CHANNEL  2
#define ADC_READ_RESOLUTION 10
#define LPF_ENABLE          0               // 1 : ENABLE, 0 : DISABLE

// User-defined: Low-pass filter
// with a divider of 32, conversion time = 75 us
// Sampling rate = 1000000 us / 75 us / No. of channels
// select filter coefficient
// y(k) = y(k-1) + Gain*[x(k) - y(k-1)]
// Gain = dt / (RC + dt)
#define _adc_SRate      (1000000.0/75.0/ADC_NUM_OF_CHANNEL) // (Hz), sampling rate
#define _adc_LPF_dt     (1000.0/_adc_SRate)             // (ms)
#define _adc_LPF_RC     (48.0*_adc_LPF_dt)              // (ms), user-defined
#define _adc_LPF_Gain   (_adc_LPF_dt/(_adc_LPF_RC+_adc_LPF_dt))

uint8_t ADC_pin[ADC_NUM_OF_CHANNEL] = {34,32};
uint16_t ADC_avg[ADC_NUM_OF_CHANNEL];
uint8_t ADC_resolution = ADC_READ_RESOLUTION;
bool f_lpf = (bool)LPF_ENABLE;

class esp_adc
{
    public:
        uint8_t adc_channel_pos = 0;
        uint8_t adc_sample_pos = 0;
        uint16_t* pointer = &ADC_value[0][0];
        float  LPFGain = 0.1;
        uint16_t ADC_value[ADC_NUM_OF_CHANNEL][ADC_NUM_OF_SAMPLE];
        void Scan_ADC();
        void Get_avg_value();
        esp_adc();
};

esp_adc::esp_adc()
{
    analogReadResolution(ADC_resolution);
}

void esp_adc::Scan_ADC()
{
    uint16_t ADC_temp = analogRead(ADC_pin[adc_channel_pos]);
    if(f_lpf)
    {
        float ADC_prev = (float)ADC_value[adc_channel_pos][(adc_sample_pos == 0 ? ADC_NUM_OF_SAMPLE - 1 : adc_sample_pos - 1)];
        ADC_temp = ADC_prev + LPFGain * ((float)ADC_temp - ADC_prev);
    }
    ADC_value[adc_channel_pos][adc_sample_pos] = ADC_temp;
    adc_channel_pos++;
    if(adc_channel_pos >= ADC_NUM_OF_CHANNEL) 
    {  
    adc_channel_pos = 0;
    adc_sample_pos++; 
    if(adc_sample_pos >= ADC_NUM_OF_SAMPLE)
    {
        adc_sample_pos = 0;
    }
    }
}

void esp_adc::Get_avg_value()
{
    for(int i = 0; i < ADC_NUM_OF_CHANNEL; i++)
    {
        ADC_avg[i] = 0;
        for(int j = 0 + ADC_NUM_OF_SAMPLE * i ; j < (ADC_NUM_OF_SAMPLE * (i + 1)); j++)
        {
            ADC_avg[i] += *(pointer + j);
        }
        ADC_avg[i] /= 5;

    }
}


