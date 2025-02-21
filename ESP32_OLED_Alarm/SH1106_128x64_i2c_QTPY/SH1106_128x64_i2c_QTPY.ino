
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#include <time.h>
#include "pitch.h"
#define i2c_Address 0x3c 
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);




const uint8_t Pointer[] PROGMEM = {
  B00000,
  B00100,
  B00010,
  B11111,
  B00010,
  B00100,
  B00000,
  B00000,
};

const uint8_t Weather_icon[] PROGMEM = {
    0x00, 0x00, 0x02, 0x20, 0x01, 0x00, 0x00, 0xF2, 
    0x00, 0x08, 0x0E, 0x44, 0x32, 0x84, 0x20, 0x87, 
    0x80, 0x78, 0x80, 0x08, 0x00, 0x08, 0x40, 0x00, 
    0x20, 0x0A, 0x00, 0x08, 0xFF, 0xF0, 0x01, 0x00
};

const unsigned char icon_test [] PROGMEM = {
	// '32, 90x90px
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x70, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xfe, 0x1c, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x8f, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x04, 0x03, 0xb0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x07, 0xfc, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x1c, 0x0e, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x07, 0x00, 0x60, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x03, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x60, 0x01, 0x80, 0x4e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x60, 0x01, 0xf0, 0xc6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0xfc, 0xc0, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 
	0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0xc0, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 
	0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0xc0, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 
	0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0e, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t activeSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00011000,
    B00100100,
    B01000010,
    B01000010,
    B00100100,
    B00011000
};

const uint8_t inactiveSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00011000,
    B00011000,
    B00000000,
    B00000000
};

/*------------------------------------
---------TiMER CONFIGURATION----------
------------------------------------*/


#define _Base_Freq          10000   // Hz
#define _Working_Freq       10      // Hz
#define _Oled_Freq          1       // Hz
#define _Alarm_Freq          1       // Hz

#define _Sys_CLK            80000000  // Hz
#define _BaseTimer_CLK      2000000   // Hz
#define _BaseTimer_PRE      (_Sys_CLK / _BaseTimer_CLK)
#define _BaseTimer_TOP      (_BaseTimer_CLK / _Base_Freq)
#define _WrkCycle_MAX       (_Base_Freq / _Working_Freq)
#define _OledCycle_MAX      (_Base_Freq / _Oled_Freq)
#define _AlarmCycle_MAX      (_Base_Freq / _Alarm_Freq)

hw_timer_t * BaseTimer = NULL;
unsigned int WrkCycle_Counter = _WrkCycle_MAX, OledCycle_Counter= _OledCycle_MAX, AlarmCycle_Counter= _AlarmCycle_MAX;
bool F_Working_Cycle = false, F_Oled_Cycle = false, F_Alarm_Cycle = false;

void IRAM_ATTR onBaseTimer(); //timer vector interrupt
void timer_int()
{
    BaseTimer = timerBegin(0, _BaseTimer_PRE, true); 
    timerAttachInterrupt(BaseTimer, &onBaseTimer, true);
    timerAlarmWrite(BaseTimer, _BaseTimer_TOP, true);
    timerAlarmEnable(BaseTimer);  
}

void IRAM_ATTR onBaseTimer()
{
  if(!(--WrkCycle_Counter))
  {
    WrkCycle_Counter = _WrkCycle_MAX;
    F_Working_Cycle = true;
  }
  if(!(--OledCycle_Counter))
  {
    OledCycle_Counter = _OledCycle_MAX;
    F_Oled_Cycle = true;
  }
  if(!(--AlarmCycle_Counter))
  {
    AlarmCycle_Counter = _AlarmCycle_MAX;
    F_Alarm_Cycle = true;
  }
}

const char* ssid = "Dat";
const char* password = "123123123";

const char* ntpServer = "pool.ntp.org"; 
const long  gmtOffset_sec = 7 * 3600;   
const int   daylightOffset_sec = 0;    

uint8_t second_count = 1;
#define DEMO_DURATION 3000
typedef void (*Demo)(void);
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
//unsigned long timerDelay = 600000;
// Set timer to 10 seconds (10000)
unsigned long timerDelay = 10000;
String httpGETRequest(const char* serverName);
String jsonBuffer;
JSONVar myJSON;

bool f_pressed, f_push;
uint16_t counter_button = 0;
enum button_state
{
  press, push
}button;
uint8_t old_button = 0;


void IRAM_ATTR  isr() 
{
  button = press;
  f_pressed = true;
}



volatile bool lastStateB = 0;  
uint8_t alarm_minute, alarm_hour, current_state = 0;
bool f_alarm_time_out = false, f_run_alarm = false;
bool f_change_up = 0, f_change_down = 0,f_new = false;
uint8_t counterr = 0;
void IRAM_ATTR  isr2() 
{
  f_new = true;
  bool currentStateB = digitalRead(27);
  if (currentStateB != lastStateB) 
  {
    if (currentStateB) {
      f_change_up = true;  // Quay theo chiều kim đồng hồ
    } else {
      f_change_up = false; // Quay ngược chiều kim đồng hồ
    }
  }
  lastStateB = currentStateB;
}




// Project function declare

void oled_print_text(char* string);
void oled_display(String city, double temperature, int humd, int presure, int state);
void drawLoadingScreen() ;
void printLocalTime();
void display_clock();
void  angle_cal(float* second,float* minute ,float* hour,
                char* array_second,char* array_minute, char* array_hour);
void draw_clock_face();
void draw_hand(float angle, int Radian);

//
#define _X_clock_center 95
#define _Y_clock_center 32
#define _clock_Radian   30

void setup()   
{
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
  Serial.print('.');
  delay(1000);
}
  pinMode(25, INPUT_PULLUP); // nut nhan chinh
  pinMode(26, INPUT_PULLUP); // encoder
  pinMode(27, INPUT_PULLUP);  // encoder
  pinMode(13, INPUT_PULLUP);  // nut nha phu
  ledcSetup(0, 490, 12); // Kênh, tần số ban đầu, độ phân giải 8-bit
  ledcAttachPin(4, 0); // Gắn chân buzzer vào kênh PWM
  attachInterrupt(25,isr,CHANGE);
  attachInterrupt(26,isr2,CHANGE);
  timer_int();
  //OLED
  display.begin(i2c_Address, true); 
  display.display();
  // Clear the buffer.
  display.clearDisplay();
  //Get real-time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

}
String city_name;
int humd, pressure;
bool f_alarm = false;
double temp;
uint8_t pointer_pos = 1; uint8_t mode = 0;
uint8_t current_location = 0; //0 : Long An, 1: TPHCM
uint8_t alarm_pos = 0;
String serverPath;
bool f_error = false;
void loop() 
{
// Send an HTTP GET request
  if(F_Working_Cycle)
  {
    F_Working_Cycle = false;
    counterr++;
    if(counterr >= 10 & f_new)
    {
      f_new = false;
    }
    printLocalTime();
    switch(current_location)
    {
      case 0:
        serverPath = "http://api.openweathermap.org/data/2.5/weather?q=Long%20An,VN&APPID=aeedde5c10c91b5f96856e0d964e4e63&units=metric";
        break;
      case 1:
      serverPath = "http://api.openweathermap.org/data/2.5/weather?q=Ho%20Chi%20Minh,VN&APPID=aeedde5c10c91b5f96856e0d964e4e63&units=metric";
        break;
    }
    jsonBuffer = httpGETRequest(serverPath.c_str());
    //Serial.println(jsonBuffer);
    JSONVar myObject = JSON.parse(jsonBuffer);
    city_name = (const char*)myObject["name"];
    temp = (double)(myObject["main"]["temp"]);
    humd = (int)(myObject["main"]["humidity"]);
    pressure = (int)(myObject["main"]["pressure"]); 
  }
    // Check WiFi connection status
  if(F_Oled_Cycle)
  {
    F_Oled_Cycle = false;
    if(!digitalRead(13))
    {
      f_run_alarm = true;
    }
    if(f_pressed)
    {
      if(!digitalRead(25))
      {
        counter_button++;
        if(counter_button >= 2)
        {
          f_push = true;
          button = push;
        }
      }
      if(digitalRead(25))
      {
        f_push = false;
        f_pressed = false;
        button = press;
        if(!old_button)
        {
          pointer_pos++;
          pointer_pos = pointer_pos % 4;
        }
        counter_button = 0;
      }
      if(button && !old_button) // sự kiện nhấn đè
      {
        if(pointer_pos == 3)
        {
          mode++; mode =  mode % 3;  
        }
        else if(pointer_pos == 0)
        {
          current_location++; current_location = current_location % 2;
        }

      }
      old_button = button;
    }
    if(!f_error)
    {
      oled_display(city_name, temp,humd,pressure,mode);
    }
    else
    {
      drawLoadingScreen();
    }
    //button = 1 là đè
   } 
}


String httpGETRequest(const char* serverName) {
  WiFiClient client;
  HTTPClient http;
    
  // Your Domain name with URL path or IP address with path
  http.begin(client, serverName);
  
  // Send HTTP POST request
  int httpResponseCode = http.GET();
  
  String payload = "{}"; 
  
  if (httpResponseCode>0) 
  {
    //Serial.print("HTTP Response code: ");
    //Serial.println(httpResponseCode);
    payload = http.getString();
    f_error = false;
  }
  else 
  {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
    f_error = true;
  }
  // Free resources
  http.end();

  return payload;
}



void oled_print_text(char* string)
{
  char* sample = string;
  for(int i = 0; i <strlen(sample); i++)
  {
    display.write(*(sample+i));
  }
}
        uint8_t a = 1;
        uint8_t b = 0;
void oled_display(String city, double temperature, int humd, int presure, int state) //
{
  switch(state)
  {
    case 0:
        display.clearDisplay();
        char temperature_buffer[10];
        char humd_buffer[10];
        char presure_buffer[10];
        char city_array[20];
        city.toCharArray(city_array,sizeof(city_array));
        sprintf(temperature_buffer, "%.1f", temperature);
        sprintf(humd_buffer, "%d", humd);
        sprintf(presure_buffer, "%d", presure);
        display.setTextSize(1);
        display.setTextColor(SH110X_WHITE);
        display.setCursor(16, 1);

        oled_print_text(city_array);
        display.setCursor(16, 16);

        oled_print_text("Temp :");
        display.setTextSize(1);
        display.setCursor(66, 16);
        oled_print_text(temperature_buffer);
        display.setCursor(92, 16);
        oled_print_text("C");

        display.setCursor(16, 32);
        display.setTextSize(1);
        oled_print_text("Hmud :");
        display.setTextSize(1);
        display.setCursor(66, 32);
        oled_print_text(humd_buffer);
        display.setCursor(92, 32);
        oled_print_text("%");

        display.setCursor(16, 48);
        display.setTextSize(1);
        oled_print_text("Pressure:");
        display.setTextSize(1);
        display.setCursor(66, 48);
        oled_print_text(presure_buffer);

        display.drawBitmap(0, pointer_pos*16, Pointer, 8, 8, SH110X_WHITE);
        display.drawBitmap(70, 0, icon_test, 90, 90, SH110X_WHITE);
        display.display();
      break;
    case 1:
        display_clock();
        break;
    case 2:
        if(f_new)
        {
          if(f_change_up)
          {
            alarm_minute++;
          }
          else
          {
            alarm_minute--;
          }
          alarm_minute = alarm_minute %  60;
        }
        alarm(alarm_minute,b);
        break;
  }
}


void drawLoadingScreen() 
{
  display.clearDisplay();
  // Hiển thị tiêu đề
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 10);
  display.println(("Loading, please wait"));
  // Hiển thị thanh tiến trình
  for (int progress = 0; progress <= 100; progress += 5) 
  {
    display.fillRect(20, 30, progress, 10, SH110X_WHITE); // Thanh tiến trình
    display.drawRect(20, 30, 100, 10, SH110X_WHITE);     // Khung
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(30, 50);
    display.println(("Design by Dat"));
    display.display();
    delay(100); // Tốc độ loading
  }
}

char timeHour[3],timeMinute[3],timeSecond[3];
char timeWeekDay[10], Month[10],Day[10],Year[10];

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  strftime(timeHour,3, "%H", &timeinfo);
  strftime(timeMinute,3, "%M", &timeinfo);
  strftime(timeSecond,3, "%S", &timeinfo);
  strftime(timeWeekDay,10, "%A", &timeinfo);
  strftime(Month,10, "%B", &timeinfo);
  strftime(Day,10, "%d", &timeinfo);
  strftime(Year,10, "%Y", &timeinfo);
}

float second_angle, minute_angle, hour_angle;
void display_clock()
{
  display.clearDisplay();
  // Hiển thị tiêu đề
  draw_clock_face();
  display.setTextSize(1.5);
  display.setTextColor(SH110X_WHITE);
  if(pointer_pos == 3)
  {
    display.drawBitmap(120, 60, Pointer, 8, 8, SH110X_WHITE);
  }
  display.setCursor(0, 30);
  oled_print_text(timeHour);
  display.setCursor(10, 30);
  oled_print_text(":");
  display.setCursor(20, 30);
  oled_print_text(timeMinute);
  display.setCursor(30, 30);
  oled_print_text(":");
  display.setCursor(40, 30);
  oled_print_text(timeSecond);
  display.setTextSize(1);
  display.setCursor(0, 40);
  oled_print_text(timeWeekDay);
  display.setCursor(0, 50);
    oled_print_text(Day);
  display.setCursor(20, 50);
    oled_print_text(Month);
  display.setCursor(70, 50);
    oled_print_text(Year);
  angle_cal(&second_angle,&minute_angle,&hour_angle,timeSecond,timeMinute,timeHour);
  draw_hand(second_angle,30);
  draw_hand(minute_angle,25);
  draw_hand(hour_angle,15);
  // draw_hand()
  display.display();
}

void  angle_cal(float* second,float* minute ,float* hour,
                char* array_second,char* array_minute, char* array_hour)
{
  char *output;
  long result = strtol(array_second, &output, 10);
  *second = (float)result*6;
  result  = strtol(array_minute, &output, 10);
  *minute = result*6 + (*second)/60;
  float temp = result;
  result  = strtol(array_hour, &output, 10);
  *hour   = result*30 + temp/2;
}


void draw_clock_face()
{
  display.drawCircle(_X_clock_center, _Y_clock_center, _clock_Radian, SH110X_WHITE);
  for(int i = 0 ; i < 12; i++)
  {
    int x1 = _X_clock_center + _clock_Radian*(cos((i*30 - 90) * 3.14 / 180));
    int y1 = _Y_clock_center + (_clock_Radian) *(sin((i*30 - 90) * 3.14 / 180));
    int x2 = _X_clock_center + (_clock_Radian - 3)*(cos((i*30 - 90) * 3.14 / 180));
    int y2 = _Y_clock_center + (_clock_Radian - 3) *(sin((i*30 - 90) * 3.14 / 180));
    display.drawLine(x1, y1, x2, y2, SH110X_WHITE);
  }
}

void draw_hand(float angle, int Radian)
{
  int x1 = _X_clock_center + Radian*(cos((angle - 90) * 3.14 / 180));
  int y1 = _Y_clock_center + Radian*(sin((angle - 90) * 3.14 / 180));
  display.drawLine(_X_clock_center, _Y_clock_center, x1, y1, SH110X_WHITE);
}

// hàm alarm

void alarm(uint8_t alarm_minute, uint8_t alarm_hour)
{
    char minute_buffer[10], hour_buffer[10],second_buffer[10];
    static unsigned long previousMillis = 0; // Lưu thời điểm trước đó
    const unsigned long interval = 1000;  
    
    while (f_run_alarm)
    { 
       unsigned long currentMillis = millis();

        // Kiểm tra xem đã đủ 1 giây chưa
      if (currentMillis - previousMillis >= interval)
      {
        previousMillis = currentMillis;
        second_count--;
        Serial.println("\t"); Serial.print(alarm_hour);
        Serial.println("\t"); Serial.print(alarm_minute);
        Serial.println("\t"); Serial.println(second_count);
        bool f_alarm_ring = false;
        if (!second_count)
        {
            second_count = 60;
            if (alarm_minute > 0)
            {
                alarm_minute--;
            }
            else if (alarm_hour > 0)
            {
                alarm_minute = 59; // Reset phút về 59 nếu còn giờ
                alarm_hour--;
            }
            else
            {
                f_alarm_ring = true; // Hết giờ và phút
                f_run_alarm = false;
                Serial.println("bao thuc");
            }
        }

        display.clearDisplay();
        display.setTextSize(1.5);
        display.setTextColor(SH110X_WHITE);
        display.setCursor(0, 5);

        if (!f_alarm_ring)
        {
            oled_print_text("Running");
        }
        else
        {   
            tune_buzzer();
            oled_print_text("End");
        }
        sprintf(second_buffer, "%d", second_count);
        sprintf(minute_buffer, "%d", alarm_minute);
        sprintf(hour_buffer, "%d", alarm_hour);
        display.setTextSize(3);
        display.setCursor(5, 24);
        oled_print_text(hour_buffer);
        display.setCursor(35, 24);
        oled_print_text(":");
        display.setCursor(50, 24);
        oled_print_text(minute_buffer);
        display.setCursor(75, 24);
        oled_print_text(":");
        display.setCursor(85, 24);
        oled_print_text(second_buffer);
        display.display();
    }
    }
        sprintf(second_buffer, "%d", second_count);
        sprintf(minute_buffer, "%d", alarm_minute);
        sprintf(hour_buffer, "%d", alarm_hour);
        display.clearDisplay();
        if(pointer_pos == 3)
        {
          display.drawBitmap(120, 60, Pointer, 8, 8, SH110X_WHITE);
        }
        display.setTextSize(1.5);
        display.setTextColor(SH110X_WHITE);
        display.setCursor(0, 5);
        oled_print_text("Ready");
        display.setTextSize(3);
        display.setCursor(5, 24);
        oled_print_text(hour_buffer);
        display.setCursor(35, 24);
        oled_print_text(":");
        display.setCursor(50, 24);
        oled_print_text(minute_buffer);
        display.setCursor(75, 24);
        oled_print_text(":");
        display.setCursor(85, 24);
        oled_print_text(second_buffer);
    
    display.display();
}

void tune_buzzer()
{
	 int size = sizeof(durations) / sizeof(int); 
	 for (int note = 0; note < size; note++) { 
	   //to calculate the note duration, take one second divided by the note type. 
	   //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc. 
	   int duration = 1000 / durations[note];  
     ledcWriteTone(0, melody[note]);
      delay(duration);  
	   //to distinguish the notes, set a minimum time between them. 
	   //the note's duration + 30% seems to work well: 
	   int pauseBetweenNotes = duration * 1.30; 
	   delay(pauseBetweenNotes); 
	   //stop the tone playing: 
	   ledcWriteTone(0, 0);
   }
}

