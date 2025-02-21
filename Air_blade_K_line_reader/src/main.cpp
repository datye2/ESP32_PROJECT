#include <Arduino.h>
#include "BluetoothSerial.h"
#include <string>
#include <stdlib.h>
#include "esp_system.h"


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

#include "VirtuinoCM.h"
VirtuinoCM virtuino;               
#define V_memory_count 32          // the size of V memory. You can change it to a number <=255)
float V[V_memory_count];           // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.


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



boolean debug = true;              // set this variable to false on the finale code to decrease the request time.

#define debug Serial
#define bike Serial2
#define PACKET_BUFFER_SIZE (128)
#define TX_PIN 17
#define RX_PIN 16

//SoftwareSerial bike(RX_PIN, TX_PIN); // RX, TX

byte ECU_WAKEUP_MESSAGE[] = {0xFE, 0x04, 0x72, 0x8C}; 
byte ECU_INIT_MESSAGE[] = {0x72, 0x05, 0x00, 0xF0, 0x99};
byte ECU_FIRST_REQUEST[] = {0x72,0x07,0x72,0x00,0x00,0x05,0x10};
byte ECU_SECOND_REQUEST[] = {0x72,0x07,0x72,0xD0,0x00,0x02,0x43};
// Request Frame
byte ECU_REQUEST_D1[] = {0x72,0x07,0x72,0xD1,0x00,0x06}; //Kickstand
byte ECU_REQUEST_17[] = {0x72,0x07,0x72,0x17,0x00,0x0F}; //Engine speed, TPS Vol, TPS, IA Vol, IAT, ECT,BatVol
byte ECU_REQUEST_20[] = {0x72,0x07,0x72,0x20,0x00,0x02}; //Oxygen sensor voltage
byte ECU_REQUEST_00[] = {0x72,0x07,0x72,0x00,0x00,0x05}; //Vehicle information

byte ECU_REQUEST_ERROR_73[] = {0x72,0x05,0x73,0x01};
byte ECU_REQUEST_ERROR_74[] = {0x72,0x05,0x74,0x01};

int ECU_SUCCESS_CHECKSUM = 0x6FB;

uint8_t kickstand_state = 0;
uint16_t engine_speed;
float TPS_Vol, MAF_Vol, Bat_Vol, TPS_Pos, IntakeAir_Temp, Coolant_Temp, Oxy_sensor_Vol;
char VIN[11];
String VIN_GUI;
String test_string = "datmen";
bool f_receive = false;

/*Function for this project*/
float calcValueDivide256(int val);
float calcValueMinus40(int val);



void virtuinoRun();
String onRequested(char variableType, uint8_t variableIndex);
void onReceived(char variableType, uint8_t variableIndex, String valueAsText);

void print_K_line_data();
void read_K_line_stream_data();
void read_K_line_error();
byte calculateChecksum(byte* data, int length);
void K_line_read_data();
void K_line_write_data(byte* request_data, int length);
int initComms();
byte initHonda();

void IRAM_ATTR onBaseTimer()
{
    if(!RequestCycle_Counter--)
    {
        F_Request_Cycle = true;
        RequestCycle_Counter = _ReqCycle_MAX;
    }
    if(!GUICycle_Counter--)
    {
        F_GUI_Cycle = true;
        GUICycle_Counter = _GUICycle_MAX;
    }
}


void setup() 
{
  debug.begin(115200);
  SerialBT.begin("K_line_esp"); //Bluetooth device name   // enable bluetooth
  virtuino.begin(onReceived,onRequested,256);  // enable virtuino
  BaseTimer = timerBegin(0, _BaseTimer_PRE, true); 
  timerAttachInterrupt(BaseTimer, &onBaseTimer, true);
  timerAlarmWrite(BaseTimer, _BaseTimer_TOP, true);
  timerAlarmEnable(BaseTimer);  
  //initHonda();
  delay(50);
}
uint8_t state = 0;
void loop() 
{
    if(!F_Request_Cycle)
    {
        read_K_line_stream_data();
        print_K_line_data();
        //Serial.println("Read data frame");
        if(V[11])
        {
          read_K_line_error();
          //Serial.println("Read error frame");
        }
        portENTER_CRITICAL_ISR(&timerMux);
        F_Request_Cycle = false;
        portEXIT_CRITICAL_ISR(&timerMux);
    }
    if(F_GUI_Cycle)
    {
        virtuinoRun();
        V[0] = kickstand_state;
        //V[1] = engine_speed;
        V[1] += 1;
        V[2] = TPS_Vol;
        V[3] = TPS_Pos;
        V[4] = MAF_Vol;
        V[5] = IntakeAir_Temp;
        V[6] = Coolant_Temp;
        V[7] = Bat_Vol;
        V[8] = Oxy_sensor_Vol;
        portENTER_CRITICAL_ISR(&timerMux);
        F_GUI_Cycle = false;
        portEXIT_CRITICAL_ISR(&timerMux);
    }

}


void K_line_write_data(byte* request_data, int length)
{
  //Serial.println("Writing Data");
  byte extendedData[length + 1];
  memcpy(extendedData, request_data, length);
  byte checksum = calculateChecksum(extendedData, length);
  extendedData[length] = checksum;
  bike.write(extendedData,sizeof(extendedData));
  delay(40);
}

byte read_buffer[256];
int result;
void K_line_read_data()
{
  delay(5);
  result = bike.available();
  if (result > 0) {
    memset(read_buffer, 0, sizeof(read_buffer));
    int bytesRead = min(result, int(sizeof(read_buffer)));
    for (int i = 0; i < bytesRead; i++) 
    {
      read_buffer[i] = bike.read();
      // debug.print(i);debug.print(":");
      // debug.print(read_buffer[i],HEX);
      // debug.print(" ");
      delayMicroseconds(1);
    }
  }
}

uint8_t calculateChecksum(uint8_t* data, int length) // based on https://www.javatpoint.com/2s-complement-in-c
{
    uint8_t sum = 0;
    for (int i = 0; i < length; i++)
    {
        sum += *(data + i);
    }
    return ~sum + 1;
}



byte* Respone_frame;
uint8_t Respone_frame_size;

void read_K_line_stream_data()
{
  // Request and get respone IDBANK 0x17
  K_line_write_data(ECU_REQUEST_17,sizeof(ECU_REQUEST_17));
  K_line_read_data();
  // Get respone frame
  Respone_frame_size = read_buffer[8];
  Respone_frame = (byte *) malloc(Respone_frame_size);
  for(int i = 0; i < Respone_frame_size; i++)
  {
      Respone_frame[i] = read_buffer[sizeof(ECU_REQUEST_17) + 1 + i];
  }
  byte cks = calculateChecksum(Respone_frame,Respone_frame_size - 1);
  if(cks == Respone_frame[Respone_frame_size - 1] && read_buffer[3] == Respone_frame[3])
  {
    
    engine_speed = Respone_frame[5]*256  + Respone_frame[6];
    TPS_Vol = calcValueDivide256(Respone_frame[7]);
    TPS_Pos = (Respone_frame[8]) * 0.5f;
    MAF_Vol = calcValueDivide256(Respone_frame[11]);
    IntakeAir_Temp = calcValueMinus40(Respone_frame[12]);
    Coolant_Temp = calcValueMinus40(Respone_frame[14]);
    Bat_Vol = Respone_frame[15] * 0.1;
    f_receive = true;
  }
  free(Respone_frame);
  // Request and get respone IDBANK 0xD1
  K_line_write_data(ECU_REQUEST_D1,sizeof(ECU_REQUEST_D1));
  K_line_read_data();
  // Get respone frame
  Respone_frame_size = read_buffer[8];
  Respone_frame = (byte *) malloc(Respone_frame_size);
  for(int i = 0; i < Respone_frame_size; i++)
  {
      Respone_frame[i] = read_buffer[sizeof(ECU_REQUEST_D1) + 1 + i];
      
      //debug.print(Respone_frame[i],HEX);debug.print(" ");
  }
  cks = calculateChecksum(Respone_frame,Respone_frame_size - 1);
  if(cks == Respone_frame[Respone_frame_size - 1] && read_buffer[3] == Respone_frame[3])
  {  
    kickstand_state = (Respone_frame[5])?1:0;
    f_receive = true;
  }
  free(Respone_frame);
  // Request and get respone IDBANK 0x20
  K_line_write_data(ECU_REQUEST_20,sizeof(ECU_REQUEST_20));
  K_line_read_data();
  // Get respone frame
  Respone_frame_size = read_buffer[8];
  Respone_frame = (byte *) malloc(Respone_frame_size);
  for(int i = 0; i < Respone_frame_size; i++)
  {
      Respone_frame[i] = read_buffer[sizeof(ECU_REQUEST_20) + 1 + i];
      
      //debug.print(Respone_frame[i],HEX);debug.print(" ");
  }
  cks = calculateChecksum(Respone_frame,Respone_frame_size - 1);
  if(cks == Respone_frame[Respone_frame_size - 1] && read_buffer[3] == Respone_frame[3])
  {
    Oxy_sensor_Vol = calcValueDivide256(Respone_frame[5]);
    f_receive = true;
  }
  free(Respone_frame);
// Request and get respone IDBANK 0x00
  K_line_write_data(ECU_REQUEST_00,sizeof(ECU_REQUEST_00));
  K_line_read_data();
  // Get respone frame
  Respone_frame_size = read_buffer[8];
  Respone_frame = (byte *) malloc(Respone_frame_size);
  for(int i = 0; i < Respone_frame_size; i++)
  {
      Respone_frame[i] = read_buffer[sizeof(ECU_REQUEST_00) + 1 + i];
  }
  cks = calculateChecksum(Respone_frame,Respone_frame_size - 1);
  if(cks == Respone_frame[Respone_frame_size - 1] && read_buffer[3] == Respone_frame[3])
  {
    snprintf(VIN, sizeof(VIN), "%02X%02X%02X%02X" , Respone_frame[5], Respone_frame[6],
                                                 Respone_frame[7], Respone_frame[8]);
    VIN_GUI = VIN;                           
    f_receive = true;
  }
  free(Respone_frame);
}

void read_K_line_error()
{
   // Request and get respone IDBANK 0x73
  K_line_write_data(ECU_REQUEST_ERROR_73,sizeof(ECU_REQUEST_ERROR_73));
  K_line_read_data();
  // Get respone frame
  Respone_frame_size = read_buffer[6];
  Respone_frame = (byte *) malloc(Respone_frame_size);
  for(int i = 0; i < Respone_frame_size; i++)
  {
      Respone_frame[i] = read_buffer[sizeof(ECU_REQUEST_ERROR_73) + 1 + i];
      debug.print(i);debug.print(":");
      debug.print(Respone_frame[i],HEX);
      debug.print(" ");
  }
  debug.println(" ");
  byte cks = calculateChecksum(Respone_frame,Respone_frame_size - 1);
  if(cks == Respone_frame[Respone_frame_size - 1] && read_buffer[3] == Respone_frame[3])
  {
    
    f_receive = true;
  }
  free(Respone_frame);
  // Request and get respone IDBANK 0x74
  K_line_write_data(ECU_REQUEST_ERROR_74,sizeof(ECU_REQUEST_ERROR_74));
  K_line_read_data();
  // Get respone frame
  Respone_frame_size = read_buffer[6];
  Respone_frame = (byte *) malloc(Respone_frame_size);
  for(int i = 0; i < Respone_frame_size; i++)
  {
    Respone_frame[i] = read_buffer[sizeof(ECU_REQUEST_ERROR_74) + 1 + i];
    debug.print(i);debug.print(":");
    debug.print(Respone_frame[i],HEX);
    debug.print(" ");
  }
  debug.println();
  cks = calculateChecksum(Respone_frame,Respone_frame_size - 1);
  if(cks == Respone_frame[Respone_frame_size - 1] && read_buffer[3] == Respone_frame[3])
  {
  f_receive = true;
  }
  free(Respone_frame);
}



void print_K_line_data()
{
  if(f_receive)
  {
    f_receive = false;
    debug.print("VIN: ");
    debug.print(VIN);
    debug.print(" ");

    debug.print("KickState: ");
    debug.print(kickstand_state);
    debug.print(" ");

    debug.print("EngSpeed: ");
    debug.print(engine_speed);
    debug.print(" RPM ");

    debug.print("TPS Vol: ");
    debug.print(TPS_Vol,2);
    debug.print(" V ");

    debug.print("TPS Pos: ");
    debug.print(TPS_Pos);
    debug.print(" % ");

    debug.print("MAF Vol: ");
    debug.print(MAF_Vol,2);
    debug.print(" V ");

    debug.print("IAT: ");
    debug.print(IntakeAir_Temp);
    debug.print(" °C ");

    debug.print("ECT: ");
    debug.print(Coolant_Temp);
    debug.print(" °C ");

    debug.print("Bat Vol: ");
    debug.print(Bat_Vol);
    debug.print(" V ");

    debug.print("Oxy Vol: ");
    debug.print(Oxy_sensor_Vol);
    debug.print(" V ");

    debug.print("\n");
  }
}

float calcValueDivide256(int val) 
{
  //convert to dec, multiple by 5, then divide result by 256
  //used for TPS Volt, ECT Volt, IAT Volt, MAP Volt
  
  return (float)(val*5)/256;
}

float calcValueMinus40(int val) {
  //value minus 40
  //used for ECT Temp, IAT Temp
  
  return val-40;
}
uint16_t counter = 0;
byte temp[] = {0};
byte initHonda() //fixed
{
  //Honda ecu communication handshake
  int initSuccess = 0;
  while(!initSuccess) 
  {
    debug.println("Starting up...");
    debug.println("Setting line low 70ms, high 120ms");
    while(!initComms())
    {
      debug.println("Initing....");
    }
    bike.begin(10400);
    debug.println("Sending ECU Wakeup");
    bike.write(ECU_WAKEUP_MESSAGE, sizeof(ECU_WAKEUP_MESSAGE));
    delay(100);
    debug.println("Sending ECU Init String");
    bike.write(ECU_INIT_MESSAGE, sizeof(ECU_INIT_MESSAGE));
    bike.flush();
    delay(50);
    bike.write(ECU_FIRST_REQUEST, sizeof(ECU_FIRST_REQUEST));
    delay(200);
    // bike.write(ECU_SECOND_REQUEST, sizeof(ECU_SECOND_REQUEST));
    // delay(200);
    K_line_read_data();
    byte temp1[] = {read_buffer[9],read_buffer[10],read_buffer[11]};
    counter++;
    if(counter > 5)
    {
      esp_restart();
    }
    if(read_buffer[12]  == 0xFA|| read_buffer[7] == 0x7C) 
    {
        debug.println("ECU init Success");
        counter = 0;
        initSuccess = 1;
        break;
    }
  }
  return initSuccess;
}

int initComms()  // fixed
{
  //Honda ECU Init sequence, Base on AB init signal
  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, HIGH); //TX High for 100ms
  delay(100);
  digitalWrite(TX_PIN, LOW); //TX Low for 70ms
  delay(70);
  digitalWrite(TX_PIN, HIGH); //TX High for 140ms
  delay(140);
  return 1;
}


  void virtuinoRun(){
   int client = SerialBT.available();
   if (!client) return;
   unsigned long timeout = millis() + 100;
   while (!SerialBT.available() && millis() < timeout) delay(1);
   if (millis() > timeout) {
    SerialBT.flush();
    return;
  }
    virtuino.readBuffer="";    // clear Virtuino input buffer. The inputBuffer stores the incoming characters
      while (SerialBT.available()>0) {        
        char c = SerialBT.read();         // read the incoming data
        virtuino.readBuffer+=c;         // add the incoming character to Virtuino input buffer
      }
     SerialBT.flush();
     String* response= virtuino.getResponse();    // get the text that has to be sent to Virtuino as reply. The library will check the inptuBuffer and it will create the response text
     SerialBT.print(*response);
     SerialBT.flush();
     delay(10);
}
 

//
void onReceived(char variableType, uint8_t variableIndex, String valueAsText)
{     
  if (variableType=='V')
  {
      float value = valueAsText.toFloat();        // convert the value to float. The valueAsText have to be numerical
      if (variableIndex<V_memory_count) V[variableIndex]=value;  // copy the received value to arduino V memory array       
  }
}

//==============================================================
/* This function is called every time Virtuino app requests to read a pin value*/

String onRequested(char variableType, uint8_t variableIndex){     
    if (variableType=='V') 
    {
      if (variableIndex<V_memory_count) return  String(V[variableIndex]);
      else if(variableIndex == 33) return VIN_GUI;
    }
  return "";
}

