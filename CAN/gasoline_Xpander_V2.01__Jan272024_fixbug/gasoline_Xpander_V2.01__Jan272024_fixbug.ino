
#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#define ESP32
#include <stdlib.h>
#include <string.h>

TaskHandle_t Task1;
TaskHandle_t Task2;



String send_frame;
String send_frame_string;

const int bufferSize = 10;  // Kích thước của buffer
char myBuffer[bufferSize];  // Khai báo mảng để lưu trữ buffer

uint8_t send_frame_size;
uint8_t send_frame_string_size;
byte ledStatus = LOW;                                                                    
volatile int interruptCounter0, FlagSerial;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
const int rx_queue_size = 8;
int LetGo , CounterObject0, CounterObject1, FlagVIN, AllVINChecked, FlagPID = 0,
           CounterPID = 1;
float Data[13];
String Data_string[2];
bool FlagFrame[15] = {false, false, false, false, false,
                     false, false, false, false, false, false, false,false, false, false}; 
CAN_device_t CAN_cfg;  


void stringToBuffer(String inputString) {
  // Chuyển đổi chuỗi ký tự thành buffer
  for (int i = 0; i < 10; i++) {
    myBuffer[i] = inputString.charAt(i);
  }
  
  // Đặt ký tự null (kết thúc chuỗi) vào cuối buffer
  myBuffer[inputString.length()] = ';';
}

void creat_send_frame()
{
  send_frame = "";
  send_frame += 'N';
  send_frame += 'A';
  send_frame += 'M';
}
  
// }
void send_num_float(float num_float, char id[2])
{
  send_frame += id[0];
  send_frame += id[1];
  byte* p_num_float = (byte*)&num_float; 
  send_frame += (char)*(p_num_float + 3);
  send_frame += (char)*(p_num_float + 2);
  send_frame += (char)*(p_num_float + 1);
  send_frame += (char)*p_num_float;
  send_frame += ';';
}
void _send()
{  
  send_frame_size = send_frame.length() + 5;
  send_frame += (char) send_frame_size;
  send_frame += 'H';
  send_frame += 'O';
  send_frame += 'A';  
  send_frame += '\n';
  Serial.println(send_frame);
}

void creat_send_frame_string()
{
  send_frame_string = "";
  send_frame_string += 'R';
  send_frame_string += 'U';
  send_frame_string += 'N';
}
  
// }
void send_string(String num_string, char id[2])
{
  send_frame_string += id[0];
  send_frame_string += id[1];
      // Chuyển đổi chuỗi ký tự thành buffer
  stringToBuffer(num_string);
  // for (int i = 0; i < num_string.length() + 1; i++) {
  for (int i = 0; i < bufferSize; i++) {
    send_frame_string += myBuffer[i];
  }
  // In buffer ra cổng Serial
  for (int i = 0; i < bufferSize; i++) {
//    Serial.print(myBuffer[i]);
  }
}
void _send_string()
{  
  send_frame_string_size = send_frame_string.length() + 5;
  send_frame_string += (char) send_frame_string_size;
  send_frame_string += 'E';
  send_frame_string += 'N';
  send_frame_string += 'D';  
  send_frame_string += '\n';
  Serial.println(send_frame_string);
}


class DataReq{
  private:
       CAN_frame_t tx_frame;
       CAN_frame_t rx_frame;
       uint8_t PIDs[4];
       char STR[200], StringBit[200];
       String MyVIN="";
       String DTCStored="";
       String NumDTCStored="";
       String DTCPending="";
       String DTCPending1="";
       String DTCPending2="";
       String NumDTCPending="";
       String DTCPermanent="";
       String NumDTCPermanent="";
       int Bi;
       byte RAWBIT;
      
     
                                        
    
       public:
       DataReq( unsigned short ID , byte length ,byte mode , byte PID)
       {
        tx_frame.FIR.B.FF = CAN_frame_std;
        tx_frame.FIR.B.RTR = CAN_no_RTR;
        tx_frame.MsgID = ID;
        tx_frame.FIR.B.DLC = 8;
        tx_frame.data.u8[0] = length;
        tx_frame.data.u8[1] = mode;
        tx_frame.data.u8[2] = PID;
        tx_frame.data.u8[3] = 0x00;
        tx_frame.data.u8[4] = 0x00;
        tx_frame.data.u8[5] = 0x00;
        tx_frame.data.u8[6] = 0x00;
        tx_frame.data.u8[7] = 0x00;
       }
       DataReq( byte mode , byte PID)
       {
        tx_frame.FIR.B.FF = CAN_frame_std;
        tx_frame.FIR.B.RTR = CAN_no_RTR;
        tx_frame.MsgID = 0x7DF;
        tx_frame.FIR.B.DLC = 8;
        tx_frame.data.u8[0] = 0x02;
        tx_frame.data.u8[1] = mode;
        tx_frame.data.u8[2] = PID;
        tx_frame.data.u8[3] = 0x00;
        tx_frame.data.u8[4] = 0x00;
        tx_frame.data.u8[5] = 0x00;
        tx_frame.data.u8[6] = 0x00;
        tx_frame.data.u8[7] = 0x00;
       }
      
       void Send()
       { 
          ESP32Can.CANWriteFrame(&tx_frame); 
          
       }
    
      void RecPID()
      {
         if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
         {
          if(rx_frame.MsgID == 0x7E8)
          {
             if(rx_frame.data.u8[2] == tx_frame.data.u8[2])
             {
               
                 for(  int i = 0; i<=3; i++)
                {
                  PIDs[i] = rx_frame.data.u8[i+3]; 
                }
               TransferBit();
               Serial.print("PIDs SUPPORTED 0x");
               Serial.print(tx_frame.data.u8[2],HEX) ;
               Serial.print(":");
               Serial.println(StringBit);
               StringBit[0] ='\0'; 
             
               
         
             }
          }
         }
      }
       
       void TransferBit()
       {
          for( int j = 0 ; j<4; j++)
            {
              RAWBIT = PIDs[j];
              for (int i = 7; i >= 0; i--)
                {
                  Bi = bitRead(RAWBIT,i);
                  sprintf(STR,"%d",Bi);
                  strcat(StringBit,STR);   
                }   
             }
             if( StringBit[31]  == '1' ){FlagPID = 0;  CounterPID++;}
             else { FlagPID = 1 ; CounterPID = 0;}
             
             
              
            
              
       }
       void CheckVIN()
       {
         if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
         {
          if(rx_frame.MsgID == 0x7E8)
          {
            if( FlagVIN == 1)
            {
              
              switch(rx_frame.data.u8[0])
              {
                case 0x21: //VIN-PART2
                  for(  int v = 1; v< sizeof(rx_frame.data.u8);v++)
                  {
                    MyVIN += String((char)rx_frame.data.u8[v]);
                  
                  }
                  break;
                case 0x22: //VIN-PART3
                  for(  int v = 1; v< sizeof(rx_frame.data.u8);v++)
                  {
                    MyVIN += String((char)rx_frame.data.u8[v]);
                  }
                  
                  Serial.println(MyVIN);
                  AllVINChecked = 1;
                  break;
              }
           }
            switch(rx_frame.data.u8[2])
            {
            
              case 0x49: // VIN-PART 1
                if( rx_frame.data.u8[0] == 0x10) // VIN
                {
                 
                  
                  
                    MyVIN += String((char)rx_frame.data.u8[5]);
                    MyVIN += String((char)rx_frame.data.u8[6]);
                    MyVIN += String((char)rx_frame.data.u8[7]);                
                   
                    FlagVIN = 1;
                   
                    if (MyVIN.length() > 10) {
                        Serial.print("VINCODE :");
                        Serial.print(MyVIN);
                        //delay(100);
                      }
                 
                } 
            }
          }
       
          
         }
       }
      
         void Receive( byte heximal)
       {
       
        if( CAN_cfg.rx_queue != NULL)
        {
       if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, ( TickType_t ) 2 ) == pdPASS)
      
        {
          if(rx_frame.MsgID == 0x7E8)
          {
           //if(rx_frame.data.u8[2] == heximal)
           // { 
              switch(rx_frame.data.u8[2])
              {
                case 0x0C: //RPM
                  { float rpm = (float)((rx_frame.data.u8[3]*256.0 + rx_frame.data.u8[4])/4.0);
                  Data[0] = float (rpm);
                  FlagFrame[0] = true;
                  break; }
                case 0x5A: //PEDAL 
                  { float pedal = (float)((rx_frame.data.u8[3]*100.0)/255.0);
                  Data[1] = float (pedal);
                  FlagFrame[1] = true;
                  break; }
                case 0x45: //THROTTLE
                  { float throttle = (float)(((rx_frame.data.u8[3]*100.0)/255.0));
                  Data[2] = float(throttle);
                  FlagFrame[2] = true;
                  break; }
                case 0x06: //STFT
                  { float stft = ((float)rx_frame.data.u8[3]/1.28 - 100);
                  Data[3] = float(stft);
                  FlagFrame[3] = true;
                  break; }
                case 0x07: //LTFT
                  { float ltft = ((float)rx_frame.data.u8[3]/1.28 - 100);
                  Data[4] = float(ltft);
                  FlagFrame[4] = true;
                  break; }
                case 0x14: //O2>Cat
                  { float O2F = (float)((rx_frame.data.u8[3])/200.0);
                  Data[5] = float(O2F);
                  FlagFrame[5] = true;
                  break; }
                case 0x15: //Cat>O2
                  { float O2R = (float)((rx_frame.data.u8[3])/200.0);
                  Data[6] = float(O2R);
                  FlagFrame[6] = true;
                  break; }
                case 0x0B: //MAP
                  { float map = (float)(rx_frame.data.u8[3]);
                  Data[7] = float(map);
                  FlagFrame[7] = true;
                  break; }    
                case 0x05: //Engine coolant temperature
                  { float ect = (float)(rx_frame.data.u8[3] - 40);
                  Data[8] = float(ect);
                  FlagFrame[8] = true;
                  break; } 
                case 0x0E: //Timing advance
                  { float timing = ((float)rx_frame.data.u8[3]/2 - 64);
                  Data[9] = float(timing);
                  FlagFrame[9] = true;
                  break; } 
                case 0x0F: //Intake air temp
                  { float iat = (float)(rx_frame.data.u8[3] - 40);
                  Data[10] = float(iat);
                  FlagFrame[10] = true;
                  break; }   
                case 0x43: //Absolute load value
                  { float abload = (float)((rx_frame.data.u8[3]*256.0 + rx_frame.data.u8[4])*100.0/255.0);
                  Data[11] = float(abload);
                  FlagFrame[11] = true;
                  break; } 
                case 0x03: //Fuel System Status
  { int check_03 = (int)(rx_frame.data.u8[3]);
  // String Data_string[0] ;

  switch (check_03) {
    case 0:
      Data_string[0] = "Engoff";
      break;
    case 1:
      Data_string[0] = "OLtemp";
      break;
    case 2:
      Data_string[0] = "CL mix";
      break;
    case 4:
      Data_string[0] = "OLdece";
      break;
    case 8:
      Data_string[0] = "OLfail";
      break;
    case 16:
      Data_string[0] = "CLfeed";
      break;
    default:
      Data_string[0] = "ECUnot";
  }

  FlagFrame[12] = true;
  break; }
                case 0x01: //MIL status
                  { int MIL_dec = (int)(rx_frame.data.u8[3]);
                  String MIL_hex =  String(MIL_dec, HEX);
                  long MIL_hexValue = strtol(MIL_hex.c_str(), NULL, 16);
                  bool MIL_bit8 = bitRead(MIL_hexValue, 7);
                  if (MIL_bit8)
                  {
                    Data_string[1] = "MIL ON";
                  } else {
                    Data_string[1] = "MILOFF";
                  }
                  FlagFrame[13] = true;
                  break; }   
                case 0x30: //Warm up count
                  { float warmcount = (float)(rx_frame.data.u8[3]);
                  Data[12] = float(warmcount);
                  FlagFrame[14] = true;
                  break; } 
              }
              if( (FlagFrame[0] == true)&&(FlagFrame[1] == true)&&(FlagFrame[2] == true)&&(FlagFrame[3] == true)&&(FlagFrame[4] == true)&&(FlagFrame[5] == true)&&(FlagFrame[6] == true)&&(FlagFrame[7] == true)&&(FlagFrame[8] == true)&&(FlagFrame[9] == true)&&(FlagFrame[10] == true)&&(FlagFrame[11] == true)&&(FlagFrame[12] == true)&&(FlagFrame[13] == true)&&(FlagFrame[14] == true))
              {
                
                 for (int p=0;p<=14;p++)
                {
                  FlagFrame[p] = false;
                }
                //Serial.println("DA NAP XONG");
                 portENTER_CRITICAL(&timerMux);
                 FlagSerial = 1 ;
                 portEXIT_CRITICAL(&timerMux);
               
                
              }
            

            
          }
         }
       }
      }
       
 
       

};

  //Creat Object

  DataReq PIDCheck1 = DataReq(0x01,0x00);// mode 1 PID 0x00
  DataReq PIDCheck2 = DataReq(0x01,0x20);  
  DataReq PIDCheck3 = DataReq(0x01,0x40);  
  DataReq PIDCheck4 = DataReq(0x01,0x60);  
  DataReq PIDCheck5 = DataReq(0x01,0x80);  
  DataReq PIDCheck6 = DataReq(0x01,0xA0);  
  DataReq PIDCheck7 = DataReq(0x01,0xC0);
  DataReq RPM       = DataReq(0x01,0x0C);
  DataReq PEDAL     = DataReq(0x01,0x5A);
  DataReq THROTTLE  = DataReq(0x01,0x45);
  DataReq STFT      = DataReq(0x01,0x06);
  DataReq LTFT      = DataReq(0x01,0x07);
  DataReq O2Front   = DataReq(0x01,0x14);
  DataReq O2Rear    = DataReq(0x01,0x15);
  DataReq Map       = DataReq(0x01,0x0B);
  DataReq ECT       = DataReq(0x01,0x05);
  DataReq ITA       = DataReq(0x01,0x0E);
  DataReq IAT       = DataReq(0x01,0x0F);
  DataReq LOAD      = DataReq(0x01,0x43);  
  DataReq SYS       = DataReq(0x01,0x03);
  DataReq MIL       = DataReq(0x01,0x01);
  DataReq WARM      = DataReq(0x01,0x30);  
  DataReq VIN       = DataReq(0x09,0x02);// mode 9 PID 0x02
  // DataReq Mode03    = DataReq(0x7DF,0x01,0x03,0x00); //mode 3 DTC Storesd
  // DataReq Mode07    = DataReq(0x7DF,0x01,0x07,0x00); //mode 7 DTC Pending
  // DataReq Mode0A    = DataReq(0x7DF,0x01,0x0A,0x00); //mode 0A DTC Permannent
  DataReq REPVIN    = DataReq(0x7E0,0x30,0x00,0x00); // ID 0x7E0 -> Respone to ECU to accept receive VIN CODE
 
 
  //add Object if need more. 
void CheckPIDANDVIN() {
   while( FlagPID != 1)
  {
    switch( CounterPID)
    {
      case 1:
        PIDCheck1.Send();
        break;
      case 2:
        PIDCheck2.Send();
        break;
      case 3:
        PIDCheck3.Send();
        break;
      case 4:
        PIDCheck4.Send();
        break;
      case 5:
        PIDCheck5.Send();
        break;
      case 6:
       PIDCheck6.Send();
        break;
      case 7:
       PIDCheck7.Send();
        break;
    }
  }

    while( FlagVIN != 1)
    {
      VIN.Send();
    }
    while ( AllVINChecked != 1)
    {
      REPVIN.Send();
      
    }
  
    if( FlagPID == 1 && AllVINChecked == 1)
    {
      LetGo = 1; 
    }
}
void ReceivePIDANDVIN() {
  while( FlagPID != 1)
        {
          switch(CounterPID)
          {
            case 1:
              PIDCheck1.RecPID();
              break;
            case 2:
              PIDCheck2.RecPID();
              break;
            case 3:
              PIDCheck3.RecPID();
              break;
            case 4:
              PIDCheck4.RecPID();
              break;
            case 5:
              PIDCheck5.RecPID();
              break;
            case 6:
              PIDCheck6.RecPID();
              break;
            case 7:
              PIDCheck7.RecPID();
              break;
          }
        }
        
    while( FlagVIN != 1)
    {
      VIN.CheckVIN();
    }
    while( AllVINChecked != 1)
    {
      REPVIN.CheckVIN();
    }
   
}
void  IRAM_ATTR onTimer() {
      portENTER_CRITICAL_ISR(&timerMux);
      interruptCounter0++;
      ledStatus = !ledStatus;
      digitalWrite(2, ledStatus);  
      portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  pinMode(2, OUTPUT);
  Serial.begin(115200);
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10000, true);
  timerAlarmEnable(timer);
  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  ESP32Can.CANInit(); 
 
       
  
  
  xTaskCreatePinnedToCore(
    TaskQuery
    ,  "SendQuery"   
    ,  10000 
    ,  NULL
    ,  1  
    ,  &Task1 
    ,  0);

  xTaskCreatePinnedToCore(
    TaskResponse
    ,  "GetResponse"
    ,  10000  
    ,  NULL
    ,  1  
    ,  &Task2 
    ,  1);

} 

void loop() {
}


void TaskQuery(void *pvParameters)  
{
  CheckPIDANDVIN();

  for (;;) 
  {
    if( LetGo == 1)
    {
      if( interruptCounter0 != 0)
      {
        portENTER_CRITICAL(&timerMux);
        interruptCounter0 = 0;
        portEXIT_CRITICAL(&timerMux);
        CounterObject0++;
        switch(CounterObject0)
        {
          case 1:
            RPM.Send();
            break;
          case 2:
            PEDAL.Send();
            break;
          case 3:
            THROTTLE.Send();
            break;
          case 4:
            STFT.Send();
            break;
          case 5:
            LTFT.Send();
          case 6:
            O2Front.Send();
            break;
          case 7:
            O2Rear.Send();
            break;
          case 8:
            Map.Send();
            break; 
          case 9:
            ECT.Send();
            break; 
          case 10:
            ITA.Send();
            break; 
          case 11:
            IAT.Send();
            break; 
          case 12:
            LOAD.Send();
            break;
          case 13:
            SYS.Send();
            break; 
          case 14:
            MIL.Send();
            break; 
          case 15:
            CounterObject0 = 0;
            WARM.Send();
            break; 
        }
      }
    }
    CounterObject1++;
    switch(CounterObject1)
    {
      case 1:
        RPM.Receive( 0x0C);
        //Serial.print("RPM: ");
        //Serial.println(Data[0]);
        //delay(100);
        break;
      case 2:
        PEDAL.Receive(0x5A);
        PEDAL.Receive(0x5A);
        //Serial.print("PEDAL: ");
        //Serial.println(Data[1]);
        //delay(100);
        break;
      case 3:
        THROTTLE.Receive(0x45);
        //Serial.print("THROTTLE: ");
        //Serial.println(Data[2]);
        //delay(100);
        break;
      case 4:
        STFT.Receive(0x06);
        //Serial.print("STFT: ");
        //Serial.println(Data[3]);
        //delay(100);
        break;
      case 5:
        LTFT.Receive(0x07);
        //Serial.print("LTFT: ");
        //Serial.println(Data[4]);
        //delay(100);
        break;
      case 6:
        O2Front.Receive(0x14);
        //Serial.print("O2 Front: ");
        //Serial.println(Data[5]);
        //delay(100);
        break;
      case 7:
        O2Rear.Receive(0x15);
        //Serial.print("O2 Rear: ");
        //Serial.println(Data[6]);
        //delay(100);
        break;
      case 8:
        Map.Receive(0x0B);
        //Serial.print("MAP: ");
        //Serial.println(Data[7]);
        //delay(100);
        break; 
      case 9:
        ECT.Receive(0x05);
        //Serial.print("ECT: ");
        //Serial.println(Data[8]);
        //delay(100);
        break; 
      case 10:
        ITA.Receive(0x0E);
        //Serial.print("Timing: ");
        //Serial.println(Data[9]);
        //delay(100);
        break; 
      case 11:
        IAT.Receive(0x0F);
        //Serial.print("IAT: ");
        //Serial.println(Data[10]);
        //delay(100);
        break;
      case 12:
        LOAD.Receive(0x43);
        //Serial.print("LOAD: ");
        //Serial.println(Data[11]);
        //delay(100);
        break;
      case 13:
        SYS.Receive(0x03);
        //Serial.print("Fuel Sytem Status: ");
        //Serial.println(Data_string[0]);
        //delay(100);
        break; 
      case 14:
        MIL.Receive(0x01);
        //Serial.print("MIL status: ");
        //Serial.println(Data_string[1]);
        //delay(100);
        break;
      case 15:
        CounterObject1 = 0;
        WARM.Receive(0x30);
        //Serial.print("Warm up count: ");
        //Serial.println(Data[12]);
        //Serial.println("//////////////////////////END FRAME///////////////////////////////////");
        //delay(100);
        break;  
    }
    
  }
}

void TaskResponse(void *pvParameters)  
{
  ReceivePIDANDVIN();
 
  
  for (;;)
  { 
    if ( FlagSerial == 1)
    {
       portENTER_CRITICAL(&timerMux);
       FlagSerial = 0;
       portEXIT_CRITICAL(&timerMux);
       creat_send_frame();
       send_num_float(Data[0], "ne");
       send_num_float(Data[1], "ap");
       send_num_float(Data[2], "tp");
       send_num_float(Data[3], "st");
       send_num_float(Data[4], "lt");
       send_num_float(Data[5], "o1");
       send_num_float(Data[6], "o2");
       send_num_float(Data[7], "ma");
       send_num_float(Data[8], "ec");
       send_num_float(Data[9], "ia");
       send_num_float(Data[10], "it");
       send_num_float(Data[11], "lo");
       send_num_float(Data[12], "wa");
       _send();
       creat_send_frame_string();
       send_string(Data_string[0], "sy");
       send_string(Data_string[1], "mi");
       _send_string();    
    }                
  }
} 