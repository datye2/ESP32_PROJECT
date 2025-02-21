#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <stdlib.h>
#include <string.h>

TaskHandle_t Task1;
TaskHandle_t Task2;


byte ledStatus = LOW;                                                                    
volatile int interruptCounter0, FlagSerial = true;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
const int rx_queue_size = 8;
int CounterObject0, CounterObject1;
bool F_request = true, F_receive = true;
uint32_t data_send[33];
CAN_device_t CAN_cfg;  
char *p_num;

void send_num_uint32_t(uint32_t num_uint32_t, char id[2])
{
  Serial.write(id[0]);
  Serial.write(id[1]);
  p_num = (char*)&num_uint32_t;
  Serial.write((char)*p_num +3);
  Serial.write((char)*p_num + 2);
  Serial.write((char)*p_num + 1);
  Serial.write((char)*p_num);
  Serial.write(';');
}

// tạo một class chứa các PID và kí hiệu PID đó

uint16_t read_pot = 0;

class DataReq
{
  private:  CAN_frame_t tx_frame;
            CAN_frame_t rx_frame;
       
  public:   DataReq( unsigned short ID , byte length ,byte mode , byte PID)
           {
              tx_frame.FIR.B.FF = CAN_frame_std;
              tx_frame.FIR.B.RTR = CAN_no_RTR;
              tx_frame.MsgID = ID;
              tx_frame.FIR.B.DLC = 8;
              tx_frame.data.u8[0] = length;
              tx_frame.data.u8[1] = mode;
              tx_frame.data.u8[2] = PID;
              tx_frame.data.u8[3] = 0x55;
              tx_frame.data.u8[4] = 0x55;
              tx_frame.data.u8[5] = 0x55;
              tx_frame.data.u8[6] = 0x55;
              tx_frame.data.u8[7] = 0x55;
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
              tx_frame.data.u8[3] = 0x55;
              tx_frame.data.u8[4] = 0x55;
              tx_frame.data.u8[5] = 0x55;
              tx_frame.data.u8[6] = 0x55;
              tx_frame.data.u8[7] = 0x55;
           }
           void Send()
           { 
              ESP32Can.CANWriteFrame(&tx_frame); 
           }
           void Receive( byte heximal)
           {
            // read_pot = analogRead(15);
            // FlagSerial = true;
              if( CAN_cfg.rx_queue != NULL)
              {
                if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, ( TickType_t ) 2 ) == pdPASS)
                {
                  if(rx_frame.MsgID == 0x7E8)
                  {
                    Serial.println(CounterObject0);
                    data_send[CounterObject0] = rx_frame.data.u8[3] + rx_frame.data.u8[4] + rx_frame.data.u8[5] + + rx_frame.data.u8[6];
                      //Data[0] = rx_frame.data.u8[3]*16777216 + rx_frame.data.u8[4]*65536 + rx_frame.data.u8[5]*256 + rx_frame.data.u8[6];
                       //data_send[CounterObject0] = rx_frame.data.u8[3] + rx_frame.data.u8[4] + rx_frame.data.u8[5] + + rx_frame.data.u8[6];
                    } 
                  }
                }
              }
           
};

 


//PIDS 1
DataReq PIDCheck1 = DataReq(0x01,0x04);  
DataReq PIDCheck2 = DataReq(0x01,0x05);  
DataReq PIDCheck3 = DataReq(0x01,0x06);  
DataReq PIDCheck4 = DataReq(0x01,0x07);  
DataReq PIDCheck5 = DataReq(0x01,0x0C);  
DataReq PIDCheck6 = DataReq(0x01,0x0D);
DataReq PIDCheck7 = DataReq(0x01,0x0E);
DataReq PIDCheck8 = DataReq(0x01,0x0F);
DataReq PIDCheck9 = DataReq(0x01,0x10);
DataReq PIDCheck10 = DataReq(0x01,0x11);
DataReq PIDCheck11 = DataReq(0x01,0x13);
DataReq PIDCheck12 = DataReq(0x01,0x15);
DataReq PIDCheck13 = DataReq(0x01,0x1F);
DataReq PIDCheck14 = DataReq(0x01,0x21);
DataReq PIDCheck15 = DataReq(0x01,0x24);
DataReq PIDCheck16 = DataReq(0x01,0x2E);
DataReq PIDCheck17 = DataReq(0x01,0x30);
DataReq PIDCheck18 = DataReq(0x01,0x31);
DataReq PIDCheck19 = DataReq(0x01,0x33);
DataReq PIDCheck20 = DataReq(0x01,0x34);
DataReq PIDCheck21 = DataReq(0x01,0x3C);
DataReq PIDCheck22 = DataReq(0x01,0x3E);
DataReq PIDCheck23 = DataReq(0x01,0x42);
DataReq PIDCheck24 = DataReq(0x01,0x49);
DataReq PIDCheck25 = DataReq(0x01,0x4D);
DataReq PIDCheck26 = DataReq(0x01,0x4F);
DataReq PIDCheck27 = DataReq(0x01,0x51);
DataReq PIDCheck28 = DataReq(0x01,0x52);
DataReq PIDCheck29 = DataReq(0x01,0x54);
DataReq PIDCheck30 = DataReq(0x01,0x55);
DataReq PIDCheck31 = DataReq(0x01,0x56);
DataReq PIDCheck32 = DataReq(0x01,0x5B);



void  IRAM_ATTR onTimer() {
      portENTER_CRITICAL_ISR(&timerMux);
      interruptCounter0++;
      ledStatus = !ledStatus;
      digitalWrite(2, ledStatus);  
      portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() 
{
  pinMode(15, INPUT);
  Serial.begin(115200);
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 5000, true); // chu kỳ timer 5 milli giây
  timerAlarmEnable(timer);
  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  ESP32Can.CANInit(); 

  xTaskCreatePinnedToCore(
    Send_require
    ,  "Send_tx_frame_require"   
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

void loop() 
{
}


void Send_require(void *pvParameters)  
{  
  for (;;) 
  {
    if(interruptCounter0 != 0)
    { 
      
      portENTER_CRITICAL(&timerMux);
      interruptCounter0 = 0;
      portEXIT_CRITICAL(&timerMux);
      if(F_request = true)
      {
      CounterObject0++;
      }
      switch (CounterObject0)
      {
        case  1:
        {
          PIDCheck1.Send();
          F_request = false;
          break;
        }
        case  2:
        {
          PIDCheck2.Send();
          F_request = false;
          break;
        }
        case  3:
        {
          PIDCheck3.Send();
          F_request = false;
          break;
        }
        case  4:
        {
          PIDCheck4.Send();
          F_request = false;
          break;
        }
        case  5:
        {
          PIDCheck5.Send();
          F_request = false;
          break;
        }
        case  6:
        {
          PIDCheck6.Send();
          break;
        }
        case  7:
        {
          PIDCheck7.Send();
          F_request = false;
          break;
        }
        case  8:
        {
          PIDCheck8.Send();
          F_request = false;
          break;
        }
        case  9:
        {
          PIDCheck9.Send();
          F_request = false;
          break;
        }
        case  10:
        {
          PIDCheck10.Send();
          break;
        }
        case  11:
        {
          PIDCheck11.Send();
          F_request = false;
          break;
        }
        case  12:
        {
          PIDCheck12.Send();
          F_request = false;
          break;
        }
        case  13:
        {
          PIDCheck13.Send();
          F_request = false;
          break;
        }
        case  14:
        {
          PIDCheck14.Send();
          F_request = false;
          break;
        }
        case  15:
        {
          PIDCheck15.Send();
          F_request = false;
          break;
        }
        case  16:
        {
          PIDCheck16.Send();
          F_request = false;
          break;
        }
        case  17:
        {
          PIDCheck17.Send();
          F_request = false;
          break;
        }
        case  18:
        {
          PIDCheck18.Send();
          F_request = false;
          break;
        }
        case  19:
        {
          PIDCheck19.Send();
          F_request = false;
          break;
        }
        case  20:
        {
          PIDCheck20.Send();
          F_request = false;
          break;
        }
        case  21:
        {
          PIDCheck21.Send();
          F_request = false;
          break;
        }
        case  22:
        {
          PIDCheck22.Send();
          F_request = false;
          break;
        }
        case  23:
        {  
          PIDCheck23.Send();
          F_request = false;
          break;
        }
        case  24:
        {
          PIDCheck24.Send();
          F_request = false;
          break;
        }
        case  25:
        {
          PIDCheck25.Send();
          F_request = false;
          break;
        }
        case  26:
        {
          PIDCheck26.Send();
          F_request = false;
          break;
        }
        case  27:
        {
          PIDCheck27.Send();
          F_request = false;
          break;
        }
        case  28:
        {
          PIDCheck28.Send();
          F_request = false;
          break;
        }
        case  29:
        {
          PIDCheck29.Send();
          F_request = false;
          break;
        }
        case  30:
        {
          PIDCheck30.Send();
          F_request = false;
          break;
        }
        case  31:
        {
          PIDCheck31.Send();
          F_request = false;
          break;
        }
        case  32:
        {
          PIDCheck32.Send();
          F_request = false;
          break;
        }
      switch (CounterObject0)
      {
        case  1:
        {
          PIDCheck1.Receive(0x04);
          F_request = true;
          break;
        }
        case  2:
        {
          PIDCheck2.Receive(0x05);F_request = true;
          break;
        }
        case  3:
        {
          PIDCheck3.Receive(0x06);F_request = true;
          break;
        }
        case  4:
        {
          PIDCheck4.Receive(0x07);F_request = true;
          break;
        }
        case  5:
        {
          PIDCheck5.Receive(0x0C);F_request = true;
          break;
        }
        case  6:
        {
          PIDCheck6.Receive(0x0D);F_request = true;
          break;
        }
        case  7:
        {
          PIDCheck7.Receive(0x0E);F_request = true;
          break;
        }
        case  8:
        {
          PIDCheck8.Receive(0x0F);F_request = true;
          break;
        }
        case  9:
        {
          PIDCheck9.Receive(0x10);F_request = true;
          break;
        }
        case  10:
        {
          PIDCheck10.Receive(0x11);F_request = true;
          break;
        }
        case  11:
        {
          PIDCheck11.Receive(0x13);F_request = true;
          break;
        }
        case  12:
        {
          PIDCheck12.Receive(0x15);F_request = true;
          break;
        }
        case  13:
        {
          PIDCheck13.Receive(0x1F);F_request = true;
          break;
        }
        case  14:
        {
          PIDCheck14.Receive(0x21);F_request = true;
          break;
        }
        case  15:
        {
          PIDCheck15.Receive(0x24);F_request = true;
          break;
        }
        case  16:
        {
          PIDCheck16.Receive(0x2E);F_request = true;
          break;
        }
        case  17:
        {
          PIDCheck17.Receive(0x30);F_request = true;
          break;
        }
        case  18:
        {
          PIDCheck18.Receive(0x31);F_request = true;
          break;
        }
        case  19:
        {
          PIDCheck19.Receive(0x33);F_request = true;
          break;
        }
        case  20:
        {
          PIDCheck20.Receive(0x34);F_request = true;
          break;
        }
        case  21:
        {
          PIDCheck21.Receive(0x3C);F_request = true;
          break;
        }
        case  22:
        {
          PIDCheck22.Receive(0x3E);F_request = true;
          break;
        }
        case  23:
        {
          PIDCheck23.Receive(0x42);F_request = true;
          break;
        }
        case  24:
        {
          PIDCheck24.Receive(0x49);F_request = true;
          break;
        }
        case  25:
        {
          PIDCheck25.Receive(0x4D);F_request = true;
          break;
        }
        case  26:
        {
          PIDCheck26.Receive(0x4F);F_request = true;
          break;
        }
        case  27:
        {
          PIDCheck27.Receive(0x51);F_request = true;
          break;
        }
        case  28:
        {
          PIDCheck28.Receive(0x52);F_request = true;
          break;
        }
        case  29:
        {
          PIDCheck29.Receive(0x54);F_request = true;
          break;
        }
        case  30:
        {
          PIDCheck30.Receive(0x55);F_request = true;
          break;
        }
        case  31:
        {
          PIDCheck31.Receive(0x56);F_request = true;
          break;
        }
        case  32:
        {
          PIDCheck32.Receive(0x5B);F_request = true;
          CounterObject0 = 0;
          portENTER_CRITICAL(&timerMux);
          FlagSerial = 1;
          portEXIT_CRITICAL(&timerMux);  
          break;
        }
      }
    }
    vTaskDelay(10);
  }
}
}

void TaskResponse(void *pvParameters)  
{
  for (;;)
  { 
    if (FlagSerial)
    {
     // gửi lên Serial
      // send_num_uint32_t(data_send[1],"04");
      // send_num_uint32_t(data_send[2],"05");
      // send_num_uint32_t(data_send[3],"06");
      // send_num_uint32_t(data_send[4],"07");
      // send_num_uint32_t(data_send[5],"0C");
      // send_num_uint32_t(data_send[6],"0D");
      // send_num_uint32_t(data_send[7],"0E");
      // send_num_uint32_t(data_send[8],"0F");
      // send_num_uint32_t(data_send[9],"10");
      //send_num_uint32_t(data_send[10],"11");
    //   send_num_uint32_t(data_send[11],"13");
    //   send_num_uint32_t(data_send[12],"15");
    //   send_num_uint32_t(data_send[13],"1F");
    //   send_num_uint32_t(data_send[14],"21");
    //   send_num_uint32_t(data_send[15],"24");
    //   send_num_uint32_t(data_send[16],"2E");
    //   send_num_uint32_t(data_send[17],"30");
    //   send_num_uint32_t(data_send[18],"31");
    //   send_num_uint32_t(data_send[19],"33");
    //   send_num_uint32_t(data_send[20],"34");
    //   send_num_uint32_t(data_send[21],"3C");
    //   send_num_uint32_t(data_send[22],"3E");
    //   send_num_uint32_t(data_send[23],"42");
    //   send_num_uint32_t(data_send[24],"49");
    //   send_num_uint32_t(data_send[25],"4D");
    //   send_num_uint32_t(data_send[26],"4F");
    //   send_num_uint32_t(data_send[27],"51");
    //   send_num_uint32_t(data_send[28],"52");
    //   send_num_uint32_t(data_send[29],"54");
    //   send_num_uint32_t(data_send[30],"55");
    //   send_num_uint32_t(data_send[31],"56");
    //   send_num_uint32_t(data_send[32],"5B");
       send_num_uint32_t(data_send[33],"4D");
    //  //
       portENTER_CRITICAL(&timerMux);
       FlagSerial = 0;
       portEXIT_CRITICAL(&timerMux);  
    }          
  vTaskDelay(10);
  }
}
