#include "WiFi.h"
#include <HTTPClient.h>

//----------------------------------------SSID and PASSWORD of your WiFi network.
const char* ssid = "Dat";  //--> Your wifi name
const char* password = "123123123"; //--> Your wifi password
//----------------------------------------

// Google script Web_App_URL.
String Web_App_URL = "https://script.google.com/macros/s/AKfycbxXxRBrbIy7e3l05FnGb1Nc6fa-o0OTMy19MHQj4j08yGHU44cZVbXPvHVIo0uCYKrJMw/exec";


String adc = "";


//________________________________________________________________________________

//________________________________________________________________________________VOID SETUP()
void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial.println();
  delay(1000);

  pinMode(36, INPUT);

  //----------------------------------------Set Wifi to STA mode
  Serial.println();
  Serial.println("-------------");
  Serial.println("WIFI mode : STA");
  WiFi.mode(WIFI_STA);
  Serial.println("-------------");
  //---------------------------------------- 

  //----------------------------------------Connect to Wi-Fi (STA).
  Serial.println();
  Serial.println("------------");
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  //:::::::::::::::::: The process of connecting ESP32 with WiFi Hotspot / WiFi Router.
  // The process timeout of connecting ESP32 with WiFi Hotspot / WiFi Router is 20 seconds.
  // If within 20 seconds the ESP32 has not been successfully connected to WiFi, the ESP32 will restart.
  // I made this condition because on my ESP32, there are times when it seems like it can't connect to WiFi, so it needs to be restarted to be able to connect to WiFi.

  int connecting_process_timed_out = 20; //--> 20 = 20 seconds.
  connecting_process_timed_out = connecting_process_timed_out * 2;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
    if (connecting_process_timed_out > 0) connecting_process_timed_out--;
    if (connecting_process_timed_out == 0) {
      delay(1000);
      ESP.restart();
    }
  }


  
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("------------");
  //::::::::::::::::::
  //----------------------------------------

  delay(2000);
}
//________________________________________________________________________________

//________________________________________________________________________________VOID LOOP()
void loop() {
  // put your main code here, to run repeatedly:
  uint16_t adc_value = analogRead(36);

  //----------------------------------------Conditions that are executed when WiFi is connected.
  // This condition is the condition for sending or writing data to Google Sheets.
  if (WiFi.status() == WL_CONNECTED) {


    // Create a URL for sending or writing data to Google Sheets.
    String Send_Data_URL = Web_App_URL + "?sts=write" + "&srs=Success";
    Send_Data_URL += "&adc=" + String(adc_value);

    Serial.println();
    Serial.println("-------------");
    Serial.println("Send data to Google Spreadsheet...");
    Serial.print("URL : ");
    Serial.println(Send_Data_URL);

    //::::::::::::::::::The process of sending or writing data to Google Sheets.
      // Initialize HTTPClient as "http".
      HTTPClient http;
  
      // HTTP GET Request.
      http.begin(Send_Data_URL.c_str());
      http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  
      // Gets the HTTP status code.
      int httpCode = http.GET(); 
      Serial.print("HTTP Status Code : ");
      Serial.println(httpCode);
  
      // Getting response from google sheets.
      String payload;
      if (httpCode > 0) {
        payload = http.getString();
        Serial.println("Payload : " + payload);    
      }
      
      http.end();
    //::::::::::::::::::
    

    Serial.println("-------------");
  }
  //----------------------------------------

  delay(10000);
}
//________________________________________________________________________________
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
