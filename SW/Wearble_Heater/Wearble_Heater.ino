#include "SevSeg.h"
#include <Wire.h>
#include "Adafruit_MCP9808.h"
SevSeg sevseg; 
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

#define UP_SW_PIN 32U
#define DOWN_SW_PIN 26U
#define TEMP_PWM_PIN 33U

int TTimer = 0U, LEDTimer = 0U;
float Temp_Sensor = 0.0, Temp_Desired = 25.0,Temp_Output = 0.0;
float PID_TimeChange = 0.0, PID_TimeLast  = 0.0, PID_Error = 0.0, PID_Intg = 0.0, PID_Derv = 0.0, PID_ErrorLast = 0.0;
float kp = 40, ki = 0.0002, kd = 50;

void setup() {
  Serial.begin(115200);

  pinMode(UP_SW_PIN, INPUT);
  pinMode(DOWN_SW_PIN, INPUT);
  pinMode(TEMP_PWM_PIN, OUTPUT);
  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);
  
  byte numDigits = 3;
  byte digitPins[] = {27,14,12};
  byte segmentPins[] = {5, 23, 16, 4, 2, 18, 19, 17};
  bool resistorsOnSegments = true; 
  bool updateWithDelaysIn = true;
  sevseg.begin(N_TRANSISTORS , numDigits, digitPins, segmentPins, resistorsOnSegments);
  sevseg.setBrightness(100); 

  if (!tempsensor.begin(0x18)) {
    Serial.println("Couldn't find MCP9808! Check your connections and verify the address is correct.");
    while (1);
  }  
  tempsensor.setResolution(3);
  delay(200);
  Temp_Desired = tempsensor.readTempC(); 
   
  digitalWrite(25, LOW);
}

void loop() {
    sevseg.setNumber((int)Temp_Sensor);
    sevseg.refreshDisplay();
    
    if(digitalRead(UP_SW_PIN) == LOW){
      if(Temp_Desired < 50.0){
        Temp_Desired = Temp_Desired + 1.0;
      } 
      while(digitalRead(UP_SW_PIN) == LOW){       
        sevseg.setNumber((int)Temp_Desired);
        sevseg.refreshDisplay(); 
        digitalWrite(25, HIGH);
      }       
    }
    if(digitalRead(DOWN_SW_PIN) == LOW){
      if(Temp_Desired > 1.0){
        Temp_Desired = Temp_Desired - 1.0;
      }  
      while(digitalRead(DOWN_SW_PIN) == LOW){     
          sevseg.setNumber((int)Temp_Desired);
          sevseg.refreshDisplay(); 
          digitalWrite(25, HIGH);
      }
    }
  
    TTimer++;
    if(TTimer > 200U){
      TTimer = 0U;
      Temp_Sensor = tempsensor.readTempC(); 
  
      /*Compute PID on sensed temperature*/
      unsigned long PID_Time = millis();    
      PID_TimeChange = (float)(PID_Time - PID_TimeLast);    
      PID_Error = Temp_Desired - Temp_Sensor;
      PID_Intg = PID_Intg + (PID_Error * PID_TimeChange);
      PID_Derv = (PID_Error - PID_ErrorLast)/PID_TimeChange;
      Temp_Output = (kp*PID_Error) + (ki*PID_Intg) + (kd*PID_Derv);        
      PID_ErrorLast = PID_Error;
      PID_TimeLast = PID_Time;  
      if(Temp_Output > 255.0){
        Temp_Output = 255.0;
      }
      if(Temp_Output < 0.0){
        Temp_Output = 0.0;
      }
  
      int Temp_Diff = 0U;
      if(Temp_Sensor > Temp_Desired){
        Temp_Diff = (int)(Temp_Sensor - Temp_Desired);
      }
      else{
        Temp_Diff = (int)(Temp_Desired - Temp_Sensor);
      }
      if(Temp_Diff < 1U){      
        digitalWrite(25, HIGH);
      }
      else{    
        LEDTimer++;
        if(LEDTimer>250U){
          LEDTimer = 0U;
         digitalWrite(25, (!digitalRead(25)));
        }
      }
      Serial.println((int)Temp_Output);
      analogWrite(TEMP_PWM_PIN, (int)Temp_Output);
    }
}
