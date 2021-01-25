#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include <Blynk.h>
#include <Adafruit_MAX31865.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

#define BLYNK_PRINT Serial

Adafruit_MAX31865 fireplace = Adafruit_MAX31865(27, 12, 13, 14);            //Pin connection MAX31865 module
#define RREF 430.0                              //Value Vref resistor 430.0 for PT100 and 4300.0 for PT1000
#define RNOMINAL 100.0                          //The nominal 0ºC resistance of sensor 100.0 for PT100 and 1000.0 for PT1000
  const float emergencyTempFireplace = 87;      //Water temperature Fireplace for activation emergency normaly 90
  const float calibrationTempFireplace = -4;    //Value to calibrate offset throug real measurement and signal transmitter
  float fireplaceTemp;                          //Water temperature fireplace
  const int emergencyHysteresis = 4;            //Hysteresis stay emergency funtion
  bool dischargeState = false;                  //State discharge security

  char auth[] = "P7tFnQt5LBVKBVK0H-__j9St2Xe9xaER";     // You should get Auth Token in the Blynk App. Go to the Project Settings (nut icon).
  char server[] = "blynk-cloud.com";
  unsigned int port = 8442;
  char ssid[] = "CASA HERMENEGILDO 2";          // Your WiFi credentials, SSID
  char pass[] = "hermenegildo";                 // Your WiFi credentials, PASSWORD

  Servo controlClap;                        //Name servo move to clapper

  int signalPot;                            //Conversion analog to digital signal potentiometer
  float clapPosition;                       //Conversion digital to position degrees clapper
  int newSetpointClap;                      //Set point clapper position
  int setpointClap;                         //Parameter before to new setpoint clapper position
  int servoSetpointClap;                    //Conversion Set point servo indication
  int speedClap = 100;                      //Delay to spin 1º for move clapper (50 to 300 recomended)
  const int spinPulse = 4;                  //Degrees movement for each pulse
  bool openClap = false;                    //Direcction clapper open
  bool closeClap = false;                   //Direcction clapper close
  bool automatic = false;                   //Clapeta funtion automatic-manual  
  bool startClap = true;                    //Initialization clapper position
  bool servoOff = true;                     //Activation to start servo
  int touchOpen;                            //Capacitive sensor open clapper
  int touchClose;                           //Capacitive sensor close clapper
  unsigned int offSetClap;                  //Diferential degrees setpoint clap and real value clapper
  const int difAlarmClap = 8;               //Trigger alarm for maximun diferential to setpoint and real value clapper

  const int emergencyRelay = 21;             //Pin activate emergency relay (Relay IN4)
  const int dischargeValve = 19;            //Pin to activate electrovalve to system discharge (Relay IN3)
  const int powerThermRelay = 18;           //Pin power supply thermometer livingroom (Relay IN2)
  const int thermLightRelay = 5;            //Pin activate light thermometer livingroom (Relay IN1)
  const int powerMax31865 = 26;             //Pin to power supply MAX31865 module
  const int buzzer = 22;                    //Pin activate buzzer emergency 
  const int pot = 39;                       //Pin connection potentiometer in 
  const int calibration0 = 1536;
  const int calibration100 = 512;

  bool a = true;
  bool b = true;
  bool c = true;

  WidgetLED emergencyLed(V6);
  WidgetLED automaticLed(V7);


TaskHandle_t TEMPERATURE, READCLAP, TOUCHREAD, TOUCHCLAP;                         //Define multitask name
  
void setup()
{
  Serial.begin(115200);                     //Activate serial monitor

  Serial.print("MAC_ADDRESS = ");Serial.println(WiFi.macAddress());
  IPAddress rapsberry_server (192,168,0,100);       //IP servidor raspberry pi
  
  connectWiFi();
  
  pinMode(pot, INPUT);
  pinMode(emergencyRelay, OUTPUT);
  digitalWrite(emergencyRelay, HIGH);
  pinMode(dischargeValve, OUTPUT);
  digitalWrite(dischargeValve, HIGH);
  pinMode(powerThermRelay, OUTPUT);
  pinMode(thermLightRelay, OUTPUT);
  digitalWrite(thermLightRelay, HIGH);
  pinMode(buzzer, OUTPUT);
  pinMode(powerMax31865, OUTPUT);           //Define pin supply to MAX31865 module
  digitalWrite(powerMax31865, HIGH);        //Power on MAX31865 module
  fireplace.begin(MAX31865_4WIRE);          //2WIRE, 3WIRE or 4WIRE depend sensor connection
  delay(500);

  xTaskCreatePinnedToCore(                  //Funtion task parameter
    temperature,                            //Funtion name to run in task
    "Task_1",                               //Task label
    1000,                                   //Cell size  
    NULL,                                   //Parameter to get
    1,                                      //Ejecution priority
    &TEMPERATURE,                           //Name to funtion to run
    0);                                     //Core procesor to ejecution

  xTaskCreatePinnedToCore(                  //Funtion task parameter
    pushClapPosition,                       //Funtion name to run in task
    "Task_2",                               //Task label
    1000,                                   //Cell size  
    NULL,                                   //Parameter to get
    2,                                      //Ejecution priority
    &READCLAP,                              //Name to funtion to run
    0);                                     //Core procesor to ejecution

  xTaskCreatePinnedToCore(                  //Funtion task parameter
    touchRead,                       //Funtion name to run in task
    "Task_3",                               //Task label
    1000,                                   //Cell size  
    NULL,                                   //Parameter to get
    3,                                      //Ejecution priority
    &TOUCHREAD,                              //Name to funtion to run
    0);                                     //Core procesor to ejecution

  xTaskCreatePinnedToCore(                  //Funtion task parameter
    touchClap,                               //Funtion name to run in task
    "Task_4",                               //Task label
    2000,                                   //Cell size  
    NULL,                                   //Parameter to get
    4,                                      //Ejecution priority
    &TOUCHCLAP,                              //Name to funtion to run
    0);                                     //Core procesor to ejecution
    
}

void connectWiFi()
{
  WiFi.begin(ssid, pass);
  int x = 0;
  while(WiFi.status() !=  WL_CONNECTED and (x <= 3))
  {
    Serial.println("WIFI CONNECTING");
    delay(5000);
    x = x+1;
  }
  if(WiFi.status() == WL_CONNECTED)
  {
    Serial.println("WIFI CONNECTION SUCCESSFUL");
    Serial.printf("CONNECTED TO WIFI SSID = ", WiFi.SSID());
    Serial.printf("IP ADDRESS = ", WiFi.localIP());
    connectBlynk();
  }
  else
  {
    Serial.println("FAIL WIFI CONNECTION");
  }
}

void connectBlynk()
{
  Blynk.config(auth);
  Blynk.connect();
  delay(500);
  Serial.println(Blynk.connect());
  Blynk.notify("SYSTEM STARTING");         //Starting app notification
}

void temperature (void *parameter)
{
  for(;;)
  {
    fireplaceTemp = (fireplace.temperature(RNOMINAL, RREF) + calibrationTempFireplace);      //Calculate water temperature fireplace
    //Blynk.virtualWrite(V0, fireplaceTemp);                                                   //Send to blynk water temperature fireplace
    delay(2000);
    Serial.print("Temperatura = "); Serial.println(fireplaceTemp);
    
    // Check and print any faults 
    uint8_t fault = fireplace.readFault();                                                                                                    
    if (fault) 
    {
      //Serial.print("Fault 0x"); Serial.println(fault, HEX);
      //Blynk.notify("Fault 0x" + (fault, HEX));
      if (fault & MAX31865_FAULT_HIGHTHRESH) 
      {
        //Blynk.notify("RTD High Threshold");                                       
      }
      if (fault & MAX31865_FAULT_LOWTHRESH) 
      {
        //Blynk.notify("RTD Low Threshold"); 
      }
      if (fault & MAX31865_FAULT_REFINLOW) 
      {
        //Blynk.notify("REFIN- > 0.85 x Bias"); 
      }
      if (fault & MAX31865_FAULT_REFINHIGH) 
      {
        //Blynk.notify("REFIN- < 0.85 x Bias - FORCE- open"); 
      }
      if (fault & MAX31865_FAULT_RTDINLOW) 
      {
        //Blynk.notify("RTDIN- < 0.85 x Bias - FORCE- open"); 
      }
      if (fault & MAX31865_FAULT_OVUV) 
      {
        //Blynk.notify("Under/Over voltage"); 
      }
      fireplace.clearFault();
      delay(2000);                                             //Delay 2 seconds
    }
  }
  vTaskDelay(10);                                          //Avoid warning watchdog
}

void pushClapPosition (void *parameter)
{
  for(;;)
  {
    signalPot = analogRead(pot);                        //Read to potentiometer signal
    delay(100);
    clapPosition = map(signalPot, (calibration0), (calibration100), 0, 100);      //Map digital signal to angular value clap position ¿¿¿¿¿¿Calibrar potenciómetro una vez montado?????
    Serial.print("Posición Clapeta = ");Serial.println(clapPosition);
    delay(1000); 
  }
  vTaskDelay(10);                                          //Avoid warning watchdog  
}

void touchRead (void *parameter)
{
  for(;;)
  { 
    touchClose = touchRead(T8);
    delay(200);
    touchOpen = touchRead(T9);
    delay(200);
  }
  vTaskDelay(10);
}
  
void touchClap (void *parameter)
{
  for(;;)
  { 
    if (touchClose < 20 and touchClose != 0)
    {
      delay(500);
      if(touchClose < 20 and touchClose != 0)
      {
        delay(2500);
        if (touchClose < 20 and touchClose != 0 and touchOpen < 20 and touchOpen != 0)
        {
          automatic = !automatic;
        }
        else
        {
          closeClap = true;
          Serial.println("Pulsador cierre clapeta on");
        }
      }
    }
    if (touchOpen < 20 and touchOpen != 0)
    {
      delay(500);
      if (touchOpen < 20 and touchOpen != 0)
      {
        delay(2500);
        if (touchOpen < 20 and touchOpen != 0 and touchClose < 20 and touchClose != 0)
        {
          automatic = !automatic;  
        }
        else
        {
          openClap = true;
          Serial.println("Pulsador apertura clapeta on");
        }
        vTaskDelay(10);
      }
      vTaskDelay(10);
    }
    vTaskDelay(10);
  }
  vTaskDelay(10);
}
          
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{

  Blynk.run();

  Blynk.virtualWrite(V3, clapPosition);               //Send to blynk angular value clap position

  Blynk.virtualWrite(V0, fireplaceTemp);              //Send to blynk water temperature fireplace

  emergency();

  moveClap();

  autState();
  
  Serial.print("                        Pulsador Tactil cierre = "); Serial.println(touchClose);
  Serial.print("                        Pulsador Tactil apertura = "); Serial.println(touchOpen);
  Serial.print("Automatico/Manual = ");Serial.println(automatic);
}

BLYNK_WRITE(V1)
{
  closeClap = true;                       //Activation close clap
}

BLYNK_WRITE(V2)
{
  openClap = true;                        //Activation open clap
}

BLYNK_WRITE(V4)
{
  if (param.asInt() == 1)
  {
    automatic = !automatic;
  }
}

BLYNK_WRITE(V5)
{
  if (param.asInt() == 1)
 {
  digitalWrite(dischargeValve, LOW); 
  Blynk.notify ("DISCHARGE_VALVE_ON");
  Serial.println("DISCHARGE_VALVE_ON");
 }
 else
 {
    digitalWrite(dischargeValve, HIGH); 
    Blynk.notify ("DISCHARGE_VALVE_OFF");
    Serial.println("DISCHARGE_VALVE_OFF"); 
  }
}

BLYNK_READ(V8)
{
  Blynk.virtualWrite (V8, setpointClap);
}

void emergency()
{
  if (fireplaceTemp >= emergencyTempFireplace)
  {
    delay(3000);
    if (fireplaceTemp >= emergencyTempFireplace)
    {
      Blynk.notify("EMERGENCY_TEMPERATURE_FIREPLACE_ON");           //Notify app activate emergency temperature 
      Serial.println("EMERGENCY_TEMPERATURE_FIREPLACE_ON");
      digitalWrite(emergencyRelay, LOW);
      digitalWrite(dischargeValve, LOW);                           //Open electrovalve to system discharge
      Blynk.notify("DISCHARGE_VALVE_ON");
      Serial.println("DISCHARGE_VALVE_ON");
      emergencyLed.on();                               
      if (newSetpointClap != 0)                                        //Close Clap
      {
        newSetpointClap = 0;
        servoSetpoint();
      }
      while (fireplaceTemp >= (emergencyTempFireplace - emergencyHysteresis))
        {
          digitalWrite(buzzer, HIGH);               //Code buzzer emergency
          delay(1000);
          digitalWrite(buzzer, LOW);
          delay(500);
          digitalWrite(buzzer, HIGH);
          delay(1000);
          digitalWrite(buzzer, LOW);
          Blynk.virtualWrite(V3, clapPosition);               //Send to blynk angular value clap position
          Blynk.virtualWrite(V0, fireplaceTemp);              //Send to blynk water temperature fireplace
        }
      digitalWrite(emergencyRelay, HIGH);
      digitalWrite(dischargeValve, HIGH);                            //Close electrovalve to system discharge
      Blynk.notify ("DISCHARGE_VALVE_OFF");
      Serial.println("DISCHARGE_VALVE_OFF");
      Blynk.notify("EMERGENCY_TEMPERATURE_FIREPLACE_OFF");         //Notify app desactivate emergency temperature
      Serial.println("EMERGENCY_TEMPERATURE_FIREPLACE_OFF");
      emergencyLed.off();
   }
}

void moveClap()
{
  if (startClap)
  {
    newSetpointClap = map(signalPot, (calibration0), (calibration100), 0, 100);
    setpointClap = newSetpointClap;
    servoSetpoint();
    startClap = false;  
  }
  if (fireplaceTemp < emergencyTempFireplace)
  {
    if (automatic)
    { 
      closeClap = false;
      openClap = false;
      newSetpointClap = map (fireplaceTemp, 30, 60, 100, 30);
      if (newSetpointClap > 100)
      {
        newSetpointClap = 100;
      }
      if (newSetpointClap < 30)
      {
        newSetpointClap = 30;
      }  
      servoSetpoint();
      delay(2000);
    }
    else
    {
      if (closeClap)
      {
        if (newSetpointClap >= 3)
        {
          newSetpointClap = (newSetpointClap - spinPulse);
          servoSetpoint();   
        }
        else
        {
          digitalWrite(buzzer, HIGH);               //Sigle long beep to indicate endstop
          delay(2000);
          digitalWrite(buzzer, LOW);
        }
        closeClap = false;
      }
      if (openClap)
      {  
        if (newSetpointClap <= 97)
        {
          newSetpointClap = (newSetpointClap + spinPulse);
          servoSetpoint(); 
        }
        else
        {
          digitalWrite(buzzer, HIGH);               //Sigle long beep to indicate endstop
          delay(2000);
          digitalWrite(buzzer, LOW);
        }
        openClap = false;
      }
    }  
  }
}  
 

void servoSetpoint()
{
  if (newSetpointClap != setpointClap)
  {
    if (newSetpointClap > setpointClap)
    {
      while (newSetpointClap != setpointClap)
      {
        setpointClap = setpointClap + 1;
        playClap();
      }
    servoDetach();  
    }  
    else
    {
      while (newSetpointClap != setpointClap)
      { 
        setpointClap = setpointClap - 1;
        playClap();
      }
    servoDetach();  
    }
  }
}

void servoDetach()
{
  if(servoOff == false)
  {
    controlClap.detach();
    servoOff = true;
    Serial.println("                                                                                    PARADA SERVO");
  }
}


void playClap()
{
  if(servoOff)
  {
    controlClap.attach(23);                       //Define Pin signal to servo actuator
    servoOff = false;
    Serial.println("                                                                                       ARRANQUE SERVO");  
  }
  servoSetpointClap = map(setpointClap, 100, 0, 45, 135);
  controlClap.write (servoSetpointClap) ;
  Serial.print("                                                                                          Servo Setpoint = ");Serial.println(setpointClap);
  delay(speedClap);
  //offSetClap = (setpointClap - clapPosition);           //Vigilance to right clapper actuation !!!!DESACTIVATED!!!!
  if (offSetClap > difAlarmClap)
  { 
    if(a = true)
    {
      Serial.println("CLAPPER_ACTUATION_ERROR_ON");         //Notify app clapper funtioning
      Blynk.notify ("CLAPPER_ACTUATION_ERROR_ON");
      a = false;
    }
  }
  else
  {
    if(a = false)
    {
      Serial.println("CLAPPER_ACTUATION_ERROR_OFF");        //Notify app clapper funtioning
      Blynk.notify ("CLAPPER_ACTUATION_ERROR_OFF");
      a = true;
    }
  }
}       

void autState()
{
  if(automatic)
  {
    if(c)
    {
      automaticLed.on();
      digitalWrite(buzzer, HIGH);               
      delay(200);
      digitalWrite(buzzer, LOW);
      delay(100);
      digitalWrite(buzzer, HIGH);
      delay(200);
      digitalWrite(buzzer, LOW);
      c = false;
    }  
  }
  else
  {
    if(c == false)
    {
      automaticLed.off();
      digitalWrite(buzzer, HIGH);               
      delay(300);
      digitalWrite(buzzer, LOW);
      c = true;
    }
  }
}
