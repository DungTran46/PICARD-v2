  /********************************
 * *****OLED screen library******
 * ******************************/
#include <SPI.h>
#include <SSD_13XX.h>

#include "_fonts/unborn_small.c"
#include "_fonts/square_small.c"
#include "_fonts/Terminal_9.c"
#include "_fonts/mono_mid.c"

#include "_icons/wifi.c"

#include "_icons/spkOn.c" 
#include "_icons/spkOff.c" 

#include "_images/sw1.c"
#include "_images/wifi2.c"

/********************************
 * *******RTC library************
 * ******************************/
#include <Arduino.h>
#include <Wire.h>
#include "RTClib.h"
/********************************
 ***Fingerprint scanner library**
 * ******************************/
#include <FPS_GT511C3.h>

/********************************
 *********WIFI library***********
 * ******************************/
#include <WiFi.h>
#include <PubSubClient.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ArduinoJson.h>
 
/********************************
 * *****OLED screen define*******
 * ******************************/
#define __CS   21 
#define __DC   25//A1
#define __RST  26//A0
#define __MOSI 18//33
#define __SCLK 5//15

/********************************
 * *********RTC define***********
 * ******************************/
#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif

/********************************
 *******RTC initialization*******
 ********************************/
RTC_Millis rtc;
int nextFlag;
DateTime now;

/********************************
 ***OLED screen initialization***
 ********************************/
SSD_13XX tft = SSD_13XX(__CS, __DC, __RST,__MOSI, __SCLK, 2);

/************************************
 *fingerprint Scanner initialization*
 ************************************/
FPS_GT511C3 fps(115200, 2);

/************************************
 **********WIFI initialization*******
 ************************************/
WiFiClient espClient;
PubSubClient client(espClient);
int inval_flat=0;
const char* ssid = "UCInet Mobile Access";
const char* password = "CalPlug:)";
const char* mqttServer = "m12.cloudmqtt.com";
const int mqttPort = 12888;
const char* mqttUser = "nlxnwobb";
const char* mqttPassword = "I_acCzjITPYj";
char device_id[50]="48:174:164:55:85:12";
int verify=0,verified_flat=0;
int timeout=20, isHold=0,message_flat=0;
int pushflat=0;
char mqttMessage[500];
char *mqtt_topic;
int ignor=0;

/************************************
 ************MOTOR INIT*************
 ************************************/
#define PWMA          14 //H-Bridge
#define AIN1          15 //H-Bridge
#define AIN2          32 //H-Bridge
#define FREQENCY      500 //for PWM configuration
#define RESOLUTION    8  //for PWM configuration
#define MOTOR_CHANNEL 0//for PWM configuration
#define ENCODER_PINC2  33
#define ENCODER_PINC1  27
#define MAXIMUM_DURATION 10
#define MAXIMUM_POWER    160
int max_duration=MAXIMUM_DURATION, max_power=MAXIMUM_POWER;
int motor_flat=0;
int temp1=0;
int temp2=0;
byte encoder0PinALast;
int duration=0;//the number of the pulses boolean Direction;//the rotation direction
int lastDuration=0;
bool Direction;
int power;
/************************************
 *************other init************
 ************************************/
#define SCANNER_BUTTON 12
#define CONTROL_BUTTON 4
int is_interrupt=0;
int numPills=40;
int timeNextPills=10; //in second
int nextSec=0,nextHour=0, nextMin=0;
int nextDoseTime = 0;
int nextDoseIn = 30;
int states=1;
int scanBut=0,controlBut=0,but_read=0;


/*********************************************
**************SET UP FUNCTION******************
**********************************************/
void setup() {
  Serial.begin(9600);// set baud rate
  
  /*********************************setup oled screen*******************************/
  tft.begin();
  
  /**********************************setup RTC*****************************/
  rtc.begin(DateTime(F(__DATE__), F(__TIME__)));
  now=rtc.now();
  nextDoseTime = now.unixtime()+nextDoseIn;

  /*********************************setup FingerPrint*******************************/
  fps.Open();
  fps.SetLED(false);
  Serial.print("Enroll count: ");
  Serial.println(fps.GetEnrollCount());

  /***********************************Setup Button*******************************/
  pinMode(SCANNER_BUTTON,INPUT);
  pinMode(CONTROL_BUTTON,INPUT);
  attachInterrupt(digitalPinToInterrupt(SCANNER_BUTTON),scannerState,RISING);
  attachInterrupt(digitalPinToInterrupt(CONTROL_BUTTON),controlState,RISING);
  pinMode(35,INPUT);
  /********Motor Setup***********/
  ledcAttachPin(PWMA,MOTOR_CHANNEL);
  ledcSetup(MOTOR_CHANNEL, FREQENCY, RESOLUTION);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(ENCODER_PINC2, INPUT_PULLUP);
  Direction = true;//default -> Forward
  pinMode(ENCODER_PINC1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINC1), wheelSpeed, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_PINC2), wheelSpeed2, CHANGE);
   starUpScreen();
   
  /******8*****************************WIFI SETUP*******************************/
  Serial.print("Connect to network: ");
  Serial.println(ssid);
  //WiFi.begin("RT-AC1200_C8_2G","sonandy2004");
  WiFi.begin(ssid);// connect to wifi
  int connect_flat=1;
  while (WiFi.status() != WL_CONNECTED){
    if(timeout<0)
    {
      connect_flat=0;
      break;
    }
    Serial.println("Connecting to Wifi.....");
    timeout--;
    delay(500);
  }
  timeout=20;
  if(connect_flat==1)
    Serial.println("Connected to the WiFi network");
  else
    Serial.println("unable to connect to Wifi");
  
  /*************************************MQTT SETUP************************************/
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  while (!client.connected()) 
  {
    if(timeout<0)
    {
      Serial.println("Unable to connect to wifi");
      break;
    }

    Serial.println("Connecting to MQTT...");
    
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) 
    {
      Serial.println("connected");
    } 
    else 
    {
      Serial.print("failed with state ");
      Serial.println(client.state());
    }
    timeout--;
    delay(500);
  }
  timeout=20;
  client.subscribe("esp/phoneToEsp");
  delay(2000);
  
  /******************************************START UP**************************/
  tft.fillRect(0,0,96,64,BLACK);
  tft.fillRect(0,0,96,64,BLACK);
  displayNumPills();
}

/*********************************************
**************MAIN LOOP STARTS*****************
***********************************************/
void loop() {
  
  client.loop();//start up MQTT LOOP
  
  /*************************CONTROL BUTTON PRESS**************************/
  if(controlBut==1)
  {
    Serial.print("states before displayMenu= ");
    Serial.println(states);
    controlBut=0;
    setNextState();
    Serial.print("states after displayMenu= ");
    Serial.println(states);
    if (states==1)
    {
      displayNumPills();
    }
    else if (states==2)
    {
      int tempHour, tempMin, tempSec, tempTime;
      DateTime nextDose(nextDoseTime);
      while(scanBut==0 && controlBut==0)
      { 
        tft.clearScreen();
        tft.clearScreen();
        tft.setCursor(0,0);
        tft.setTextScale(2);
        tft.setTextColor(WHITE);
        tft.println("Next pill in:");
        tft.setCursor(10,20);
        now=rtc.now();
        //clockDisplay(now.hour(),now.minute(),now.second());
        tempHour=nextHour-now.hour();
        tempMin=nextMin-now.minute();
        tempSec=nextSec-now.second();
        tempTime = nextDose.unixtime() - now.unixtime();
        if(isNeg(tempTime))
          tft.print(0, DEC);
        else
          tft.print(nextDose.hour()-now.hour(), DEC);
        tft.print(':');
        if(isNeg(tempTime))
          tft.print(0, DEC);
        else
          tft.print(nextDose.minute()-now.minute(), DEC);
        tft.print(':');
        if(isNeg(tempTime))
          tft.print(0, DEC);
        else
          tft.print(nextDose.second()-now.second(),DEC);
        delay(500);
        Serial.print("controlBut= ");Serial.println(controlBut);
        Serial.print("scanBut= ");Serial.println(scanBut);
      }
      Serial.println("Exit time to next pills");
    }
    else if(states==3)
    {
      displayBatLevel();
    }
    else if(states==4)
    {
      displayWiFiStatus();
    }
    delay(10);
  }

  /*************************************SCANNER BUTTON PUSH/HOLD**************************/
  if(scanBut==1)
  {
    delay(500);
    Serial.println("Scanner state");
    int isEnroll=0;
    but_read=digitalRead(SCANNER_BUTTON);
    Serial.print("but_read= ");
    Serial.println(but_read);
    if(but_read==HIGH)
    {
      Serial.println("button is high in hold");
      delay(3000);
      but_read=digitalRead(SCANNER_BUTTON);
      if(but_read==HIGH)
      {
        isEnroll=1;
      }
    }
    if(isEnroll==1){
      /*char key[50]="password";
      char value[50]="0";
      char topic[50]="esp/espToPhone";
      send_message(key,value,topic);
      Serial.println("wait for the verification");
      printTextMessage("Verify through phone");
      verified_flat=1;*/
      fps.SetLED(true);
      enroll();
      
    }
    else
    {
      verify_finger();
    }
    scanBut=0; 
    //displayMenu();
    displayNumPills();
  }

  
  /*******************************ENROLL VERIFICATION(NOT READY) **************************/
  if(message_flat==1)
  {
    decode_message(mqtt_topic,mqttMessage);
    message_flat=0;
    timeout=20;
  }
  if(verified_flat==1)
  {
    Serial.print("verified_flat= ");Serial.println(verified_flat);
    if(timeout<0)
    {
      Serial.println("timeout, cannot verify");
      timeout=20;
      verified_flat=0;
      fps.SetLED(false);
    }
    if(verify==1)
    {
      Serial.println("verified id");
      enroll();
      timeout=10;
      verified_flat=0;
      verify=0;
      //displayMenu();
      displayNumPills();
    }
    if(verify==0 && timeout>=0)
    {
      printTextMessage("verifying.....");
      Serial.println("verifying.....");
    }
    timeout--;
    delay(500);
  }


  /******************************THE END OF MAIN LOOP**************************/
}



void controlState()
{
  controlBut=1;
  scanBut=0;

}
void setNextState()
{
  if(states==1)
    states=2;
  else if(states==2)
    states=3;
  else if(states==3)
    states=4;
  else if(states==4)
    states=1;
  else
    states=1;
}
void scannerState()
{
  if(verified_flat!=1)
  {
    scanBut=1;
    controlBut=0;
  }
}

bool isNeg(int input)
{
  if(input<0)return true;
  else return false;
}

/**********************************************************************************
**************************************ENROLL FUNCTION*******************************
***********************************************************************************/
void enroll()
{
  fps.SetLED(true);
  //ignore the interrupt because we don't want interrupt signal affect the enroll process
  detachInterrupt(SCANNER_BUTTON);
  detachInterrupt(CONTROL_BUTTON);
  // find open enroll id
  int enrollid = 0;
  bool usedid = true;
  while (usedid == true)
  {
    usedid = fps.CheckEnrolled(enrollid);
    if (usedid==true) enrollid++;
  }
  fps.EnrollStart(enrollid);
  displayEnrollMesssage(1); //display (ready to enroll)
  delay(2000);
  displayEnrollMesssage(2); //display (press finger to enroll)
  while(fps.IsPressFinger() == false) delay(1000);
  bool bret = fps.CaptureFinger(true);
  int iret = 0;
  if (bret != false)
  {
    displayEnrollMesssage(3); //display (remove finger)
    delay(2000);
    fps.Enroll1(); 
    displayEnrollMesssage(7); //display (press same finger again)
    while(fps.IsPressFinger() == false) delay(100);
    bret = fps.CaptureFinger(true);
    if (bret != false)
    {
      displayEnrollMesssage(3); //display (remove finger)
      fps.Enroll2();
      while(fps.IsPressFinger() == true) 
        delay(100);
      displayEnrollMesssage(7); //display (press same finger again)
      while(fps.IsPressFinger() == false) 
        delay(100);
      bret = fps.CaptureFinger(true);
      if (bret != false)
      {
        displayEnrollMesssage(3);//display (remove finger)
        iret = fps.Enroll3();
        if (iret == 0)
        {
          displayEnrollMesssage(4); //display (enroll successful)
        }
        else if(iret>0)
        {
          displayEnrollMesssage(5); //display (already enroll this finger)
        }
        else
        {
         displayEnrollMesssage(6); //display(fail to enroll)
         
        }
      }
      else 
      {
        displayEnrollMesssage(8); //display(FAIL TO ENROLL)
        delay(2000);
        displayEnrollMesssage(9); //display(TRY AGAIN)
        delay(1000);
      }
    }
    else 
    {
      displayEnrollMesssage(8); //display(FAIL TO ENROLL)
      delay(2000);
      displayEnrollMesssage(9); //display(TRY AGAIN)
      delay(1000);
    }
  }
  else 
  {
    displayEnrollMesssage(8); //display(FAIL TO ENROLL)
    delay(2000);
    displayEnrollMesssage(9); //display(TRY AGAIN)
    delay(1000);
  }

  attachInterrupt(digitalPinToInterrupt(SCANNER_BUTTON),scannerState,RISING);
  attachInterrupt(digitalPinToInterrupt(CONTROL_BUTTON),controlState,RISING);
  fps.SetLED(false);  //turn off the LED in the finger print scanner

  
  
}
/******************************************************************************
*************************************VERIFY FINGER FUNCTION********************
*******************************************************************************/
void verify_finger()
{
  fps.SetLED(true);
  // Identify fingerprint test
  int timeout_temp=0;
  int flat=0;
  diplayVerifyMessage(1);
  delay(2000);
  while(timeout_temp<=20)
  {
    if (fps.IsPressFinger())
    {
      Serial.println("read finger");
      flat=1;
      break;
    }
    timeout_temp++;
    delay(1000);
  }
  if (flat==1)
  {
    fps.CaptureFinger(false);
    int id = fps.Identify1_N();
    if (id <200)
    {//if the fingerprint matches, provide the matching template ID
      printTextMessage("Verified ID:");
      delay(2000);
      //Serial.println(id);
      dispense();
    }
    else
    {//if unable to recognize
      diplayVerifyMessage(2);
      delay(3000);
    }
  }
  else{
    printTextMessage("TIME OUT");
    delay(3000);
  }
  fps.SetLED(false);
}
void dispense()
{
  int flat=0;
  now=rtc.now();

  
  Serial.println("now.unixtime()");
  Serial.println(now.unixtime());
  if(now.unixtime()>=nextDoseTime)
  {
    Serial.println(nextDoseTime);
    Serial.println(now.unixtime());    

        flat=1;
    
  }
  Serial.println(flat);
  if(flat==1)
  {
    Serial.print("Next hour "); Serial.println(nextHour);
    Serial.print("now.hour ");  Serial.println(now.hour());
    Serial.print("next min ");  Serial.println(nextMin);
    Serial.print("now.min ");   Serial.println(now.minute());
    Serial.print("nextsec ");   Serial.println(nextSec);
    Serial.print("now second"); Serial.println(now.second());
    
    Serial.println("DISPENSING");
    printTextMessage("Dispensing");
    nextSec=now.second()+timeNextPills;
    nextDoseTime = now.unixtime()+ nextDoseIn;
    Serial.print("Next hour "); Serial.println(nextHour);
    Serial.print("now.hour ");  Serial.println(now.hour());
    Serial.print("next min ");  Serial.println(nextMin);
    Serial.print("now.min ");   Serial.println(now.minute());
    Serial.print("nextsec ");   Serial.println(nextSec);
    Serial.print("now second "); Serial.println(now.second());
    char temp[10];
    numPills--;
    sprintf(temp,"%d",numPills);
    char key[50]="remaining_pill";
    char topic[50]="esp/espToPhone";
    send_message(key,temp,topic);
    motor();
    delay(3000);
    flat=0;
    tft.scroll(false);
  }
  else
  {
    Serial.print("Next hour "); Serial.println(nextHour);
    Serial.print("now.hour ");  Serial.println(now.hour());
    Serial.print("next min ");  Serial.println(nextMin);
    Serial.print("now.min ");   Serial.println(now.minute());
    Serial.print("nextsec ");   Serial.println(nextSec);
    Serial.print("now second"); Serial.println(now.second());
    diplayVerifyMessage(3);
    delay(3000);
  }
  
}

/*********************************************************************************
*****************************************MOTOR FUNCTION****************************
***********************************************************************************/
void wheelSpeed() {
  motor_flat=1;
  temp1=temp2;
}
void motor() {
  //add pwr=35, duration=40 for #1 helix
  max_duration=MAXIMUM_DURATION;
  max_power=MAXIMUM_POWER;
  power=95;
  int adjust_power=55;
  int counter=0;
  if(numPills<=2)
  {
    max_duration=8;
    max_power=80;
  }
  int adjust_flat=0;
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  Serial.println("in motor function");
  Serial.print("power= ");
  Serial.println(power);
  Serial.print("duration= ");
  Serial.println(duration);
  Serial.print("flat ");
  Serial.println(motor_flat);
  //ledcWrite(MOTOR_CHANNEL, adjust_power);
  //delay(40);
  duration = 0;
  lastDuration=0;
  /*
  while(adjust_flat==0)
  {
    tft.defineScrollArea(1,0,64,0,0);
    tft.scroll(true);
    Serial.println("adjust");
    Serial.print("temp1= ");
    Serial.println(temp1);
    Serial.print("temp2= ");
    Serial.println(temp2);
    counter++;
    ledcWrite(MOTOR_CHANNEL, adjust_power);
    delay(40);
    if(temp1==temp2)
    {
      temp1++;
      if(numPills==1 && counter>=40)
      {
         adjust_flat=0;
         break;
      }
    }
    else
    {
      adjust_flat=1;
      temp1=0;
      temp2=0;
      ledcWrite(MOTOR_CHANNEL, 0);
      break;
    }
  }
  delay(500);
  ledcWrite(MOTOR_CHANNEL, power);
  */
  while ((duration < max_duration) && (power < max_power)) 
  {
    tft.defineScrollArea(1,0,64,0,0);
    tft.scroll(true);
  
    ledcWrite(MOTOR_CHANNEL, power);
    Serial.print("Power:"); 
    Serial.print(power); 
    Serial.print("  duration:");
    Serial.print(duration); 
    Serial.print("  last:"); 
    Serial.println(lastDuration);
    Serial.print(" motor_flat:"); 
    Serial.println(motor_flat);
    /*if(duration==lastDuration)
    {
      power++;
    }*/
    
    lastDuration = duration;
    delay(10);
    if(motor_flat==1)
    {
      int Lstate = digitalRead(ENCODER_PINC2);
      if ((encoder0PinALast == LOW) && Lstate == HIGH) 
      {
        int val = digitalRead(ENCODER_PINC1);
        if (val == LOW && Direction)
          Direction = false; //Reverse
        else if (val == HIGH && !Direction)
          Direction = true; //Forward
      }
      encoder0PinALast = Lstate;
      lastDuration = duration;
      duration++;
      motor_flat=0;
    }
    /*else
    {
      power++;
    }*/
  }
  backward();
  delay(100);
  power=0;
  duration=0;
  lastDuration=0;
  motor_flat=0;
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  delay(40);
}
void backward() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(MOTOR_CHANNEL, 60);
}


/*******************************************************************************
*************************************MQTTT CALL BACK FUNCTION*******************
********************************************************************************/
void callback(char* topic, byte* payload, unsigned int length) {

  Serial.println("In call back");
  if(ignor==0)
  {
    Serial.println("ignore the first message");
    ignor=1;
    return;
  }
  else
  {
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);
  
    Serial.print("Message:");
    for (int i = 1; i <length-1; i++) {
      Serial.println((char)payload[i]);
      mqttMessage[i-1]=(char)payload[i];
    }
    Serial.println();
    
    //copy from payload to tempoary message
    for (int i = 0; i < length-2; i++) {
      Serial.print(mqttMessage[i]);
    }
    message_flat=1;
    mqtt_topic=topic;
    Serial.println();
  }
}

/***************************************************************************
************8**************DECODE MESSAGE FUNCTIONS*************************
****************************************************************************/
void decode_message(char *topic, char *message)
{
  StaticJsonBuffer<300> JSONBuffer;
  Serial.print("topic: ");
  Serial.println(topic);
  
  if(strcmp(topic,"esp/phoneToEsp")==0)
  {
    char *value;
    char key1[]="password";
    char key2[]="request";
    char key3[]="set_pill";
    Serial.println("extract message");
    //parse the message
    JsonObject& parsed=JSONBuffer.parseObject(message);
    //Check for errors in parsing
    delay(100);
    if (!parsed.success()) 
    {   
      Serial.println("Parsing failed");
      message_flat=0;
      verified_flat=0;
      timeout=20;
      delay(1000);
    }
    else
    {
      Serial.println("Parsed success");
      bool hashKey1=parsed.containsKey(key1);
      bool hashKey2=parsed.containsKey(key2);
      bool hashKey3=parsed.containsKey(key3);
      Serial.print("hashKey1= ");Serial.println(hashKey1);
      Serial.print("hashKey2=");Serial.println(hashKey2);
      if(hashKey1==true && verified_flat==1)
      {
        const char *id=parsed[key1];
        value=strdup(id);
        Serial.print("key= ");
        Serial.print(key1);
        Serial.println("Done extract");
        Serial.println("value");
        Serial.println(value);
        Serial.println(verify);
        if(strcmp(value,"1")==0)
          verify=1;
        Serial.println(verify);
      }
      else if(hashKey2==true)
      {
        const char *id=parsed[key1];
        Serial.println("send request");
        char temp[10];
        sprintf(temp,"%d",numPills);
        char temp_topic[]="esp/espToPhone";
        char temp_key[]="remaining_pill";
        send_message(temp_key,temp,temp_topic);
      }
      else if(hashKey3==true)
      {
        const char *id=parsed[key3];
        Serial.print("new numPills= "); Serial.println(id);
        int temp=atoi(id);
        Serial.print("new numPills= "); Serial.println(temp);
        numPills=temp;
        Serial.println("display number of pills"); 
        Serial.print("states= ");
        Serial.println(states);
        displayNumPills();
      }
    }
  }
  else if(strcmp(topic,"esp/serverToEsp")==0)
  {
    Serial.println("get message from the server");
  }
}

/****************************************************************************
***********************************SEND MESSAGE FUNCTION**********************
******************************************************************************/
void send_message(char *key, char *value, char*topic)
{
  char JSONmessageBuffer[100];
  StaticJsonBuffer<300> JSONBuffer;
  JsonObject& JSONencoder = JSONBuffer.createObject();
  JSONencoder["registration_id"]=device_id;
  JSONencoder[key]=value;
  JSONencoder.printTo(JSONmessageBuffer,sizeof(JSONmessageBuffer));
  Serial.print("Sending message to MQTT topic: ");
  Serial.println(topic);
  Serial.println(JSONmessageBuffer);
  client.publish(topic,JSONmessageBuffer);
}

/*****************************************************************************
**********************************DISPLAY TEXT FUNCTIONS**********************
******************************************************************************/
void printTextMessage(String m)
{
  tft.clearScreen();
  tft.clearScreen();
  tft.setTextScale(2);
  tft.setTextColor(BLUE);
  tft.print(m);
}
void printNumMessage(int m)
{
  tft.clearScreen();
  tft.clearScreen();
  tft.setTextScale(2);
  tft.setTextColor(BLUE);
  tft.println(m);
}
void displayNumPills()
{
    Serial.println("display number of pills"); 
    Serial.print("states= ");
    Serial.println(states);
    tft.fillRect(0,0,96,64,BLACK);
    tft.fillRect(0,0,96,64,BLACK);
    tft.setCursor(0,0);
    tft.setTextScale(2);
    tft.setTextColor(WHITE);
    tft.setCursor(5,0);
    tft.println("Remaining");
    tft.setCursor(30,20);
    tft.println("pills:");
    tft.setCursor(35,40);
    tft.println(numPills);
}
void displayBatLevel()
{
    float battery=(((analogRead(35)/4095.0)*2.0*3.3*1100)/(10*10*10))/4.2*100.0;
    int  temp=battery;
    Serial.print("battery level= ");
    Serial.println(battery);
    String s_battery= String(temp)+"%";
    tft.clearScreen();
    tft.clearScreen();
    tft.setCursor(10,0);
    tft.setTextScale(2);
    tft.setTextColor(WHITE);
    tft.println("Battery:");
    tft.setCursor(20,20);
    tft.println(s_battery);
}
void displayWiFiStatus()
{
    tft.clearScreen();
    tft.clearScreen();
    tft.setCursor(0,0);
    tft.setTextScale(2);
    tft.setTextColor(WHITE);
    tft.setCursor(20,0);
    tft.println("WiFi:");
    if(WiFi.status() != WL_CONNECTED)
      tft.println("Not connected");
    else
      tft.println("Connected");
}
/* h is hour
 * m is minute
 * s is second
 * the function display the time on the screen
 */
void clockDisplay(int h, int m, int s)
{
  tft.clearScreen();
  tft.setTextScale(2);
  tft.setTextColor(BLUE);
  tft.print(h);
  tft.print(':');
  tft.print(m);
  tft.print(':');
  tft.print(s);
}
void starUpScreen()
{
  tft.fillRect(0,0,96,64,BLACK);
  tft.fillRect(0,0,96,64,BLACK);
  tft.setCursor(20,10);
  tft.setTextScale(2);
  tft.setTextColor(RED);
  tft.print("P");
  delay(500);
  tft.setTextColor(GREEN);
  tft.print("I");
  delay(500);
  tft.setTextColor(BLUE);
  tft.print("C");
  delay(500);
  tft.setTextColor(YELLOW);
  tft.print("A");
  delay(500);
  tft.setTextColor(WHITE);
  tft.print("R");
  delay(500);
  tft.setTextColor(PURPLE);
  tft.print("D");
}

void diplayVerifyMessage(int message_code)
{
  if(message_code==1)
  {
    tft.fillRect(0,0,96,64,BLACK);
    tft.fillRect(0,0,96,64,BLACK);;
    tft.setTextScale(2);
    tft.setTextColor(BLUE);
    tft.setCursor(0,0);
    tft.println("READ");
    tft.println("FINGER");
  }
  else if(message_code==2)
  {
    tft.fillRect(0,0,96,64,BLACK);
    tft.fillRect(0,0,96,64,BLACK);;
    tft.setTextScale(2);
    tft.setTextColor(RED);
    tft.setCursor(0,0);
    tft.println("FINGER");
    tft.println("NOT");
    tft.println("FOUND");
  }
  else if(message_code==3)
  {
    tft.fillRect(0,0,96,64,BLACK);
    tft.fillRect(0,0,96,64,BLACK);;
    tft.setTextScale(2);
    tft.setTextColor(RED);
    tft.setCursor(0,0);
    tft.println("CANNOT");
    tft.println("DISPENSE");
  }
}


void displayEnrollMesssage(int message_code)
{
  if(message_code==1) //"READY TO ENROLL"
  {
    tft.fillRect(0,0,96,64,BLACK);
    tft.fillRect(0,0,96,64,BLACK);;
    tft.setTextScale(2);
    tft.setTextColor(BLUE);
    tft.setCursor(0,0);
    tft.println("READY ");
    tft.println("TO");
    tft.println("ENROLL");
  }
  else if(message_code==2) //"PRESS FINGER TO ENROLL
  {
    tft.fillRect(0,0,96,64,BLACK);
    tft.fillRect(0,0,96,64,BLACK);;
    tft.setTextScale(2);
    tft.setTextColor(BLUE);
    tft.setCursor(0,0);
    tft.println("PRESS ");
    tft.print("FINGER ");
    tft.print("TO ");
    tft.print("ENROLL");
  }
  else if(message_code==3)  //REMOVE FINGER
  {
    tft.fillRect(0,0,96,64,BLACK);
    tft.fillRect(0,0,96,64,BLACK);;
    tft.setTextScale(2);
    tft.setTextColor(BLUE);
    tft.setCursor(0,0);
    tft.println("REMOVE ");
    tft.println("FINGER ");
  }
  else if(message_code==4)  //ENROLL SUCCESSFUL
  {
    tft.fillRect(0,0,96,64,BLACK);
    tft.fillRect(0,0,96,64,BLACK);;
    tft.setTextScale(2);
    tft.setTextColor(BLUE);
    tft.setCursor(0,0);
    tft.println("Enroll ");
    tft.println("Successful");
  }
  else if(message_code==5)  // ALREADY ENROLL THIS FINGER
  {
    tft.fillRect(0,0,96,64,BLACK);
    tft.fillRect(0,0,96,64,BLACK);;
    tft.setTextScale(2);
    tft.setTextColor(RED);
    tft.setCursor(0,0);
    tft.println("ALREADY");
    tft.println("ENROLL");
    tft.println("THIS");
    tft.println("FINGER");
  }
  else if(message_code==6)  //FAIL TO ENROLL
  {
    tft.fillRect(0,0,96,64,BLACK);
    tft.fillRect(0,0,96,64,BLACK);;
    tft.setTextScale(2);
    tft.setTextColor(RED);
    tft.setCursor(0,0);
    tft.println("FAIL");
    tft.println("TO");
    tft.println("ENROLL");
  }
  else if(message_code==7)  //PRESS SAME FINGER AGAIN
  {
    tft.fillRect(0,0,96,64,BLACK);
    tft.fillRect(0,0,96,64,BLACK);;
    tft.setTextScale(2);
    tft.setTextColor(BLUE);
    tft.setCursor(0,0);
    tft.println("PRESS");
    tft.println("SAME");
    tft.println("FINGER"); 
    tft.println("AGAIN");    
  }
  else if(message_code==8) //FAIL TO CAPTURE
  {
    tft.fillRect(0,0,96,64,BLACK);
    tft.fillRect(0,0,96,64,BLACK);;
    tft.setTextScale(2);
    tft.setTextColor(RED);
    tft.setCursor(0,0);
    tft.println("FAIL");
    tft.println("TO");
    tft.println("CAPTURE"); 
  }
  else if(message_code==9) //TRY AGAIN
  {
    tft.fillRect(0,0,96,64,BLACK);
    tft.fillRect(0,0,96,64,BLACK);;
    tft.setTextScale(2);
    tft.setTextColor(BLUE);
    tft.setCursor(0,0);
    tft.println("TRY");
    tft.println("AGAIN");
  }
}