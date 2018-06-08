#define DEBUG 1
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
*********DEEP SLEEP library***********
* ******************************/
#include "esp_deep_sleep.h"

/********************************
 * *********RTC define***********
 * ******************************/
#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif
RTC_Millis rtc;
DateTime now;

#define STR_LENGTH  50
/********************************
 ***OLED screen initialization***
 ********************************/
#define __CS   21 
#define __DC   25//A1
#define __RST  26//A0
#define __MOSI 18//33
#define __SCLK 5//15
SSD_13XX tft = SSD_13XX(__CS, __DC, __RST,__MOSI, __SCLK, 2);

/************************************
 *fingerprint Scanner initialization*
 ************************************/
FPS_GT511C3 fps(2); // chose UART 2

/************************************
 **********WIFI initialization*******
 ************************************/
WiFiClient espClient;
PubSubClient client(espClient);
int inval_flat=0;
//const char* ssid = "UCInet Mobile Access";
char ssid[STR_LENGTH] = "NETGEAR44";
char passwords[STR_LENGTH] = "xuatcanh01";
const char* mqttServer = "m12.cloudmqtt.com";
const int mqttPort = 12888;
const char* mqttUser = "nlxnwobb";
const char* mqttPassword = "I_acCzjITPYj";
char device_id[STR_LENGTH]="48:174:164:55:85:12";
int verify=          0,
    verified_flat=   0,
    timeout=         20, 
    message_flat=    0,
    pushflat=        0,
    ignor=           0;
char mqttMessage[500],
    *mqtt_topic;

/************************************
 ************MOTOR INIT*************
 ************************************/
#define PWMA              14 //H-Bridge
#define AIN1              15 //H-Bridge
#define AIN2              32 //H-Bridge
#define FREQENCY          500 //for PWM configuration
#define RESOLUTION        8  //for PWM configuration
#define MOTOR_CHANNEL     0//for PWM configuration
#define ENCODER_PINC2     33
#define ENCODER_PINC1     27
#define MAXIMUM_DURATION  50
#define MAXIMUM_POWER     200
int max_duration=         MAXIMUM_DURATION, 
    max_power=            MAXIMUM_POWER,
    motor_flat=           0,
    temp1=                0,
    temp2=                0,
    duration=             0,//the number of the pulses boolean Direction;//the rotation direction
    power;

/************************************
 *************other init************
 ************************************/
#define SCANNER_BUTTON  12
#define CONTROL_BUTTON  4
#define BATTERY_VOLTAGE 35
#define MAX_NUMBER_OF_PILLS 30
#define SLEEP_TIME 30 //amount of time before sleep
int numPills=       40, //number of pills
    nextDoseTime=   0,
    nextDoseIn=     30, //time to next pills in seconds
    states=         1,  //Menu ID
    scanBut=        0,  //scanner button flat
    controlBut=     0,  //control button flat
    but_read=       0,  // button press flat
    sleep_counter=  0,
    set_state=      0;
RTC_DATA_ATTR int bootCount = 0;      //number of boots and reboots
RTC_DATA_ATTR int numPills_sleep =0; //memorize number of pills during sleep
RTC_DATA_ATTR char ssid_sleep[STR_LENGTH];      //number of boots and reboots
RTC_DATA_ATTR char passwords_sleep[STR_LENGTH]; //memorize number of pills during sleep

/*********************************************
**************SET UP FUNCTION******************
**********************************************/
void setup() {
  Serial.begin(9600);// set baud rate
  if(bootCount>0)
  {
    numPills=numPills_sleep;
    strcpy(ssid,ssid_sleep);
    strcpy(passwords,passwords_sleep);
  }
  esp_deep_sleep_enable_ext0_wakeup(GPIO_NUM_12,HIGH);
  
  /*********************************setup oled screen*******************************/
  tft.begin();
  
  /**********************************setup RTC*****************************/
  rtc.begin(DateTime(F(__DATE__), F(__TIME__)));
  now=rtc.now();
  nextDoseTime = now.unixtime()+nextDoseIn;

  /*********************************setup FingerPrint*******************************/
  fps.Open();
  fps.SetLED(false);
  //uncomment if need to delete all finger
  //fps.DeleteAll();

  /***********************************Setup Button*******************************/
  pinMode(SCANNER_BUTTON,INPUT);  //Setup SCANNER_BUTTON pin as input
  pinMode(CONTROL_BUTTON,INPUT);  //Setup CONTROL_BUTTON pin as input
  pinMode(BATTERY_VOLTAGE,INPUT); //setup BATTERY_VOLTAGE pin as input
  attachInterrupt(digitalPinToInterrupt(SCANNER_BUTTON),scannerState,RISING); // attach interrupt signal to SCANNER_BUTTON pin
  attachInterrupt(digitalPinToInterrupt(CONTROL_BUTTON),controlState,RISING); // attach interrupt signal to CONTROL_BUTTON pin
  
  /********Motor Setup***********/
  ledcAttachPin(PWMA,MOTOR_CHANNEL);                //attach pin to PWM signal
  ledcSetup(MOTOR_CHANNEL, FREQENCY, RESOLUTION);   //Setup PWM signal
  pinMode(AIN1, OUTPUT);                            //Setup AN1 pin as output to the MOTOR controller
  pinMode(AIN2, OUTPUT);                            //Setup AN2 pin as output to the MOTOR controller
  pinMode(ENCODER_PINC2, INPUT_PULLUP);             //Setup ENCODER_PINC2 pin as inout from the MOTOR 
  pinMode(ENCODER_PINC1, INPUT_PULLUP);             //Setup ENCODER_PINC1 pin as inout from the MOTOR 
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINC1), wheelSpeed, CHANGE); //attatch ENCODER_PIN to interrupt signal
  starUpScreen(); //display startup screen
  
  /***********************************WIFI SETUP*******************************/
  Serial.print("Connect to network: ");
  Serial.println(ssid);
  //if wifi has password use this function: WiFi.begin(SSID,password);
  WiFi.begin(ssid,passwords);       // connect to wifi
  int connect_flat=1;     //falt is to know whether the WiFi is connected
  /*  check if the device successfully connect to WiFi. 
   *  Timeout period is 10 second. If the time expires, the device will ignore the connection
   */
  while (WiFi.status() != WL_CONNECTED){
    if(timeout<0)
    {
      connect_flat=0;
      break;
    }
    
//#ifdef DEBUG
      Serial.println("Connecting to Wifi.....");
//#endif

    timeout--;
    delay(500);
  }
  timeout=20;
  
#ifdef DEBUG
    if(connect_flat==1)
      Serial.println("Connected to the WiFi network");
    else
      Serial.println("unable to connect to Wifi");
#endif
  
  /*************************************MQTT SETUP************************************/
  client.setServer(mqttServer, mqttPort); //connect to server
  client.setCallback(callback); //set callback function, so whenever the device receives the message from MQTT broker, the callback function is called.
  //Timeout period is 10 second. If the time expires, the device will ignore the connection
  if(connect_flat==1)
  {
    while (!client.connected()) 
    {
      if(timeout<0)
      {
#ifdef DEBUG
        Serial.println("Unable to connect to wifi");
#endif
        break;
      }
#ifdef DEBUG
      Serial.println("Connecting to MQTT...");
#endif     
      if (client.connect("ESP32Client", mqttUser, mqttPassword )) 
      {
#ifdef DEBUG
        Serial.println("connected");
#endif
      } 
      else 
      {
#ifdef DEBUG        
        Serial.print("failed with state ");
        Serial.println(client.state());
#endif
      }
      timeout--;
      delay(500);
    }
    client.subscribe("esp/phoneToEsp");
  }
  timeout=20;
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
  delay(500);
 
  /*************************CONTROL BUTTON PRESS/HOLD**************************/
  if(controlBut==1)
  {
    controlBut=0;
    if(isHold(CONTROL_BUTTON))  //hold
    {
      set_state=1;
      //set number of pills.
      if(states==1)
      {
#ifdef DEBUG
        Serial.println("holding");
#endif
        fps.SetLED(true);
        while(!isHold(CONTROL_BUTTON))
        {
          displaySetPills();
          delay(500);
        }
      }
      set_state=0;
      displayNumPills();
      fps.SetLED(false);
      
    }
    else  //single press
    {
      setNextState();
      if (states==1)
      {
        displayNumPills();
      }
      else if (states==2)
      {
        int tempTime;
        int toHour, toMin, toSec;
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
          tempTime = nextDose.unixtime()-now.unixtime();
          
          if(isNeg(tempTime))
            tft.print("0:00:00");
          else
          {
            toHour = tempTime;
            if(toHour/3600 < 10)
            {
              tft.print("0");
              tft.print(toHour/3600);
            }
            else
              tft.print(toHour/3600);
            tft.print(":");
            toMin = tempTime%3600;
            if(toMin/60 < 10)
            {
              tft.print("0");
              tft.print(toMin/60);
            }
            else
              tft.print(toMin/60);
            tft.print(":");
            toSec = tempTime%60;
            if(toSec < 10){
              tft.print("0");
              tft.print(toSec);
              }
            else
              tft.print(toSec);
          }
          delay(500);
#ifdef DEBUG          
          Serial.print("controlBut= ");Serial.println(controlBut);
          Serial.print("scanBut= ");Serial.println(scanBut);
#endif
        }
#ifdef DEBUG
        Serial.println("Exit time to next pills");
#endif
      }
      else if(states==3)
      {
        displayBatLevel();
      }
      else if(states==4)
      {
        displayWiFiStatus();
      }      
    }
  }

  /*************************************SCANNER BUTTON PUSH/HOLD**************************/
  if(scanBut==1)
  {
#ifdef DEBUG
    Serial.println("Scanner state");
#endif
    if(isHold(SCANNER_BUTTON)){
      enroll();
    }
    else
    {
      verify_finger();
    }
    scanBut=0; 
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
#ifdef DEBUG
    Serial.print("verified_flat= ");Serial.println(verified_flat);
#endif
    if(timeout<0)
    {
#ifdef DEBUG
      Serial.println("timeout, cannot verify");
#endif
      timeout=20;
      verified_flat=0;
      fps.SetLED(false);
    }
    if(verify==1)
    {
#ifdef DEBUG
      Serial.println("verified id");
#endif
      enroll();
      timeout=10;
      verified_flat=0;
      verify=0;
      displayNumPills();
    }
    if(verify==0 && timeout>=0)
    {
      printTextMessage("verifying.....");
#ifdef DEBUG
      Serial.println("verifying.....");
#endif
    }
    timeout--;
    delay(500);
  }
   /*************************checking for sleep time**************************/
  if(sleep_counter>SLEEP_TIME*2) //*2 because the 1 loop is 0.5 second
  {
    numPills_sleep=numPills;
    strcpy(ssid_sleep,ssid);
    strcpy(passwords_sleep,passwords);
    //turn off the screen
    tft.fillRect(0,0,96,64,BLACK);
    tft.fillRect(0,0,96,64,BLACK);
    //Go to sleep now
    Serial.println("Going to sleep now");
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
    
  }
  sleep_counter++;
  
}
/******************************THE END OF MAIN LOOP**************************/




//Interrupted function for control button
void controlState()
{
  sleep_counter=0;
  if(!set_state)
  {
    controlBut=1;
    scanBut=0;
  }

}

//Interrupted function for fingerprint button
void scannerState()
{
  sleep_counter=0;
  if(!set_state)
  {
    if(verified_flat!=1)
    {
      scanBut=1;
      controlBut=0;
    }
  }
   else
   {
    if(states==1)
    {
      if(numPills<MAX_NUMBER_OF_PILLS)
         numPills++;
      else 
        numPills=0;
    }  
   }

  
}


bool isHold(int button_pin)
{
  int but_read;
  bool is_hold=false;
  delay(100);
  // checking for hold action
  but_read=digitalRead(button_pin);
  if(but_read==HIGH)
  {
    delay(3000);
    but_read=digitalRead(button_pin);
    if(but_read==HIGH)
      is_hold=true;
  }
  return is_hold;
}

void print_wakeup_reason() {
  esp_deep_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_deep_sleep_get_wakeup_cause();

  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("EXT0 Test");

  switch (wakeup_reason)
  {
    case 1  : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case 2  : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case 3  : Serial.println("Wakeup caused by timer"); break;
    case 4  : Serial.println("Wakeup caused by touchpad"); break;
    case 5  : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.println("Wakeup was not caused by deep sleep"); break;
  }
}
//Set next menu for the display
/*  states=1 display number of pills
 *  states=2 display remaining time
 *  states=3 display battery level
 *  states=4 display WiFi status
 */
void setNextState()
{
  if(controlBut==0)
  {
    if(states==1)
      states=2;     //timer menu
    else if(states==2)
      states=3;      //battery level menu
    else if(states==3)
      states=4;     //Wifi menu
    else if(states==4)  
      states=1;     //number of pills menu
    else
      states=1;   //number of pills menu
  }
}

//checking if input is negative
/*  @input: input of the function
 *  @return: return true if input is negative, else return false
 */
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
          delay(1000);
        }
        else if(iret>0)
        {
          displayEnrollMesssage(5); //display (already enroll this finger)
          delay(1000);
        }
        else
        {
         displayEnrollMesssage(6); //display(fail to enroll)
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
#ifdef DEBUG
  Serial.println("now.unixtime()");
  Serial.println(now.unixtime());
#endif

  if(now.unixtime()>=nextDoseTime)
  {
#ifdef DEBUG
    Serial.println(nextDoseTime);
    Serial.println(now.unixtime()); 
#endif   
    flat=1;
  }
#ifdef DEBUG
  Serial.println(flat);
#endif
  if(flat==1)
  {
#ifdef DEBUG
    Serial.println("DISPENSING");
#endif
    printTextMessage("Dispensing");
    nextDoseTime = now.unixtime()+ nextDoseIn;
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
  duration++;
}

void backward() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(MOTOR_CHANNEL, 80);
}

void motor() {
  int adjust_power=             80,
      counter=                  0,
      adjust_flat=              0;
      
  max_duration= MAXIMUM_DURATION;
  max_power=    MAXIMUM_POWER;
  power=        70;
  duration=     0;
  
  if(numPills<=2)//used when there is only 2 pills remain
  {
    max_duration=8;
    max_power=80;
  }
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  Serial.println("in motor function"); 
#ifdef DEBUG
  Serial.print("power= ");    Serial.println(power);
  Serial.print("duration= "); Serial.println(duration);
  Serial.print("flat ");      Serial.println(motor_flat);
#endif
  //start adjusting phase
  ledcWrite(MOTOR_CHANNEL, 200);
  delay(1000);
  while(adjust_flat==0)
  {
    tft.defineScrollArea(1,0,64,0,0);
    tft.scroll(true);
#ifdef DEBUG
    Serial.println("adjust phase");
    Serial.print("temp1= ");  Serial.println(temp1);
    Serial.print("temp2= ");  Serial.println(temp2);
#endif
    if(numPills==1 && counter>=40)
    {
       adjust_flat=0;
       break;
    }
    ledcWrite(MOTOR_CHANNEL, adjust_power);
    delay(40);
    if(temp1==temp2)
    {
      temp1++;
    }
    else
    {
      adjust_flat=1;
      temp1=0;
      temp2=0;
      duration=0;
      ledcWrite(MOTOR_CHANNEL, 0);
      break;
    }
  }
  //end adjusting period

  //transition from adjusting phase to dispensing phase
  delay(500);
  ledcWrite(MOTOR_CHANNEL, power);
  delay(10);
  
  //start dispensing phase
  while ((duration < max_duration)) 
  {
    //scrolling dispensing text
    tft.defineScrollArea(1,0,64,0,0); //config scrolling
    tft.scroll(true);                 //activate scrolling
    power++;
    ledcWrite(MOTOR_CHANNEL, power);
    delay(10);
    
#ifdef DEBUG
    Serial.print("Power: ");                    Serial.print(power); 
    Serial.print("  duration: ");               Serial.print(duration); 
    Serial.println();
#endif
  }
  
  backward();
  delay(4000);
  power=0;
  duration=0;
  motor_flat=0;
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  delay(40);
}

/*******************************************************************************
*************************************MQTTT CALL BACK FUNCTION*******************
********************************************************************************/
void callback(char* topic, byte* payload, unsigned int length) {

  Serial.println("In call back");
  if(ignor==0)
  {
#ifdef DEBUG
    Serial.println("ignore the first message");
#endif

    ignor=1;
    return;
  }
  else
  {
#ifdef DEBUG
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);
    Serial.print("Message:");
#endif
    //copy from payload to tempoary message
    for (int i = 1; i <length-1; i++) 
    {
#ifdef DEBUG
      Serial.println((char)payload[i]);
#endif
      mqttMessage[i-1]=(char)payload[i];
    }
    Serial.println();
    
#ifdef DEBUG
    for (int i = 0; i < length-2; i++) {
      Serial.print(mqttMessage[i]);
    }
    Serial.println();
#endif
    message_flat=1;
    mqtt_topic=topic;
  }
}

/***************************************************************************
************8**************DECODE MESSAGE FUNCTIONS*************************
****************************************************************************/
void decode_message(char *topic, char *message)
{
  StaticJsonBuffer<300> JSONBuffer;
#ifdef DEBUG
  Serial.print("topic: ");
  Serial.println(topic);
#endif
  
  if(strcmp(topic,"esp/phoneToEsp")==0)
  {
    char *value;
    char key1[]="password";
    char key2[]="request";
    char key3[]="set_pill";
    char key4[]="set_time";
#ifdef DEBUG
    Serial.println("extract message");
#endif
    //parse the message
    JsonObject& parsed=JSONBuffer.parseObject(message);
    //Check for errors in parsing
    delay(100);
    if (!parsed.success()) 
    {   
#ifdef DEBUG
      Serial.println("Parsing failed");
#endif
      message_flat=0;
      verified_flat=0;
      timeout=20;
      delay(1000);
    }
    else
    {
#ifdef DEBUG
      Serial.println("Parsed success");
#endif
      bool hashKey1=parsed.containsKey(key1); //looking for key in parsed messages
      bool hashKey2=parsed.containsKey(key2); //looking for key in parsed messages
      bool hashKey3=parsed.containsKey(key3); //looking for key in parsed messages
      bool hashKey4=parsed.containsKey(key4); //looking for key in parsed messages
#ifdef DEBUG
      Serial.print("hashKey1= ");Serial.println(hashKey1);
      Serial.print("hashKey2=");Serial.println(hashKey2);
      Serial.print("hashKey2=");Serial.println(hashKey3);
      Serial.print("hashKey2=");Serial.println(hashKey4);
#endif
      if(hashKey1==true && verified_flat==1)
      {
        const char *id=parsed[key1];
        value=strdup(id);
#ifdef DEBUG
        Serial.print("key= ");
        Serial.print(key1);
        Serial.println("Done extract");
        Serial.println("value");
        Serial.println(value);
        Serial.println(verify);
#endif
        if(strcmp(value,"1")==0)
          verify=1;
#ifdef DEBUG
        Serial.println(verify);
#endif
      }
      else if(hashKey2==true)
      {
        const char *id=parsed[key1];
#ifdef DEBUG
        Serial.println("send request");
#endif
        char temp[10];
        sprintf(temp,"%d",numPills);
        char temp_topic[]="esp/espToPhone";
        char temp_key[]="remaining_pill";
        send_message(temp_key,temp,temp_topic);
      }
      else if(hashKey3==true)
      {
        const char *id=parsed[key3];
#ifdef DEBUG
        Serial.print("new numPills= "); Serial.println(id);
#endif
        int temp=atoi(id);
#ifdef DEBUG        
        Serial.print("new numPills= "); Serial.println(temp);
#endif
        numPills=temp;
        
#ifdef DEBUG        
        Serial.println("display number of pills"); 
        Serial.print("states= ");
        Serial.println(states);
#endif        

        displayNumPills();
      }
      else if(hashKey4==true)
      {
        const char *id=parsed[key3];
        
#ifdef DEBUG
        Serial.print("new time= "); Serial.println(id);
#endif

        int temp=atoi(id); //convert string to integer

#ifdef DEBUG
        Serial.print("new time= "); Serial.println(temp);
#endif
        numPills=temp;
#ifdef DEBUG
        Serial.println("display number of pills"); 
        Serial.print("states= ");
        Serial.println(states);
#endif
        displayNumPills();
      }
    }
  }
  else if(strcmp(topic,"esp/serverToEsp")==0)
  {
#ifdef DEBUG
    Serial.println("get message from the server");
#endif
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
#ifdef DEBUG
  Serial.print("Sending message to MQTT topic: ");
  Serial.println(topic);
  Serial.println(JSONmessageBuffer);
#endif
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
    tft.fillRect(0,0,96,64,BLACK);
    tft.fillRect(0,0,96,64,BLACK);
    tft.setCursor(0,0);
    tft.setTextScale(2);
    tft.setTextColor(WHITE);
    tft.setCursor(5,0);
    tft.println("Remaining");
    tft.setCursor(30,20);
    tft.println("Pills:");
    tft.setCursor(35,40);
    tft.println(numPills);
}
void displaySetPills()
{
    tft.fillRect(0,0,96,64,BLACK);
    tft.fillRect(0,0,96,64,BLACK);
    tft.setCursor(0,0);
    tft.setTextScale(2);
    tft.setTextColor(BLUE);
    tft.setCursor(20,0);
    tft.println("Setting");
    tft.setCursor(30,20);
    tft.println("Pills:");
     tft.setTextColor(RED);
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
    tft.println("Finger");
    tft.println("Not");
    tft.println("Auth.");
  }
  else if(message_code==3)
  {
    tft.fillRect(0,0,96,64,BLACK);
    tft.fillRect(0,0,96,64,BLACK);;
    tft.setTextScale(2);
    tft.setTextColor(RED);
    tft.setCursor(0,0);
    tft.println("WRONG");
    tft.println("TIME");
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
