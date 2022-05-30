// RFID code adapted from MFRC522 library examples and documentation!
// Stepper motor library not controlling motor as expected, needed to controll wire by wire
#define ESP_DRD_USE_SPIFFS true
#include <ESP8266WiFi.h>            // Wifi Library
#include <WiFiManager.h>
// #include <WiFi.h>
#include <FS.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Ethernet.h>
#include <BlynkSimpleEsp8266.h>

#include "ESP8266TimerInterrupt.h"  // Timer libraries
#include "ESP8266_ISR_Timer.hpp"   

#include <SPI.h>                    // RFID libraries
#include <MFRC522.h>

#include <Stepper.h>                // Stepper library

//==========================

// Timer definitions (Prescalers)
#define USING_TIM_DIV1                false           // for shortest and most accurate timer
#define USING_TIM_DIV16               false           // for medium time and medium accurate timer
#define USING_TIM_DIV256              true            // for longest timer but least accurate. Default

// RFID definitions
#define RST_PIN 2
#define SS_PIN  15
// #define IRQ_PIN 9

// Stepper definitions
#define inOne 16
#define inTwo 5
#define inThr 4
#define inFou 0

// Motion Sensor definitions
#define moSens 10

// WIFI definitions
#define JSON_CONFIG_FILE "/test_config.json"

// Blynk definitions
#define BLYNK_TEMPLATE_ID           "TMPLmoqm3e2E"
#define BLYNK_DEVICE_NAME           "Quickstart Device"
#define BLYNK_AUTH_TOKEN            "uRi1Nb1g2Zk4JCakmvTZub8mUyJATaHh"
// #define BLYNK_PRINT Serial

//==========================


// Init ESP8266 only and only Timer 1
ESP8266Timer ITimer; 
// Timer interval (in microseconds)
bool setInterval(unsigned long interval, timer_callback callback);
// Timer interval (in microseconds)
bool attachInterruptInterval(unsigned long interval, timer_callback callback);

// Init RFID
MFRC522 mfrc522(SS_PIN, RST_PIN);
// RFID Interrupt helpers
// volatile bool bNewInt = false;
// byte regVal = 0x7F;
// void activateRec(MFRC522 mfrc522);
// void clearInt(MFRC522 mfrc522); 
// void readCard();
int write_byte_array(byte *buffer, byte bufferSize);

// Init WiFi
WiFiManager wm;
void saveConfigFile(); // From http://dronebotworkshop.com/wifimanager/
bool loadConfigFile();
void saveConfigCallback();
void configModeCallback(WiFiManager *myWiFiManager);

// Init Server
// WiFiServer server(80); // didnt work
// String prepareHtmlPage();
// BlynkTimer timer;

// // This function is called every time the device is connected to the Blynk.Cloud
// BLYNK_CONNECTED();

// // This function sends Arduino's uptime every second to Virtual Pin 2.
// void myTimerEvent();
// couldnt get Blynk to work for now


// Task Struct for Task Scheduler
typedef struct task{
  int state;
  unsigned long period;
  unsigned long elapsedTime;
  int (*TickFct) (int);
} task;

task taskArray[5];

//==========================

// Variables
// Scheduler variables
const unsigned char tasksNum = 3;
const unsigned long tasksPeriodGDC = 3000;
const unsigned long periodMos = 6000;
const unsigned long periodRFID = 3000;
const unsigned long periodStep = 3000;

// Motion Sensor variables
volatile short motionFlag = 0;
volatile int motionCntr = 0;
volatile short motionFilter = 0;

// RFID variables
int tempUid;
const int collarUid = 0xe3cd820d;
const int backwardsCollarUid = 0x0D82CDE3;
volatile int plate = 0;
volatile int rfidRead = 0;

// Stepper Vars
volatile short openLid = 2;
unsigned long numFeed = 0;

// WIFI vars
char catNameResp[50] = ""; 
bool shouldSaveConfig = false;

// Blynk vars
// char auth[] = BLYNK_AUTH_TOKEN;
// char ssid[] = "";
// char pass[] = "";

//==========================

volatile int tFlag = 0;
void IRAM_ATTR TimerHandler(){
  tFlag = 1;
}

//==========================
// Tick Function Declarations 
enum MoS_States{Mos_Start, Mos_Wait, Mos_Detd, Mos_Hold};
int TickFct_MoSensor(int state);

enum RFID_States{RFID_Start, RFID_off, RFID_waitRead};
int TickFct_RFID(int state);

enum Step_States{Step_Start, Step_off, Step_open, Step_hold, Step_close};
int TickFct_Step(int state);

void setup(){ 
  // wifi init
  bool forceConfig = false;
  bool spiffsSetup = loadConfigFile();
  if (!spiffsSetup)
  {
    Serial.println(F("Forcing config mode as there is no saved config"));
    forceConfig = true;
  }
  wm.resetSettings(); // TODO: remove when ready
  WiFi.mode(WIFI_STA);
  Serial.begin(9600);
  while (!Serial);
  
  wm.setPreSaveConfigCallback(saveConfigCallback);
  wm.setAPCallback(configModeCallback);
  wm.setDebugOutput(false);
  WiFiManagerParameter catName("catNameIn", "Enter the cat's name here", catNameResp, 50);
  // Need to convert numerical input to string to display the default value.
  char convertedValue[6];
  sprintf(convertedValue, "%lu", numFeed); 
  WiFiManagerParameter numTimesFed("numTimesCatFed", "How many times cat has been fed", convertedValue, 7); 
  wm.addParameter(&catName);
  wm.addParameter(&numTimesFed);
  // wm.setConfigPortalBlocking(false);
  // wm.setConfigPortalTimeout(60);
  //automatically connect using saved credentials if they exist
  //If connection fails it starts an access point with the specified name
  if (forceConfig)
    // Run if we need a configuration
  {
    if (!wm.startConfigPortal("CatFeeder", "password"))
    {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }
  }
  else
  {
    while(!wm.autoConnect("CatFeeder", "password"));
  }
  Serial.println("Wifi connected");
  Serial.println(WiFi.localIP());
  strncpy(catNameResp, catName.getValue(), sizeof(catNameResp));
  Serial.print("Name of Users Cat: ");
  Serial.println(catNameResp);
  numFeed = atoi(numTimesFed.getValue());
  Serial.print("Num times fed: ");
  Serial.println(numFeed);
  if(shouldSaveConfig){saveConfigFile();}
  // server.begin();
  // wm.getWiFiSSID().toCharArray(ssid, wm.getWiFiSSID().length());
  // wm.getWiFiPass().toCharArray(pass, wm.getWiFiPass().length());
  // Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);

  // Setup a function to be called every second
  // timer.setInterval(1000L, myTimerEvent);
  //catNameResp = catName.getValue();
  // required 1 minute delay for the IR sensor to warm up
  // reduced for wifi startup
  Serial.println("Start up");
  delay(30000);
  Serial.println("Delay over");
  
  // Timer Init
  // Set Interval in microsecs
  if (!ITimer.attachInterruptInterval(tasksPeriodGDC * 1000, TimerHandler))
  {
    exit(1);
  }
  // RFID Init
  SPI.begin();
  mfrc522.PCD_Init();
  delay(4);
  mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max); // max read distance
  /* setup the IRQ pin*/
  // pinMode(IRQ_PIN, INPUT_PULLUP);
  /*
   * Allow the ... irq to be propagated to the IRQ pin
   * For test purposes propagate the IdleIrq and loAlert
   */
  // regVal = 0xA0; //rx irq
  // mfrc522.PCD_WriteRegister(mfrc522.ComIEnReg, regVal);

  // bNewInt = false; //interrupt flag

  /*Activate the interrupt*/
  // attachInterrupt(digitalPinToInterrupt(IRQ_PIN), readCard, FALLING);

  // do { //clear a spourious interrupt at start
  //   ;
  // } while (!bNewInt);
  // bNewInt = false;
  
  // Stepper Motor Init
  pinMode(inOne, OUTPUT);
  pinMode(inTwo, OUTPUT);
  pinMode(inThr, OUTPUT);
  pinMode(inFou, OUTPUT);
  // Motion Sensor Init
  pinMode(moSens, INPUT);
  // pinMode(LED_BUILTIN, OUTPUT);

  // Scheduler Setup // TODO: Finish once the tasks are done
  unsigned i = 0;
  taskArray[i].state = Mos_Start;
  taskArray[i].period = periodMos;
  taskArray[i].elapsedTime = periodMos;
  taskArray[i].TickFct = &TickFct_MoSensor;
  i++;
  taskArray[i].state = RFID_Start;
  taskArray[i].period = periodRFID;
  taskArray[i].elapsedTime = periodRFID;
  taskArray[i].TickFct = &TickFct_RFID;
  i++;
  taskArray[i].state = Step_Start;
  taskArray[i].period = periodStep;
  taskArray[i].elapsedTime = periodStep;
  taskArray[i].TickFct = &TickFct_Step;
}  

volatile unsigned long timeTill = 0;
const int stepCheck = 5000;
void loop(){
  if(tFlag){
    // Serial.println("Time");
    unsigned char i;
    for(i = 0; i < tasksNum; ++i){
      if(taskArray[i].elapsedTime >= taskArray[i].period){
        taskArray[i].state = taskArray[i].TickFct(taskArray[i].state);
        taskArray[i].elapsedTime = 0;
      }
      taskArray[i].elapsedTime += tasksPeriodGDC;
    }
    tFlag = 0;
  }

  if( millis() >= (timeTill + stepCheck)){
    timeTill += stepCheck;
    if(openLid == 1){
      for(unsigned i = 0; i < 129; i++){
        digitalWrite(inOne, LOW);
        digitalWrite(inTwo, LOW);
        digitalWrite(inThr, HIGH);
        digitalWrite(inFou, HIGH);
        delay(10);
        digitalWrite(inOne, LOW);
        digitalWrite(inTwo, HIGH);
        digitalWrite(inThr, HIGH);
        digitalWrite(inFou, LOW);
        delay(10);
        digitalWrite(inOne, HIGH);
        digitalWrite(inTwo, HIGH);
        digitalWrite(inThr, LOW);
        digitalWrite(inFou, LOW);
        delay(10);
        digitalWrite(inOne, HIGH);
        digitalWrite(inTwo, LOW);
        digitalWrite(inThr, LOW);
        digitalWrite(inFou, HIGH);
        delay(10);
      }
      digitalWrite(inOne, LOW);
      digitalWrite(inTwo, LOW);
      digitalWrite(inThr, LOW);
      digitalWrite(inFou, LOW);
      // Serial.println("Lid Open");
      openLid = 2;
    }
    else if(openLid == 0){
      for(unsigned i = 0; i < 129; i++){
        digitalWrite(inOne, HIGH);
        digitalWrite(inTwo, LOW);
        digitalWrite(inThr, LOW);
        digitalWrite(inFou, HIGH);
        delay(10);
        digitalWrite(inOne, HIGH);
        digitalWrite(inTwo, HIGH);
        digitalWrite(inThr, LOW);
        digitalWrite(inFou, LOW);
        delay(10);
        digitalWrite(inOne, LOW);
        digitalWrite(inTwo, HIGH);
        digitalWrite(inThr, HIGH);
        digitalWrite(inFou, LOW);
        delay(10); 
        digitalWrite(inOne, LOW);
        digitalWrite(inTwo, LOW);
        digitalWrite(inThr, HIGH);
        digitalWrite(inFou, HIGH);
        delay(10);
      }
      digitalWrite(inOne, LOW);
      digitalWrite(inTwo, LOW);
      digitalWrite(inThr, LOW);
      digitalWrite(inFou, LOW);
      // Serial.println("Lid Closed");
      openLid = 2;
    }
  }
  if(rfidRead){
    if(!mfrc522.PICC_IsNewCardPresent()){return;}
    // Serial.println("Trying to Read");
    if(mfrc522.PICC_ReadCardSerial()){
        // Serial.println("Read");
        mfrc522.PICC_HaltA();
        tempUid = write_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
        if(tempUid == backwardsCollarUid){
          plate = 1;
          // Serial.println("\tMatch");
        }
        else{
          Serial.print(tempUid); Serial.print(" not "); Serial.println(backwardsCollarUid);
        }
      }
  }

  // WiFiClient client = server.available();
  // // wait for a client (web browser) to connect
  // if (client)
  // {
  //   Serial.println("\n[Client connected]");
  //   while (client.connected())
  //   {
  //     // read line by line what the client (web browser) is requesting
  //     if (client.available())
  //     {
  //       String line = client.readStringUntil('\r');
  //       Serial.print(line);
  //       // wait for end of client's request, that is marked with an empty line
  //       if (line.length() == 1 && line[0] == '\n')
  //       {
  //         client.println(prepareHtmlPage());
  //         break;
  //       }
  //     }
  //   }

  //   while (client.available()) {
  //     // but first, let client finish its request
  //     // that's diplomatic compliance to protocols
  //     // (and otherwise some clients may complain, like curl)
  //     // (that is an example, prefer using a proper webserver library)
  //     client.read();
  //   }

  //   // close the connection:
  //   client.stop();
  //   Serial.println("[Client disconnected]");
  // }
  // Blynk.run();
  // timer.run();
}

int TickFct_MoSensor(int state){
  switch(state){
    case Mos_Start:
      state = Mos_Wait;
      break;
    case Mos_Wait:
      if((digitalRead(moSens) == LOW)){ // set to low due to testing
        if(!motionFilter){
          motionFilter = 1;
          state = Mos_Wait;
        }
        else{
          motionFilter = 0;
          state = Mos_Detd;
        }
      }  
      break;
    case Mos_Detd:
      state = Mos_Hold;
      break;
    case Mos_Hold:
      state = (motionCntr < 30) ? Mos_Wait : Mos_Hold;  // Set to 1 min. TODO: Set to meaningful number
      break;
    default:
      state = Mos_Start;
      break;
  }
  switch(state){
    case Mos_Start:
      break;
    case Mos_Wait:
      // Serial.println("Mos Wait");
      motionFlag = 0;
      motionCntr = 0;
      break;
    case Mos_Detd:
      // Serial.println("Mos Detected");
      motionFlag = 1;
      motionCntr = 0;
      break;
    case Mos_Hold:
      motionCntr++;
      break;
    default:
      motionFlag = 0;
      motionCntr = 0;
      break;
  }
  return state;
}

int TickFct_RFID(int state){
  switch(state){
    case RFID_Start:
      state = RFID_off;
      // mfrc522.PCD_SoftPowerDown();
      break;
    case RFID_off:
      if(motionFlag){
        state = RFID_waitRead;
        // mfrc522.PCD_SoftPowerUp();
      }
      else{
        state = RFID_off;
      }
      break;
    case RFID_waitRead:
      if(motionFlag){
        state = RFID_waitRead;
      }
      else{
        state = RFID_off;
        // mfrc522.PCD_SoftPowerDown();
      }
      break;
    default:
      state = RFID_Start;
      break;
  }
  switch(state){
    case RFID_Start:
      break;
    case RFID_off:
      // Serial.println("Not Reading");
      rfidRead = 0;
      break;
    case RFID_waitRead:
      rfidRead = 1;
      // tempUid = 0;
      // if (bNewInt) { //new read interrupt
      //   mfrc522.PICC_ReadCardSerial(); //read the tag data
      //   clearInt(mfrc522);
      //   mfrc522.PICC_HaltA();
      //   bNewInt = false;
      // }
      // // The receiving block needs regular retriggering (tell the tag it should transmit??)
      // // (mfrc522.PCD_WriteRegister(mfrc522.FIFODataReg,mfrc522.PICC_CMD_REQA);)
      // activateRec(mfrc522);
      // if(write_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size) == collarUid){
      //   if(plate == 0){plate = 1;};
      // }
      // if(mfrc522.PICC_ReadCardSerial()){
      //   Serial.println("Read");
      //   mfrc522.PICC_HaltA();
      //   tempUid = write_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
      //   if(tempUid == backwardsCollarUid){
      //     plate = 1;
      //     Serial.println("\tMatch");
      //   }
      //   else{
      //     Serial.print(tempUid); Serial.print(" not "); Serial.println(backwardsCollarUid);
      //   }
      // }
      break;
    default:
      break;
  }
  return state;
}

int TickFct_Step(int state){ // TODO: needs testing
  switch(state){
    case Step_Start:
      state = Step_off;
      break;
    case Step_off:
      state = plate ? Step_open : Step_off;
      break;
    case Step_open:
      state = Step_hold;
      break;
    case Step_hold:
      state = (plate > 30) ? Step_close : Step_hold; // TODO: set to more meaningful num than 30
      break;
    case Step_close:
      state = Step_off;
      break;
    default:
      state = Step_Start;
      break;  
  }
  switch(state){
    case Step_Start:
      break;
    case Step_off:
      // Serial.println("Step Off");
      break;
    case Step_open:
      // Serial.println("Step Open\n");
      openLid = 1;
      numFeed++;
      Serial.print(catNameResp);
      Serial.print(" has eaten ");
      Serial.print(numFeed);
      Serial.println(" times");
      break;
    case Step_hold:
      plate++;
      // Serial.print("Step hold... ");
      // Serial.println(plate);
      break;
    case Step_close:
      // Serial.println("Step Close");
      plate = 0;
      openLid = 0;
      break;
    default:
      break;  
  }
  return state;
}

// RFID Helper Functions =======================================================================
/**
 * Helper routine to return a byte array as string.
 */
int write_byte_array(byte *buffer, byte bufferSize) {
  int retId = 0x00;
  for (byte i = 0; i < bufferSize; i++) {
    retId = retId | buffer[i] << (i*8);
  }
  return retId;
}
/**
 * MFRC522 interrupt serving routine
 */
// void readCard() {
//   bNewInt = true;
// }

/*
 * The function sending to the MFRC522 the needed commands to activate the reception
 */
// void activateRec(MFRC522 mfrc522) {
//   mfrc522.PCD_WriteRegister(mfrc522.FIFODataReg, mfrc522.PICC_CMD_REQA);
//   mfrc522.PCD_WriteRegister(mfrc522.CommandReg, mfrc522.PCD_Transceive);
//   mfrc522.PCD_WriteRegister(mfrc522.BitFramingReg, 0x87);
// }

/*
 * The function to clear the pending interrupt bits after interrupt serving routine
 */
// void clearInt(MFRC522 mfrc522) {
//   mfrc522.PCD_WriteRegister(mfrc522.ComIrqReg, 0x7F);
// }
// END RFID Helper Functions ==================================================================

// Wifi helper Functions=======================================================================
void saveConfigFile(){
  StaticJsonDocument<512> json;
  json["userCatName"] = catNameResp; 
  json["numFeed"] = numFeed; 
  File configFile = LittleFS.open(JSON_CONFIG_FILE, "w");
  if(!configFile){Serial.println("Error spiff");}
  serializeJsonPretty(json, Serial);
  if(serializeJson(json, configFile) == 0){
    Serial.println(F("Fauled to write to file"));
  }
  configFile.close();
}

bool loadConfigFile()
// Load existing configuration file
{
  // Uncomment if we need to format filesystem
  // SPIFFS.format();
 
  // Read configuration from FS json
  Serial.println("Mounting File System...");
 
  // May need to make it begin(true) first time you are using SPIFFS
  if (LittleFS.begin())
  {
    Serial.println("mounted file system");
    if (LittleFS.exists(JSON_CONFIG_FILE))
    {
      // The file exists, reading and loading
      Serial.println("reading config file");
      File configFile = LittleFS.open(JSON_CONFIG_FILE, "r");
      if (configFile)
      {
        Serial.println("Opened configuration file");
        StaticJsonDocument<512> json;
        DeserializationError error = deserializeJson(json, configFile);
        serializeJsonPretty(json, Serial);
        if (!error)
        {
          Serial.println("Parsing JSON");
 
          strcpy(catNameResp, json["userCatName"]);
          numFeed = json["numFeed"].as<int>();
 
          return true;
        }
        else
        {
          // Error loading JSON data
          Serial.println("Failed to load json config");
        }
      }
    }
  }
  else
  {
    // Error mounting file system
    Serial.println("Failed to mount FS");
  }
 
  return false;
}

void saveConfigCallback()
// Callback notifying us of the need to save configuration
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}
 
void configModeCallback(WiFiManager *myWiFiManager)
// Called when config mode launched
{
  Serial.println("Entered Configuration Mode");
 
  Serial.print("Config SSID: ");
  Serial.println(myWiFiManager->getConfigPortalSSID());
 
  Serial.print("Config IP Address: ");
  Serial.println(WiFi.softAPIP());
}

////////////////////////////////////////////////////////////////////////

// Wifi Server Helper Functions ==========================================
// prepare a web page to be send to a client (web browser)
// String prepareHtmlPage()
// {
//   String htmlPage;
//   htmlPage.reserve(1024);               // prevent ram fragmentation
//   htmlPage = F("HTTP/1.1 200 OK\r\n"
//                "Content-Type: text/html\r\n"
//                "Connection: close\r\n"  // the connection will be closed after completion of the response
//                "Refresh: 60\r\n"         // refresh the page automatically every 5 sec
//                "\r\n"
//                "<!DOCTYPE HTML>"
//                "<html>"
//                "");
//   htmlPage = htmlPage + catNameResp + " has eaten " + static_cast<String>(numFeed) + " times";
//   htmlPage += numFeed;
//   htmlPage += F("</html>"
//                 "\r\n");
//   return htmlPage;
// }

// This function is called every time the device is connected to the Blynk.Cloud
// BLYNK_CONNECTED()
// {
//   // Change Web Link Button message to "Congratulations!"
//   Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
//   Blynk.setProperty(V3, "onImageUrl",  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
//   Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
// }

// // This function sends Arduino's uptime every second to Virtual Pin 2.
// void myTimerEvent()
// {
//   // You can send any value at any time.
//   // Please don't send more that 10 values per second.
//   Blynk.virtualWrite(V4, catNameResp);
//   Blynk.virtualWrite(V2, numFeed);
// }
