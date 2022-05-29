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
#define IRQ_PIN 9

// Stepper definitions
#define stepsPerRev 64

// Motion Sensor definitions
#define moSens 10

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
volatile bool bNewInt = false;
byte regVal = 0x7F;
void activateRec(MFRC522 mfrc522);
void clearInt(MFRC522 mfrc522); 
void readCard();

// Init Stepper Motor
Stepper lidMotor(stepsPerRev, 16, 5, 4, 0);

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
const unsigned char tasksNum = 4;
const unsigned long tasksPeriodGDC = 50;

// Motion Sensor variables
volatile short motionFlag = 0;
volatile int motionCntr = 0;

// RFID variables
String tempUid = "";
const String collarUid = "";
int plates[3] = {0,0,0};
//==========================

void IRAM_ATTR TimerHandler(){
  unsigned char i;
  for(i = 0; i < tasksNum; ++i){
    if(taskArray[i].elapsedTime >= taskArray[i].period){
      taskArray[i].state = taskArray[i].TickFct(taskArray[i].state);
      taskArray[i].elapsedTime = 0;
    }
    taskArray[i].elapsedTime += tasksPeriodGDC;
  }
}

//==========================
// Tick Function Declarations 
enum MoS_States{Mos_Start, Mos_Wait, Mos_Detd, Mos_Hold};
int TickFct_MoSensor(int state);

enum RFID_States {RFID_Start, RFID_off, RFID_waitRead};
int TickFct_RFID(int state);

void setup(){ 
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
  pinMode(IRQ_PIN, INPUT_PULLUP);

  /*
   * Allow the ... irq to be propagated to the IRQ pin
   * For test purposes propagate the IdleIrq and loAlert
   */
  regVal = 0xA0; //rx irq
  mfrc522.PCD_WriteRegister(mfrc522.ComIEnReg, regVal);

  bNewInt = false; //interrupt flag

  /*Activate the interrupt*/
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), readCard, FALLING);

  do { //clear a spourious interrupt at start
    ;
  } while (!bNewInt);
  bNewInt = false;
  // Stepper Motor Init
  lidMotor.setSpeed(30); // arbitrary speed, 30 rpm
  // Scheduler Setup // TODO: Finish once the tasks are done
  // unsigned i = 0;
  // taskArray[i].state = ST_Start;
  // taskArray[i].period = periodLCD;
  // taskArray[i].elapsedTime = periodLCD;
  // taskArray[i].TickFct = &lcdScreenSet;
  // i++;
}  

void loop(){
  // Empty, code will only be executed on interrupt :D
}

int TickFct_MoSensor(int state){
  switch(state){
    case Mos_Start:
      state = Mos_Wait;
      break;
    case Mos_Wait:
      state = digitalRead(moSens) ? Mos_Detd : Mos_Wait;
      break;
    case Mos_Detd:
      state = Mos_Hold;
      break;
    case Mos_Hold:
      state = (motionCntr < 30) ? Mos_Wait : Mos_Hold;  // Arbitrarily set to 30. TODO: Set to meaningful number
      break;
    default:
      state = Mos_Start;
      break;
  }
  switch(state){
    case Mos_Start:
      break;
    case Mos_Wait:
      motionFlag = 0;
      break;
    case Mos_Detd:
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
      mfrc522.PCD_SoftPowerDown();
      break;
    case RFID_off:
      if(motionFlag){
        state = RFID_waitRead;
        mfrc522.PCD_SoftPowerUp();
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
        mfrc522.PCD_SoftPowerDown();
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
      break;
    case RFID_waitRead:
      tempUid = "";
      if (bNewInt) { //new read interrupt
        mfrc522.PICC_ReadCardSerial(); //read the tag data
        clearInt(mfrc522);
        mfrc522.PICC_HaltA();
        bNewInt = false;
      }
      // The receiving block needs regular retriggering (tell the tag it should transmit??)
      // (mfrc522.PCD_WriteRegister(mfrc522.FIFODataReg,mfrc522.PICC_CMD_REQA);)
      activateRec(mfrc522);
      if(write_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size) == collarUid){
        plates[0] = 1;
      }
      break;
    default:
      break;
  }
}

// RFID Helper Functions =======================================================================
/**
 * Helper routine to return a byte array as string.
 */
String write_byte_array(byte *buffer, byte bufferSize) {
  String retId;
  for (byte i = 0; i < bufferSize; i++) {
    retId = retId + (buffer[i] < 0x10 ? " 0" : " ");
    retId = retId + String(buffer[i], HEX);
  }
  return retId;
}
/**
 * MFRC522 interrupt serving routine
 */
void readCard() {
  bNewInt = true;
}

/*
 * The function sending to the MFRC522 the needed commands to activate the reception
 */
void activateRec(MFRC522 mfrc522) {
  mfrc522.PCD_WriteRegister(mfrc522.FIFODataReg, mfrc522.PICC_CMD_REQA);
  mfrc522.PCD_WriteRegister(mfrc522.CommandReg, mfrc522.PCD_Transceive);
  mfrc522.PCD_WriteRegister(mfrc522.BitFramingReg, 0x87);
}

/*
 * The function to clear the pending interrupt bits after interrupt serving routine
 */
void clearInt(MFRC522 mfrc522) {
  mfrc522.PCD_WriteRegister(mfrc522.ComIrqReg, 0x7F);
}
// END RFID Helper Functions ==================================================================