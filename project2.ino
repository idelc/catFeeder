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

// Stepper definitions
#define stepsPerRev 64

// Motion Sensor definitions
#define moSens -1

//==========================

// Init ESP8266 only and only Timer 1
ESP8266Timer ITimer; 
// Timer interval (in microseconds)
bool setInterval(unsigned long interval, timer_callback callback);
// Timer interval (in microseconds)
bool attachInterruptInterval(unsigned long interval, timer_callback callback);

// Init RFID
MFRC522 mfrc522(SS_PIN, RST_PIN); 

// Init Stepper Motor
Stepper lidMotor(stepsPerRev, 16, 5, 4, 0);

// scheduler
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
