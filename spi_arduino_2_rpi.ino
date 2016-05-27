/*

  Author : Vincent Jaunet


 This sketch is made for
  - Get RC from Rx receiver
  - transmit to Rpi pilot
  - Receive ESC and Servos input from Rpi Pilot

 based onrcarduino.blogspot.com

*/

// include the pinchangeint library - see the links in the related topics section above for details
#include <PinChangeInt.h>
#include <SPI.h>
#include <Servo.h>

//LED pin for checking
#define LED_PIN 13

//For ease of programming
#define THR 0
#define YAW 1
#define PITCH 2
#define ROLL 3
#define SW1 4
#define SW2 5
#define SW3 6
// Assign your channel in pins
#define THROTTLE_IN_PIN 2
#define YAW_IN_PIN 3
#define PITCH_IN_PIN 4
#define ROLL_IN_PIN 5
#define SW1_IN_PIN  A1  //   15 -> A1
#define SW2_IN_PIN  A2  //   16 -> A2 
#define SW3_IN_PIN  A3  //   16 -> A3 

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unYawInShared;
volatile uint16_t unPitchInShared;
volatile uint16_t unRollInShared;
volatile uint16_t unSw1InShared;
volatile uint16_t unSw2InShared;
volatile uint16_t unSw3InShared;
// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulYawStart;
uint32_t ulPitchStart;
uint32_t ulRollStart;
uint32_t ulSw1Start;
uint32_t ulSw2Start;
uint32_t ulSw3Start;
//servo variables
//Number of servos
#define SERVO_NUM 4
#define RC_MIN 1000
// Assign your channel out pins
#define FL_MOTOR_OUT_PIN 6
#define FR_MOTOR_OUT_PIN 7
#define BL_MOTOR_OUT_PIN 8
#define BR_MOTOR_OUT_PIN 9

//define Servo variables
Servo MOTOR[SERVO_NUM];

//define SPI data
volatile union int_byt{
  uint8_t u8[2];
  uint16_t u16;
} rc_data[6], esc_data[4];

volatile byte rx_buf[SERVO_NUM*2+1];
uint8_t pos=0;
uint8_t checksum=0;

bool start=false;
volatile bool update_servo=false;

//setup function
void setup()
{
  Serial.begin(9600); //for debugging...

  //pinMode(LED_PIN, OUTPUT);

  /*
    PWM measurement settings
  */

  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle,CHANGE);
  PCintPort::attachInterrupt(YAW_IN_PIN, calcYaw,CHANGE);
  PCintPort::attachInterrupt(PITCH_IN_PIN, calcPitch,CHANGE);
  PCintPort::attachInterrupt(ROLL_IN_PIN, calcRoll,CHANGE);
  
  
  //pinMode(SW2_IN_PIN, INPUT);
  //digitalWrite(SW2_IN_PIN, LOW);
  PCintPort::attachInterrupt(SW1_IN_PIN, calcSw1,CHANGE);
  PCintPort::attachInterrupt(SW2_IN_PIN, calcSw2,CHANGE);
  PCintPort::attachInterrupt(SW3_IN_PIN, calcSw3,CHANGE);
  // attach servo objects, these will generate the correct
  // pulses for driving Electronic speed controllers, servos or other devices
  // designed to interface directly with RC Receivers
  MOTOR[0].attach(FL_MOTOR_OUT_PIN);
  MOTOR[1].attach(FR_MOTOR_OUT_PIN);
  MOTOR[2].attach(BL_MOTOR_OUT_PIN);
  MOTOR[3].attach(BR_MOTOR_OUT_PIN);

  //Set servo values to min
  for (int i=0;i<SERVO_NUM;i++)
    {
      MOTOR[i].writeMicroseconds(RC_MIN);
    }

  /*
    SPI settings
  */

  // Declare MISO as output : have to send on
  //master in, *slave out*
  pinMode(MISO, OUTPUT);

  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // now turn on interrupts
  SPI.attachInterrupt();

//digitalWrite(LED_PIN, HIGH);

}

void loop()
{
  //Constantly update rc_data
  noInterrupts();
  rc_data[THR].u16 = unThrottleInShared;
  rc_data[YAW].u16 = unYawInShared;
  rc_data[PITCH].u16 = unPitchInShared;
  rc_data[ROLL].u16 = unRollInShared;
  rc_data[SW1].u16 = unSw1InShared;
  rc_data[SW2].u16 = unSw2InShared;
  rc_data[SW3].u16 = unSw3InShared;
  interrupts();
//Serial.print("SW1 :" );  
//Serial.println(unSw1InShared);
//Serial.print("SW2 :" );
//Serial.println(unSw2InShared);
//Serial.println(unSw3InShared);
//Serial.println(unYawInShared);
//Serial.println(start);
//digitalWrite(LED_PIN, HIGH);



  if ((unThrottleInShared < 1200) &
      (unYawInShared      < 1200) ) {
    uint32_t t_old = millis();
    while (millis()-t_old < 500 ){
      //wait to be sure that sticks are still in position
    }
    // if so change current status
    if ((unThrottleInShared < 1200) &
	(unYawInShared      < 1200) ) {

      start = true;
      //change LED status
      //digitalWrite(13, start);
    }
  }
  
   if ((unThrottleInShared < 1200) &
      (unYawInShared      > 1800) ) {
    uint32_t t_old = millis();
    while (millis()-t_old < 500 ){
      //wait to be sure that sticks are still in position
    }
    // if so change current status
    if ((unThrottleInShared < 1200) &
	(unYawInShared      > 1800) ) {

      start = false;
      //change LED status
      //digitalWrite(13, start);
    }
  }

  //Update servo (ESC) if necessary and started
  if (update_servo & start){
    uint8_t ipos=0;
    byte checksum2=0;
    for (int i=0;i<SERVO_NUM;i++)
      {
	//process buffer values
	for (int ibyte = 0;ibyte<2;ibyte++){
	  esc_data[i].u8[ibyte] = rx_buf[ipos];
	  checksum2+=rx_buf[ipos];
	  ipos++;
	}
      }

    if (rx_buf[ipos] == checksum2) {
      //write ESC output
      for (int i=0;i<SERVO_NUM;i++)
	MOTOR[i].writeMicroseconds(esc_data[i].u16);
    }

    update_servo = false;

  } else if (!start) {
    for (int i=0;i<SERVO_NUM;i++)
      {
	MOTOR[i].writeMicroseconds(RC_MIN);
      }
  }

}

/*-----------------------
  SPI interrupt routine
------------------------*/
ISR (SPI_STC_vect)
{

  //grab a new command and process it
  byte cmd = SPDR;

  // Serial.println(cmd);

  if (cmd == 'S') {
    //STOP do nothing : end of sending RC data
    pos=0;
    return;
  }

  if (cmd == 'P') {
    //process it !
    update_servo = true;
    pos=0;
    return;
  }

  if (cmd == 'C') {
    //push Cheksum into the register
    SPDR = checksum;
    checksum = 0;
    pos=0;
    return;
  }


  //push data into a byte buffer
  rx_buf[pos++] = cmd;

  // 10-11 -> send 2bytes channel 1 value
  // ...
  // 40-41 -> send 2bytes channel 4 value
  // ...
  // 60-61 -> send 2bytes channel 6 value
  switch (cmd){
  case 10:
    SPDR = rc_data[THR].u8[0];
    checksum += rc_data[THR].u8[0];
    
  //   Serial.println(rc_data[THR].u8[0]);
    break;

  case 11:
    SPDR = rc_data[THR].u8[1];
    checksum += rc_data[THR].u8[1];
    
    // Serial.println(rc_data[THR].u8[1]);
    break;

  case 20:
    SPDR = rc_data[YAW].u8[0];
    checksum += rc_data[YAW].u8[0];
    break;

  case 21:
    SPDR = rc_data[YAW].u8[1];
    checksum += rc_data[YAW].u8[1];
    break;

  case 30:
    SPDR = rc_data[PITCH].u8[0];
    checksum += rc_data[PITCH].u8[0];
    break;

  case 31:
    SPDR = rc_data[PITCH].u8[1];
    checksum += rc_data[PITCH].u8[1];
    break;

  case 40:
    SPDR = rc_data[ROLL].u8[0];
    checksum += rc_data[ROLL].u8[0];
    break;

  case 41:
    SPDR = rc_data[ROLL].u8[1];
    checksum += rc_data[ROLL].u8[1];
    break;
  case 50:
    SPDR = rc_data[SW1].u8[0];
    checksum += rc_data[SW1].u8[0];
    break;

  case 51:
    SPDR = rc_data[SW1].u8[1];
    checksum += rc_data[SW1].u8[1];
    break;
  case 60:
    SPDR = rc_data[SW2].u8[0];
    checksum += rc_data[SW2].u8[0];
    break;
  case 61:
    SPDR = rc_data[SW2].u8[1];
    checksum += rc_data[SW2].u8[1];
    break;
  case 70:
    SPDR = rc_data[SW3].u8[0];
    checksum += rc_data[SW3].u8[0];
    break;
  case 71:
    SPDR = rc_data[SW3].u8[1];
    checksum += rc_data[SW3].u8[1];
    break;
 }
 
 

}

/*----------------------------------
  simple interrupt service routine
  for PWM measurements
*/
void calcThrottle()
{
  uint32_t tmp;
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(THROTTLE_IN_PIN) == HIGH)
    {
      ulThrottleStart = micros();
    }
  else
    {
      // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
      // this gives use the time between the rising and falling edges i.e. the pulse duration.
      tmp = micros();
      if(tmp > ulThrottleStart)
         unThrottleInShared = (uint16_t)(tmp - ulThrottleStart);

    }
}

void calcYaw()
{
  uint32_t tmp;
  if(digitalRead(YAW_IN_PIN) == HIGH)
    {
      ulYawStart = micros();
    }
  else
    {
      tmp = micros();
      if(tmp > ulYawStart)
         unYawInShared = (uint16_t)(tmp - ulYawStart);
    }
}

void calcPitch()
{
  uint32_t tmp;
  
  if(digitalRead(PITCH_IN_PIN) == HIGH)
    {
      ulPitchStart = micros();
    }
  else
    {
      tmp = micros();
      if(tmp > ulPitchStart)
         unPitchInShared = (uint16_t)(tmp - ulPitchStart);
    }
}

void calcRoll()
{
  uint32_t tmp;
  if(digitalRead(ROLL_IN_PIN) == HIGH)
    {
      ulRollStart = micros();
    }
  else
    {
      tmp = micros();
      if(tmp > ulRollStart)
        unRollInShared = (uint16_t)(tmp - ulRollStart);
    }
}

void calcSw1()
{
  uint32_t tmp;
  if(digitalRead(SW1_IN_PIN) == HIGH)
    {
      ulSw1Start = micros();
    }
  else
    {
      tmp = micros();
      if(tmp > ulSw1Start)
        unSw1InShared = (uint16_t)(tmp - ulSw1Start);
      //  Serial.println(unSw1InShared);
    }
}

void calcSw2()
{
  uint32_t tmp;
  if(digitalRead(SW2_IN_PIN) == HIGH)
    {
      ulSw2Start = micros();
    }
  else
    {
      tmp = micros();
      if(tmp > ulSw2Start)
        unSw2InShared = (uint16_t)(tmp - ulSw2Start);
    }
}

void calcSw3()
{
  uint32_t tmp;
  if(digitalRead(SW3_IN_PIN) == HIGH)
    {
      ulSw3Start = micros();
    }
  else
    {
      tmp = micros();
      if(tmp > ulSw3Start)
        unSw3InShared = (uint16_t)(tmp - ulSw3Start);
    }
}

