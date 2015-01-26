/*
 * @file roverBot.ino
 * @brief ITEC-4501 Special Projects Rover.
 * @author Timothy Locke
 * 
 *
 *
 *   
 *
 * Created August 28th, 2014
 * Updated October 17th, 2014
 * v0.7.2
 

Required connections between Arduino Mega and qik 2s9v1:

      Arduino   qik 2s9v1
-------------------------
           5V - VCC
          GND - GND
Digital Pin 69 - TX
Digital Pin 68 - RX
Digital Pin 67 - RESET

IR Connections:
---------------
Left Sensor A0
Right Sensor A1

Sonar Connections:  Trigger  Echo
---------------------------------

Sonar 0 - Left          24      25
Sonar 1 - Center 		22      23
Sonar 2 - Right         26      27


Mode Pushbutton:
---------------------------
Digital Pin 2

CC3000
------
IRQ   21  // MUST be an interrupt pin!
VBAT  30
CS    31
MISO  50  //SPI
MOSI  51  //SPI
CLK   52  //SPI

Emic 2 Text To Speech
---------------------
RX  62  Serial input (connects to Emic 2 SOUT - Green) This is also A8
TX  63  Serial output (connects to Emic 2 SIN - Blue) This is also A9

*/

// Enables:
#define ENABLE_EMIC2
//#define ENABLE_CC3000
#define ENABLE_SERVO_DRIVER

#ifdef ENABLE_EMIC2
#include "Emic2TtsModule.h"
#endif

#ifdef ENABLE_SERVO_DRIVER
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  140 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600
#endif

#ifdef ENABLE_CC3000
#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include "utility/debug.h"
#include <stdlib.h>
#include <StopWatch.h>
#endif

#include <NewPing.h>
#include <SoftwareSerial.h>
#include <PololuQik.h>
#include "moving_average.h"

// constants
#define SONAR_NUM     3 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define TOO_CLOSE 20     // distance to obstacle in centimeters 
#define FAST 255,255
#define MED 210,210
#define SLOW 190,190
#define FAST_RIGHT 255,-255
#define FAST_LEFT -255,255
#define MED_RIGHT 210,-210
#define MED_LEFT -210,210
#define SLOW_RIGHT 190,-190
#define SLOW_LEFT -190,190
#define BACKUP_TIME 500
#define IR_TURN 30
#define LEFT 0
#define RIGHT 1
#define LEFT_RIGHT 2
#define RANDOM_ANALOG_PIN 5
#define rxPin 62    // Serial input (connects to Emic 2 SOUT - Green)
#define txPin 63    // Serial output (connects to Emic 2 SIN - Blue)

//connect gp2d120x to analog pins
#define irLeft A0
#define irRight A1

#ifdef ENABLE_CC3000
// Define CC3000 chip pins
#define ADAFRUIT_CC3000_IRQ   21
#define ADAFRUIT_CC3000_VBAT  32
#define ADAFRUIT_CC3000_CS    33
#endif

//Create objects and variables
PololuQik2s9v1 qik(69, 68, 67);

#ifdef ENABLE_EMIC2
// set up a new serial port
SoftwareSerial emicSerial =  SoftwareSerial(rxPin, txPin);
Emic2TtsModule emic2TtsModule = Emic2TtsModule(&emicSerial);
#endif

#ifdef ENABLE_CC3000
StopWatch MySW;
StopWatch SWarray[5];
#endif

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where distance data is stored.
boolean bSensor[SONAR_NUM];         // Where sensor status is stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
enum state_t { stateStopped, stateMoving, stateTurning, stateBackward, stateRemote };
state_t state; // declare states
unsigned long endStateTime;
MovingAverage<unsigned int, 3> distanceAverageRight; //create averaging object for IR distance
MovingAverage<unsigned int, 3> distanceAverageLeft; //create averaging object for IR distance
int lastTurn; //keep track of last turn for corner oscillation algorithm

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(24, 25, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(22, 23, MAX_DISTANCE),
  NewPing(26, 27, MAX_DISTANCE)
  };

void setup() {
    Serial.begin(115200);
    pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
    for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
      pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
    qik.init();  //initialize the Pololu qik motor controller
    randomSeed(analogRead(RANDOM_ANALOG_PIN));
    pinMode(rxPin, INPUT);  //Set pins to input or output
    pinMode(txPin, OUTPUT);
    pinMode(irRight, INPUT);
    pinMode(irLeft, INPUT);
	
    //Servo driver initialization
    int degrees;
    pwm.begin();
    pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
	
        

 #ifdef ENABLE_EMIC2 //Emic 2 text to speech initialization  
    emicSerial.begin(9600); // set the data rate for the SoftwareSerial port
    
    emic2TtsModule.init();
    emic2TtsModule.setVolume(18);
    emic2TtsModule.setWordsPerMinute(200);
    emic2TtsModule.setVoice(PerfectPaul);
    Serial.print(F("OK"));
    Serial.println();
	
    emicSerial.print('S');
    emicSerial.print("Hello. My name is Cyber Knight, the Row Bot. I can do a lot of Row Bot tricks such as");  // Send the desired string to convert to speech
    emicSerial.print('\n');
    while (emicSerial.read() != ':');   // Wait here until the Emic 2 responds with a ":" indicating it's ready to accept the next command
 #endif
 
 #ifdef ENABLE_EMIC2 && #ifdef ENABLE_SERVO_DRIVER //speaking and arm movement code
    sdAttention();
    emicSerial.print('S');
    emicSerial.print("Extend my arm to you. ");  // Send the desired string to convert to speech
    emicSerial.print('\n');
    while (emicSerial.read() != ':');
    sdExtendArmRight();
    
    delay(1500);    
    
    emicSerial.print('S');
    emicSerial.print("stand at attention. ");  // Send the desired string to convert to speech
    emicSerial.print('\n');
    while (emicSerial.read() != ':');
    sdAttention();
    
    delay(1500);
        
    emicSerial.print('S');
    emicSerial.print("Salute you. ");  // Send the desired string to convert to speech
    emicSerial.print('\n');
    while (emicSerial.read() != ':');
    sdSalute();
        
    delay(1500);
    
    sdAttention();
    emicSerial.print('S');
    emicSerial.print("Try to fli like a bird. by flapping my arms. ");  // Send the desired string to convert to speech
    emicSerial.print('\n');
    while (emicSerial.read() != ':');
    for (int i = 0; i <= 4; i++)
    sdFlapArms();
       
    delay(1500);
    
    sdAttention();
    emicSerial.print('S');
    emicSerial.print("Wave at you. ");  // Send the desired string to convert to speech
    emicSerial.print('\n');
    while (emicSerial.read() != ':');
    for (int i = 0; i <= 4; i++)
    sdWaveHandRight();
        
    delay(1000);
    
    sdAttention();
    emicSerial.print('S');
    emicSerial.print("So you can see how strong I am, I can show you my muscles. ");  // Send the desired string to convert to speech
    emicSerial.print('\n');
    while (emicSerial.read() != ':');
    sdShowMuscles();
        
    delay(3000);
   
    sdAttention();
    emicSerial.print('S');
    emicSerial.print("And finally as a good will gesture, I can shake hands. ");  // Send the desired string to convert to speech
    emicSerial.print('\n');
    while (emicSerial.read() != ':');
    for (int i = 0; i <= 4; i++)
    sdShakeHand();
    
    emicSerial.print('S');
    emicSerial.print("Now. I will sing a song to you. ");  // Send the desired string to convert to speech
    emicSerial.print('\n');
    while (emicSerial.read() != ':');
    
    // Sing a song
    emicSerial.print("D1\n");
    while (emicSerial.read() != ':');
    
    emicSerial.print('S');
    emicSerial.print("Thank you for your attention.");  // Send the desired string to convert to speech
    emicSerial.print('\n');
    while (emicSerial.read() != ':');
	sdBow();
	
	delay(1500);
 
    sdAttention(); 
    emicSerial.print('S');
    emicSerial.print("Now. In one second, I will show you how I drive around. ");  // Send the desired string to convert to speech
    emicSerial.print('\n');
    while (emicSerial.read() != ':');
    
 #endif 
 
    delay(1000);
    
    qik.setSpeeds(FAST);  
    state = stateMoving;
   
  }

void loop() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is cancelled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 200;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
  // non Sonar routines.
  unsigned long currentTime = millis();
  
  if (moving()){
    double irDistanceLeft = distanceAverageLeft.add(getDistance(irLeft)); // get IR distance
    double irDistanceRight = distanceAverageRight.add(getDistance(irRight));
    
    //Serial.print(irDistanceLeft);
    //Serial.print("  ");
    //Serial.print(irDistanceRight);
    //Serial.println();
    
      if (irDistanceLeft < 14 || irDistanceRight < 14){ //If IR fires (too close)
        stop();
        moveBackward(currentTime, BACKUP_TIME);
      }
         

  }	
  else if (turning()){
    if (doneTurning(currentTime, getNum()))
        move();
  }
  else if (backward()){
    if (doneBackward(currentTime))
		if (lastTurn == LEFT) // prevent corner oscillation
			turnLeft(currentTime, IR_TURN);
			else if (lastTurn == RIGHT)
				turnRight(currentTime, IR_TURN);
				else
					turnLeftRight(currentTime,IR_TURN);
  }
}


void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer()){
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
    if (sonar[currentSensor].ping_result / US_ROUNDTRIP_CM <= TOO_CLOSE)
      bSensor[currentSensor] = true;
    else bSensor[currentSensor] = false;
  }
}

void oneSensorCycle(){ // Sensor ping cycle complete, do something with the results.
  
  int num = getNum();
  unsigned long currentTime = millis();
  
  if (moving()){
      // turn states
      if (num == 2)
          turnLeftRight(currentTime, 3); //Turn randomly left or right
          
      if (num == 1 || num == 3) // Turn right
          turnRight(currentTime, 3);
          
      if (num == 4 || num == 6) // Turn left 
          turnLeft(currentTime, 3);
  }
  else if (turning()) {
      if (doneTurning(currentTime, num)){
		  lastTurn = LEFT_RIGHT;
          move();
	  }
  }
  
  //Serial monitor for test purposes
  //for (uint8_t i = 0; i < SONAR_NUM; i++) {
    //Serial.print(i);
    //Serial.print("|");
    //Serial.print(bSensor[i]);
    //Serial.print("  ");
    //Serial.print("cm ");
   //}
   
    //Serial.print("  ");
    //Serial.print("Num = ");
    //Serial.print(num);
    //Serial.println();
  
}
  
void move()
{
  qik.setSpeeds(FAST);  
  state = stateMoving;
}

void moveBackward(unsigned long currentTime, int backupTime)
{
  qik.setSpeeds(-255,-255);
  state = stateBackward;
  endStateTime = currentTime + (backupTime);
}

void stop()
{
  qik.setCoasts();
  state = stateStopped;
}


bool turnRight(unsigned long currentTime, int degrees)
{
    qik.setSpeeds(FAST_RIGHT);
    state = stateTurning;
    endStateTime = currentTime + (degrees * 11.94); 
    
}

bool turnLeft(unsigned long currentTime, int degrees)
{
    qik.setSpeeds(FAST_LEFT);
    state = stateTurning;
    endStateTime = currentTime + (degrees * 11.94); 
    
}

bool turnLeftRight(unsigned long currentTime, int degrees)
{
  if (random(2) == 0){
    qik.setSpeeds(FAST_LEFT);
    lastTurn = LEFT;
  }
  else{
    qik.setSpeeds(FAST_RIGHT);
    lastTurn = RIGHT;
  }
    
  state = stateTurning;
  endStateTime = currentTime + (degrees * 11.94);
}

bool doneTurning(unsigned long currentTime, int num)
{
    if (currentTime >= endStateTime && num == 0)
        return true;
    else return false;
}

bool doneBackward(unsigned long currentTime)
{
    if (currentTime >= endStateTime)
        return true;
    else return false;
}

int getNum()
{
  int num = 0;
  
  for (int i = 0; i < SONAR_NUM; i++){
    if (bSensor[i] == true){
      if (i == 2){
        num += pow(2,2.0);
        return num;
      }
      num += pow(2,i);
    }
   }
    
  return num;
  
}

//return IR distance
double getDistance(char sensor)
{
  double irDistance = MAX_DISTANCE;
  uint16_t value = analogRead (sensor);
  irDistance = get_gp2d120x (value);
  return irDistance; 
 }

//return IR distance (cm)
double get_gp2d120x (uint16_t value){
    if (value < 16)  value = 16;
    return 2076.0 / (value - 11.0);
}

void sdAttention(){ // This is the default starting posture
    pwm.setPWM(0, 0, map(75, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(1, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(2, 0, map(80, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(3, 0, map(0, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(4, 0, map(91, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(5, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(6, 0, map(176, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(7, 0, map(135, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(8, 0, map(180, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(9, 0, map(91, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(10, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(11, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(12, 0, map(45, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(13, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
}

void sdFlapArms(){
    pwm.setPWM(0, 0, map(75, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(1, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(2, 0, map(80, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(3, 0, map(0, 0, 180, SERVOMIN, SERVOMAX));
    
    for (int degrees = 90; degrees <= 180; degrees++) //flap motion
    pwm.setPWM(4, 0, map(degrees, 0, 180, SERVOMIN, SERVOMAX));
    
    for (int degrees = 90; degrees >= 0; degrees--)
    pwm.setPWM(9, 0, map(degrees, 0, 180, SERVOMIN, SERVOMAX));
    
    delay(500);
    
    for (int degrees = 180; degrees >= 90; degrees--)
    pwm.setPWM(4, 0, map(degrees, 0, 180, SERVOMIN, SERVOMAX));
    
    for (int degrees = 0; degrees <= 90; degrees++)
    pwm.setPWM(9, 0, map(degrees, 0, 180, SERVOMIN, SERVOMAX));
    
    delay(500);
    
    pwm.setPWM(5, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(6, 0, map(176, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(7, 0, map(135, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(8, 0, map(180, 0, 180, SERVOMIN, SERVOMAX));
    
    pwm.setPWM(10, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(11, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(12, 0, map(45, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(13, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
}

void sdSalute(){
  
    pwm.setPWM(0, 0, map(75, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(1, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(2, 0, map(80, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(3, 0, map(0, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(4, 0, map(91, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(5, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(6, 0, map(176, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(7, 0, map(135, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(8, 0, map(45, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(9, 0, map(86, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(10, 0, map(85, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(11, 0, map(155, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(12, 0, map(45, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(13, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));

}

void sdExtendArmRight(){
    pwm.setPWM(0, 0, map(75, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(1, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(2, 0, map(80, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(3, 0, map(0, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(4, 0, map(91, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(5, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(6, 0, map(176, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(7, 0, map(135, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(8, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(9, 0, map(91, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(10, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(11, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(12, 0, map(45, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(13, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
}

void sdWaveHandRight(){
    pwm.setPWM(0, 0, map(75, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(1, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(2, 0, map(80, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(3, 0, map(0, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(4, 0, map(91, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(5, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(6, 0, map(176, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(7, 0, map(135, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(8, 0, map(0, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(10, 0, map(180, 0, 180, SERVOMIN, SERVOMAX));
    
    for (int degrees = 60; degrees <= 110; degrees++) //Wave hand
    pwm.setPWM(9, 0, map(degrees, 0, 180, SERVOMIN, SERVOMAX));
    delay(1000);
    for (int degrees = 110; degrees >= 60; degrees--)
    pwm.setPWM(9, 0, map(degrees, 0, 180, SERVOMIN, SERVOMAX));
    delay(1000);
    
    pwm.setPWM(11, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(12, 0, map(45, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(13, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
}

void sdShowMuscles(){
    pwm.setPWM(0, 0, map(75, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(1, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(2, 0, map(80, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(3, 0, map(180, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(4, 0, map(78, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(5, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(6, 0, map(125, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(7, 0, map(135, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(8, 0, map(0, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(9, 0, map(100, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(10, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(11, 0, map(120, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(12, 0, map(45, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(13, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
}

void sdShakeHand(){
    pwm.setPWM(0, 0, map(75, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(1, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(2, 0, map(80, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(3, 0, map(0, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(4, 0, map(91, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(5, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(6, 0, map(176, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(7, 0, map(135, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(8, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    
    for (int degrees = 80; degrees <= 100; degrees++) //move arm
    pwm.setPWM(8, 0, map(degrees, 0, 180, SERVOMIN, SERVOMAX));
    delay(1000);
    for (int degrees = 100; degrees >= 80; degrees--)
    pwm.setPWM(8, 0, map(degrees, 0, 180, SERVOMIN, SERVOMAX));
    delay(1000);
    
    pwm.setPWM(9, 0, map(91, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(10, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(11, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(12, 0, map(45, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(13, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
}

void sdBow(){
    pwm.setPWM(0, 0, map(75, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(1, 0, map(100, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(3, 0, map(0, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(4, 0, map(65, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(5, 0, map(35, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(6, 0, map(176, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(7, 0, map(135, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(8, 0, map(120, 0, 180, SERVOMIN, SERVOMAX)); //right arm forward
    pwm.setPWM(9, 0, map(170, 0, 180, SERVOMIN, SERVOMAX)); //right arm to chest
    pwm.setPWM(10, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(11, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(12, 0, map(45, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(13, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(2, 0, map(125, 0, 180, SERVOMIN, SERVOMAX)); //bow forward
}

bool moving() { return (state == stateMoving); }
bool turning() { return (state == stateTurning); }
bool stopped() { return (state == stateStopped); }
bool backward() { return (state == stateBackward); }
bool remoteControlled() { return (state == stateRemote); } 


