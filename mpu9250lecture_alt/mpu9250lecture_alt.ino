
#include "IMU.h"
IMU imu;
int16_t imusensor[3][3];




#include <TimerOne.h>                                 // Header file for TimerOne library
#define trigPin A6                                    // Pin 12 trigger output
#define echoPin 2                                     // Pin 2 Echo input
#define echo_int 0                                    // Interrupt id for echo pulse

#define TIMER_US 50                                   // 50 uS timer duration 
#define TICK_COUNTS 4000                              // 200 mS worth of timer ticks

volatile long echo_start = 0;                         // Records start of echo pulse
volatile long echo_end = 0;                           // Records end of echo pulse
volatile long echo_duration = 0;                      // Duration - difference between end and start
volatile int trigger_time_count = 0;                  // Count down counter to trigger pulse time

void trigger_pulse() {
  static volatile int state = 0;                 // State machine variable
  if (!(--trigger_time_count))                   // Count to 200mS
  { // Time out - Initiate trigger pulse
    trigger_time_count = TICK_COUNTS;           // Reload
    state = 1;                                  // Changing to state 1 initiates a pulse
  }
  switch (state)                                 // State machine handles delivery of trigger pulse
  {
    case 0:                                      // Normal state does nothing
      break;
    case 1:                                      // Initiate pulse
      digitalWrite(trigPin, HIGH);              // Set the trigger output high
      state = 2;                                // and set state to 2
      break;
    case 2:                                      // Complete the pulse
    default:
      digitalWrite(trigPin, LOW);               // Set the trigger output low
      state = 0;                                // and return state to normal 0
      break;
  }
}

void echo_interrupt(){
  switch (digitalRead(echoPin))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_end = 0;                                 // Clear the end time
      echo_start = micros();                        // Save the start time
      break;
    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_end = micros();                          // Save the end time
      echo_duration = echo_end - echo_start;        // Calculate the pulse duration
      break;
  }
}

// Initializations
void setup()
{
  Serial.begin(115200);
  imu.initialize();

  pinMode(trigPin, OUTPUT);                           // Trigger pin set to output
  pinMode(echoPin, INPUT);                            // Echo pin set to input
  Timer1.initialize(TIMER_US);                        // Initialise timer 1
  Timer1.attachInterrupt(trigger_pulse);                 // Attach interrupt to the timer service routine
  attachInterrupt(echo_int, echo_interrupt, CHANGE);  // Attach interrupt to the sensor echo input

}

unsigned long time_new, time_old, delta_t;

void loop()
{

  imu.readsensor(imusensor);

  time_new = millis();
  delta_t = time_new - time_old;
  time_old = time_new;
  float distance = echo_duration*344*1e-3/2.0;

  Serial.print (delta_t);
  Serial.print ('\t');
  Serial.print (distance);
  Serial.print ('\t');
  //  // Accelerometer
  //  Serial.print (imusensor[0][0],DEC);
  //  Serial.print ("\t");
  //  Serial.print (imusensor[0][1],DEC);
  //  Serial.print ("\t");
  //  Serial.print (imusensor[0][2],DEC);
  //  Serial.print ("\t");
  //  // Gyroscope
  //  Serial.print (imusensor[1][0],DEC);
  //  Serial.print ("\t");
  //  Serial.print (imusensor[1][1],DEC);
  //  Serial.print ("\t");
  //  Serial.print (imusensor[1][2],DEC);
  //  Serial.print ("\t");
  // Magnetometer
  Serial.print (imusensor[2][0], DEC);
  Serial.print ("\t");
  Serial.print (imusensor[2][1], DEC);
  Serial.print ("\t");
  Serial.print (imusensor[2][2], DEC);
  Serial.print ("\t");
  // End of line
  Serial.println("");
}
