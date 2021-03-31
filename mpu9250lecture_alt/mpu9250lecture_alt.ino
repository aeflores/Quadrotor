
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





float imudata[3][3];
float height;
float deg2rad = atan(-1)/180.0;


//#include "SingleEMAFilterLib.h"
//
//SingleEMAFilter<float> height_filt(0.6);
//
//SingleEMAFilter<float> accx_filt(0.6);
//SingleEMAFilter<float> accy_filt(0.6);
//SingleEMAFilter<float> accz_filt(0.6);
//
//
//SingleEMAFilter<float> magx_filt(0.1);
//SingleEMAFilter<float> magy_filt(0.1);
//SingleEMAFilter<float> magz_filt(0.1);
//
//
//void filterandscale(){
//  height        = height_filt.AddValue(echo_duration*344*1e-3/2.0);
//  
//  imudata[0][0] = accx_filt.AddValue(imusensor[0][0]*4.0f/32767.5f);
//  imudata[0][1] = accy_filt.AddValue(imusensor[0][1]*4.0f/32767.5f);
//  imudata[0][2] = accz_filt.AddValue(imusensor[0][2]*4.0f/32767.5f);
//  
//  imudata[1][0] = imusensor[1][0]*1000.0f*deg2rad/32767.5f;
//  imudata[1][1] = imusensor[1][1]*1000.0f*deg2rad/32767.5f;
//  imudata[1][2] = imusensor[1][2]*1000.0f*deg2rad/32767.5f;
//  
//  imudata[2][0] = magx_filt.AddValue(imusensor[2][0]*48000.0f/32767.5f);
//  imudata[2][1] = magx_filt.AddValue(imusensor[2][1]*48000.0f/32767.5f);
//  imudata[2][2] = magx_filt.AddValue(imusensor[2][2]*48000.0f/32767.5f);
//}








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
  // filterandscale();

  time_new = millis();
  delta_t = time_new - time_old;
  time_old = time_new;


//  Serial.print (delta_t);
//  Serial.print ('\t');
//  Serial.print (echo_duration);
//  Serial.print ('\t');
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
//  Serial.print (imusensor[2][0], DEC);
//  Serial.print ("\t");
//  Serial.print (imusensor[2][1], DEC);
//  Serial.print ("\t");
//  Serial.print (imusensor[2][2], DEC);
//  Serial.print ("\t");

  Serial.print (delta_t);
  Serial.print ('\t');
//  Serial.print (height);
//  Serial.print ('\t');
  // Accelerometer
//  Serial.print (imudata[0][0],DEC);
//  Serial.print ("\t");
//  Serial.print (imudata[0][1],DEC);
//  Serial.print ("\t");
//  Serial.print (imudata[0][2],DEC);
//  Serial.print ("\t");
  // Gyroscope
//  Serial.print (imudata[1][0],3);
//  Serial.print ("\t");
//  Serial.print (imudata[1][1],3);
//  Serial.print ("\t");
//  Serial.print (imudata[1][2],3);
//  Serial.print ("\t");
  // Magnetometer
//  Serial.print (imudata[2][0], 2);
//  Serial.print ("\t");
//  Serial.print (imudata[2][1], 2);
//  Serial.print ("\t");
//  Serial.print (imudata[2][2], 2);
  
  // End of line
  Serial.println("");
}
