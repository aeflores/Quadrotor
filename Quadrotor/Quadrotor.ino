//-------------------------------------------------------------//
//---------------------QUADROTOR CODE -------------------------//
//-------------------------------------------------------------//
//Authors: Enrique Flores and Antonio Flores


bool diagnosis_mode = true;

// Including libraries

#include "IMU.h"
#include "Attitude.h"
#include "EngineControl.h"
#include "Radio.h"
#include "PID.h"


#include "MedianFilterLib.h"
#include "SingleEMAFilterLib.h"

MedianFilter<int> heightmedian(5);
SingleEMAFilter<float> heightEMA(0.5);


// Defining objects
IMU imu;
EngineControl engines;
Radio RadioCOM;
Attitude attitude;
PID pitchPID;

float PID_output;



// Global variables
int16_t       imusensor[3][3];
float         height;
Euler         yawpitchroll;
ControlData   control;          // Control telecomands
unsigned long tiempo0;
int           delta_t;



// Constants
float radtodeg = 180 / acos(-1);  // Radtodeg conversion
float deg2rad = acos(-1) / 180.0; // Degtorad conversion
float pi = acos(-1);
//----------------------------------HEIGHT SENSOR CODE---------------------------------------
// Use of interrupts to parallelize ultrasonic sensor measurements
// ------------------------------------------------------------------------------------------

#define trigPin A6                                    // Pin A6 trigger output
#define echoPin 2                                     // Pin 2 Echo input
#define echo_int 0                                    // Interrupt id for echo pulse
volatile long echo_start = 0;                         // Records start of echo pulse
volatile long echo_end = 0;                           // Records end of echo pulse
volatile long echo_duration = 0;                      // Duration - difference between end and start
volatile long echo_median = 0;

void echo_interrupt() {
  switch (digitalRead(echoPin))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_end = 0;                                 // Clear the end time
      echo_start = micros();                        // Save the start time
      break;
    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_end = micros();                          // Save the end time
      echo_duration = echo_end - echo_start;        // Calculate the pulse duration
      echo_median = heightmedian.AddValue(echo_duration);
      height = heightEMA.AddValue(echo_median* 0.344 / 2);
      sendpulse(true);
      break;
  }
}
// Pulse generator
// Creates a pulse in the trigger pin whether timeout is true or an echo has been received
// when sendpulse is called from the main loop it is called with InterruptTriggered = falso so it only sends a pulse if timeout
// when interrupt is called from echo_interrupt() InterruptTriggered is true so it always sends a pulse
void sendpulse(bool InterruptTriggered){
  if (micros()-echo_start>=200000 || InterruptTriggered == true){
    digitalWrite(trigPin, HIGH);
    digitalWrite(trigPin, LOW);
  }
}


// ----------------------------------------------------------------------------
// ---------------------------Finite State Machine-----------------------------
// ----------------------------------------------------------------------------

// current state initialized to STANDBY
State curr_state = STANDBY;
// Next state function
State next_state(State curr_state, StateChange change) {
  if (change == StateChange::NEXT && curr_state != ABORT)
    return (State) ((curr_state + 1) % NUM_STATES);
  if (change == StateChange::PREV && curr_state != STANDBY)
    return (State) ((curr_state - 1) % NUM_STATES);
  return curr_state;
}

// ----------------------------------------------------------------------------
// --------------------------------- Functions --------------------------------
// ----------------------------------------------------------------------------

void read_sensors() {
  // Get sensors raw values
  // 9 DOF IMU
  imu.readsensor(imusensor);
}



void Print_data() {

//  Serial.print(curr_state);
//  Serial.print("\t");
//  Serial.print(yawpitchroll.yaw_deg());
//  Serial.print("\t");
//  Serial.print(yawpitchroll.pitch_deg());
//  Serial.print("\t");
//  Serial.print(yawpitchroll.roll_deg());
//  Serial.print("\t");
  //Serial.print(echo_duration * 0.344 / 2, 3);
  Serial.print("\t");
  Serial.print(echo_median * 0.344 / 2, 3);
  Serial.print("\t");
  Serial.print(height, 3);
  Serial.print("\t");
  //
//  Serial.print (delta_t/1000.,3);
//  Serial.print ('\t');
//  //  Serial.print (echo_duration);
//  //  Serial.print ('\t');
//  // Accelerometer
//      Serial.print (imusensor[0][0]*4./32767., 2);
//      Serial.print ("\t");
//      Serial.print (imusensor[0][1]*4./32767.,2);
//      Serial.print ("\t");
//      Serial.print (imusensor[0][2]*4./32767.,2);
//      Serial.print ("\t");
////   Gyroscope
//      Serial.print (imusensor[1][0],DEC);
//      Serial.print ("\t");
//      Serial.print (imusensor[1][1],DEC);
//      Serial.print ("\t");
//      Serial.print (imusensor[1][2],DEC);
//      Serial.print ("\t");
//  // Magnetometer
////      Serial.print (imusensor[2][0]*47./39./32.767, DEC);
////      Serial.print ("\t");
////      Serial.print (imusensor[2][1]*47./39./32.767, DEC);
////      Serial.print ("\t");
////      Serial.print (imusensor[2][2]*47./39./32.767, DEC);
////      Serial.print ("\t");
//      Serial.print (imusensor[2][0]*4.7/39./32.767, 2);
//      Serial.print ("\t");
//      Serial.print (imusensor[2][1]*4.7/39./32.767, 2);
//      Serial.print ("\t");
//      Serial.print (imusensor[2][2]*4.7/39./32.767, 2);
//      Serial.print ("\t");
//
//  Serial.print("  Yaw = ");
//  Serial.print(yawpitchroll.yaw_deg());
//  Serial.print("   Pitch = ");
  //    Serial.print("   Pitch = ");
  //    Serial.print(yawpitchroll_triad.pitch*radtodeg);
  //    Serial.print("   Roll = ");
  //    Serial.print(yawpitchroll_triad.roll*radtodeg);
  //    Serial.print("JSRX= ");
  //    Serial.print(control[0]);
  //    Serial.print("  JSRY= ");
  //    Serial.print(control[1]);
  //    Serial.print("  JSLX= ");
  //    Serial.print(control[2]);
  //    Serial.print("  JSLY= ");
  //    Serial.print(control[3]);
  //  Serial.print("  state= ");
  //  Serial.print(control[4]);
  //  Serial.print("  Delta Time  ");
  //  Serial.println(delta_t);
  //  Serial.print("  Ref pitch=  ");
  //  Serial.print(engines.reference.pitch);
  //  Serial.print("  Ref roll=  ");
  //  Serial.print(engines.reference.roll);
  //  Serial.print("  Ref vertical speed=  ");
  //  Serial.print(engines.reference.altitude_rate);
  //  Serial.print("  Ref yaw rate=  ");
  //  Serial.print(engines.reference.yaw_rate);
  //  Serial.print(" Error_pitch  ");
  //  Serial.print(engines.error_pitch);
  //  Serial.print(" Error_roll  ");
  //  Serial.print(engines.error_roll);
  //  Serial.print(" Derivative_Error_pitch  ");
  //  Serial.print(engines.derivative_error_pitch);
  //  Serial.print(" Derivative_Error_roll  ");
  //  Serial.print(engines.derivative_error_roll);
  //  Serial.print(" Base power  ");
  //  Serial.print(engines.power);
  //
  //  Serial.print("  Height =  ");
  //  Serial.print(Height.value, 4);
  //  Serial.print("  Height rate  =  ");
  //  Serial.print(Height.derivative, 4);
  //
  //  Serial.print(" Control_Coeff= ");
  //  Serial.print(engines.error2CorrectionCoeff);
  //  Serial.print(" DerivativeControl_Coeff= ");
  //  Serial.print(engines.derivativeError2CorrectionCoeff);
  //  Serial.print(" upper_range= ");
  //  Serial.print(engines.upperUnbalanceRange);
  //  Serial.print("lower_range= ");
  //  Serial.print(engines.lowerUnbalanceRange);
  //  Serial.print("FFUn14= ");
  //  Serial.print(engines.feedforwardunbalance14);
  //  Serial.print("FFUn23= ");
  //  Serial.print(engines.feedforwardunbalance23);
  //  Serial.print("  Yaw raw=  ");
  //  Serial.print(Yaw.raw);
  //  Serial.print("  Yaw=  ");
  //  Serial.print(Yaw.value);
  //  Serial.print("  Yaw rate=  ");
  //  Serial.print(Yaw.derivative);
  //  Serial.print(" Engine 1: ");
  //  Serial.print(engines.engine_speed[0]);
  //  Serial.print(" Engine 2: ");
  //  Serial.print(engines.engine_speed[1]);
  //  Serial.print(" Engine 3: ");
  //  Serial.print(engines.engine_speed[2]);
  //  Serial.print(" Engine 4 :");
  //  Serial.print(engines.engine_speed[3]);
  Serial.println("\t");

}

// -----------------------------------------------------------------------------
// ---------------------------------- Inicialization ---------------------------
// -----------------------------------------------------------------------------

void setup() {
  if (diagnosis_mode){
      Serial.begin(115200);
  }

  // start communication with IMU
  imu.initialize();
  RadioCOM.initialize();
  engines.init();
  attitude.initial_cond();

  // Distance sensor configuration
  pinMode(trigPin, OUTPUT);                           // Trigger pin set to output
  pinMode(echoPin, INPUT);                            // Echo pin set to input
  attachInterrupt(echo_int, echo_interrupt, CHANGE);  // Attach interrupt to the sensor echo input

  // PIDinitialization
  pitchPID.init(5.0, 1.0, 0.1);


}

// -----------------------------------------------------------------------------
// ---------------------------------- Main Loop --------------------------------
// -----------------------------------------------------------------------------
short cycle_counter, cycle = 4;
ControllerConfiguration configuration;



void loop() {
  
  // La radio transmite y recibe cada 15 ciclos
  cycle_counter = (cycle_counter + 1) % cycle;
  delta_t = micros() - tiempo0;
  tiempo0 += delta_t;
  // En el ciclo 14 recibe datos
  if (cycle_counter == (cycle - 1)) {
    RadioCOM.radiolisten(control, configuration);
    curr_state = next_state(curr_state, control.change);
  }

  
  sendpulse(false);
  read_sensors();
  attitude.get_attitude(imusensor, yawpitchroll, delta_t);
  
  pitchPID.pid_step(yawpitchroll.pitch_deg(), 0, PID_output, delta_t);

  switch (curr_state) {
    case STANDBY:
      engines.stop();
      break;
    case CALIBRATION:
      {
        engines.stop();
        engines.configure(configuration);
        break;
      }
    case FLYMODE:
      if (yawpitchroll.pitch_deg() > 25 ||  yawpitchroll.pitch_deg() < -25 || yawpitchroll.roll_deg() > 25 || yawpitchroll.roll_deg() < -25)
        curr_state = ABORT;
      else
        engines.pdControl(control.movement, yawpitchroll, delta_t);
      break;
    case ABORT:
      engines.stop();
      break;
  }

  // En el ciclo 1 empieza el envio
  if (cycle_counter == 1) {
    RadioCOM.radiosend(curr_state, yawpitchroll, engines, delta_t);
  }
  // En el ciclo 7 termina el envio
  // y empieza a escuchar hasta el ciclo 14
  if (cycle_counter == cycle / 2) {
    RadioCOM.finishSend();
  }
  engines.updateEngines();
  
  if (diagnosis_mode){
    //Print_data();
  }
  
}
