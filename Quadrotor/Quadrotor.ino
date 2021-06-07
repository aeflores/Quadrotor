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

MedianFilter<int> heightmedian(3);
SingleEMAFilter<float> heightEMA(0.8);


// Defining objects
IMU imu;
EngineControl engines;
Radio RadioCOM;
Attitude attitude;






// Global variables
int16_t       imusensor[3][3];
float         height;
Euler         yawpitchroll;
ControlData   control;          // Control telecomands
float         height0;
unsigned long tiempo0;
int           delta_t;
QuadState     state;
short cycle_counter, cycle = 4;
ControllerConfiguration configuration;



// Constants
float radtodeg  = 180 / acos(-1);   // Radtodeg conversion
float deg2rad   = acos(-1) / 180.0; // Degtorad conversion
float pi        = acos(-1);



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
      height = heightEMA.AddValue((echo_median* 0.344 / 2)/cos(yawpitchroll.pitch)/cos(yawpitchroll.roll));
      state.height = height - height0;
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





void Print_data() {
//  Serial.print(curr_state);
  Serial.print("\t");
  Serial.print(state.pitch);
  Serial.print("\t");
  Serial.print(state.roll);
  Serial.print("\t");
  Serial.print(state.yaw_rate);
  Serial.print("\t");
  Serial.print(state.height);
  Serial.print("\t");
  Serial.println("\t");
}

void init_state_vector(){
  sendpulse(true);
  attitude.initial_cond(yawpitchroll);
  state.pitch = yawpitchroll.pitch_deg();
  state.roll = yawpitchroll.roll_deg();
  state.yaw_rate = 0;
  state.height = 0;
  height0 = height;
}

// -----------------------------------------------------------------------------
// ---------------------------------- Inicialization ---------------------------
// -----------------------------------------------------------------------------

void setup() {
  if (diagnosis_mode){
      Serial.begin(115200);
  }

  
  imu.initialize();       // start communication with IMU
  RadioCOM.initialize();  // start radio
  engines.init();         // initialize engines
  // Distance sensor configuration
  pinMode(trigPin, OUTPUT);                           // Trigger pin set to output
  pinMode(echoPin, INPUT);                            // Echo pin set to input
  attachInterrupt(echo_int, echo_interrupt, CHANGE);  // Attach interrupt to the sensor echo input
  init_state_vector(); 

}

// -----------------------------------------------------------------------------
// ---------------------------------- Main Loop --------------------------------
// -----------------------------------------------------------------------------




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
  imu.readsensor(imusensor);
  attitude.get_attitude(imusensor, yawpitchroll, delta_t);
  state.pitch = yawpitchroll.pitch_deg();
  state.roll = yawpitchroll.roll_deg();
  state.yaw_rate = imusensor[1][2]/ 32.7675f / 60;
  
  

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
        engines.pdControl(control.movement, yawpitchroll, state, delta_t);
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
  Print_data();
  
}
