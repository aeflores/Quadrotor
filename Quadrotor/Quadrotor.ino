//-------------------------------------------------------------//
//---------------------QUADROTOR CODE -------------------------//
//-------------------------------------------------------------//
//Authors: Enrique Flores and Antonio Flores


// Including libraries
#include "Acelerometro.h"
#include "EngineControl.h"
#include "Radio.h"
#include "Distancia.h"
#include "Signal.h"
#include "Triad.h"

// Defining objects
Acelerometro acelerometro;
Radio RadioCOM;
Distancia Dist_sensor;
Triad triad;

// Global variables
float Acc_raw_val[9]; // Raw IMU measurements accelerometer, magnetometer and gyroscope
Attitude yawpitchroll_int; // Yaw pitch roll angles from integrator
Attitude yawpitchroll_triad; // Yaw pitch roll angles from Triad algorithm
Attitude yawpitchroll_offset;
Attitude yawpitchroll; // Filtered yaw pitch roll angles
Attitude yawpitchroll_deg; // yaw pitch roll in degrees
ControlData control; // Control telecomands
// Signal Yaw(0.90,0.75);
// Signal Height(0.1, 0.5);
int first_iteration = 0;
unsigned long tiempo, tiempo0;
int delta_t;
EngineControl engines;

// Constants
float radtodeg = 180 / acos(-1); // Radtodeg conversion
float pi = acos(-1);



// ----------------------------------------------------------------------------
// ---------------------------Finite State Machine-----------------------------
// ----------------------------------------------------------------------------

// current state initialized to STANDBY
State curr_state = STANDBY;
// Next state function
State next_state(State curr_state, StateChange change) {
  if (change== StateChange::NEXT && curr_state!= ABORT)
    return (State) ((curr_state+1) % NUM_STATES);
  if (change== StateChange::PREV && curr_state!= STANDBY)
     return (State) ((curr_state-1) % NUM_STATES);
  return curr_state;
}

// ----------------------------------------------------------------------------
// --------------------------------- Functions --------------------------------
// ----------------------------------------------------------------------------

void read_sensors() {
  // Lectura de los sensores
  // Lectura de los valores raw de la IMU 9 DOF
  acelerometro.get_raw_val(Acc_raw_val);
  // Determinacion de los angulos de Euler utilizando el algoritmo TRIAD
  triad.get_ypr_triad(Acc_raw_val,yawpitchroll_triad);

  // Determinación de los angulos de Euler utilizando el integrador
  if (first_iteration <= 20){
    triad.integrator(Acc_raw_val, yawpitchroll_triad, delta_t / 1000.0, yawpitchroll_int);
    if (first_iteration < 20){
    yawpitchroll_offset.pitch+= yawpitchroll_triad.pitch;
    yawpitchroll_offset.roll+= yawpitchroll_triad.roll;
    }else{
      yawpitchroll_offset.pitch = yawpitchroll_offset.pitch/first_iteration;
      yawpitchroll_offset.roll = yawpitchroll_offset.roll/first_iteration;
    }
    first_iteration++;
  } else {
    yawpitchroll_triad.pitch -= yawpitchroll_offset.pitch;
    yawpitchroll_triad.roll -= yawpitchroll_offset.roll;
    triad.integrator(Acc_raw_val, yawpitchroll, delta_t / 1000.0, yawpitchroll_int);
  }
  //Aplicacion del filtro que combina ambas medidas
  triad.filter(yawpitchroll_triad, yawpitchroll_int, yawpitchroll);
  // Yaw.update(yawpitchroll.yaw*radtodeg, delta_t);
  //Height.update(Dist_sensor.update_distance(),delta_t);
  // Lectura del dato de altura  
  //Height[0]=Dist_sensor.update_distance();
  // Transformamos la altura medida en altura con respecto al suelo
  //Height[0]=Height[0]*cos(yawpitchroll[1])*cos(yawpitchroll[2]);

  yawpitchroll_deg.yaw= yawpitchroll.yaw * radtodeg;
  yawpitchroll_deg.pitch= yawpitchroll.pitch * radtodeg;
  yawpitchroll_deg.roll= yawpitchroll.roll * radtodeg;


}

void Print_data() {      
//    Serial.print("Yaw = ");
//    Serial.print(yawpitchroll[0]*radtodeg);
    Serial.print("   Pitch = ");
    Serial.print(yawpitchroll_deg.pitch);
    Serial.print("   Roll = ");
    Serial.print(yawpitchroll_deg.roll);
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
//  Serial.println(millis());
  Serial.print("  Ref pitch=  ");
  Serial.print(engines.reference.pitch);
  Serial.print("  Ref roll=  ");
  Serial.print(engines.reference.roll);
  //Serial.print("  Ref vertical speed=  ");
  //Serial.print(engines.reference.altitude_rate);
  //Serial.print("  Ref yaw rate=  ");
  //Serial.print(engines.reference.yaw_rate);
  Serial.print(" Error_pitch");
  Serial.print(engines.error_pitch);
  Serial.print(" Error_roll");
  Serial.print(engines.error_roll);
  Serial.print(" Base power");
  Serial.print(engines.power);

  Serial.print(" Control_Coeff= ");
  Serial.print(engines.error2CorrectionCoeff);
  Serial.print(" upper_range= ");
  Serial.print(engines.upperUnbalanceRange);
  Serial.print("lower_range= ");
  Serial.print(engines.lowerUnbalanceRange);
//  Serial.print("  Yaw raw=  ");
//  Serial.print(Yaw.raw);
//  Serial.print("  Yaw=  ");
//  Serial.print(Yaw.value);
//  Serial.print("  Yaw rate=  ");
//  Serial.print(Yaw.derivative);
  Serial.print(" Engine 1: ");
  Serial.print(engines.engine_speed[0]);
  Serial.print(" Engine 2: ");
  Serial.print(engines.engine_speed[1]);
  Serial.print(" Engine 3: ");
  Serial.print(engines.engine_speed[2]);
  Serial.print(" Engine 4 :");
  Serial.print(engines.engine_speed[3]);
  Serial.println(" ");
}

// -----------------------------------------------------------------------------
// ---------------------------------- Inicialization ---------------------------
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  // start communication with IMU
  acelerometro.initialize();
  acelerometro.default_cal();
  acelerometro.settings();
  RadioCOM.initialize();
  engines.init();
  //Dist_sensor.initialize();
  triad.initialize();
}

// -----------------------------------------------------------------------------
// ---------------------------------- Main Loop --------------------------------
// -----------------------------------------------------------------------------

void loop() {
  tiempo = millis();
  delta_t = tiempo - tiempo0;
  tiempo0 = tiempo;
  ControllerConfiguration configuration;
  RadioCOM.radiolisten(control, configuration);
  read_sensors();
  curr_state = next_state(curr_state, control.change);
  switch (curr_state) {
  case STANDBY:
    //Serial.print("STANDBY   ");
    engines.stop();
    break;
  case CALIBRATION:
  {
    //Serial.print("CALIBRATION   ");
    engines.stop();
    engines.configure(configuration);
    // acelerometro.acelerometro_cal();
    // acelerometro.magnetometro_cal();
    // acelerometro.gyroscope_cal();
    break;
  }
  case FLYMODE:
    //Serial.print("FLYMODE   ");
    if(yawpitchroll_deg.pitch > 25 ||  yawpitchroll_deg.pitch < -25 || yawpitchroll_deg.roll > 25 || yawpitchroll_deg.roll < -25)
      curr_state = ABORT;
    else
      engines.proportionalControl(control.movement, yawpitchroll_deg);
    break;
  case ABORT:
    //Serial.print("ABORT  ");
    engines.stop();
    break;
  }
  RadioCOM.radiosend(curr_state, yawpitchroll_deg, engines, delta_t);
//  Print_data();
  engines.updateEngines();

}
