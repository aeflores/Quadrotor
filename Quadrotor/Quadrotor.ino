//-------------------------------------------------------------//
//---------------------QUADROTOR CODE -------------------------//
//-------------------------------------------------------------//
//Authors: Enrique Flores and Antonio Flores


// Including libraries
#include "Acelerometro.h"
#include "Radio.h"
#include "Distancia.h"
#include "Signal.h"

// Defining objects
Acelerometro acelerometro;
Radio RadioCOM;
Distancia Dist_sensor;


// Global variables
float Acc_raw_val[9]; // Raw IMU measurements accelerometer, magnetometer and gyroscope
float yawpitchroll_int[3]; // Yaw pitch roll angles from integrator
float yawpitchroll_triad[3]; // Yaw pitch roll angles from Triad algorithm
float yawpitchroll[3]; // Filtered yaw pitch roll angles
int control[5]; // Control telecomands
float ref_pitch, ref_roll, ref_yaw_rate, ref_altitude_rate;
Signal Yaw(0.90,0.75);
//Signal Height(0.1, 0.5);
int first_iteration = 0;
unsigned long tiempo, tiempo0;
int delta_t;


// Constants
float radtodeg = 180 / acos(-1); // Radtodeg conversion






// ----------------------------------------------------------------------------
// ---------------------------Finite State Machine-----------------------------
// ----------------------------------------------------------------------------
// Possible states
enum state { STANDBY = 0, CALIBRATION = 1, FLYMODE = 2, ABORT = 3 };
// current state initialized to STANDBY
state curr_state = STANDBY;
// Next state function
state next_state() {
  if (control[4] == 0)
    return STANDBY;
  if (control[4] == 1)
    return CALIBRATION;
  if (control[4] == 2)
    return FLYMODE;
  if (control[4] == 3)
    return ABORT;
}

// ----------------------------------------------------------------------------
// --------------------------------- Functions --------------------------------
// ----------------------------------------------------------------------------

void read_sensors() {
  // Lectura de los sensores
  // Lectura de los valores raw de la IMU 9 DOF
  acelerometro.get_raw_val(Acc_raw_val);
  // Determinacion de los angulos de Euler utilizando el algoritmo TRIAD
  get_ypr_triad(Acc_raw_val,yawpitchroll_triad);

  // Determinación de los angulos de Euler utilizando el integrador
  if (first_iteration <= 20){
    integrator(Acc_raw_val, yawpitchroll_triad, delta_t / 1000.0, yawpitchroll_int);
    first_iteration++;
  } else {
    integrator(Acc_raw_val, yawpitchroll_int, delta_t / 1000.0, yawpitchroll_int);
  }
  //Aplicacion del filtro que combina ambas medidas
  filter(Acc_raw_val, yawpitchroll_triad, yawpitchroll_int, yawpitchroll);
  Yaw.update(yawpitchroll[0]*radtodeg, delta_t);
  //Height.update(Dist_sensor.update_distance(),delta_t);
  // Lectura del dato de altura  
  //Height[0]=Dist_sensor.update_distance();
  // Transformamos la altura medida en altura con respecto al suelo
  //Height[0]=Height[0]*cos(yawpitchroll[1])*cos(yawpitchroll[2]);




}



void Reference(){
  // Calculating the pitch and roll references from the joystick data
  ref_pitch = (-float(control[3])+515.)/1024.*30.;
  ref_roll = (-float(control[2])+502.)/1024.*30.;
  // Calculating the height rate and the roll variation rate references from data
  ref_altitude_rate = (-float(control[1])+518.)/1024.*20.; // º/s
  ref_yaw_rate = (-float(control[0])+518.)/1024.*10.; // cm/s

}

void Print_data() {      
//    Serial.print("Yaw = ");
//    Serial.print(yawpitchroll[0]*radtodeg);
//    Serial.print("   Pitch = ");
//    Serial.print(yawpitchroll[1]*radtodeg);
//    Serial.print("   Roll = ");
//    Serial.print(yawpitchroll[2]*radtodeg);
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
  Serial.print(ref_pitch);
  Serial.print("  Ref roll=  ");
  Serial.print(ref_roll);
  Serial.print("  Ref vertical speed=  ");
  Serial.print(ref_altitude_rate);
  Serial.print("  Ref yaw rate=  ");
  Serial.print(ref_yaw_rate);
//  Serial.print("  Yaw raw=  ");
//  Serial.print(Yaw.raw);
//  Serial.print("  Yaw=  ");
//  Serial.print(Yaw.value);
//  Serial.print("  Yaw rate=  ");
//  Serial.print(Yaw.derivative);
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
  //Dist_sensor.initialize();
}

// -----------------------------------------------------------------------------
// ---------------------------------- Main Loop --------------------------------
// -----------------------------------------------------------------------------

void loop() {
  tiempo = millis();
  delta_t = tiempo - tiempo0;
  tiempo0 = tiempo;
  switch (curr_state) {
  case STANDBY:
    Serial.print("STANDBY   ");
    RadioCOM.radiolisten(control);
    read_sensors();
    RadioCOM.radiosend(yawpitchroll);
    curr_state = next_state();
    break;

  case CALIBRATION:
    Serial.print("CALIBRATION   ");
    RadioCOM.radiolisten(control);
    read_sensors();
    RadioCOM.radiosend(yawpitchroll);
    // acelerometro.acelerometro_cal();
    // acelerometro.magnetometro_cal();
    // acelerometro.gyroscope_cal();
    curr_state = next_state();
    break;

  case FLYMODE:
    Serial.print("FLYMODE   ");
    RadioCOM.radiolisten(control);
    read_sensors();
    RadioCOM.radiosend(yawpitchroll);
    curr_state = next_state();
    break;
  case ABORT:
    Serial.print("ABORT  ");
    RadioCOM.radiolisten(control);
    read_sensors();
    RadioCOM.radiosend(yawpitchroll);
    curr_state = next_state();
    break;
  }
  Reference();
  Print_data();

}
