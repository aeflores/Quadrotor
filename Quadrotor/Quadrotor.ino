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
float *Acc_raw_val; // Raw IMU measurements accelerometer, magnetometer and gyroscope
float *yawpitchroll_int; // Yaw pitch roll angles from integrator
float *yawpitchroll_triad; // Yaw pitch roll angles from Triad algorithm
float *yawpitchroll; // Filtered yaw pitch roll angles
int *control; // Control telecomands
float ref_pitch, ref_roll, ref_yaw_rate, ref_altitude_rate;
Signal Yaw(0.98,0.05);
Signal Height(0.1, 0.5);



// Constants
float radtodeg = 180 / acos(-1); // Radtodeg conversion
int first_iteration = 0;
unsigned long tiempo, tiempo0=0;
int delta_t;





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
  Acc_raw_val = acelerometro.get_raw_val();
  // Determinacion de los angulos de Euler utilizando el algoritmo TRIAD
  yawpitchroll_triad = get_ypr_triad(Acc_raw_val);

  // Determinación de los angulos de Euler utilizando el integrador
  if (first_iteration <= 20){
    yawpitchroll_int = Integrator(Acc_raw_val, yawpitchroll_triad);
    first_iteration++;
  } else {
    yawpitchroll_int = Integrator(Acc_raw_val, yawpitchroll_int);
  }
  //Aplicacion del filtro que combina ambas medidas
  yawpitchroll = filter(Acc_raw_val, yawpitchroll_triad, yawpitchroll_int);
  
  Yaw.update(yawpitchroll[0], delta_t);
  Height.update(Dist_sensor.update_distance(),delta_t);
  // Lectura del dato de altura  
  //Height[0]=Dist_sensor.update_distance();
  // Transformamos la altura medida en altura con respecto al suelo
  //Height[0]=Height[0]*cos(yawpitchroll[1])*cos(yawpitchroll[2]);
  //Filtering signals
  //Height[1]=LPF(Height[0],Height[3], Height[1]);  



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
//  Serial.print("JSRX= ");
//  Serial.print(control[0]);
//  Serial.print("  JSRY= ");
//  Serial.print(control[1]);
//  Serial.print("  JSLX= ");
//  Serial.print(control[2]);
//  Serial.print("  JSLY= ");
//  Serial.print(control[3]);
//  Serial.print("  state= ");
//  Serial.print(control[4]);
  Serial.print("  Delta Time  ");
  Serial.println(millis());
  Serial.print("  Ref pitch=  ");
  Serial.print(ref_pitch);
  Serial.print("  Ref roll=  ");
  Serial.print(ref_roll);
  Serial.print("  Yaw=  ");
  Serial.print(Yaw.value);
  Serial.print("  Yaw rate=  ");
  Serial.print(Yaw.derivative);
//  Serial.print("  Height =  ");
//  Serial.print(Height[1]);
//  Serial.print("  Height raw = ");
//  Serial.print(Height[0]);
//  Serial.print(" Height rate =  ");
//  Serial.print(Height_rate[0]);
//  Serial.print(" Height rate =  ");
//  Serial.print(Height_rate[1]);
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
  Dist_sensor.initialize();
}

// -----------------------------------------------------------------------------
// ---------------------------------- Main Loop --------------------------------
// -----------------------------------------------------------------------------

void loop() {
  switch (curr_state) {
  case STANDBY:
    Serial.print("STANDBY   ");
    control = RadioCOM.radiolisten();
    read_sensors();
    RadioCOM.radiosend(yawpitchroll);
    curr_state = next_state();
    break;

  case CALIBRATION:
    Serial.print("CALIBRATION   ");
    control = RadioCOM.radiolisten();
    read_sensors();
    RadioCOM.radiosend(yawpitchroll);
    // acelerometro.acelerometro_cal();
    // acelerometro.magnetometro_cal();
    // acelerometro.gyroscope_cal();
    curr_state = next_state();
    break;

  case FLYMODE:
    Serial.print("FLYMODE   ");
    control = RadioCOM.radiolisten();
    read_sensors();
    RadioCOM.radiosend(yawpitchroll);
    curr_state = next_state();
    break;
  case ABORT:
    Serial.print("ABORT  ");
    control = RadioCOM.radiolisten();
    read_sensors();
    RadioCOM.radiosend(yawpitchroll);
    curr_state = next_state();
    break;
  }
  Reference();
  Print_data();

}
