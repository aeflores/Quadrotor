//-------------------------------------------------------------//
//---------------------QUADROTOR CODE -------------------------//
//-------------------------------------------------------------//
//Authors: Enrique Flores and Antonio Flores

#include "Acelerometro.h"
#include "Radio.h"
#include "Distancia.h"

Acelerometro acelerometro;
Radio RadioCOM;
Distancia Dist_sensor;

// unsigned long tiempo;
float radtodeg = 180 / acos(-1);
int first_iteration = 0;

float *Acc_raw_val;
float *yawpitchroll_int;
float *yawpitchroll_triad;
float *yawpitchroll;
int *control;
double altura;

// possible states
enum state { STANDBY = 0, CALIBRATION = 1, FLYMODE = 2, ABORT = 3 };
// current state initialized to STANDBY
state curr_state = STANDBY;

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
  
  // Definicion del angulo de guiñada
  if (yawpitchroll[0]<=0){
    yawpitchroll[0]=2*acos(-1.)+yawpitchroll[0];    
  }

  
  altura=Dist_sensor.update_distance();
  // Transformamos la altura medida en altura con respecto al suelo
  altura = altura*cos(yawpitchroll[1])*cos(yawpitchroll[2]);
}

void print_control() {
  Serial.print("JSRX= ");
  Serial.print(control[0]);
  Serial.print("  JSRY= ");
  Serial.print(control[1]);
  Serial.print("  JSLX= ");
  Serial.print(control[2]);
  Serial.print("  JSLY= ");
  Serial.print(control[3]);
  Serial.print("  state= ");
  Serial.print(control[4]);
  Serial.print("  Altura  ");
  Serial.print(altura);
  Serial.print("  Delta Time  ");
  Serial.println(millis());
}

void setup() {
  Serial.begin(115200);
  // start communication with IMU
  acelerometro.initialize();
  acelerometro.default_cal();
  acelerometro.settings();
  RadioCOM.initialize();
  Dist_sensor.initialize();
  // acelerometro.acelerometro_cal();
  // acelerometro.magnetometro_cal();
  // acelerometro.gyroscope_cal();
}

void loop() {
  switch (curr_state) {
  case STANDBY:
    //Serial.print("STANDBY   ");
    control = RadioCOM.radiolisten();
    read_sensors();
    RadioCOM.radiosend(yawpitchroll);
    curr_state = next_state();
    break;

  case CALIBRATION:
    //Serial.print("CALIBRATION   ");
    control = RadioCOM.radiolisten();
    read_sensors();
    RadioCOM.radiosend(yawpitchroll);
    // acelerometro.acelerometro_cal();
    // acelerometro.magnetometro_cal();
    // acelerometro.gyroscope_cal();
    curr_state = next_state();
    break;

  case FLYMODE:
    //Serial.print("FLYMODE   ");
    control = RadioCOM.radiolisten();
    read_sensors();
    RadioCOM.radiosend(yawpitchroll);
    curr_state = next_state();
    break;
  case ABORT:
    //Serial.print("ABORT MODE  ");
    control = RadioCOM.radiolisten();
    read_sensors();
    RadioCOM.radiosend(yawpitchroll);
    curr_state = next_state();
    break;
  }
  print_control();

}
