
#include "Acelerometro.h"
#include "Radio.h"

Acelerometro acelerometro(Wire);
Radio RadioCOM;

// unsigned long tiempo;
float radtodeg = 180 / acos(-1);
int first_iteration = 0;

float *Acc_raw_val;
float *yawpitchroll_int;
float *yawpitchroll_triad;
float *yawpitchroll;
float *control;

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
  Acc_raw_val = acelerometro.get_raw_val();
  yawpitchroll_triad = get_ypr_triad(Acc_raw_val);
  if (first_iteration <= 10) {
    yawpitchroll_int = Integrator(Acc_raw_val, yawpitchroll_triad);
    first_iteration++;
  } else {
    yawpitchroll_int = Integrator(Acc_raw_val, yawpitchroll_int);
  }
  yawpitchroll = filter(Acc_raw_val, yawpitchroll_triad, yawpitchroll_int);
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
  // acelerometro.acelerometro_cal();
  // acelerometro.magnetometro_cal();
  // acelerometro.gyroscope_cal();
}

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
    // acelerometro.acelerometro_cal();
    // acelerometro.magnetometro_cal();
    // acelerometro.gyroscope_cal();
    curr_state = next_state();
    Serial.println(curr_state);
    break;

  case FLYMODE:
    Serial.print("FLYMODE   ");
    control = RadioCOM.radiolisten();
    read_sensors();
    RadioCOM.radiosend(yawpitchroll);
    curr_state = next_state();
    break;
  case ABORT:
    Serial.println("ABORT MODE");
    control = RadioCOM.radiolisten();
    curr_state = next_state();
    break;
  }
  print_control();

  //  Serial.print("Yaw: ");
  //  Serial.print(yawpitchroll_triad[0]*radtodeg,3);
  //  Serial.print("  ");
  //  Serial.print("Pitch: ");
  //  Serial.print(yawpitchroll_triad[1]*radtodeg,3);
  //  Serial.print("  ");
  //  Serial.print("Roll: ");
  //  Serial.print(yawpitchroll_triad[2]*radtodeg,3);
}
