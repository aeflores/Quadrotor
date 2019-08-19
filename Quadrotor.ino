
#include "Acelerometro.h"
#include "Radio.h"

Acelerometro acelerometro(Wire);
Radio RadioCOM;

//unsigned long tiempo;
float radtodeg = 180 / acos(-1);
int first_iteration = 0;

float *Acc_raw_val;
float *yawpitchroll_int;
float *yawpitchroll_triad;
float *yawpitchroll;
float *control;

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
  //Envio de telemetria por radio
  RadioCOM.radiosend(yawpitchroll);
  // Recepcion de Telecomandos por radio
  control = RadioCOM.radiolisten();
  //
  Serial.print("JSRX= " );
  Serial.print(control[0]);
  Serial.print("  JSRY= " );
  Serial.print(control[1]);
  Serial.print("  JSLX= " );
  Serial.print(control[2]);
  Serial.print("  JSLY= " );
  Serial.print(control[3]);
  Serial.print("  Delta Time  ");
  Serial.println(millis());


//  Serial.print("Yaw: ");
//  Serial.print(yawpitchroll_triad[0]*radtodeg,3);
//  Serial.print("  ");
//  Serial.print("Pitch: ");
//  Serial.print(yawpitchroll_triad[1]*radtodeg,3);
//  Serial.print("  ");
//  Serial.print("Roll: ");
//  Serial.print(yawpitchroll_triad[2]*radtodeg,3);

}
