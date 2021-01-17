//-------------------------------------------------------------//
//---------------------QUADROTOR CODE -------------------------//
//-------------------------------------------------------------//
//Authors: Enrique Flores and Antonio Flores


// Including libraries
#include "Acelerometro.h"
#include "Attitude.h"
#include "EngineControl.h"
#include "Radio.h"
// #include "Distancia.h"
// #include "Signal.h"
#include "blackbox.h"


// Defining objects
Acelerometro acelerometro;
EngineControl engines;
Radio RadioCOM;
Attitude attitude;
// Distancia Dist_sensor;
// blackbox mySDbb;

// Global variables
float Acc_raw_val[3][3]; // Raw IMU measurements accelerometer, magnetometer and gyroscope
Euler yawpitchroll;
ControlData control; // Control telecomands
// Signal Yaw(0.90,0.75);
// Signal Height(0.05, 0.9);

unsigned long tiempo0;
int delta_t;


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
  acelerometro.get_raw_val(Acc_raw_val);
  // Height sensor
  //  Height.update(Dist_sensor.update_distance()*cos(yawpitchroll.pitch)*cos(yawpitchroll.roll), delta_t);
  // Lectura del dato de altura
  // Height.update(Dist_sensor.update_distance(), delta_t);
  //Height[0]=Dist_sensor.update_distance();
  // Transformamos la altura medida en altura con respecto al suelo
  //Height[0]=Height[0]*cos(yawpitchroll[1])*cos(yawpitchroll[2]);
}

void Print_data() {
//      Serial.print("  dt = ");
//      Serial.print(delta_t/1e3 , 3);
//      Serial.print("  Accc_x = ");
//      Serial.print(Acc_raw_val[0][0]);
//      Serial.print("  Acc_y = ");
//      Serial.print(Acc_raw_val[0][1]);
//      Serial.print("  Acc_z = ");
//      Serial.print(Acc_raw_val[0][2]);


  //    Serial.print("  Gyro_x = ");
  //    Serial.print(Acc_raw_val[1][0], 5);
  //    Serial.print("  Gyro_y = ");
  //    Serial.print(Acc_raw_val[1][1], 5);
  //    Serial.print("  Gyro_z = ");
  //    Serial.print(Acc_raw_val[1][2], 5);

//      Serial.print("  Mag_x = ");
//      Serial.print(Acc_raw_val[2][0],3);
//      Serial.print("  Mag_y = ");
//      Serial.print(Acc_raw_val[2][1],3);
//      Serial.print("  Mag_z = ");
//      Serial.print(Acc_raw_val[2][2],3);
//
      Serial.print("  Yaw = ");
      Serial.print(yawpitchroll.yaw_deg());
      Serial.print("   Pitch = ");
      Serial.print(yawpitchroll.pitch_deg());
      Serial.print("   Roll = ");
      Serial.print(yawpitchroll.roll_deg());
  //
  //    Serial.print("   Yaw = ");
  //    Serial.print(yawpitchroll_int.yaw*radtodeg);
//  Serial.print("   Pitch = ");
//  Serial.print(yawpitchroll_int.pitch * radtodeg);
//  Serial.print("   Roll = ");
//  Serial.print(yawpitchroll_int.roll * radtodeg);

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

}

// -----------------------------------------------------------------------------
// ---------------------------------- Inicialization ---------------------------
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  // start communication with IMU
  acelerometro.initialize();
  // acelerometro.acelerometro_cal();
  // acelerometro.magnetometro_cal();
  // acelerometro.gyroscope_cal();
  acelerometro.default_cal();
  acelerometro.settings();
  RadioCOM.initialize();
  engines.init();
  //  Dist_sensor.initialize();
  attitude.initial_cond();
//   mySDbb.init();


}

// -----------------------------------------------------------------------------
// ---------------------------------- Main Loop --------------------------------
// -----------------------------------------------------------------------------
short cycle_counter;
ControllerConfiguration configuration;

void loop() {
  // La radio transmite y recibe cada 15 ciclos
  cycle_counter = (cycle_counter + 1) % 15;
  delta_t = micros() - tiempo0;
  tiempo0 += delta_t;
  // En el ciclo 14 recibe datos
  if (cycle_counter == 14) {
    RadioCOM.radiolisten(control, configuration);
    curr_state = next_state(curr_state, control.change);
  }
  read_sensors();
  attitude.get_attitude(Acc_raw_val, yawpitchroll, delta_t/1e6);

  switch (curr_state) {
    case STANDBY:
      Serial.print("STANDBY   ");
      engines.stop();
      break;
    case CALIBRATION:
      {
        Serial.print("CALIBRATION   ");
        engines.stop();
        engines.configure(configuration);
        // acelerometro.acelerometro_cal();
        // acelerometro.magnetometro_cal();
        // acelerometro.gyroscope_cal();
        break;
      }
    case FLYMODE:
      Serial.print("FLYMODE   ");
      if (yawpitchroll.pitch_deg() > 25 ||  yawpitchroll.pitch_deg() < -25 || yawpitchroll.roll_deg() > 25 || yawpitchroll.roll_deg() < -25)
        curr_state = ABORT;
      else
        engines.pdControl(control.movement, yawpitchroll, delta_t);
      break;
    case ABORT:
      Serial.print("ABORT  ");
      engines.stop();
      break;
  }
  
  // En el ciclo 1 empieza el envio
  if (cycle_counter == 1) {
    RadioCOM.radiosend(curr_state, yawpitchroll, engines, delta_t);
  }
  // En el ciclo 7 termina el envio
  // y empieza a escuchar hasta el ciclo 14
  if (cycle_counter == 7) {
    RadioCOM.finishSend();
  }
  
  Print_data();
  engines.updateEngines();
  Serial.println(" ");
}
