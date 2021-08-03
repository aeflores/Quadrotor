#include "EngineControl.h"


void EngineControl::init() {
  for (int i = 0; i < 4; i++) {
    engine[i].attach(engine_port[i]);
    engine[i].writeMicroseconds(engine_speed[i]);
  }
  pitchPID.init(2.0, 2.0, 0.5, 100);
  rollPID.init(2.0, 2.0, 0.5, 100);
  yawratePID.init(5.0, 0.0, 0., 20);
  heightPID.init(2.0, 0.5, 2, 700);

}

void EngineControl::updateEngines() {
  for (int i = 0; i < 4; i++) {
    engine[i].writeMicroseconds(engine_speed[i]);
  }
}

void EngineControl::testControl(const int control[5]) {
  for (int i = 0; i < 4; i++) {
    engine_speed[i] = (control[i] - 512) * 2 + 1000;
    if (engine_speed[i] < MIN_SPEED)
      engine_speed[i] = MIN_SPEED;
    if (engine_speed[i] > MAX_SPEED)
      engine_speed[i] = MAX_SPEED;
  }
}

void EngineControl::configure(ControllerConfiguration &conf) {
  upperUnbalanceRange             = conf.upperUnbalanceRange;
  lowerUnbalanceRange             = conf.lowerUnbalanceRange;
  error2CorrectionCoeff           = conf.error2CorrectionCoeff;
  derivativeError2CorrectionCoeff = conf.derivativeError2CorrectionCoeff;
  feedforwardunbalance14          = conf.feedforwardunbalance14;
  feedforwardunbalance23          = conf.feedforwardunbalance23;
}

void EngineControl::computeReference(const int control[5], QuadState &reference) {
  // control[0] --> RV --> height_rate --> 517
  // control[1] --> RH --> yaw_rate --> 518
  // control[2] --> LV --> pitch --> 514
  // control[3] --> LH --> yaw --> 502
  // control[4] --> VR --> power
  // Calculating state references from the joystick data
  reference.pitch     = -(-float(control[2]) + 515.) / 1024.*20.;
  reference.roll      = (-float(control[3]) + 502.) / 1024.*20.;
  reference.height    = control[4]; // for testing
  reference.yaw_rate  = (-float(control[1]) + 518.) / 1024.*10.;

}


void EngineControl::pdControl(const int control[5], const Euler &yawpitchroll, const QuadState &state, const int delta_t) {

  computeReference(control, reference);


  float unbalance_pitch    = pitchPID.pid_step(state.pitch, reference.pitch, delta_t);
  float unbalance_roll     = rollPID.pid_step(state.roll, reference.roll, delta_t);
  float unbalance_yaw_rate = yawratePID.pid_step(state.yaw_rate, reference.yaw_rate, delta_t);
  //float power              = heightPID.pid_step(state.height, reference.height, delta_t);
  float power              = control[4];

  

  engine_speed[0] = max(int(MIN_SPEED + power + unbalance_pitch - unbalance_roll), MIN_SPEED);
  engine_speed[1] = max(int(MIN_SPEED + power - unbalance_pitch - unbalance_roll), MIN_SPEED);
  engine_speed[2] = max(int(MIN_SPEED + power - unbalance_pitch + unbalance_roll), MIN_SPEED);
  engine_speed[3] = max(int(MIN_SPEED + power + unbalance_pitch + unbalance_roll), MIN_SPEED);
  //
  //    Serial.print(control[0]);
  //    Serial.print("\t");
  //    Serial.print(control[1]);
  //    Serial.print("\t");
  //    Serial.print(control[2]);
  //    Serial.print("\t");
  //    Serial.print(control[3]);
  //    Serial.println("\t");

  
//      Serial.print(engine_speed[0]);
//      Serial.print("\t");
//      Serial.print(engine_speed[1]);
//      Serial.print("\t");
//      Serial.print(engine_speed[2]);
//      Serial.print("\t");
//      Serial.print(engine_speed[3]);
//      Serial.println("\t");


//      Serial.print(unbalance_pitch);
//      Serial.print("\t");
//      Serial.print(unbalance_roll);
//      Serial.print("\t");
//      Serial.print(unbalance_yaw_rate);
//      Serial.print("\t");
//      Serial.print(power);
//      Serial.println("\t");


  //
  //    float old_error_pitch, old_error_roll, old_derivative_error_pitch, old_derivative_error_roll;
  //    // save old values
  //    old_error_pitch = error_pitch;
  //    old_error_roll = error_roll;
  //
  //    old_derivative_error_pitch = derivative_error_pitch;
  //    old_derivative_error_roll = derivative_error_roll;
  //
  //    float derivative_pitch_filter_coeff = 0.2;
  //    float derivative_roll_filter_coeff = 0.2;
  //
  //    float pitch_filter_coeff = 0.8;
  //    float roll_filter_coeff = 0.05;
  //
  //    // compute new values
  //    error_pitch = reference.pitch - yawpitchroll.pitch_deg();
  //    error_roll = reference.roll - yawpitchroll.roll_deg();
  //    // Filtering errors
  //    error_pitch = pitch_filter_coeff*error_pitch + (1-pitch_filter_coeff)*old_error_pitch;
  //    error_roll = roll_filter_coeff*error_roll + (1-roll_filter_coeff)*old_error_roll;
  //
  //    // compute derivative
  //    derivative_error_pitch = (error_pitch - old_error_pitch) *1000000 / delta_t;
  //    derivative_error_roll = (error_roll - old_error_roll) *1000000 / delta_t;
  //    // Filtering error derivatives
  //    derivative_error_pitch = derivative_pitch_filter_coeff *derivative_error_pitch + (1 - derivative_pitch_filter_coeff)*old_derivative_error_pitch;
  //    derivative_error_roll = derivative_roll_filter_coeff *derivative_error_roll + (1 - derivative_roll_filter_coeff)*old_derivative_error_roll;
  //    // compute power based on error
  //    int unbalance_pitch=error2Correction(error_pitch, derivative_error_pitch);
  //    int unbalance_roll=error2Correction(error_roll, derivative_error_roll);

}

void EngineControl::stop() {
  for (int i = 0; i < 4; i++) {
    engine_speed[i] = MIN_SPEED;
  }
}
