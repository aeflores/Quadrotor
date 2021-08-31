#include "EngineControl.h"


void EngineControl::init() {
  for (int i = 0; i < 4; i++) {
    engine[i].attach(engine_port[i]);
    engine[i].writeMicroseconds(engine_speed[i]);
  }
  pitchPID.init(0.12, 0.2, 0.0, 5, 10, 100);
  rollPID.init(0.12, 0.2, 0.0, 5, 10, 100);
  yawratePID.init(1.0, 0.0, 0., 10, 10,  20);
  heightPID.init(0.04, 0.12, 0.002, 5, 5,  200);

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

  pitchPID.init(conf.PIDattitude_Kp,       conf.PIDattitude_Kd,     conf.PIDattitude_Ki, pitchPID.parameters.error_fc,     pitchPID.parameters.error_dot_fc,     pitchPID.parameters.saturation);
  rollPID.init( conf.PIDattitude_Kp,       conf.PIDattitude_Kd,     conf.PIDattitude_Ki, rollPID.parameters.error_fc,      rollPID.parameters.error_dot_fc,      rollPID.parameters.saturation);
  heightPID.init(conf.PIDheight_Kp,     conf.PIDheight_Kd,    conf.PIDheight_Ki,    conf.PIDheight_error_fc,    conf.PIDheight_error_dot_fc,    heightPID.parameters.saturation);
  //yawratePID.init(conf.PIDyawrate_Kp,   conf.PIDyawrate_Kd,   conf.PIDyawrate_Ki,   conf.PIDyawrate_error_fc,   conf.PIDyawrate_error_dot_fc,   conf.PIDyawrate_saturation);

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
  //float unbalance_yaw_rate = yawratePID.pid_step(state.yaw_rate, reference.yaw_rate, delta_t);
  float power              = 400 +  heightPID.pid_step(state.height, reference.height, delta_t);
  //power                    = control[4];

  engine_speed[0] = max(int(MIN_SPEED + power + unbalance_pitch - unbalance_roll), MIN_SPEED);
  engine_speed[1] = max(int(MIN_SPEED + power - unbalance_pitch - unbalance_roll), MIN_SPEED);
  engine_speed[2] = max(int(MIN_SPEED + power - unbalance_pitch + unbalance_roll), MIN_SPEED);
  engine_speed[3] = max(int(MIN_SPEED + power + unbalance_pitch + unbalance_roll), MIN_SPEED);

}

void EngineControl::stop() {
  for (int i = 0; i < 4; i++) {
    engine_speed[i] = MIN_SPEED;
  }
}



//  Serial.print(heightPID.error);
//  Serial.print("\t");
//  Serial.print(heightPID.derivative);
//  Serial.print("\t");

//  Serial.print(pitchPID.error);
//  Serial.print("\t");
//  Serial.print(pitchPID.derivative);
//  Serial.print("\t");
//  Serial.print(rollPID.error);
//  Serial.print("\t");
//  Serial.print(rollPID.derivative);
//  Serial.print("\t");

//  Serial.print(pitchPID.parameters.Kp);
//  Serial.print("\t");
//  Serial.print(pitchPID.parameters.Kd);
//  Serial.print("\t");
//  Serial.print(pitchPID.parameters.Ki);
//  Serial.print("\t");
//  Serial.print(pitchPID.parameters.error_fc);
//  Serial.print("\t");
//  Serial.print(pitchPID.parameters.error_dot_fc);
//  Serial.print("\t");
//  Serial.print(pitchPID.parameters.saturation);
//  Serial.print("\t");
//  Serial.print("\t");
//  Serial.print(rollPID.parameters.Kp);
//  Serial.print("\t");
//  Serial.print(rollPID.parameters.Kd);
//  Serial.print("\t");
//  Serial.print(rollPID.parameters.Ki);
//  Serial.print("\t");
//  Serial.print(rollPID.parameters.error_fc);
//  Serial.print("\t");
//  Serial.print(rollPID.parameters.error_dot_fc);
//  Serial.print("\t");
//  Serial.print(rollPID.parameters.saturation);
//  Serial.print("\t");
//  Serial.print("\t");
//  Serial.print(heightPID.parameters.Kp);
//  Serial.print("\t");
//  Serial.print(heightPID.parameters.Kd);
//  Serial.print("\t");
//  Serial.print(heightPID.parameters.Ki);
//  Serial.print("\t");
//  Serial.print(heightPID.parameters.error_fc);
//  Serial.print("\t");
//  Serial.print(heightPID.parameters.error_dot_fc);
//  Serial.print("\t");
//  Serial.print(heightPID.parameters.saturation);
//  Serial.print("\t");


//
//      Serial.print(control[0]);
//      Serial.print("\t");
//      Serial.print(control[1]);
//      Serial.print("\t");
//      Serial.print(control[2]);
//      Serial.print("\t");
//      Serial.print(control[3]);
//      Serial.println("\t");

//      Serial.print(1000);
//      Serial.print("\t");
//      Serial.print(2000);
//      Serial.print("\t");
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
