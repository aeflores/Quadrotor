#ifndef PID_H_
#define PID_H_


struct PIDparameters
{
  float Kp, Kd, Ki, error_fc, error_dot_fc, saturation;
};

class PID {
  private:
    float error_n_1, derivative_n_1;
  public:
    float error;
    float derivative;
    float integral = 0;
    PIDparameters parameters;
    bool integral_saturation;

    void init(float Kp, float Kd, float Ki, float error_fc, float error_dot_fc, float saturation);
    float pid_step(const float state, const float reference, const int delta_t);
};

#endif /* PID_H_*/
