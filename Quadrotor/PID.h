#ifndef PID_H_
#define PID_H_


struct PIDparameters
{
  float Kp, Kd, Ki, saturation, alpha;
};

class PID {
  private:
    float error_n_1, derivative_n_1, dt = 10;
  public:
    float error;
    float derivative;
    float integral;
    PIDparameters parameters;
    bool integral_saturation;

    void init(float Kp, float Kd, float Ki, float saturation);
    float pid_step(const float state, const float reference, const int delta_t);
};

#endif /* PID_H_*/
