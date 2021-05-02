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
    float output;


    void init(float Kp, float Kd, float Ki);
    void get_output(float error, float derivative, float integral, PIDparameters& parameters, float output);
    void pid_step(const float state, const float reference, float output, int delta_t);
};

#endif /* PID_H_*/
