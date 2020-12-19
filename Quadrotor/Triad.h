
#ifndef TRIAD_H_
#define TRIAD_H_

struct Attitude{
    float yaw;
    float pitch;
    float roll;
};

class Triad{
  private:
    float B[3][3];
  public:
    void initialize();
    void get_ypr_triad(float Acc_raw_val[],Attitude &yawpitchroll);
    void integrator(float Acc_raw_val[], const Attitude &yawpitchroll, float delta_t, Attitude &yawpitchroll_integrator);
    void filter(const Attitude yawpitchroll_triad, const Attitude &yawpitchroll_int, Attitude &yawpitchroll);

};

#endif /* TRIAD_H_ */
