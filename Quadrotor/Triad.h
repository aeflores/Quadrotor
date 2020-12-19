
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
    void get_ypr_triad(const float Acc_raw_val[3][3],Attitude &yawpitchroll);
    void integrator(const float gyro[3], const Attitude &yawpitchroll, float delta_t, Attitude &yawpitchroll_integrator);
    void filter(const Attitude yawpitchroll_triad, const Attitude &yawpitchroll_int, Attitude &yawpitchroll);

};

#endif /* TRIAD_H_ */
