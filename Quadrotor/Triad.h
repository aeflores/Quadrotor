
#ifndef TRIAD_H_
#define TRIAD_H_

struct Attitude{
    float yaw;
    float pitch;
    float roll;
};

void get_ypr_triad(float Acc_raw_val[],Attitude &yawpitchroll);
void Integrator(float Acc_raw_val[], Attitude &yawpitchroll, Attitude &yawpitchroll_integrator);
void filter(float Acc_raw_val[], const Attitude &yawpitchroll_triad,
              const Attitude &yawpitchroll_int, float delta_t, Attitude &yawpitchroll);


#endif /* TRIAD_H_ */
