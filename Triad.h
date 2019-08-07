float *get_ypr_triad(float Acc_raw_val[]);
float *Integrator(float Acc_raw_val[], float yawpitchroll[]);
float *filter(float Acc_raw_val[], float yawpitchroll_triad[],
              float yawpitchroll_int[]);
