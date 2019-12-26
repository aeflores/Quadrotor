
#ifndef TRIAD_H_
#define TRIAD_H_

float *get_ypr_triad(float Acc_raw_val[],float (&yawpitchroll)[3]);
float *Integrator(float Acc_raw_val[], float yawpitchroll[], float (&yawpitchroll_integrator)[3]);
float *filter(float Acc_raw_val[], float yawpitchroll_triad[],
              float yawpitchroll_int[], float delta_t, float (&yawpitchroll)[3]);


#endif TRIAD_H_