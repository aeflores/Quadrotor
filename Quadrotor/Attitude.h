#ifndef ATTITUDE_H_
#define ATTITUDE_H_

struct Euler{
    float yaw;
    float pitch;
    float roll;

    float yaw_deg() const;
    float pitch_deg() const;
    float roll_deg() const;
};


class Attitude{
  private:
    float B[3][3];
    float q0[4], q1[0];
    float qoffset[4] = {0.6412,    0.4425,   -0.5160,   -0.3561};

  public:
  void initial_cond();
  void triad_algorithm(const float Acc[3], const float Mag[3],  float q[4]);
  void get_attitude(const float Acc_raw_val[3][3], Euler& yawpitchroll, float delta_t);

};

#endif /* ATTITUDE_H_ */
