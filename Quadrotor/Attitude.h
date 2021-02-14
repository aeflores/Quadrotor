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
    float q0[4], q1[4];
    float qoffset[4] = {0.9986, 0.0355, -0.0389, -0.0014};
    float alpha = 0.95;

  public:
  void initial_cond();
  void triad_algorithm(const float Acc[3], const float Mag[3],  float q[4]);
  void get_attitude(const float Acc_raw_val[3][3], Euler& yawpitchroll, float delta_t);

};

#endif /* ATTITUDE_H_ */
