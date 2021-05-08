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
    float qoffset[4] = {1, 0, 0, 0};
    float alpha = 0.95;

  public:
  void initial_cond(Euler &yawpitchroll);
  void triad_algorithm(const int Acc[3], const int Mag[3],  float q[4]);
  void get_attitude(const int imusensor[3][3], Euler& yawpitchroll, int delta_t);

};

#endif /* ATTITUDE_H_ */
