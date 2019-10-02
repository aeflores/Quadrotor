// Vcc-->3.3V
// Gnd-->Gnd
// SDA--> A4
// SCL--> A5

class Acelerometro {
  private:
    int status = -1;
  public:
    void magnetometro_cal();
    void acelerometro_cal();
    void gyroscope_cal();
    void initialize();
    void default_cal();
    void settings();
    float *get_raw_val();
};

