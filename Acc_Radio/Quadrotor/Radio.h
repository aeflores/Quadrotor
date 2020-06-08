#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>

class Radio{
  private:
  //Variable con la direccion del canal por donde se va a transmitir
  const uint64_t pipe_read = 0xE8E8F0F0E1LL;
  const uint64_t pipe_send = 0xF0F0F0F0D2LL;

  public:
    void initialize();
    void radiosend(float datos[]);
    float* radiolisten();
};

