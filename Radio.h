#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>


#ifndef RADIO_H_
#define RADIO_H_

//Declaremos los pines CE y el CSN
#define CE_PIN 7
#define CSN_PIN 8

class Radio{
  private:
  RF24 radio;
  //Variable con la direccion del canal por donde se va a transmitir
  const uint64_t pipe_read = 0xE8E8F0F0E1LL;
  const uint64_t pipe_send = 0xF0F0F0F0D2LL;

  public:
    Radio():radio(CE_PIN,CSN_PIN){};
    void initialize();
    void radiosend(float datos[]);
    float* radiolisten();
};

#endif /* RADIO_H */
