

#ifndef RADIO_H_
#define RADIO_H_

#include <RF24.h>
#include "Attitude.h"
#include "EngineControl.h"
#include "TransmitData.h"

// Declaremos los pines CE y el CSN
#define CE_PIN 7
#define CSN_PIN 8

class Radio {
private:
  RF24 radio;
  // Variable con la direccion del canal por donde se va a transmitir
  const uint64_t pipe_read = 0xE8E8F0F0E1LL;
  const uint64_t pipe_send = 0xF0F0F0F0D2LL;
  void radiolisten(void* data, int numBytes);
public:
  Radio() : radio(CE_PIN, CSN_PIN){};
  void initialize();
  void radiosend(const State curr_state, const QuadState &datos, const EngineControl &engines, int delta_t);
  void finishSend();
  void radiolisten(ControlData &datos,ControllerConfiguration& conf);
};

#endif /* RADIO_H */
