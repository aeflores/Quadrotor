
#include "Radio.h"
#include "TransmitData.h"

void Radio::initialize() {
  radio.begin();
  radio.setDataRate(RF24_2MBPS);
  // radio.setRetries(15,15);
  // radio.setPayloadSize(8);
  // Abrimos un canal de escritura
  // radio.openWritingPipe(direccion);
  radio.openWritingPipe(pipe_send);
  // radio.openReadingPipe(1,direccion);
  radio.openReadingPipe(1, pipe_read);
  radio.startListening();
}

void Radio::radiosend(const State curr_state, const QuadState& datos, const EngineControl &engines, int delta_t) {
  // EMISION DE DATOS
  TransmitData data;
  data.state        = curr_state;
  data.delta_t      = delta_t;
  data.quadstate[0] = datos.yaw_rate;
  data.quadstate[1] = datos.pitch;
  data.quadstate[2] = datos.roll;
  data.quadstate[3] = datos.height;
  
  for (int i=0; i<4 ; i++){
    data.engines[i]= engines.engine_speed[i];
  }
  data.errors[0] = engines.pitchPID.error;
  data.errors[1] = engines.rollPID.error;

  radio.stopListening();
  radio.startWrite(&data, sizeof(data), false);
}

void Radio::finishSend(){
  radio.txStandBy();
  radio.startListening();
}

void Radio::radiolisten(void *data, int numBytes) {
  // RECEPCION DE DATOS
  // Empezamos a escuchar por el canal
  //radio.startListening();
  unsigned long started_waiting_at = millis();
  bool timeout = false;
  while (!radio.available() && !timeout) { // Esperamos 10ms
    if (millis() - started_waiting_at > 5)
      timeout = true;
  }
  if (timeout) {
//    Serial.println("Error, No ha habido respuesta a tiempo");
  } else {
    // Leemos los datos y los guardamos en la variable datos[]
    radio.read(data, numBytes);
  }
  //radio.stopListening();
}

void Radio::radiolisten(ControlData& data, ControllerConfiguration &conf) {
  radiolisten(&data, sizeof(data));
  if(data.moreData)
    radiolisten(&conf, sizeof(conf));
}
