
#include "Radio.h"
#include "TransmitData.h"

void Radio::initialize() {
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(5,5);
  radio.stopListening();
  radio.openWritingPipe(pipe_send);
}

void Radio::radiosend(const State curr_state, const QuadState& datos, const EngineControl &engines, int delta_t) {
  // EMISION DE DATOS
  TransmitData data;
  data.state        = curr_state;
  data.delta_t      = delta_t;
  data.quadstate[0] = int(datos.yaw_rate*100);
  data.quadstate[1] = int(datos.pitch*100);
  data.quadstate[2] = int(datos.roll*100);
  data.quadstate[3] = int(datos.height);
  
  for (int i=0; i<4 ; i++){
    data.engines[i]= engines.engine_speed[i];
  }
  data.errors[0] = int(engines.pitchPID.error*100);
  data.errors[1] = int(engines.rollPID.error*100);

  
  radio.startWrite(&data, sizeof(data),false);
}

void Radio::finishSend(ControlData& data){
  radio.txStandBy();
  if(radio.isAckPayloadAvailable()){
    radio.read(&data,sizeof(data));
  }
}
