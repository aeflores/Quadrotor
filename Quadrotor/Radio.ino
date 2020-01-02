
#include "Radio.h"

void Radio::initialize() {
  radio.begin();
  // radio.setRetries(15,15);
  // radio.setPayloadSize(8);
  // Abrimos un canal de escritura
  // radio.openWritingPipe(direccion);
  radio.openWritingPipe(pipe_send);
  // radio.openReadingPipe(1,direccion);
  radio.openReadingPipe(1, pipe_read);
  radio.startListening();
}

void Radio::radiosend(const Attitude& datos) {
  // EMISION DE DATOS
  float data[3];
  data[0] = datos.yaw * radtodeg;
  data[1] = datos.pitch * radtodeg;
  data[2] = datos.roll * radtodeg;
  radio.stopListening();
  radio.write(data, sizeof(data));
  radio.startListening();
}

void Radio::radiolisten(int (&datos)[5]) {
  // RECEPCION DE DATOS
  // Empezamos a escuchar por el canal
  //radio.startListening();
  unsigned long started_waiting_at = millis();
  bool timeout = false;
  while (!radio.available() && !timeout) { // Esperamos 200ms
    if (millis() - started_waiting_at > 200)
      timeout = true;
  }
  if (timeout) {
    Serial.println("Error, No ha habido respuesta a tiempo");
  } else {
    // Leemos los datos y los guardamos en la variable datos[]
    radio.read(datos, sizeof(datos));
  }
  //radio.stopListening();
}
