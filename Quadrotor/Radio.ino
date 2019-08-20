
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
  // radio.startListening();
}

void Radio::radiosend(float datos[]) {
  // EMISION DE DATOS
  float data[3];
  datos[0] = datos[0] * radtodeg;
  datos[1] = datos[1] * radtodeg;
  datos[2] = datos[2] * radtodeg;

  // radio.write(datos, sizeof(datos));
  radio.write(datos, sizeof(data));
  // radio.write(, sizeof(datos));
}

float *Radio::radiolisten() {
  static float datos[4];
  // RECEPCION DE DATOS
  // Empezamos a escuchar por el canal
  radio.startListening();
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
  radio.stopListening();
  return datos;
}
