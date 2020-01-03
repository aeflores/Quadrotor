//-------------------------------------------------------------//
//---------------------CONTROLLER CODE-------------------------//
//-------------------------------------------------------------//
//Authors: Enrique Flores and Antonio Flores

//Libraries
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>
#include <nRF24L01.h>
#include "TransmitData.h"

// Declaremos los pines CE y el CSN
#define CE_PIN 9
#define CSN_PIN 10

// Creamos el objeto radio (NRF24L01)
RF24 radio(CE_PIN, CSN_PIN);

// Variable con la dirección del canal por donde se va a transmitir
const uint64_t pipe_send = 0xE8E8F0F0E1LL;
const uint64_t pipe_read = 0xF0F0F0F0D2LL;

//Variables necearias para la maquina de estados
unsigned long tiempo;
const int pinJoyButtonRight = 3;
const int pinJoyButtonLeft = 2;
// possible states
enum state { STANDBY = 0, CALIBRATION = 1, FLYMODE = 2, ABORT = 3 };
// current state initialized to STANDBY
state curr_state = STANDBY;

// Vectores con los datos a enviar
TransmitData datos_rec;
int datos_send[5];


state next_state() {
  if ((digitalRead(pinJoyButtonLeft) == LOW && curr_state == CALIBRATION && (millis()>=tiempo+150)) || (digitalRead(pinJoyButtonRight) == LOW && curr_state == FLYMODE && (millis()>=tiempo+150))) {
    tiempo=millis();
    return STANDBY;
  }
  if (digitalRead(pinJoyButtonRight) == LOW && curr_state == STANDBY && (millis()>=tiempo+150)) {
    tiempo=millis();
    return CALIBRATION;
  }

  if ((digitalRead(pinJoyButtonLeft) == LOW && curr_state == STANDBY && (millis()>=tiempo+150)) || (digitalRead(pinJoyButtonRight) == LOW && curr_state == ABORT && (millis()>=tiempo+150))) {
    tiempo=millis();    
    return FLYMODE;
  }
  if (digitalRead(pinJoyButtonLeft) == LOW && curr_state == FLYMODE && (millis()>=tiempo+150)) {
    tiempo=millis();
    return ABORT;
  }
  else
    return curr_state;
}

void setup() {
  pinMode(pinJoyButtonRight , INPUT_PULLUP);
  pinMode(pinJoyButtonLeft , INPUT_PULLUP);
  // Inicializamos el puerto serie
  Serial.begin(115200);
  // Inicializamos el NRF24L01
  radio.begin();
  // radio.setRetries(15,15);
  // radio.setPayloadSize(8);
  // Abrimos un canal de escritura
  // radio.openWritingPipe(direccion);
  radio.openWritingPipe(pipe_send);
  // radio.openReadingPipe(1,direccion);
  radio.openReadingPipe(1, pipe_read);
}

void loop() {
    switch (curr_state) {
    case STANDBY:
      Serial.print("STANDBY   ");
      datos_send[4] = 0;
      curr_state = next_state();
      break;

    case CALIBRATION:
      Serial.print("CALIBRATION   ");
      datos_send[4] = 1;
      curr_state = next_state();
      break;

    case FLYMODE:
      Serial.print("FLYMODE   ");
      datos_send[4] = 2;
      curr_state = next_state();
      break;
    case ABORT:
      Serial.print("ABORT   ");
      datos_send[4] = 3;
      curr_state = next_state();
      break;
  }
  // EMISION DE DATOS
  // cargamos los datos en la variable datos[]
  datos_send[0] = analogRead(A0);
  datos_send[1] = analogRead(A1);
  datos_send[2] = analogRead(A2);
  datos_send[3] = analogRead(A3);

  datos_send[1]= analogRead(A4);
  Serial.print("Power= ");
  Serial.print(datos_send[1]);
  // enviamos los datos
  radio.write(datos_send, sizeof(datos_send));

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
    radio.read(&datos_rec, sizeof(datos_rec));

    // reportamos por el puerto serial los datos recibidos
    datos_rec.print(Serial);
    Serial.println("");
  }
  radio.stopListening();
}
