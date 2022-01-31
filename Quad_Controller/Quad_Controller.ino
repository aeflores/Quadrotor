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
const int CE_PIN = 7;
const int CSN_PIN = 8;

// Creamos el objeto radio (NRF24L01)
RF24 radio(CE_PIN, CSN_PIN);

// Variable con la dirección del canal por donde se va a transmitir
const uint64_t pipe_read = 0xF0F0F0F0D2LL;

//Variables necesarias para la maquina de estados
const int pinJoyButtonRight = 3;
const int pinJoyButtonLeft = 4;

// current state initialized to STANDBY
State curr_state = STANDBY;

// Vectores con los datos a enviar
TransmitData datos_rec;
ControlData datos_send;

enum Button { HI = 0, PUSHED = 1, DONE = 2};

StateChange stateChange() {
  static unsigned long tiempo_left, tiempo_right;
  static Button left_button, right_button;

  StateChange change = StateChange::NO;
  int left = digitalRead(pinJoyButtonLeft);
  int right = digitalRead(pinJoyButtonRight);

  // Left button state machine
  // si encontramos un flanco de bajada vamos a PUSHED y empezamos a contar tiempo.
  // Cambiamos the estado si hemos estado 100 ms con el boton pulsado.
  // Una vez cambiado de estado, no hacemos nada (en el estado DONE) hasta que el boton
  // se deja de pulsar.
  switch (left_button) {
    case Button::HI:
      if (left == LOW) {
        left_button = Button::PUSHED;
        tiempo_left = millis();
      }
      break;
    case Button::PUSHED:
      if (millis() >= tiempo_left + 50) {
        left_button = Button::DONE;
        change = StateChange::PREV;
      }
      if (left == HIGH)
        left_button = Button::HI;
      break;
    case Button::DONE:
      if (left == HIGH)
        left_button = Button::HI;
      break;
  }

  // right button state machine
  switch (right_button) {
    case Button::HI:
      if (right == LOW) {
        right_button = Button::PUSHED;
        tiempo_right = millis();
      }
      break;
    case Button::PUSHED:
      if (millis() >= tiempo_right + 50) {
        right_button = Button::DONE;
        change = StateChange::NEXT;
      }
      if (right == HIGH)
        right_button = Button::HI;
      break;
    case Button::DONE:
      if (right == HIGH)
        right_button = Button::HI;
      break;
  }

  return change;
}

void setup() {
  pinMode(pinJoyButtonRight , INPUT_PULLUP);
  pinMode(pinJoyButtonLeft , INPUT_PULLUP);
  // Inicializamos el puerto serie
  Serial.begin(115200);
  // Inicializamos el NRF24L01
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, pipe_read);
  // Envia respuesta con la confirmacion
  // de recibo.
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.startListening();
  radio.writeAckPayload(1,&datos_send,sizeof(datos_send));
}

void loop() {
  

  if (radio.available()){
    
    // Lectura de datos
    radio.read(&datos_rec, sizeof(datos_rec));
    curr_state = datos_rec.state;
    Serialbufferprint(datos_rec, datos_send.movement[4]);

    // Emision de datos
    datos_send.movement[0] = analogRead(A3);
    datos_send.movement[1] = analogRead(A2);
    datos_send.movement[2] = analogRead(A1);
    datos_send.movement[3] = analogRead(A0);
    // This is for callibrating
    //datos_send.movement[1]= int(analogRead(A4)*3/4);
    datos_send.movement[4] = int(analogRead(A7));
    datos_send.change = stateChange();
    radio.writeAckPayload(1,&datos_send,sizeof(datos_send)); 
  }
}


void Serialbufferprint(TransmitData &data, int power) {
  int intstate;
  switch (data.state) {
    case STANDBY:
      intstate = 1;
      break;
    case CALIBRATION:
      intstate = 2;
      break;
    case FLYMODE:
      intstate = 3;
      break;
    case ABORT:
      intstate = 4;
      break;
  }
  int nvar = 13;
  int nbytesvar = 2;
  int totalbytes = nvar * nbytesvar;
  byte buff[totalbytes] = { ((uint8_t*)&intstate)[0], ((uint8_t*)&intstate)[1],
                            ((uint8_t*)&power)[0], ((uint8_t*)&power)[1],
                            ((uint8_t*)&data.delta_t)[0], ((uint8_t*)&data.delta_t)[1],
                            ((uint8_t*)&data.quadstate[0])[0], ((uint8_t*)&data.quadstate[0])[1],
                            ((uint8_t*)&data.quadstate[1])[0], ((uint8_t*)&data.quadstate[1])[1],
                            ((uint8_t*)&data.quadstate[2])[0], ((uint8_t*)&data.quadstate[2])[1],
                            ((uint8_t*)&data.quadstate[3])[0], ((uint8_t*)&data.quadstate[3])[1],
                            ((uint8_t*)&data.errors[0])[0], ((uint8_t*)&data.errors[0])[1],
                            ((uint8_t*)&data.errors[1])[0], ((uint8_t*)&data.errors[1])[1],
                            ((uint8_t*)&data.engines[0])[0], ((uint8_t*)&data.engines[0])[1],
                            ((uint8_t*)&data.engines[1])[0], ((uint8_t*)&data.engines[1])[1],
                            ((uint8_t*)&data.engines[2])[0], ((uint8_t*)&data.engines[2])[1],
                            ((uint8_t*)&data.engines[3])[0], ((uint8_t*)&data.engines[3])[1],};
  Serial.write(buff, totalbytes);
}
