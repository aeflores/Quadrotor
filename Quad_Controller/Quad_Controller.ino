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
const int CE_PIN=9;
const int CSN_PIN=10;

// Creamos el objeto radio (NRF24L01)
RF24 radio(CE_PIN, CSN_PIN);

// Variable con la dirección del canal por donde se va a transmitir
const uint64_t pipe_send = 0xE8E8F0F0E1LL;
const uint64_t pipe_read = 0xF0F0F0F0D2LL;

//Variables necearias para la maquina de estados
unsigned long tiempo;
const int pinJoyButtonRight = 3;
const int pinJoyButtonLeft = 2;

// current state initialized to STANDBY
State curr_state = STANDBY;

// Vectores con los datos a enviar
TransmitData datos_rec;
ControlData datos_send;


StateChange stateChange() {
  if ((digitalRead(pinJoyButtonLeft) == LOW && (millis()>=tiempo+150))) {
    tiempo=millis();
    return StateChange::PREV;
  }
  if (digitalRead(pinJoyButtonRight) == LOW && (millis()>=tiempo+150)) {
    tiempo=millis();
    return StateChange::NEXT;
  }
  return StateChange::NO;
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
  // EMISION DE DATOS
  // cargamos los datos en la variable datos[]
  datos_send.movement[0] = analogRead(A0);
  datos_send.movement[1] = analogRead(A1);
  datos_send.movement[2] = analogRead(A2);
  datos_send.movement[3] = analogRead(A3);
  // This is for callibrating
  datos_send.movement[1]= analogRead(A4);
  datos_send.change= stateChange();

  if(curr_state== CALIBRATION){
      datos_send.moreData=true;
      radio.write(&datos_send, sizeof(datos_send));
      // send configuration
      ControllerConfiguration conf;
      conf.error2CorrectionCoeff=10;
      conf.upperUnbalanceRange=100;
      conf.lowerUnbalanceRange=70;
      radio.write(&conf,sizeof(conf));
      Serial.print("written conf: err2correct_coeff= ");
      Serial.print(conf.error2CorrectionCoeff);
      Serial.print(" upperRange= ");
      Serial.print(conf.upperUnbalanceRange);
      Serial.print(" lowerRange= ");
      Serial.println(conf.lowerUnbalanceRange);
  }else{
      datos_send.moreData=false;
      radio.write(&datos_send, sizeof(datos_send));
  }

  Serial.print("Power= ");
  Serial.print(datos_send.movement[1]);
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
    curr_state=datos_rec.state;
    // reportamos por el puerto serial los datos recibidos
    datos_rec.print(Serial);
    Serial.println("");
  }
  radio.stopListening();
}
