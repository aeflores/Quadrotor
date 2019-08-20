#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>
#include <nRF24L01.h>

// Declaremos los pines CE y el CSN
#define CE_PIN 9
#define CSN_PIN 10

// Creamos el objeto radio (NRF24L01)
RF24 radio(CE_PIN, CSN_PIN);

// Variable con la dirección del canal por donde se va a transmitir
const uint64_t pipe_send = 0xE8E8F0F0E1LL;
const uint64_t pipe_read = 0xF0F0F0F0D2LL;
// byte direccion[5] ={'c','a','n','a','l'};

// vector con los datos a enviar
float datos_send[4], datos_rec[4];
int state;
void setup() {
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
  datos_send[0] = analogRead(A0);
  datos_send[1] = analogRead(A1);
  datos_send[2] = analogRead(A2);
  datos_send[3] = analogRead(A3);

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
    radio.read(datos_rec, sizeof(datos_rec));

    // reportamos por el puerto serial los datos recibidos
    Serial.print("Yaw = ");
    Serial.print(datos_rec[0]);
    Serial.print("   Pitch = ");
    Serial.print(datos_rec[1]);
    Serial.print("   Roll = ");
    Serial.println(datos_rec[2]);
  }
  radio.stopListening();
}
