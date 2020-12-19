#include "Distancia.h"

void Distancia::initialize() {
   pinMode(TriggerPin, OUTPUT);
   pinMode(EchoPin, INPUT);
}
double Distancia::update_distance(){

   digitalWrite(TriggerPin, HIGH);  //generamos Trigger (disparo) de 10us
   delayMicroseconds(10);
   digitalWrite(TriggerPin, LOW);
   duration = pulseIn(EchoPin, HIGH, 11765);//medimos el tiempo entre pulsos, en microsegundos
   if (duration==0){
    duration=11765;
   }
   distanceCm = double(duration)*340./10000./2.;   //convertimos a distancia, en cm
   return distanceCm;
}
