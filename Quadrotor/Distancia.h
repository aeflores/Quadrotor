#ifndef DISTANCIA_H_
#define DISTANCIA_H_

// Declaremos los pines CE y el CSN
#define EchoPin 5
#define TriggerPin 6

class Distancia {
private:
   long duration;
   double distanceCm;
public:
  void initialize();
  double update_distance();

};

#endif /* DISTANCIA_H */
