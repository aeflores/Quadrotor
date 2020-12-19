#ifndef BLACKBOX_H_
#define BLACKBOX_H_


#include <SPI.h>
#include <SD.h>

// Declaramos el pin CS
#define CS_PIN A3

class blackbox{
  private:
    int status = -1;
  public:
    void init();
    void calibration();
    void savedata();
};

#endif /*BLACKBOX_H_*/
