#ifndef BLACKBOX_H_
#define BLACKBOX_H_

#include <SdFat.h>

class blackbox{
  private:
    SdFat sd;
    int status = -1;
  public:
    void init();
    void calibration();
    void savedata();
};

#endif /*BLACKBOX_H_*/
