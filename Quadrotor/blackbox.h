#ifndef BLACKBOX_H_
#define BLACKBOX_H_


class blackbox{
  private:
    int status = -1;
  public:
    void init();
    void calibration();
    void savedata();
};

#endif /*BLACKBOX_H_*/
