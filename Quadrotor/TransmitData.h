#ifndef TRANSMIT_DATA_H_
#define TRANSMIT_DATA_H_


enum State { STANDBY = 0, CALIBRATION = 1, FLYMODE = 2, ABORT = 3 };
const int NUM_STATES = 4;

enum StateChange{ NO = 0, NEXT = 1, PREV = 2};

struct ControlData
{
  int movement[5];
  StateChange change;
  bool moreData;
};

struct TransmitData
{
    State state;
    int delta_t;
    float quadstate[4];
    float errors[2];
    int engines[4];

    

    void print(HardwareSerial& Serial){
       switch (state) {
        case STANDBY:
          Serial.print("STANDBY ");
          break;
        case CALIBRATION:
          Serial.print("CALIBRATION ");
          break;
        case FLYMODE:
          Serial.print("FLYMODE ");
          break;
        case ABORT:
          Serial.print("ABORT ");
          break;
        }
        Serial.print(" Yaw= ");
        Serial.print(quadstate[0]);
        Serial.print(" Pitch= ");
        Serial.print(quadstate[1]);
        Serial.print(" Roll= ");
        Serial.print(quadstate[2]);

        Serial.print(" Error pitch= ");
        Serial.print(errors[0]);
        Serial.print(" Error roll= ");
        Serial.print(errors[1]);

        for(int i=0; i<4; i++){
            Serial.print(" Eng");
            Serial.print(i+1);
            Serial.print("= ");
            Serial.print(engines[i]);
        }
        Serial.print("Delta T=");
        Serial.print(delta_t);
    }
};

struct ControllerConfiguration{
    float error2CorrectionCoeff           = 0;
    float derivativeError2CorrectionCoeff = 0;
    float upperUnbalanceRange             = 100;
    float lowerUnbalanceRange             = 100;
    float feedforwardunbalance14          = 0;
    float feedforwardunbalance23          = 0;
};


#endif
