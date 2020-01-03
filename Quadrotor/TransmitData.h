

struct TransmitData
{
    float yawpitchroll[3];
    float errors[2];
    int engines[4];
    int delta_t;

    void print(HardwareSerial& Serial){
        Serial.print(" Yaw= ");
        Serial.print(yawpitchroll[0]);
        Serial.print(" Pitch= ");
        Serial.print(yawpitchroll[1]);
        Serial.print(" Roll= ");
        Serial.print(yawpitchroll[2]);

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
