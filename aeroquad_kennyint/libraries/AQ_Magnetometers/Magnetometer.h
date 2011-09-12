/******************************************************/

float magBias[3];
byte  newMagData = 0;
float rawMagTemporary[3];


union {float value[3];
        byte bytes[12];} mag;

union {int value[3];
      byte bytes[6];} rawMag;

/******************************************************/