/******************************************************/

float accelScaleFactor[3];
long  accelSum[3] = {0, 0, 0};
long  accelSummedSamples[3] = {0, 0, 0};
float oneG;
float runTimeAccelBias[3] = {0, 0, 0};

union {float value[3];
        byte bytes[12];} accel;

union {float value[3];
        byte bytes[12];} filteredAccel;

union {int value[3];
      byte bytes[6];} rawAccel;

/******************************************************/