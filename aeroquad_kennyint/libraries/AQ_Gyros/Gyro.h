/******************************************************/

long  gyroSum[3]  = {0, 0, 0};
long  gyroSummedSamples[3];
float runTimeGyroBias[3];

union {float value[3];
        byte bytes[12];} gyro;

union {int value[3];
      byte bytes[6];} rawGyro;

/******************************************************/
