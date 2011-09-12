#define ROLL      0
#define PITCH     1
#define YAW       2

#define XAXIS     0
#define YAXIS     1
#define ZAXIS     2

#define LAST_AXIS 3

#define FINDZERO 49

typedef struct {
    
  float XAXIS_ACCEL_BIAS_ADR;
  float XAXIS_ACCEL_SCALE_FACTOR_ADR;
  float YAXIS_ACCEL_BIAS_ADR;
  float YAXIS_ACCEL_SCALE_FACTOR_ADR;
  float ZAXIS_ACCEL_BIAS_ADR;
  float ZAXIS_ACCEL_SCALE_FACTOR_ADR;
  
  float XAXIS_MAG_BIAS_ADR;
  float YAXIS_MAG_BIAS_ADR;
  float ZAXIS_MAG_BIAS_ADR;

} t_NVR_Data;  

void nvrWriteFloat(float value, int address); // defined in DataStorage.h

#define GET_NVR_OFFSET(param) ((int)&(((t_NVR_Data*) 0)->param))
#define writeFloat(value, addr) nvrWriteFloat(value, GET_NVR_OFFSET(addr))

