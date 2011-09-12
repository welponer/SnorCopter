#include <EEPROM.h>
#include "TwiMaster.h"
TwiMaster twiMaster;

#include "AeroQuadCalibrator.h"
#include "DataStorage.h"

//#include "HMC5843.h"
#include "HMC5883.h"

int maxX = 0;
int minX = 0;
int maxY = 0;
int minY = 0;
int maxZ = 0;
int minZ = 0;

int temp;

void setup() {
  
  Serial.begin(115200);
  
  twiMaster.init(false);
  
  initializeCompass();
  
  Serial.println("AHRS Mag Compass Calibrator");
  Serial.println();
  
  Serial.println("Rotate sensor shield around each axis a few times");
  Serial.println("  Send a character when complete");
  Serial.println();
  
  do {
    readCompass();
    if (maxX < magRaw[XAXIS]) maxX = magRaw[XAXIS];
    if (minX > magRaw[XAXIS]) minX = magRaw[XAXIS];
    if (maxY < magRaw[YAXIS]) maxY = magRaw[YAXIS];
    if (minY > magRaw[YAXIS]) minY = magRaw[YAXIS];
    if (maxZ < magRaw[ZAXIS]) maxZ = magRaw[ZAXIS];
    if (minZ > magRaw[ZAXIS]) minZ = magRaw[ZAXIS];
    delay(20);
  } while(Serial.available()==0);
  
  temp = Serial.read();
  
  magBias[XAXIS] = (float(maxX - minX) / 2.0) - maxX;
  magBias[YAXIS] = (float(maxY - minY) / 2.0) - maxY;
  magBias[ZAXIS] = (float(maxZ - minZ) / 2.0) - maxZ;
  
  Serial.print(minX); Serial.print(","); Serial.print(maxX); Serial.print(","); Serial.println(magBias[XAXIS]);
  Serial.print(minY); Serial.print(","); Serial.print(maxY); Serial.print(","); Serial.println(magBias[YAXIS]);
  Serial.print(minZ); Serial.print(","); Serial.print(maxZ); Serial.print(","); Serial.println(magBias[ZAXIS]);
  Serial.println();
  
  Serial.println("Send 'A' to accept this data and write to EEPROM");
  Serial.println("  Any other character exits without writting");
  Serial.println();
  
  while(Serial.available() == 0) {};
  temp = Serial.read();

  if (temp == 'A') {
    writeFloat(magBias[XAXIS], XAXIS_MAG_BIAS_ADR);
        
    writeFloat(magBias[YAXIS], YAXIS_MAG_BIAS_ADR);
        
    writeFloat(magBias[ZAXIS], ZAXIS_MAG_BIAS_ADR);
        
    Serial.println("Data written to EEPROM");
  }
  else
    Serial.println("Exited without writting");
}

void loop() {}


