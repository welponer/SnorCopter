// Choose processor type here

#define AeroQuad_Mini
//#define AeroQuadMega_v2

#include <EEPROM.h>
#include "TwiMaster.h"
TwiMaster twiMaster;

#include "AeroQuad.h"
#include "AQMath.h"
#include "Accel.h"

// Choose accelerometer type here

Accel_AeroQuadMini accel;
//Accel_AeroQuadMega_v2 accel;

#include "DataStorage.h"

float noseUp = 0;
float noseDown = 0;
float leftWingDown = 0;
float rightWingDown = 0;
float upSideDown = 0;
float rightSideUp = 0;

float xAxisAccelBias;
float xAxisAccelScaleFactor;
float yAxisAccelBias;
float yAxisAccelScaleFactor;
float zAxisAccelBias;
float zAxisAccelScaleFactor;

float m;
float biased;

int temp;

void setup() {
  
  Serial.begin(115200);
  
  twiMaster.init(false);
  
  accel.initialize(); // defined in Accel.h
  
  Serial.println("AHRS Mini Shield Accerometer Calibrator");
  Serial.println();
  
  Serial.println("Place Sensor Shield right side up");
  Serial.println("  Send a character when ready to proceed");
  Serial.println();
  
  while(Serial.available() == 0) {};
  temp = Serial.read();
  
  Serial.println("  Gathering Data...");
  Serial.println();
  for (int i=0; i<1000; i++) {
    accel.measure();
    rightSideUp += accel.getRaw(ZAXIS);
    delay(10);
  }
  rightSideUp /= 1000.0;
  Serial.println(rightSideUp);

  Serial.println("Place Sensor Shield up side down");
  Serial.println("  Send a character when ready to proceed");
  Serial.println();
  
  while(Serial.available() == 0) {};
  temp = Serial.read();
  
  Serial.println("  Gathering Data...");
  Serial.println();
  for (int i=0; i<1000; i++) {
    accel.measure();
    upSideDown += accel.getRaw(ZAXIS);
    delay(10);
  }
  upSideDown /= 1000.0;
  Serial.println(upSideDown);
  
  m = (upSideDown - rightSideUp)/(9.8065 - (-9.8065));
  zAxisAccelBias = upSideDown - m * 9.8065;
  biased = upSideDown - zAxisAccelBias;
  zAxisAccelScaleFactor = 9.8065 / biased;
  
  Serial.println("Place Sensor Shield left edge down");
  Serial.println("  Send a character when ready to proceed");
  Serial.println();
  
  while(Serial.available() == 0) {};
  temp = Serial.read();
  
  Serial.println("  Gathering Data...");
  Serial.println();
  for (int i=0; i<1000; i++) {
    accel.measure();
    leftWingDown += accel.getRaw(YAXIS);
    delay(10);
  }
  leftWingDown /= 1000.0;
  Serial.println(leftWingDown);

  Serial.println("Place Sensor Shield right edge down");
  Serial.println("  Send a character when ready to proceed");
  Serial.println();
  
  while(Serial.available() == 0) {};
  temp = Serial.read();
  
  Serial.println("  Gathering Data...");
  Serial.println();
  for (int i=0; i<1000; i++) {
    accel.measure();
    rightWingDown += accel.getRaw(YAXIS);
    delay(10);
  }
  rightWingDown /= 1000.0;
  Serial.println(rightWingDown);
  
  m = (leftWingDown - rightWingDown)/(9.8065 - -9.8065);
  yAxisAccelBias = leftWingDown - m * 9.8065;
  biased = leftWingDown - yAxisAccelBias;
  yAxisAccelScaleFactor = 9.8065 / biased;
  
  Serial.println("Place Sensor Shield rear edge down");
  Serial.println("  Send a character when ready to proceed");
  Serial.println();
  
  while(Serial.available() == 0) {};
  temp = Serial.read();
  
  Serial.println("  Gathering Data...");
  Serial.println();
  for (int i=0; i<1000; i++) {
    accel.measure();
    noseUp += accel.getRaw(XAXIS);
    delay(10);
  }
  noseUp /= 1000.0;
  Serial.println(noseUp);

  Serial.println("Place Sensor Shield front edge down");
  Serial.println("  Send a character when ready to proceed");
  Serial.println();
  
  while(Serial.available() == 0) {};
  temp = Serial.read();
  
  Serial.println("  Gathering Data...");
  Serial.println();
  for (int i=0; i<1000; i++) {
    accel.measure();
    noseDown += accel.getRaw(XAXIS);
    delay(10);
  }
  noseDown /= 1000.0;
  Serial.println(noseDown);
  
  m = (noseUp - noseDown)/(9.8065 - -9.8065);
  xAxisAccelBias = noseUp - m * 9.8065;
  biased = noseUp - xAxisAccelBias;
  xAxisAccelScaleFactor = 9.8065 / biased;
  
  Serial.print(noseUp); Serial.print(", ");
  Serial.print(noseDown); Serial.print(", ");
  Serial.print(xAxisAccelScaleFactor); Serial.print(", ");
  Serial.print(xAxisAccelBias); Serial.print(", ");
  Serial.print((noseUp-xAxisAccelBias)*xAxisAccelScaleFactor); Serial.print(", ");
  Serial.println((noseDown-xAxisAccelBias)*xAxisAccelScaleFactor);
  Serial.println();
  
  Serial.print(leftWingDown); Serial.print(", ");
  Serial.print(rightWingDown); Serial.print(", ");
  Serial.print(yAxisAccelScaleFactor); Serial.print(", ");
  Serial.print(yAxisAccelBias); Serial.print(", ");
  Serial.print((leftWingDown-yAxisAccelBias)*yAxisAccelScaleFactor); Serial.print(", ");
  Serial.println((rightWingDown-yAxisAccelBias)*yAxisAccelScaleFactor);
  Serial.println();
  
  Serial.print(upSideDown); Serial.print(", ");
  Serial.print(rightSideUp); Serial.print(", ");
  Serial.print(zAxisAccelScaleFactor); Serial.print(", ");
  Serial.print(zAxisAccelBias); Serial.print(", ");
  Serial.print((upSideDown-zAxisAccelBias)*zAxisAccelScaleFactor); Serial.print(", ");
  Serial.println((rightSideUp-zAxisAccelBias)*zAxisAccelScaleFactor);
  Serial.println();
  
  Serial.println("Send 'A' to accept this data and write to EEPROM");
  Serial.println("  Any other character exits without writting");
  Serial.println();
  
  while(Serial.available() == 0) {};
  temp = Serial.read();

  if (temp == 'A') {
    writeFloat(xAxisAccelScaleFactor, XAXIS_ACCEL_SCALE_FACTOR_ADR);
    writeFloat(xAxisAccelBias,        XAXIS_ACCEL_BIAS_ADR);
    
    writeFloat(yAxisAccelScaleFactor, YAXIS_ACCEL_SCALE_FACTOR_ADR);
    writeFloat(yAxisAccelBias,        YAXIS_ACCEL_BIAS_ADR);
    
    writeFloat(zAxisAccelScaleFactor, ZAXIS_ACCEL_SCALE_FACTOR_ADR);
    writeFloat(zAxisAccelBias,        ZAXIS_ACCEL_BIAS_ADR);
    
    Serial.println("Data written to EEPROM");
  }
  else
    Serial.println("Exited without writting");
}

void loop() {}


