void nvrWriteFloat(float value, int address) {
  union floatStore {
    byte floatByte[4];
    float floatVal;
  } floatIn;

  floatIn.floatVal = value;
  for (int i = 0; i < 4; i++)
    EEPROM.write(address + i, floatIn.floatByte[i]);
}
