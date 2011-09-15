/******************************************************/

#if defined(quadPlusConfig)
  #define FRONT 0
  #define RIGHT 1
  #define REAR  2
  #define LEFT  3

  #define FIRSTMOTOR 0
  #define LASTMOTOR  4
#endif

#if defined(quadXConfig)
  #define FRONT_LEFT  0
  #define FRONT_RIGHT 1
  #define REAR_RIGHT  2
  #define REAR_LEFT   3

  #define FIRSTMOTOR  0
  #define LASTMOTOR   4
#endif

#if defined(y4Config)
  #define FRONT_LEFT  0
  #define FRONT_RIGHT 1
  #define UPPER_REAR  2
  #define LOWER_REAR  3

  #define FIRSTMOTOR  0
  #define LASTMOTOR   4
#endif

#if defined(hexPlusConfig)
  #define FRONT       0
  #define FRONT_RIGHT 1
  #define REAR_RIGHT  2
  #define REAR        3
  #define REAR_LEFT   4
  #define FRONT_LEFT  5

  #define FIRSTMOTOR  0
  #define LASTMOTOR   6
#endif

#if defined(hexXConfig)
  #define FRONT_LEFT  0
  #define FRONT_RIGHT 1
  #define RIGHT       2
  #define REAR_RIGHT  3
  #define REAR_LEFT   4
  #define LEFT        5

  #define FIRSTMOTOR  0
  #define LASTMOTOR   6
#endif

#if defined(y6Config)
  #define UPPER_FRONT_LEFT  0
  #define UPPER_FRONT_RIGHT 1
  #define UPPER_REAR        2
  #define LOWER_FRONT_LEFT  3
  #define LOWER_FRONT_RIGHT 4
  #define LOWER_REAR        5

  #define FIRSTMOTOR  0
  #define LASTMOTOR   6
#endif

/******************************************************/

boolean escsCalibrating = OFF;
int motorAxisCommand[3];
int motorCommand[LASTMOTOR];
int minCommand[LASTMOTOR];
int maxCommand[LASTMOTOR];
int remoteMotorCommand[LASTMOTOR];

/******************************************************/

void writeMotors(void);

/******************************************************/

void commandAllMotors(int _motorCommand) {   // Sends commands to all motors
  for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
    motorCommand[motor] = _motorCommand;

  writeMotors();
}

/******************************************************/

void pulseMotors(byte quantity) {
  for (byte i = 0; i < quantity; i++) {
  commandAllMotors(MINCOMMAND + 100);
  delay(250);
  commandAllMotors(MINCOMMAND);
  delay(250);
  }
}

/******************************************************/
