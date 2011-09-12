/******************************************************/

#define TIMEOUT     25000
#define MINCOMMAND  1000
#define MIDCOMMAND  1500
#define MAXCOMMAND  2000
#define MINDELTA    200
#define MINCHECK    MINCOMMAND + 100
#define MAXCHECK    MAXCOMMAND - 100
#define MINTHROTTLE MINCOMMAND + 100
#define LEVELOFF    100
int delta;

#define RISING_EDGE  1
#define FALLING_EDGE 0
#define MINONWIDTH   950
#define MAXONWIDTH   2075
#define MINOFFWIDTH  12000
#define MAXOFFWIDTH  24000

/******************************************************/

int receiverData[LASTCHANNEL];

/******************************************************/