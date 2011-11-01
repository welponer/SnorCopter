/*
  AeroQuad v3.0 - Nov 2011
 www.AeroQuad.com
 Copyright (c) 2011 Ted Carancho.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/* 
 Special thanks to Alamo for contributing this capability! You will need to upload the correct character set to the MAX7456 first.  Please see:
 http://aeroquad.com/showthread.php?2942-OSD-implementation-using-MAX7456
 
 This class provides a basic on-screen display (OSD) for flying the AeroQuad in FPV. It can display
 battery voltage, altitude, compass heading, a flight clock, a centre reticle and an attitude indicator. Display elements will appear
 if defined below or the corresponding sensor is defined in AeroQuad.pde.
 
 The user must connect a MAX7456 OSD chip to the appropriate header pins on their Arduino. These pins are
 marked 'OSD' on the AeroQuad Shield v2. If the chip is not connected properly, this code may hang.
 If using the SparkFun MAX7456 breakout board, you should add a 10kOhm pull up resistor between 5V and the reset pin.
 You will need to update the character memory of your MAX7456 or you'll see garbage on screen. See the AQ forum thread.
 
 It's a good idea to use an external 5V linear regulator, such as an LM7805/LM317 or similar to power to the MAX7456. The MAX7456 can draw up to 100mA @ 5V
 - this would result in an extra ~0.7W being dissipated across the Arduino onboard regulator (for a 3S battery). That's a lot of power for a little regulator with minimal heatsinking!
 */

#ifndef _AQ_OSD_MAX7456_H_
#define _AQ_OSD_MAX7456_H_


/*********************** User configuration section ***************************************/
#define ShowReticle            //Displays a reticle in the centre of the screen. 
#define ShowFlightTimer        //Displays how long the motors have been armed for since the Arduino was last reset
#define ShowAttitudeIndicator
#define ShowCallSign
#define ShowRSSI
//#define feet                   //Comment this line out for altitude measured in metres, uncomment it for feet

//Choose your (default in case autodetect enabled) video standard: default=NTSC
//#define PAL
#define AUTODETECT // detect automatically

//You can configure positioning of various display elements below. #defines for elements which will not be displayed, can be ignored.
//The MAX7456 overlays characters in a grid 30 characters wide, 16/13 high (PAL/NTSC respectively). The row/column defines below
// correspond to positions in the grid of characters, with the origin at the top left. 0-origin indexing is used - ie row 0, col 0
// is the highest, leftmost character on the screen while row 15, col 29 is the bottom right (for PAL).
//Display elements start at the position you give and print to the right. They will wrap around to the next row if there are too few
// columns remaining on the row you specify.

//Battery info - 5-16 characters long
#define VOLTAGE_ROW 2
#define VOLTAGE_COL 1

//Compass reading - 5 characters long
#define COMPASS_ROW 1
#define COMPASS_COL 13

//Altitude reading - up to 8 characters long (32768 max)
#define ALTITUDE_ROW 1
#define ALTITUDE_COL 1

//Flight timer - 6 characters long
#define TIMER_ROW 1
#define TIMER_COL 23

//Callsign
#define CALLSIGN_ROW 2
#define CALLSIGN_COL 23
#ifdef ShowCallSign
  const char *callsign = "AeroQD";
#endif

// RSSI monitor
#define RSSI_ROW     3
#define RSSI_COL     23
#define RSSI_PIN     A6     // analog pin to read
#define RSSI_RAWVAL         // show raw A/D value instead of percents (for tuning)
#define RSSI_100P    1023   // A/D value for 100%
#define RSSI_0P      0      // A/D value for 0%
#define RSSI_WARN    20     // show alarm at %

// Notify
#define NOTIFY_ROW MAX_screen_rows-3
#define NOTIFY_COL 1 // don't change this, it needs a full line

/********************** End of user configuration section ********************************/

//OSD pins on AQ v2 shield:
#define CS   22 //SS_OSD
#define DOUT 51 //MOSI
#define DIN  50 //MISO
#define SCK  52 //SCLK

//MAX7456 register write addresses - see datasheet for lots of info
#define DMM   0x04 //Display memory mode register - for choosing 16bit/8bit write mode, clearing display memory, enabling auto-increment
#define DMAH  0x05 //Holds MSB of display memory address, for setting location of a character on display
#define DMAL  0x06 //Holds remaining 8 bits of display memory address - 480 characters displayed -> 9 bits req'd for addressing
#define DMDI  0x07 //Display memory data in - character address or attribute byte, depending on 8b/16b mode and DMAH[1]
#define VM0   0x00 //Video mode 0 register - for choosing, NTSC/PAL, sync mode, OSD on/off, reset, VOUT on/off
#define VM1   0x01 //Video mode 1 register - nothing very interesting in this one
#define RB0   0x10 //Row 0 brightness register - 15 more follow sequentially (ending at 0x1F)
#define STAT  0xA2 //Status register read address

//MAX7456 commands - provided in datasheet.
#define CLEAR_display      0x04
#define CLEAR_display_vert 0x06
#define END_string         0xff

#define WHITE_level_90     0x02

unsigned MAX_screen_size = 0;
unsigned MAX_screen_rows = 0;
byte ENABLE_display      = 0;
byte ENABLE_display_vert = 0;
byte MAX7456_reset       = 0;
byte DISABLE_display     = 0;

//configuration for AI
#define LINE_ROW_0 0x80                             // character address of a character with a horizontal line in row 0. Other rows follow this one
#define AI_MAX_PITCH_ANGLE (PI/4)                   // bounds of scale used for displaying pitch. When pitch is >= |this number|, the pitch lines will be at top or bottom of bounding box
static const byte ROLL_COLUMNS[4] = {10,12,17,19};  // columns where the roll line is printed
#define PITCH_L_COL 7
#define PITCH_R_COL 22
#define AI_DISPLAY_RECT_HEIGHT 9                    // Height of rectangle bounding AI. Should be odd so that there is an equal space above/below the centre reticle

#define RETICLE_ROW (MAX_screen_rows/2)             // centre row - don't change this
#define RETICLE_COL 14                              // reticle will be in this col, and col to the right

#define AI_TOP_PIXEL ((RETICLE_ROW - AI_DISPLAY_RECT_HEIGHT/2)*18)
#define AI_BOTTOM_PIXEL ((RETICLE_ROW + AI_DISPLAY_RECT_HEIGHT/2)*18)
#define AI_CENTRE (RETICLE_ROW*18+10)               // row, in pixels, corresponding to zero pitch/roll.


//////////////////////////////////////////////////////////////
//Performs an 8-bit SPI transfer operation
byte spi_transfer( byte data ) {

  SPDR = data; //transfer data with hardware SPI
  while ( !(SPSR & _BV(SPIF)) ) ;
  return SPDR;
}

void spi_writereg(byte r, byte d) {

  spi_transfer(r);
  spi_transfer(d);
}

byte spi_readreg(byte r) {

  spi_transfer(r);
  return spi_transfer(0);
} 

void spi_select() {
  digitalWrite( CS, LOW );
}

void spi_deselect() {
  digitalWrite( CS, HIGH );
}

// Writes 'len' character address bytes to the display memory corresponding to row y, column x
// - uses autoincrement mode when writing more than one character
// - will wrap around to next row if 'len' is greater than the remaining cols in row y
// - buf=NULL can be used to write zeroes (clear)
// - flags: 0x01 blink, 0x02 invert (can be combined)
void writeChars( const char* buf, byte len, byte flags, byte y, byte x ) {

  unsigned offset = y*30 + x; 
  spi_select();    
  // 16bit transfer, transparent BG, autoincrement mode (if len!=1)
  spi_writereg(DMM, ((flags&1) ? 0x10 : 0x00) | ((flags&2) ? 0x08 : 0x00) | ((len!=1)?0x01:0x00) ); 

  // send starting display memory address (position of text)
  spi_writereg(DMAH, offset >> 8 );
  spi_writereg(DMAL, offset & 0xff );

  // write out data
  for ( int i = 0; i < len; i++ ) {
    spi_writereg(DMDI, buf==NULL?0:buf[i] );
  }
  
  // Send escape 11111111 to exit autoincrement mode
  if (len!=1) {
    spi_writereg(DMDI, END_string );
  }
  // finished writing
  spi_deselect();
}

void detectVideoStandard() {

  // First set the default
  boolean pal = false;
  #ifdef PAL
    pal = true;
  #endif
  #ifdef AUTODETECT
    // if autodetect enabled modify the default if signal is present on either standard 
    // otherwise default is preserved
    spi_select();
    byte stat=spi_readreg(STAT);
    if (stat & 0x01) {
      pal = true;
    }
    if (stat & 0x02) {
      pal = false;
    }
    spi_deselect();
  #endif

  if (pal) {
    MAX_screen_size=480;
    MAX_screen_rows=16;
    ENABLE_display=0x48;
    ENABLE_display_vert=0x4c;
    MAX7456_reset=0x42;
    DISABLE_display=0x40;
  } 
  else {
    MAX_screen_size=390;
    MAX_screen_rows=13;
    ENABLE_display=0x08;
    ENABLE_display_vert=0x0c;
    MAX7456_reset=0x02;
    DISABLE_display=0x00;
  }
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// Battery voltage Display //////////////////////////
//////////////////////////////////////////////////////////////////////////////
#ifdef BattMonitor

void displayVoltage() {
  int currentVoltage = batteryVoltage*10.0;
  char buf[7];
  snprintf(buf,7,"\4%2d.%1dV",currentVoltage/10,currentVoltage%10);
  writeChars( buf, 6, ((armed==ON)&&(batteryStatus!=OK))?1:0, VOLTAGE_ROW, VOLTAGE_COL );    
}
#endif

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// AltitudeHold Display /////////////////////////////
//////////////////////////////////////////////////////////////////////////////
#ifdef AltitudeHold

int lastAltitude     = 12345;     // bogus initial values to force update
int lastHoldAltitude = 12345;
byte lastHoldState   = 6;

void displayAltitude() {
  #ifdef feet
    int currentAltitude = getBaroAltitude()*3.281;
    int currentHoldAltitude = altitudeToHoldTarget*3.281;
  #else // metric
    int currentAltitude = getBaroAltitude()*10.0; // 0.1m accuracy!!
    int currentHoldAltitude = altitudeToHoldTarget*10.0;
  #endif
  char buf[7];

  if ( lastAltitude != currentAltitude ) {
    #ifdef feet
      snprintf(buf,7,"\10%4df",currentAltitude);
    #else
      if (abs(currentAltitude)<100) {
        snprintf(buf,7,"\010%c%1d.%1dm",currentAltitude < 0 ? '-' : ' ', abs(currentAltitude/10),abs(currentAltitude%10));
      } 
      else {
        snprintf(buf,7,"\010%4dm",currentAltitude/10);
      }
    #endif
    writeChars( buf, 6, 0, ALTITUDE_ROW, ALTITUDE_COL );
    lastAltitude=currentAltitude;    
  }

  // AltitudeHold handling:
  // - show hold altitude when it is active
  // - show "panic" if 'paniced' out
  byte __write = 0;
  switch (altitudeHoldState) {
  case OFF:
    if (lastHoldState != OFF) {
      lastHoldState=OFF;
      memset(buf,0,6);
      __write++;
    }
    break;
  case ON:
    if ((lastHoldState != ON) || (lastHoldAltitude != currentHoldAltitude)) {
      lastHoldState=ON;
      lastHoldAltitude=currentHoldAltitude;
      #ifdef feet
        snprintf(buf,7,"\11%4df",currentHoldAltitude);
      #else
        if (abs(currentHoldAltitude)<100) {
          snprintf(buf,7,"\011%c%1d.%1dm", currentHoldAltitude < 0 ? '-' : ' ',abs(currentHoldAltitude/10),abs(currentHoldAltitude%10));
        } 
        else {
          snprintf(buf,7,"\011%4dm",currentHoldAltitude/10);
        }
      #endif
      __write++;
    }
    break;
  case ALTPANIC:
    if (lastHoldState!=ALTPANIC) {
      lastHoldState=ALTPANIC;
      snprintf(buf,7,"\11panic");
      __write++;
    }
    break;
  }
  if (__write) {
    writeChars( buf, 6, 0, ALTITUDE_ROW, ALTITUDE_COL+6 );
  }
}
#endif

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// HeadingMagHold Display ///////////////////////////
//////////////////////////////////////////////////////////////////////////////
#ifdef HeadingMagHold

int lastHdg=361; // bogus to force update

void displayHeading() {
  int currentHdg = kinematicsGetDegreesHeading(YAW);
  if (currentHdg!=lastHdg) {
    char buf[6];
    snprintf(buf,6,"\6%3d\7",currentHdg); // \6 is compass \7 is degree symbol
    writeChars( buf, 5, 0, COMPASS_ROW, COMPASS_COL );
  }
}
#endif

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// Flight time Display //////////////////////////////
//////////////////////////////////////////////////////////////////////////////
#ifdef ShowFlightTimer

unsigned long prevTime = 0;           // previous time since start when OSD.update() ran
unsigned int prevArmedTimeSecs = 111; // bogus to force update
unsigned long armedTime = 0;          // time motors have spent armed

void displayFlightTime() {
  if (armed == ON) {
    armedTime += ( currentTime-prevTime );
  }

  prevTime = currentTime;
  unsigned int armedTimeSecs=armedTime/1000000;
  if (armedTimeSecs!=prevArmedTimeSecs) {
    prevArmedTimeSecs=armedTimeSecs;
    char buf[7];
    snprintf(buf,7,"\5%02u:%02u",armedTimeSecs/60,armedTimeSecs%60);
    writeChars(buf, 6, 0, TIMER_ROW, TIMER_COL );
  }
}
#endif

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// RSSI Display /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Show RSSI information (analog input value optionally mapped to percents.)
#ifdef ShowRSSI

short lastRSSI=1234; //forces update at first run

void displayRSSI() {

  int val = analogRead(RSSI_PIN);
  #ifndef RSSI_RAWVAL
    val = (val-RSSI_0P)*100/(RSSI_100P-RSSI_0P);
    if (val<0) {
      val=0;
    }
    if (val>100) {
      val=100;
    }
  #endif
  if (val!=lastRSSI) {
    lastRSSI=val;
    char buf[6];
    #ifdef RSSI_RAWVAL
      snprintf((char *)buf,6,"\372%4u",val);
      writeChars(buf,5,0,RSSI_ROW,RSSI_COL);
    #else
      snprintf((char *)buf,6,"\372%3u%%",val);
      writeChars(buf,5,(RSSI_WARN>val)?1:0,RSSI_ROW,RSSI_COL);
    #endif
  }
}
#endif

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// ATTITUDE Display /////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
#ifdef ShowAttitudeIndicator

byte AIoldline[5] = {0,0,0,0,0};

void displayArtificialHorizon() {

  const float roll = kinematicsAngle[ROLL];
  const float pitch = kinematicsAngle[PITCH];
  short  AIrows[5] = {0,0,0,0,0};  //Holds the row, in pixels, of AI elements: pitch then roll from left to right.

  //Calculate row of new pitch lines
  AIrows[0] = constrain( (int)AI_CENTRE + (int)((pitch/AI_MAX_PITCH_ANGLE)*(AI_CENTRE-AI_TOP_PIXEL) ), AI_TOP_PIXEL, AI_BOTTOM_PIXEL );  //centre + proportion of full scale
  char pitchLine = LINE_ROW_0 + (AIrows[0] % 18);

  if (AIoldline[0] != AIrows[0]/18) {
    //Remove old pitch lines if not overwritten by new ones
    writeChars( NULL, 1, 0, AIoldline[0], PITCH_L_COL );
    writeChars( NULL, 1, 0, AIoldline[0], PITCH_R_COL );
    AIoldline[0] = AIrows[0]/18;
  }

  //Write new pitch lines
  writeChars( &pitchLine, 1, 0, AIoldline[0], PITCH_L_COL );
  writeChars( &pitchLine, 1, 0, AIoldline[0], PITCH_R_COL );

  //Calculate row (in pixels) of new roll lines
  int distFar  = (ROLL_COLUMNS[3] - (RETICLE_COL + 1))*12 + 6; //horizontal pixels between centre of reticle and centre of far angle line
  int distNear = (ROLL_COLUMNS[2] - (RETICLE_COL + 1))*12 + 6;
  float gradient = 1.4 * roll; // was "tan(roll)", yes rude but damn fast !!
  AIrows[4] = constrain( AI_CENTRE - (int)(((float)distFar)*gradient), AI_TOP_PIXEL, AI_BOTTOM_PIXEL );
  AIrows[3] = constrain( AI_CENTRE - (int)(((float)distNear)*gradient), AI_TOP_PIXEL, AI_BOTTOM_PIXEL );
  AIrows[1] = constrain( 2*AI_CENTRE - AIrows[4], AI_TOP_PIXEL, AI_BOTTOM_PIXEL );
  AIrows[2] = constrain( 2*AI_CENTRE - AIrows[3], AI_TOP_PIXEL, AI_BOTTOM_PIXEL );

  //writing new roll lines to screen
  for (byte i=1; i<5; i++ ) {
    // clear previous roll lines if not going to overwrite
    if (AIoldline[i] != AIrows[i]/18) {
      writeChars( NULL, 1, 0, AIoldline[i], ROLL_COLUMNS[i-1] );
      AIoldline[i] =  AIrows[i]/18;
    }
    //converting rows (in pixels) to character addresses used for the 'lines'
    char RollLine = LINE_ROW_0 + (AIrows[i] % 18);
    writeChars( &RollLine, 1, 0, AIoldline[i], ROLL_COLUMNS[i-1] );
  }
}
#endif

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// Reticle Display //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Reticle on the center of the screen
// We have two reticles empty one for ACRO and one with (s) for 'STABLE' mode
#ifdef ShowReticle

byte lastFlightMode = 9;

void displayReticle() {

  if (lastFlightMode!=flightMode) {
    writeChars( (ACRO==flightMode)?"\1\2":"\21\22", 2, 0, RETICLE_ROW, RETICLE_COL ); //write 2 chars to row (middle), column 14
    lastFlightMode=flightMode;
  }
}
#endif

// void notifyOSD(byte flags, char *fmt, ...)
//   - display notification string on OSD
// void notifyOSDmenu(byte flags, byte cstart, byte cstop, char *fmt, ...) 
//   - display notification with blinking region = 'cursor'
//   - characters between cstart and cstop will blink if OSD_CURSOR flag is used
//
//   fmt == NULL will clear
//   flags -- message priority and options
#define OSD_INFO    0x00
#define OSD_WARN    0x40
#define OSD_ERR     0x80
#define OSD_CRIT    0xc0
#define OSD_NOCLEAR 0x20 // do not clear the message after ~5s
#define OSD_CURSOR  0x10 // enable cursor
#define OSD_BLINK   0x08 // blinking message
#define OSD_INVERT  0x04 // inverted message
#define OSD_NOW     0x02 // show message immediately instead of waiting for next OSD
                         // refresh, will block until transfer done. Use with care!
                         // Could be used for emergency messages as message will remain on 
                         // screen even if Arduino crashes
#define OSD_CENTER  0x01 // Justify at center

byte _n_cprio=0;
byte _n_ttl=0;
char _n_buf[29]; // note we leave one character off from both sides on screen
byte _n_flags;
byte _n_cstart;
byte _n_cstop;

byte displayNotify(){

  if (_n_flags&OSD_NOW) {
    _n_flags&=~OSD_NOW;
    // we have new message to show

    if ((_n_flags&OSD_CURSOR) && (_n_cstart!=255)) {

      if (_n_cstart>0) {
        writeChars(_n_buf,_n_cstart,
          ((_n_flags&OSD_INVERT)?2:0)|((_n_flags&OSD_BLINK)?1:0),
          NOTIFY_ROW,NOTIFY_COL);
      }

      writeChars(_n_buf+_n_cstart,_n_cstop-_n_cstart+1,
        ((_n_flags&OSD_INVERT)?2:0)|((_n_flags&OSD_BLINK)?0:1),
        NOTIFY_ROW,NOTIFY_COL+_n_cstart);

      if (_n_cstop<27) {
        writeChars(_n_buf+_n_cstop+1,27-_n_cstop,
          ((_n_flags&OSD_INVERT)?2:0)|((_n_flags&OSD_BLINK)?1:0),
          NOTIFY_ROW,NOTIFY_COL+_n_cstop+1);
      }
    }
    else {
      writeChars(_n_buf,28,
        ((_n_flags&OSD_INVERT)?2:0)|((_n_flags&OSD_BLINK)?1:0),
        NOTIFY_ROW,NOTIFY_COL);
    }
    return 1;
  }

  if ((_n_ttl>0)&&(_n_ttl!=255)) {
    if (_n_ttl--==1) {
      writeChars(NULL,28,0,NOTIFY_ROW,NOTIFY_COL);
      return 1;
    }
  }
  return 0; // nothing was done
}

#define notifyOSD(flags,fmt,args...) notifyOSDmenu(flags,255,255,fmt, ## args)

byte notifyOSDmenu(byte flags, byte cstart, byte cstop, const char *fmt, ...) {

  va_list ap;
  if ((_n_ttl>0) && ((flags>>6)<_n_cprio)) return 1; // drop message, we tell it to caller
  if (fmt==NULL) { 
    // clear
    memset(_n_buf,0,28);
    _n_flags=0;
  } 
  else {
    _n_cprio=flags>>6;
    _n_ttl=(flags&OSD_NOCLEAR)?255:50; // set timeout for message
    va_start(ap,fmt);
    byte len=vsnprintf(_n_buf,29,fmt,ap);
    if (len>28) len=28; 
    va_end(ap);
    memset(_n_buf+len,0,28-len);
    if (flags&OSD_CENTER) {
      byte i=(28-len)/2;
      if (i) {
        // move text right to center it
        memmove(_n_buf+i,_n_buf,strlen(_n_buf));
        memset(_n_buf,0,i);
        // adjust cursor position also
        if (flags&OSD_CURSOR) {
          cstart+=i;
          cstop+=i;
        }
      } 
    }
    _n_flags=flags;
    _n_cstart=cstart<27?cstart:27;
    _n_cstop=cstop<27?cstop:27;
  }
  if (flags&OSD_NOW) {
    displayNotify();
  } else {
    _n_flags|=OSD_NOW; // this will tell next update to show message
  }
  return 0;
}

byte OSDsched = 0;

void updateOSD() {

  // OSD is updated fully in 4 rounds, these are (using bit)
  // 0x01 - Attitude Indicator
  // 0x02 - Altitude, Heading, Timer, RSSI, Reticle
  // 0x04 - Attitude Indicator
  // 0x08 - Battery info

  // Check notify first, if it did something we dont't have time for other stuff
  if (displayNotify()) {
    return;
  }

  if ((OSDsched&0x01) || (OSDsched&0x04)) {
    #ifdef ShowAttitudeIndicator
      displayArtificialHorizon();
    #endif
  }

  if (OSDsched&0x02) {
    #ifdef AltitudeHold
      displayAltitude();
    #endif
    #ifdef HeadingMagHold
      displayHeading();
    #endif
    #ifdef ShowFlightTimer
      displayFlightTime();
    #endif
    #ifdef ShowRSSI
      displayRSSI();
    #endif
    #ifdef ShowReticle
      displayReticle();
    #endif
  }

  if (OSDsched&0x08) {
    #ifdef BattMonitor
      displayVoltage();
    #endif
  }
  OSDsched<<=1;
  if (OSDsched&0x10) {
    OSDsched=0x01;
  }
}
  
void initializeOSD() {

  int i = 0;
  pinMode( CS, OUTPUT );
  pinMode( 53, OUTPUT ); //Default CS pin - needs to be output or SPI peripheral will behave as a slave instead of master
  digitalWrite( CS, HIGH );

  pinMode( DOUT, OUTPUT );
  pinMode( DIN, INPUT );
  pinMode( SCK, OUTPUT );

  // SPCR = 01010000
  //interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
  //sample on leading edge of clk,system clock/4 rate (fastest)
  SPCR = (1 << SPE) | (1 << MSTR);
  i = SPSR;
  i = SPDR;
  delay( 10 ); 

  detectVideoStandard();

  //Soft reset the MAX7456 - clear display memory
  spi_select();
  spi_writereg( VM0, MAX7456_reset );
  spi_deselect();
  delay( 1 ); //Only takes ~100us typically

  //Set white level to 90% for all rows
  spi_select();
  for( i = 0; i < MAX_screen_rows; i++ ) {
    spi_writereg( RB0 + i, WHITE_level_90 );
  }

  //ensure device is enabled
  spi_writereg( VM0, ENABLE_display );
  delay(100);
  //finished writing
  spi_deselect();  

  OSDsched=0xff; // This will make everything to be updated next round
  updateOSD(); // Make first update now
  #ifdef ShowCallSign
    writeChars(callsign,strlen(callsign),0,CALLSIGN_ROW,CALLSIGN_COL);
  #endif
  // push notification of active video format
  notifyOSD(OSD_NOW, "VIDEO: %s", (DISABLE_display)?"PAL":"NTSC");
}

#endif  // #define _AQ_OSD_MAX7456_H_
