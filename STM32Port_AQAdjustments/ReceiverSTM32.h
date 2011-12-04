#if defined(AeroQuadMega_v2STM32)

/*
  Copyright (c) 2011 ala42.  All rights reserved.

  STM32 receiver class by ala42 using time input capture
  for use with AeroQuad software and Maple library
  V 1.0 Oct 15 2011

  Define the pin numbers used for the receiver in receiverPin[]

  Timer and timer channels are accessed using the Maple PIN_MAP array.
  Make sure libmaple and this receiver class are compiled using the
  same structure alignment mode. When in doubt, change the stm32_pin_info 
  declaration in wirish_types.h to align the size to a multiple of 4 byte
  by adding a filler byte at the end of the structure declaration.
*/


#include "wirish.h"

//#define STM32_TIMER_DEBUG // enable debug messages

///////////////////////////////////////////////////////////////////////////////
// configuration part starts here
// definition of pins used for PWM receiver input

#ifdef BOARD_aeroquad32
static byte receiverPin[] = {
	Port2Pin('D', 12),
	Port2Pin('D', 13),
	Port2Pin('D', 14),
	Port2Pin('D', 15),
	Port2Pin('E',  9),
	Port2Pin('E', 11),
	Port2Pin('E', 13),
	Port2Pin('E', 14)
};
#endif

#ifdef BOARD_aeroquad32mini
static byte receiverPin[] = {
	2, // PB7
	4, // PB6
	5, // PB8
	6, // PB9
	7, // PA15    JTDI   TIM2_CH1
	8  // PB3     JTDO   SPI3SCK /TIM2_CH2
};
#endif


///////////////////////////////////////////////////////////////////////////////
// implementation part starts here.

// forward declaration, array is defined at the end of this file
extern voidFuncPtr PWM_in_handler[];

// interrupt handler needs access to receiver class
class Receiver_AeroQuad_STM32 *STM32_receiver;


class Receiver_AeroQuad_STM32 : public Receiver {
private:
	typedef struct {
		timer_dev   *TimerDev;
		timer_gen_reg_map *TimerRegs;
		__io uint32	*Timer_ccr;
		int			Low;
		int			High;
		uint16		HighTime;
		uint16		RiseTime;
		int			Channel;
		int			TimerChannel;
		int			PolarityMask;
		int			Valid;
		int			Debug;
	} tFrqData;

    #define FRQInputs 8
	volatile tFrqData FrqData[FRQInputs];

public:
	void initialize(void) {
		STM32_receiver = this; // give interrupt handler access to this class
		InitFrqMeasurement();
		this->_initialize(); // load in calibration xmitFactor from EEPROM
	}

	/*
    #define ROLL     0
    #define PITCH    1
    #define YAW      2
    #define THROTTLE 3
    #define MODE     4
    #define AUX      5
    #define AUX2     6
    #define AUX3     7
	*/

	uint16_t getReceiverChannel(const byte channel) {
		//static byte ReceiverChannelMap[] = {3, 2, 0, 1, 4, 6, 5, 7}; // mapping for ala42
		static byte ReceiverChannelMap[] = {0, 1, 2, 3, 4, 5, 6, 7}; // default mapping
		volatile tFrqData *f = &FrqData[ReceiverChannelMap[channel]];

		uint16_t PulsLength = f->HighTime;

		return PulsLength;
	}


	// hide the class details from the interupt handler
	void IrqChangeValue(int chan)
	{
		FrqChange(&FrqData[chan]);
	}


private:
	void FrqInit(int aChannel, int aDefault, volatile tFrqData *f, timer_dev *aTimer, int aTimerChannel)
	{
		aTimerChannel--;  // transfor timer channel numbering from 1-4 to 0-3

		f->Channel      = aChannel;
		f->Valid        = false;

		f->TimerDev     = aTimer;
		timer_gen_reg_map *timer = aTimer->regs.gen;
		f->TimerRegs    = timer;

		f->Timer_ccr    = &timer->CCR1 + aTimerChannel;
		f->Debug        = false;
		f->HighTime     = aDefault;
		f->TimerChannel = aTimerChannel;

		int TimerEnable = (1 << (4*aTimerChannel));
		f->PolarityMask = TimerEnable << 1;

		timer->PSC	= 72-1;
		timer->ARR	= 0xffff;
		timer->CR1	= 0;
		timer->DIER &= ~(1);

		timer->CCER &= ~TimerEnable; // Disable timer
		timer->CCER &= ~(f->PolarityMask);

#ifdef STM32_TIMER_DEBUG
		Serial.print("  timer ");
		Serial.print((int32)timer, 16);

		Serial.print("  CCMR0 ");
		Serial.print(timer->CCMR1, 16);
#endif

		volatile uint32 *mr;
		if(aTimerChannel < 2) {
			mr = &(timer->CCMR1);
		} else {
			mr = &(timer->CCMR2);
		}
		*mr &= ~(0xFF << (8*(aTimerChannel&1)));   // prescaler 1
		*mr |= 0x61<< (8*(aTimerChannel&1));// 6=filter, 1=inputs 1,2,3,4


#ifdef STM32_TIMER_DEBUG
		Serial.print("  CCER0 ");
		Serial.print(timer->CCER, 16);
		Serial.print("  CCMR1 ");
		Serial.print(timer->CCMR1, 16);
#endif

		timer->CCER |= TimerEnable; // Enable

#ifdef STM32_TIMER_DEBUG
		Serial.print("  CCER2 ");
		Serial.print(timer->CCER, 16);
#endif

		timer->CR1 = 1;

#ifdef STM32_TIMER_DEBUG
		Serial.print("  CCER3 ");
		Serial.print(timer->CCER, 16);
		Serial.print("  CCMR1 ");
		Serial.print(timer->CCMR1, 16);
		Serial.println("");
#endif
	}

	void InitFrqMeasurement()
	{
#ifdef STM32_TIMER_DEBUG
		Serial.println("InitFrqMeasurement");
#endif
		for(int rcLine = 0; rcLine < (int)(sizeof(receiverPin) / sizeof(receiverPin[0])); rcLine++) {
			int pin = receiverPin[rcLine];
			timer_dev *timer_num = PIN_MAP[pin].timer_device;
			if(timer_num == NULL) {
#ifdef STM32_TIMER_DEBUG
				Serial.print("InitFrqMeasurement: invalid PWM input ");
				Serial.print(pin);
				Serial.println();
#endif
			} else {
				pinMode(pin, INPUT);
#ifdef STM32_TIMER_DEBUG
				timer_gen_reg_map *timer = PIN_MAP[pin].timer_device->regs.gen;
				Serial.print("pin ");
				Serial.print(pin);
				Serial.print(" timerbase ");
				Serial.print((int32)timer,16);
				Serial.println();
#endif
				FrqInit(rcLine, 1500, &FrqData[rcLine], timer_num, PIN_MAP[pin].timer_channel);

				timer_attach_interrupt(timer_num, PIN_MAP[pin].timer_channel, PWM_in_handler[rcLine]);
			}
		}
#ifdef STM32_TIMER_DEBUG
		Serial.println("InitFrqMeasurement done");
#endif
	}


	void PWMInvertPolarity(volatile tFrqData *f)
	{
		f->TimerRegs->CCER ^= f->PolarityMask; // invert polarity
	}


	void FrqChange(volatile tFrqData *f)
	{
		timer_gen_reg_map *timer = f->TimerRegs;
		uint16_t c = *(f->Timer_ccr);
		if(f->Debug) {
			Serial.print(f->Channel);
			Serial.print(" ");
			Serial.print((int)c, 16);
			Serial.println("");
		}

		bool rising = (timer->CCER & f->PolarityMask) == 0;
		if(f->Valid) {
			if(rising) {
				// rising edge, store start time
				f->RiseTime = c;
				//Serial.print("  r ");
				//Serial.println(f->RiseTime, 10);
			} else {
				f->HighTime = c - f->RiseTime;
				//Serial.print("  f ");
				//Serial.println(f->HighTime, 10);
			}
		} else {
			if(rising) {
				// rising egde, store start time
				f->RiseTime = c;
				f->Valid = true;
			}
		}

		PWMInvertPolarity(f);
	}
}; // class Receiver_AeroQuad_STM32


///////////////////////////////////////////////////////////////////////////////
// definition of interrupt handler funtions, one for each channel
void PWM_in_0() { STM32_receiver->IrqChangeValue(0); }
void PWM_in_1() { STM32_receiver->IrqChangeValue(1); }
void PWM_in_2() { STM32_receiver->IrqChangeValue(2); }
void PWM_in_3() { STM32_receiver->IrqChangeValue(3); }
void PWM_in_4() { STM32_receiver->IrqChangeValue(4); }
void PWM_in_5() { STM32_receiver->IrqChangeValue(5); }
void PWM_in_6() { STM32_receiver->IrqChangeValue(6); }
void PWM_in_7() { STM32_receiver->IrqChangeValue(7); }

voidFuncPtr PWM_in_handler[] = { PWM_in_0, PWM_in_1, PWM_in_2, PWM_in_3, PWM_in_4, PWM_in_5, PWM_in_6, PWM_in_7 };

#endif
