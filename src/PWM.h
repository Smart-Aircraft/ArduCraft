/*
	https://www.arduino.cn/thread-3012-1-1.html
*/

#ifndef PWM_H_
#define PWM_H_

#include "avr/pgmspace.h"
#include "math.h"

//bit offset from timer 1 in the register
#define TIMER1_OFFSET	0x00
#define TIMER3_OFFSET	0x10
#define TIMER4_OFFSET	0x20
#define TIMER5_OFFSET	0xA0

//generic register-accessing macros (eliminates some branching)
#define TCCRA_16(tmr_offset)	_SFR_MEM8(0x80  + tmr_offset)
#define TCCRB_16(tmr_offset)	_SFR_MEM8(0x81  + tmr_offset)
#define TCCRC_16(tmr_offset)	_SFR_MEM8(0x82  + tmr_offset)
#define ICR_16(tmr_offset)		_SFR_MEM16(0x86 + tmr_offset)

//physical memory locations. Used in pwmWrite()
#define OCR1A_MEM	0x88
#define OCR1B_MEM	0x8A
#define OCR1C_MEM	0x8C
#define OCR3A_MEM	0x98
#define OCR3B_MEM	0x9A
#define OCR3C_MEM	0x9C
#define OCR4A_MEM	0xA8
#define OCR4B_MEM	0xAA
#define OCR4C_MEM	0xAC
#define OCR5A_MEM	0x128
#define OCR5B_MEM	0x12A
#define OCR5C_MEM	0x12C

#define OCR0A_MEM	0x47
#define OCR0B_MEM	0x48
#define OCR2A_MEM	0xB3
#define OCR2B_MEM	0xB4

#define TCCR1A_MEM	0x80
#define TCCR1B_MEM	0x81
#define TCCR1C_MEM	0x82
#define TCCR3A_MEM	0x90
#define TCCR3B_MEM	0x91
#define TCCR4A_MEM	0xA0
#define TCCR4B_MEM	0xA1
#define TCCR4C_MEM	0xA2
#define TCCR5A_MEM	0x120
#define TCCR5B_MEM	0x121
#define TCCR5C_MEM	0x122

#define TCCR0A_MEM	0x44
#define TCCR0B_MEM	0x45
#define TCCR2A_MEM	0xB0
#define TCCR2B_MEM	0xB1

#define ICR1_MEM	0x86
#define ICR3_MEM	0x96
#define ICR4_MEM	0xA6
#define ICR5_MEM	0x126

//8 bit timers
#define TIMER0_OFFSET	0x00
#define TIMER2_OFFSET	0x6C

#define TCCRA_8(tmr_offset)		_SFR_MEM8(0x44 + tmr_offset)
#define TCCRB_8(tmr_offset)		_SFR_MEM8(0x45 + tmr_offset)
#define OCRA_8(tmr_offset)		_SFR_MEM8(0x47 + tmr_offset)

//macros for each timer 'object'
#define Timer0_GetFrequency()		GetFrequency_8(TIMER0_OFFSET)
#define Timer0_SetFrequency(x)		SetFrequency_8(TIMER0_OFFSET, x)
#define Timer0_GetPrescaler()		GetPrescaler_8(TIMER0_OFFSET)
#define Timer0_SetPrescaler(x)		SetPrescaler_8(TIMER0_OFFSET, x)
#define Timer0_GetTop()				GetTop_8(TIMER0_OFFSET)
#define Timer0_SetTop(x)			SetTop_8(TIMER0_OFFSET, x)
#define Timer0_Initialize()			Initialize_8(TIMER0_OFFSET)

#define Timer1_GetFrequency()		GetFrequency_16(TIMER1_OFFSET)
#define Timer1_SetFrequency(x)		SetFrequency_16(TIMER1_OFFSET, x)
#define Timer1_GetPrescaler()		GetPrescaler_16(TIMER1_OFFSET)
#define Timer1_SetPrescaler(x)		SetPrescaler_16(TIMER1_OFFSET, x)
#define Timer1_GetTop()				GetTop_16(TIMER1_OFFSET)
#define Timer1_SetTop(x)			SetTop_16(TIMER1_OFFSET, x)
#define Timer1_Initialize()			Initialize_16(TIMER1_OFFSET)

#define Timer2_GetFrequency()		GetFrequency_8(TIMER2_OFFSET)
#define Timer2_SetFrequency(x)		SetFrequency_8(TIMER2_OFFSET, x)
#define Timer2_GetPrescaler()		GetPrescaler_8(TIMER2_OFFSET)
#define Timer2_SetPrescaler(x)		SetPrescalerAlt_8(TIMER2_OFFSET, x)
#define Timer2_GetTop()				GetTop_8(TIMER2_OFFSET)
#define Timer2_SetTop(x)			SetTop_8(TIMER2_OFFSET, x)
#define Timer2_Initialize()			Initialize_8(TIMER2_OFFSET)

#define Timer3_GetFrequency()		GetFrequency_16(TIMER3_OFFSET)
#define Timer3_SetFrequency(x)		SetFrequency_16(TIMER3_OFFSET, x)
#define Timer3_GetPrescaler()		GetPrescaler_16(TIMER3_OFFSET)
#define Timer3_SetPrescaler(x)		SetPrescaler_16(TIMER3_OFFSET, x)
#define Timer3_GetTop()				GetTop_16(TIMER3_OFFSET)
#define Timer3_SetTop(x)			SetTop_16(TIMER3_OFFSET, x)
#define Timer3_Initialize()			Initialize_16(TIMER3_OFFSET)

#define Timer4_GetFrequency()		GetFrequency_16(TIMER4_OFFSET)
#define Timer4_SetFrequency(x)		SetFrequency_16(TIMER4_OFFSET, x)
#define Timer4_GetPrescaler()		GetPrescaler_16(TIMER4_OFFSET)
#define Timer4_SetPrescaler(x)		SetPrescaler_16(TIMER4_OFFSET, x)
#define Timer4_GetTop()				GetTop_16(TIMER4_OFFSET)
#define Timer4_SetTop(x)			SetTop_16(TIMER4_OFFSET, x)
#define Timer4_Initialize()			Initialize_16(TIMER4_OFFSET)

#define Timer5_GetFrequency()		GetFrequency_16(TIMER5_OFFSET)
#define Timer5_SetFrequency(x)		SetFrequency_16(TIMER5_OFFSET, x)
#define Timer5_GetPrescaler()		GetPrescaler_16(TIMER5_OFFSET)
#define Timer5_SetPrescaler(x)		SetPrescaler_16(TIMER5_OFFSET, x)
#define Timer5_GetTop()				GetTop_16(TIMER5_OFFSET)
#define Timer5_SetTop(x)			SetTop_16(TIMER5_OFFSET, x)
#define Timer5_Initialize()			Initialize_16(TIMER5_OFFSET)

static const uint16_t pscLst[] = { 0, 1, 8, 64, 256, 1024};
static const uint16_t pscLst_alt[] = {0, 1, 8, 32, 64, 128, 256, 1024};

struct TimerData //each instance is 4 bytes
{
	uint16_t	TimerTopRegLoc:		9;
	uint16_t	ChannelRegLoc:		9;
	uint16_t	PinConnectRegLoc:	9;
	uint8_t		PinConnectBits:		4;
	bool		Is16Bit:			1;
};

//4 bytes each, 18 elements, 72 Bytes total
const TimerData timer_to_pwm_data[] = {
	{0, 0, 0, 0},										//NOT_ON_TIMER
	{0, 0, 0, 0},										//TIMER0A	disabled when initialized
	{OCR0A_MEM, OCR0B_MEM, TCCR0A_MEM, COM0B1, false},	//TIMER0B
		
	{ICR1_MEM, OCR1A_MEM, TCCR1A_MEM, COM1A1, true},	//TIMER1A
	{ICR1_MEM, OCR1B_MEM, TCCR1A_MEM, COM1B1, true},	//TIMER1B	no C channel on timer 1?

	{0, 0, 0, 0, 0},									//TIMER2	
	{0, 0, 0, 0, 0},									//TIMER2A	disabled when initialized
	{OCR2A_MEM, OCR2B_MEM, TCCR2A_MEM, COM2B1, false},	//TIMER2B
		
	{ICR3_MEM, OCR3A_MEM, TCCR3A_MEM, COM3A1, true},	//TIMER3A
	{ICR3_MEM, OCR3B_MEM, TCCR3A_MEM, COM3B1, true},	//TIMER3B
	{ICR3_MEM, OCR3C_MEM, TCCR3A_MEM, COM3C1, true},	//TIMER3C
		
	{ICR4_MEM, OCR4A_MEM, TCCR4A_MEM, COM4A1, true},	//TIMER4A
	{ICR4_MEM, OCR4B_MEM, TCCR4A_MEM, COM4B1, true},	//TIMER4B
	{ICR4_MEM, OCR4C_MEM, TCCR4A_MEM, COM4C1, true},	//TIMER4C
	{0, 0, 0, 0, 0},									//TIMER4D	
		
	{ICR5_MEM, OCR5A_MEM, TCCR5A_MEM, COM5A1, true},	//TIMER5A
	{ICR5_MEM, OCR5B_MEM, TCCR5A_MEM, COM5B1, true},	//TIMER5B
	{ICR5_MEM, OCR5C_MEM, TCCR5A_MEM, COM5C1, true},	//TIMER5C
};

enum prescaler
{
	ps_1	=	1,
	ps_8	=	2,
	ps_64	=	3,
	ps_256	=	4,
	ps_1024 =	5
};

//certain 8 bit timers read the CSn register differently
enum prescaler_alt
{
	psalt_1		=	1,
	psalt_8		=	2,
	psalt_32	=	3,
	psalt_64	=	4,
	psalt_128	=	5,
	psalt_256	=	6,
	psalt_1024	=	7
};

// 16 bit timers
extern uint32_t	GetFrequency_16(const int16_t timerOffset);
extern bool		SetFrequency_16(const int16_t timerOffset, uint32_t f);
extern uint16_t GetPrescaler_16(const int16_t timerOffset);
extern void		SetPrescaler_16(const int16_t timerOffset, prescaler psc);
extern void		SetTop_16(const int16_t timerOffset, uint16_t top);
extern uint16_t GetTop_16(const int16_t timerOffset);
extern void		Initialize_16(const int16_t timerOffset);

// 8 bit timers
extern uint32_t	GetFrequency_8(const int16_t timerOffset);
extern bool		SetFrequency_8(const int16_t timerOffset, uint32_t f);
extern uint16_t GetPrescaler_8(const int16_t timerOffset);
extern void		SetPrescaler_8(const int16_t timerOffset, prescaler psc);
extern void		SetPrescalerAlt_8(const int16_t timerOffset, prescaler_alt psc);
extern void		SetTop_8(const int16_t timerOffset, uint8_t top);
extern uint8_t	GetTop_8(const int16_t timerOffset);
extern void		Initialize_8(const int16_t timerOffset);

//common functions
extern void		InitTimers();
extern void		InitTimersSafe();										//doesn't init timers responsible for time keeping functions
extern void		pwmWrite(uint8_t pin, uint8_t val);
extern void		pwmWriteHR(uint8_t pin, uint16_t val);					//accepts a 16 bit value and maps it down to the timer for maximum resolution
extern bool		SetPinFrequency(int8_t pin, uint32_t frequency);
extern bool		SetPinFrequencySafe(int8_t pin, uint32_t frequency);	//does not set timers responsible for time keeping functions

#endif /* PWM_H_ */