/*
 * Hand Module (Transmitter).c
 *
 * Created: 11/12/2026 4:10:14 PM
 * Author : adham
 */ 

#define F_CPU 16000000ul

#include <util/delay.h>
#include "../service/std_types.h"
#include "../service/bit_math.h"
#include "../MCAL/regdef.h"
#include "../MCAL/ADC/adc.h"
#include "../MCAL/USART/usart.h"

// Protocol & Calibrated Drop Thresholds
#define PACKET_HEADER           0xAA

#define THUMB_DEADZONE_DROP     30		// Ignore <30 counts
#define THUMB_MAX_DROP          135     // 135+ counts drop = 100% PWM throttle
#define FINGER_BENT_DROP        70      // >=70 counts drop counts a steering finger as bent

// ADC Channel Mapping (PA0 - PA4)
#define CH_THUMB                0
#define CH_INDEX                1
#define CH_MIDDLE               2
#define CH_RING                 3
#define CH_PINKY                4

static u16 Global_u16InitVal[5] = {0};	// Stores initial bend-sensor values (flat-hand)

// Discards first read after ADMUX switch so Sample & Hold capacitor settles cleanly
static u16 APP_u16ReadAdcClean(u8 Copy_u8Channel, u8 Copy_u8Samples)
{
	// Dummy read to switch ADMUX and flush previous channel charge
	(void)ADC_u16ReadValue(Copy_u8Channel);
	_delay_us(20); // Allow analog voltage to finish settling

	// Accumulate multiple samples in a 32-bit variable to prevent overflow
	u32 Local_u32Sum = 0;
	for (u8 i = 0; i < Copy_u8Samples; i++)
	{
		Local_u32Sum += ADC_u16ReadValue(Copy_u8Channel);
	}
	// Return the averaged 10-bit ADC value (0 - 1023)
	return (u16)(Local_u32Sum / Copy_u8Samples);
}

static void APP_voidSendPacket(u8 Copy_u8Cmd, u8 Copy_u8PwmPercent)
{
	UART_voidSend(PACKET_HEADER);
	UART_voidSend(Copy_u8Cmd);
	UART_voidSend(Copy_u8PwmPercent);
}

int main(void)
{
	s16 Local_s16Drop[5];
	u8  Local_u8PostureMask;
	u8  Local_u8ThumbPwm;
	u8  Local_u8Cmd;

	ADC_voidInit();
	UART_voidInit();    // 9600 baud
	
	// Hold hand flat for ~310 ms after power-on/reset:
	// 300 ms power-rail stabilization + ~9 ms to sample 16-read baselines for all 5 fingers 
	_delay_ms(300);
	for (u8 i = 0; i < 5; i++)
	{
		Global_u16InitVal[i] = APP_u16ReadAdcClean(i, 16);
	}
	
	while (1)
	{
		// Read clean drops (Init - Runtime) and clamp negative hyper-extension to 0
		for (u8 i = 0; i < 5; i++)
		{
			u16 Local_u16Curr = APP_u16ReadAdcClean(i, 4);
			Local_s16Drop[i]  = (s16)Global_u16InitVal[i] - (s16)Local_u16Curr;

			if (Local_s16Drop[i] < 0)
			{
				Local_s16Drop[i] = 0;
			}
		}

		// Convert thumb ADC values to percentages for PWM [0..100%]
		if (Local_s16Drop[CH_THUMB] <= THUMB_DEADZONE_DROP)
		{
			Local_u8ThumbPwm = 0;
		}
		else if (Local_s16Drop[CH_THUMB] >= THUMB_MAX_DROP)
		{
			Local_u8ThumbPwm = 100;
		}
		else
		{
			u32 Local_u32Active = (u32)(Local_s16Drop[CH_THUMB] - THUMB_DEADZONE_DROP);
			u32 Local_u32Span   = (u32)(THUMB_MAX_DROP - THUMB_DEADZONE_DROP);
			Local_u8ThumbPwm    = (u8)((Local_u32Active * 100UL) / Local_u32Span);
		}

		// 4-bit posture mask [Pinky | Ring | Middle | Index]
		Local_u8PostureMask = 0;
		if (Local_s16Drop[CH_INDEX]  >= FINGER_BENT_DROP) SET_BIT(Local_u8PostureMask, 0);
		if (Local_s16Drop[CH_MIDDLE] >= FINGER_BENT_DROP) SET_BIT(Local_u8PostureMask, 1);
		if (Local_s16Drop[CH_RING]   >= FINGER_BENT_DROP) SET_BIT(Local_u8PostureMask, 2);
		if (Local_s16Drop[CH_PINKY]  >= FINGER_BENT_DROP) SET_BIT(Local_u8PostureMask, 3);
		
		// Decode 4-finger posture into direction command
		switch (Local_u8PostureMask)
		{
			case 0b1111: Local_u8Cmd = 'F'; break;  // Closed fist  -> Forward
			case 0b1100: Local_u8Cmd = 'B'; break;  // Peace sign   -> Backward
			case 0b1110: Local_u8Cmd = 'R'; break;  // Point index  -> Right
			case 0b0111: Local_u8Cmd = 'L'; break;  // Pinky out    -> Left
			default:     Local_u8Cmd = 'S'; break;  // Open palm / neutral -> Stop
		}
		
		// Straight thumb (0% throttle) overrides any posture to Stop
		if (Local_u8ThumbPwm == 0 || Local_u8Cmd == 'S')
		{
			Local_u8Cmd      = 'S';
			Local_u8ThumbPwm = 0;
		}
		
		APP_voidSendPacket(Local_u8Cmd, Local_u8ThumbPwm);
		
		// ~40 Hz transmission rate
		_delay_ms(22);
	}
	
	return 0;
}
