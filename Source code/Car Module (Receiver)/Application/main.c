/*
 * Car Module (Receiver).c
 *
 * Created: 11/12/2026 4:41:20 PM
 * Author : adham
 */ 

#define F_CPU 16000000ul

#include <util/delay.h>
#include "../service/std_types.h"
#include "../service/bit_math.h"
#include "../MCAL/USART/usart.h"
#include "../HAL/DCMOTOR/dcmotor.h"
#include "../HAL/CLCD/CLCD_int.h"

// Protocol Definitions
#define PACKET_HEADER           0xAA

// UART Packet Parser States
#define RX_STATE_HEADER         0
#define RX_STATE_CMD            1
#define RX_STATE_PWM            2

// Failsafe Timeout: Stop car if no valid packet arrives for ~600 ms
#define RX_TIMEOUT_TICKS        300     // 300 * 2 ms loop delay = 600 ms

static u8 APP_u8IsValidCmd(u8 Copy_u8Cmd)
{
	return (Copy_u8Cmd == 'F' ||
			Copy_u8Cmd == 'B' ||
			Copy_u8Cmd == 'L' ||
			Copy_u8Cmd == 'R' ||
			Copy_u8Cmd == 'S');
}

static void APP_voidDriveMotors(u8 Copy_u8Cmd, u8 Copy_u8Pwm)
{
	if (Copy_u8Pwm == 0 || Copy_u8Cmd == 'S')
	{
		DCMOTOR_voidStopAll();
		return;
	}

	switch (Copy_u8Cmd)
	{
		case 'F': DCMOTOR_voidMoveForward(Copy_u8Pwm);  break;
		case 'B': DCMOTOR_voidMoveBackward(Copy_u8Pwm); break;
		case 'R': DCMOTOR_voidTurnRight(Copy_u8Pwm);    break;
		case 'L': DCMOTOR_voidTurnLeft(Copy_u8Pwm);     break;
		default : DCMOTOR_voidStopAll();                break;
	}
}

static void APP_voidUpdateLcd(u8 Copy_u8Cmd, u8 Copy_u8Pwm, u8 Copy_u8LinkLost)
{
	// Write Direction at Col 6, Row 0
	CLCD_voidSetCursorPos(6, 0);
	if (Copy_u8LinkLost == 1)
	{
		CLCD_voidSendString((const u8 *)"NO SIGNAL ");
	}
	else
	{
		switch (Copy_u8Cmd)
		{
			case 'F': CLCD_voidSendString((const u8 *)"FORWARD   "); break;
			case 'B': CLCD_voidSendString((const u8 *)"BACKWARD  "); break;
			case 'R': CLCD_voidSendString((const u8 *)"RIGHT     "); break;
			case 'L': CLCD_voidSendString((const u8 *)"LEFT      "); break;
			default:  CLCD_voidSendString((const u8 *)"STOP      "); break;
		}
	}

	// Write Speed at Col 6, Row 1
	CLCD_voidSetCursorPos(6, 1);
	CLCD_voidSendNumber((s32)Copy_u8Pwm);
	CLCD_voidSendString((const u8 *)"%   ");
}
int main(void)
{
	u8  Local_u8RxByte      = 0;
	u8  Local_u8RxState     = RX_STATE_HEADER;
	u8  Local_u8ParsedCmd   = 'S';

	u8  Local_u8ActiveCmd   = 'S';
	u8  Local_u8ActivePwm   = 0;
	u8  Local_u8LinkLost    = 1;

	u8  Local_u8LastLcdCmd  = 0xFF;     // Force initial LCD draw on first loop
	u8  Local_u8LastLcdPwm  = 0xFF;
	u8  Local_u8LastLcdLink = 0xFF;

	u16 Local_u16IdleTicks  = 0;

	UART_voidInit();        // 9600 baud, 8N1 (HC-05)
	DCMOTOR_voidInit();     // Timer1 Fast PWM (2 kHz) on OC1A/OC1B, motors stopped
	CLCD_voidInit();

	// Draw static labels once at boot
	CLCD_voidClearScreen();
	CLCD_voidSetCursorPos(0, 0);
	CLCD_voidSendString((const u8 *)"Dir : NO SIGNAL ");
	CLCD_voidSetCursorPos(0, 1);
	CLCD_voidSendString((const u8 *)"PWM : 0%        ");
	
    while (1) 
    {
		// Drain all pending bytes from the UART hardware buffer
		while (UART_u8ReceiveNonBlocking(&Local_u8RxByte) == 1)
		{
			switch (Local_u8RxState)
			{
				// STATE 1: Expecting Byte 1 of 3 -> Header (0xAA)
				case RX_STATE_HEADER:
				if (Local_u8RxByte == PACKET_HEADER)
				{
					// Valid start-of-packet marker found; expect Command next
					Local_u8RxState = RX_STATE_CMD;
				}
				break;
				
				// STATE 2: Expecting Byte 2 of 3 -> Direction ('F','B','L','R','S')
				case RX_STATE_CMD:
				if (APP_u8IsValidCmd(Local_u8RxByte))
				{
					// Hold direction temporarily; wait for PWM byte before moving motors
					Local_u8ParsedCmd = Local_u8RxByte;
					Local_u8RxState   = RX_STATE_PWM;
				}
				else if (Local_u8RxByte == PACKET_HEADER)
				{
					// Previous packet was cut off after 0xAA, and a new packet just started.
					// Treat this 0xAA as the new header and stay in RX_STATE_CMD.
					Local_u8RxState = RX_STATE_CMD;
				}
				else
				{
					// Corrupted command byte -> abort packet and wait for next 0xAA
					Local_u8RxState = RX_STATE_HEADER;
				}
				break;
				
				// STATE 3: Expecting Byte 3 of 3 -> Speed Duty Cycle (0..100%)
				case RX_STATE_PWM:
				if (Local_u8RxByte <= 100)
				{
					// All 3 bytes [0xAA, Cmd, PWM] are valid -> commit to active state
					Local_u8ActiveCmd  = Local_u8ParsedCmd;
					Local_u8ActivePwm  = Local_u8RxByte;
					Local_u8LinkLost   = 0;
					Local_u16IdleTicks = 0;
					
					// Keep direction and speed consistent when stopped
					if (Local_u8ActiveCmd == 'S' || Local_u8ActivePwm == 0)
					{
						Local_u8ActiveCmd = 'S';
						Local_u8ActivePwm = 0;
					}
					
					// Apply new direction and PWM duty cycle to L293D / Timer1
					APP_voidDriveMotors(Local_u8ActiveCmd, Local_u8ActivePwm);
					
					// Packet complete -> return to State 1 to wait for next 0xAA
					Local_u8RxState = RX_STATE_HEADER;
				}
				else if (Local_u8RxByte == PACKET_HEADER)
				{
					// PWM byte was dropped and a new packet already started.
					// Discard the incomplete packet and immediately look for the new Cmd byte.
					Local_u8RxState = RX_STATE_CMD;
				}
				else
				{
					Local_u8RxState = RX_STATE_HEADER;
				}
				break;
				
				// FALLBACK: Recover safely if state variable ever corrupts
				default:
				Local_u8RxState = RX_STATE_HEADER;
				break;
			}
		}
		
		// Connection-loss failsafe: stop motors if no valid packet arrives for 600 ms
		if (Local_u16IdleTicks < RX_TIMEOUT_TICKS)
		{
			Local_u16IdleTicks++;
		}
		else if (Local_u8LinkLost == 0)
		{
			Local_u8LinkLost  = 1;
			Local_u8ActiveCmd = 'S';
			Local_u8ActivePwm = 0;
			DCMOTOR_voidStopAll();
		}
		
		// Refresh LCD only when direction, speed, or link status changes
		if (Local_u8ActiveCmd  != Local_u8LastLcdCmd ||
			Local_u8ActivePwm  != Local_u8LastLcdPwm ||
			Local_u8LinkLost   != Local_u8LastLcdLink)
		{
			APP_voidUpdateLcd(Local_u8ActiveCmd, Local_u8ActivePwm, Local_u8LinkLost);
			Local_u8LastLcdCmd  = Local_u8ActiveCmd;
			Local_u8LastLcdPwm  = Local_u8ActivePwm;
			Local_u8LastLcdLink = Local_u8LinkLost;
		}

		_delay_ms(2);
    }
	
	return 0;
}

