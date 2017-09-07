/*
 * main.c
 *
 *  Created on: 2017 Mar 05 21:05:50
 *  Author: vmsma
 */

#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)
#include <stdio.h>

/**

 * @brief main() - Application entry point
 *
 * <b>Details of function</b><br>
 * This routine is the application entry point. It is invoked by the device startup code. It is responsible for
 * invoking the APP initialization dispatcher routine - DAVE_Init() and hosting the place-holder for user application
 * code.
 */

#define PORT_0_CONTROL_4_REGISTER 0x40040014
#define PORT_0_CONTROL_8_REGISTER 0x40040018

#define START_DELAY 5000000
#define FRONT_SAMPLE_NUMBER 2
#define LINE_CAP_DELAY 1000
#define LINE_CHECK_TIME 1000
#define SURR_SAMPLE_TIME 5000
#define LINE_SAMPLE_TIME 5000
#define ACT_TIME 5000

#define DF0_TRIG 870 //700
#define DF1_TRIG 870 //650
#define CLOSE_TRIG 2100
#define SPEED_TIME 400000  //250000
#define GIVE_UP_TIME 80000
#define REV_TIME 450000
#define LINE_TURN_TIME 150000
#define SIDE_TURN_TIME 100000
#define BACK_TURN_TIME 200000
#define SEEK_TIME 500000
#define SCAN_TIME 350000
#define ALIGNMENT_FACTOR 0.9
#define V_MAX 90
#define V_TURN 70
#define V_SPEED 60
#define V_MIN 40
#define V_SEEK 55
#define V_SCAN 70
#define V_REV 50

uint8_t front_interrupt_no = 0;
uint8_t front_samples = 0;
uint8_t df0_status = 0, df1_status = 0, ds0_status = 0, ds1_status = 0, db_status = 0, l0_status = 0, l1_status = 0, close = 0;
uint8_t line_seen, side_seen;
uint8_t shp;
uint8_t alignment_v_max;
uint8_t button_status = 0;
uint32_t df0_sum = 0, df1_sum = 0;

uint32_t line_cap_timer, line_cap_timer_status = 1;
uint32_t line_check_timer, line_check_timer_status = 1;

uint32_t current_time, last_front_sample_time, last_sides_sample_time, last_line_sample_time, last_act_time, last_seen_time,
            seek_start_time, speed_start_time, scan_start_time, rev_start_time, line_turn_start_time, back_turn_start_time, side_turn_start_time;

char value[10];
char data[100];

void frontInterrupt(void)
{
	XMC_VADC_RESULT_SIZE_t result;

	uint32_t df0_avg, df1_avg;

	result = ADC_MEASUREMENT_GetGlobalResult();
	result = result >> ((uint32_t)ADC_MEASUREMENT_0.iclass_config_handle->conversion_mode_standard * (uint32_t)2);

	if(front_interrupt_no == 0)
	{
		df0_sum = df0_sum + result;
	}
	else
	{
		df1_sum = df1_sum + result;

		front_samples++;

		if(front_samples == FRONT_SAMPLE_NUMBER)
		{
			df0_avg = df0_sum / FRONT_SAMPLE_NUMBER;
			df1_avg = df1_sum / FRONT_SAMPLE_NUMBER;

			if(df0_avg > DF0_TRIG)
			{
				last_seen_time = current_time;

				df0_status = 1;
			}
			else
			{
				df0_status = 0;
			}

			if(df1_avg > DF1_TRIG)
			{
				last_seen_time = current_time;

				df1_status = 1;
			}
			else
			{
				df1_status = 0;
			}

			if((df0_avg > CLOSE_TRIG) || (df1_avg > CLOSE_TRIG))
			{
				close = 1;
			}
			else
			{
				close = 0;
			}

			df0_sum = 0;
			df1_sum = 0;

			front_samples = 0;
		}
	}

	front_interrupt_no = !front_interrupt_no;
}

void buttonInterrupt(void)
{
	if(PIN_INTERRUPT_GetPinValue(&BUTTON_INTERR) == 1)
	{
		button_status = 1;
	}
	else
	{
		button_status = 0;
	}
}

void dbInterrupt(void)
{
	if(PIN_INTERRUPT_GetPinValue(&DB_INTERR) == 0)
	{
		db_status = 1;
	}
	else
	{
		db_status = 0;
	}
}

void delay(uint32_t delay_time)
{
	uint32_t enter_time;
	uint32_t current_time;

	current_time = SYSTIMER_GetTime();

	enter_time = current_time;

	while((current_time - enter_time) < delay_time)
	{
		current_time = SYSTIMER_GetTime();
	}
}

void motors(int8_t v0, int8_t v1)
{
	if(v0 < 0)
	{
		DIGITAL_IO_SetOutputLow(&DIGITAL_0);
		PWM_SetDutyCycle(&PWM_0, (0 - 100 * v0));
	}
	else
	{
		DIGITAL_IO_SetOutputHigh(&DIGITAL_0);
		PWM_SetDutyCycle(&PWM_0, (10000 - 100 * v0));
	}

	if(v1 < 0)
	{
		DIGITAL_IO_SetOutputHigh(&DIGITAL_1);
		PWM_SetDutyCycle(&PWM_1, (10000 + 100 * v1));
	}
	else
	{
		DIGITAL_IO_SetOutputLow(&DIGITAL_1);
		PWM_SetDutyCycle(&PWM_1, (100 * v1));
	}
}

void lineCapCharged(void)
{
	uint32_t port_0_ctrl_4 = *(volatile uint32_t*)(PORT_0_CONTROL_4_REGISTER);
	uint32_t port_0_ctrl_8 = *(volatile uint32_t*)(PORT_0_CONTROL_8_REGISTER);

	port_0_ctrl_4 = port_0_ctrl_4 & (~(1 << 31));
	port_0_ctrl_8 = port_0_ctrl_8 & (~(1 << 7));

	*(volatile uint32_t*)(PORT_0_CONTROL_4_REGISTER) = port_0_ctrl_4;
	*(volatile uint32_t*)(PORT_0_CONTROL_8_REGISTER) = port_0_ctrl_8;

	line_check_timer_status = SYSTIMER_StartTimer(line_check_timer);
}

void lineEval(void)
{
	if(DIGITAL_IO_GetInput(&L0) == 0)
	{
		l0_status = 1;
	}
	else
	{
		l0_status = 0;
	}

	if(DIGITAL_IO_GetInput(&L1) == 0)
	{
		l1_status = 1;
	}
	else
	{
		l1_status = 0;
	}
}

void frontSample(void)
{
	ADC_MEASUREMENT_StartConversion(&ADC_MEASUREMENT_0);
}

void sidesSample(void)
{
	if(DIGITAL_IO_GetInput(&DS0) == 0)
	{
		ds0_status = 1;
	}
	else
	{
		ds0_status = 0;
	}

	if(DIGITAL_IO_GetInput(&DS1) == 0)
	{
		ds1_status = 1;
	}
	else
	{
		ds1_status = 0;
	}
}

void lineSample(void)
{
	uint32_t port_0_ctrl_4 = *(volatile uint32_t*)(PORT_0_CONTROL_4_REGISTER);
	uint32_t port_0_ctrl_8 = *(volatile uint32_t*)(PORT_0_CONTROL_8_REGISTER);

	port_0_ctrl_4 = port_0_ctrl_4 | (1 << 31);
	port_0_ctrl_8 = port_0_ctrl_8 | (1 << 7);

	*(volatile uint32_t*)(PORT_0_CONTROL_4_REGISTER) = port_0_ctrl_4;
	*(volatile uint32_t*)(PORT_0_CONTROL_8_REGISTER) = port_0_ctrl_8;

	DIGITAL_IO_SetOutputHigh(&L0);
	DIGITAL_IO_SetOutputHigh(&L1);

	line_cap_timer_status = SYSTIMER_StartTimer(line_cap_timer);
}

uint8_t checkLine(void)
{
	if(l0_status != 0)
	{
		line_seen = 0;

		return 1;
	}
	if(l1_status != 0)
	{
		line_seen = 1;

		return 1;
	}

	return 0;
}

uint8_t checkFront(void)
{
	if((df0_status == 1) || (df1_status == 1))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t checkBack(void)
{
	if(db_status == 1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t checkSides(void)
{
	if(ds0_status == 1)
	{
		side_seen = 0;

		return 1;
	}
	if(ds1_status == 1)
	{
		side_seen = 1;

		return 1;
	}

	return 0;
}

void seek(void);
void speed(void);
void scan(void);
void line(void);
void lineTurn(void);
void seenFront(void);
void seenBack(void);
void seenSides(void);
uint8_t checks(uint8_t kind);

void seek(void)
{
	motors(V_SEEK, V_SEEK);
}

void speed(void)
{
	motors(V_SPEED, V_SPEED);
}

void scan(void)
{
	motors(V_SCAN, (0 - V_SCAN));
}

void line(void)
{
	if(line_seen == 0)
	{
		motors((0 - V_REV), (0 - V_REV / 2));
	}
	else
	{
		motors((0 - V_REV / 2), (0 - V_REV));
	}
}

void lineTurn(void)
{
	if(line_seen == 0)
	{
		motors((0 - V_TURN), V_TURN);
	}
	else
	{
		motors(V_TURN, (0 - V_TURN));
	}
}

void seenFront(void)
{
	if((df0_status == 1) && (df1_status == 0))
	{
		alignment_v_max = ((V_MAX * 1000) * ALIGNMENT_FACTOR) / 1000;

		motors(alignment_v_max, V_MAX);
	}
	if((df0_status == 0) && (df1_status == 1))
	{
		alignment_v_max = ((V_MAX * 1000) * ALIGNMENT_FACTOR) / 1000;

		motors(V_MAX, alignment_v_max);
	}
	else
	{
		motors(V_MAX, V_MAX);
	}
}

void seenBack(void)
{
	motors((0 - V_MAX), V_MAX);
}

void seenSides(void)
{
	if(side_seen == 0)
	{
		motors((0 - V_MAX), V_MAX);
	}
	else
	{
		motors(V_MAX, (0 - V_MAX));
	}
}

uint8_t checks(uint8_t kind)
{
	if(checkLine() == 1)
	{
		rev_start_time = current_time;

		line();

		return 4;
	}
	if(kind != 6)
	{
		if(checkFront() == 1)
		{
			seenFront();

			return 6;
		}
	}
	if(kind != 7)
	{
		if(checkBack() == 1)
		{
			back_turn_start_time = current_time;

			seenBack();

			return 7;
		}
	}
	if(kind != 8)
	{
		if(checkSides() == 1)
		{
			side_turn_start_time = current_time;

			seenSides();

			return 8;
		}
	}

	return 0;
}

int main(void)
{
	uint8_t mode = 2;

	DAVE_Init();           /* Initialization of DAVE APPs  */

	motors(0, 0);

	line_cap_timer = SYSTIMER_CreateTimer(LINE_CAP_DELAY, SYSTIMER_MODE_ONE_SHOT, (void*)lineCapCharged, NULL);
	line_check_timer = SYSTIMER_CreateTimer(LINE_CHECK_TIME, SYSTIMER_MODE_ONE_SHOT, (void*)lineEval, NULL);

	DIGITAL_IO_SetOutputHigh(&ENABLE_DF0);
	DIGITAL_IO_SetOutputHigh(&ENABLE_DF1);

	while(button_status == 0)
	{

	}

	delay(START_DELAY);

	/*
	while(DIGITAL_IO_GetInput(&IR_CONTROL) == 0)
	{

	}
	*/

	current_time = SYSTIMER_GetTime();

	last_front_sample_time = 0;
	last_sides_sample_time = 0;
	last_line_sample_time = 0;
	last_act_time = 0;
	last_seen_time = 0;
	seek_start_time = 0;
	speed_start_time = current_time;
	scan_start_time = 0;
	rev_start_time = 0;
	line_turn_start_time = 0;
	back_turn_start_time = 0;
	side_turn_start_time = 0;

	while(1U)
	{
		/*
		if(DIGITAL_IO_GetInput(&IR_CONTROL) == 0)
		{
			motors(0, 0);
			while(1)
			{

			}
		}
		*/

		current_time = SYSTIMER_GetTime();

		if((current_time - last_front_sample_time) > 17000)
		{
			frontSample();

			last_front_sample_time = current_time;
		}

		if((current_time - last_sides_sample_time) > SURR_SAMPLE_TIME)
		{
			sidesSample();

			last_sides_sample_time = current_time;
		}

		if((current_time - last_line_sample_time) > LINE_SAMPLE_TIME)
		{
			lineSample();

			last_line_sample_time = current_time;
		}

		if((current_time - last_act_time) > ACT_TIME)
		{
			if(l0_status == 1)
			{
				DIGITAL_IO_SetOutputHigh(&LED_0);
			}
			else
			{
				DIGITAL_IO_SetOutputLow(&LED_0);
			}

			if(l1_status == 1)
			{
				DIGITAL_IO_SetOutputHigh(&LED_1);
			}
			else
			{
				DIGITAL_IO_SetOutputLow(&LED_1);
			}

			switch(mode)
			{
				case 1:

					shp = checks(1);

					if(shp != 0)
					{
						mode = shp;

						break;
					}

					if((current_time - seek_start_time) > SEEK_TIME)
					{
						scan_start_time = current_time;

						scan();

						mode = 3;

						break;
					}

					seek();

					break;

				case 2:

					shp = checks(2);

					if(shp != 0)
					{
						mode = shp;

						break;
					}

					if((current_time - speed_start_time) > SPEED_TIME)
					{
						scan_start_time = current_time;

						scan();

						mode = 3;

						break;
					}

					speed();

					break;

			    case 3:

			    	shp = checks(3);

					if(shp != 0)
					{
						mode = shp;

						break;
					}

			    	if((current_time - scan_start_time) > SCAN_TIME)
			    	{
			    		seek_start_time = current_time;

			    		seek();

			    		mode = 1;

			    		break;
			    	}

			    	scan();

			    	break;

			    case 4:

			    	line();

			    	if((current_time - rev_start_time) > REV_TIME)
			    	{
			    		line_turn_start_time = current_time;

			    		lineTurn();

			    		mode = 5;

			    		break;
			    	}

			    	break;

			    case 5:

			    	lineTurn();

			    	if((current_time - line_turn_start_time) > LINE_TURN_TIME)
			    	{
			    		speed_start_time = current_time;

			    		speed();

			    		mode = 2;

			    		break;
			    	}

			    	break;

			    case 6:

			    	shp = checks(6);

			    	if(checkFront() == 1)
			    	{
				    	if(shp == 4)
				    	{
				    		if(close == 1)
				    		{
					    		seenFront();
				    		}
				    		else
				    		{
				    			mode = shp;

				    			break;
				    		}
				    	}
				    	else
				    	{
				    		seenFront();
				    	}
			    	}
			    	else
			    	{
						if(shp != 0)
						{
							mode = shp;

							break;
						}

			    	    if((current_time - last_seen_time) > GIVE_UP_TIME)
			    	    {
			    	    	seek_start_time = current_time;

			    	    	seek();

			    	    	mode = 1;

			    	    	break;
			    	    }
			    	}

			    	break;

			    case 7:

			    	seenBack();

			    	shp = checks(7);

					if(shp != 0)
					{
						mode = shp;

						break;
					}

			    	if((current_time - back_turn_start_time) > BACK_TURN_TIME)
			    	{
			    		seek_start_time = current_time;

			    		seek();

			    		mode = 1;

			    		break;
			    	}

			    	break;

			    case 8:

			    	seenSides();

			    	shp = checks(8);

					if(shp != 0)
					{
						mode = shp;

						break;
					}

			        if((current_time - side_turn_start_time) > SIDE_TURN_TIME)
			        {
			    		seek_start_time = current_time;

			    		seek();

			    		mode = 1;

			    		break;
			        }

			    	break;

			    default:

			    	break;
			}

			/*
			sprintf(value, "%d", (int)l0_status);

			strcpy(data, value);
			strcat(data, "\r\n");

			UART_Transmit(&UART_0, (uint8_t *)data, sizeof(uint8_t));
			*/

			last_act_time = current_time;
		}
	}

	return 0;
}
