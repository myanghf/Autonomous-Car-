/*****************************************************************************
 *   EE2024 Assignment 2
 *
 *   Wednesday Lab Group
 *   Yang Hanfu Michael A0155097X
 *	 Yeo Wen Jie A0162193H
 ******************************************************************************/

//LPC17xx Libraries
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_clkpwr.h"

//Generic Libraries
#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "stdio.h"
#include "temp.h"
#include "led7seg.h"
#include "light.h"
#include "stdbool.h"
#include "inttypes.h"
#include "uart2.h"
#include "string.h"
#include "float.h"

//Declaration of Constants
static const double TEMP_HIGH_THRESHOLD = 28.0; //temp threshold 28 degrees celsius
static const double ACC_THRESHOLD = 0.4; //acceleration of 0.4g
static const int OBSTACE_NEAR_THRESHOLD = 3000; //light threshold of 3000 lux
static const int ONE_SECOND_TICKS = 1000; //1000 ticks of 1ms each in 1 second
static const int BLINK_TICKS = 333; //333ms blinky

//Declaration of the 3 modes of vehicle: Stationary, Forward, Reverse.
//Default to Stationary when powered on.
typedef enum{
	Stationary,
	Forward,
	Reverse,
} Mode;
Mode currMode = Stationary;

//Initialisation of variables
uint8_t MODE_TOGGLE =0;
int8_t x=0, y=0, z=0, xoff = 0;
double currTemp =0;
uint32_t currLight =0;
int currSeg =0;
int RATcount = 0;
bool exceedTemp = 0;
bool exceedAcc = 0;
bool exceedLight = 0;
volatile bool oneSecond = 0;
volatile bool lighted = 0;
volatile uint8_t counter;
volatile bool wait = 0;

//Declaration of character strings for OLED and UART
char OLED_STATIONARY[] = "STATIONARY";
char OLED_FORWARD[] = "FORWARD";
char OLED_REVERSE[] = "REVERSE";
char OLED_UPDATE[64] = {};
char UART_UPDATE[64] = {};

static uint32_t getMsTicks (){
	return LPC_TIM0->TC;
}

void myOneSecondTimer (){
	TIM_TIMERCFG_Type TIM_ConfigStruct;
	TIM_MATCHCFG_Type TIM_MatchConfigStruct ;

// Initialize timer0, prescale count time of 1ms
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1000;

	TIM_MatchConfigStruct.MatchChannel = 0;
	TIM_MatchConfigStruct.IntOnMatch   = TRUE;
	TIM_MatchConfigStruct.ResetOnMatch = FALSE;
	TIM_MatchConfigStruct.StopOnMatch  = FALSE;
	TIM_MatchConfigStruct.ExtMatchOutputType =TIM_EXTMATCH_NOTHING;
	TIM_MatchConfigStruct.MatchValue = 1000; //1000ms

	TIM_Init(LPC_TIM0, TIM_TIMER_MODE,&TIM_ConfigStruct);
	TIM_ConfigMatch(LPC_TIM0,&TIM_MatchConfigStruct);
	TIM_Cmd(LPC_TIM0,ENABLE);
}

void my333Timer (){
	TIM_TIMERCFG_Type TIM_ConfigStruct1;
	TIM_MATCHCFG_Type TIM_MatchConfigStruct1;

// Initialize timer1, prescale count time of 1ms
	TIM_ConfigStruct1.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct1.PrescaleValue	= 1000;

	TIM_MatchConfigStruct1.MatchChannel = 1;
	TIM_MatchConfigStruct1.IntOnMatch   = TRUE;
	TIM_MatchConfigStruct1.ResetOnMatch = TRUE;
	TIM_MatchConfigStruct1.StopOnMatch  = FALSE;
	TIM_MatchConfigStruct1.ExtMatchOutputType =TIM_EXTMATCH_NOTHING;
	TIM_MatchConfigStruct1.MatchValue = 333; //333ms

	TIM_Init(LPC_TIM1, TIM_TIMER_MODE,&TIM_ConfigStruct1);
	TIM_ConfigMatch(LPC_TIM1,&TIM_MatchConfigStruct1);
	TIM_Cmd(LPC_TIM1,ENABLE);
}

void EINT3_IRQHandler(void){
}

void EINT0_IRQHandler(void) {
	//sw3
	LPC_SC->EXTINT = 1;
	MODE_TOGGLE = 1;
}

void TIMER0_IRQHandler(void) {
	TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
	wait = 0;
}

void TIMER1_IRQHandler(void) {
	TIM_ClearIntPending(LPC_TIM1, TIM_MR1_INT);

	if (counter == 3) {
		oneSecond = 1;
		counter = 1;
	} else {
		counter++;
	}

	lighted = !lighted;
    if (exceedTemp && exceedAcc) {
		if (lighted) {
			GPIO_ClearValue (0, (1<<26));
			GPIO_SetValue( 2, 1);
		} else {
			GPIO_ClearValue( 2, 1 );
			GPIO_SetValue (0, (1<<26));
		}
	} else if (exceedTemp) {
		if (lighted) {
			GPIO_SetValue( 2, 1);
		} else {
			GPIO_ClearValue( 2, 1 );
		}
	} else if (exceedAcc) {
		if (lighted) {
			GPIO_SetValue( 0, (1<<26));
		} else {
			GPIO_ClearValue( 0, (1<<26));
		}
	}
}

//SSP Initialisation
static void init_ssp(void)
{
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);
}

//I2C Initialisation
static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 10; //SDA P0.10
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11; //SCL P0.11
	PINSEL_ConfigPin(&PinCfg);

	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 3; //ACC 1 P0.3
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 5; //ACC 2 & LIGHT P2.5
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 1; //OLED 1 P2.1
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

//GPIO Initialisation
static void init_GPIO(void)
{
	PINSEL_CFG_Type PinCfg;

	//sw4 push button P1.31
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1,1<<31,0);

	//RGB RED P2.0
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2,1,1);

	//sw3 push button P2.10
	PinCfg.Funcnum = 1; //EINT0
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2,1<<10,0);


	//temp sensor P0.2
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(0,1<<2,0);
}

//UART Pin Selection
void pinsel_uart3(void){
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 0;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);
}

//UART Initialisation
void init_uart(void) {
	UART_CFG_Type uartCfg;
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;
	uartCfg.Baud_rate = 115200;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity = UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;
	//pin select for uart3;
	pinsel_uart3();
	//supply power & setup working parameters for uart3
	UART_Init(LPC_UART3, &uartCfg);

	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
	// Init FIFO for UART3
	UART_FIFOConfig(LPC_UART3, &UARTFIFOConfigStruct);
	//enable transmit for uart3
	UART_TxCmd(LPC_UART3, ENABLE);
}

//Sending of message to UART terminal
void uartSendTempWarning () {
	char UART_TEMP_WARNING[64] = "Temperature too high. \r\n";
	UART_Send(LPC_UART3, (uint32_t *)UART_TEMP_WARNING, strlen(UART_TEMP_WARNING), BLOCKING);
}

void uartSendAccWarning () {
	char UART_ACC_WARNING[64] = "Collision has been detected. \r\n";
	UART_Send(LPC_UART3, (uint32_t *)UART_ACC_WARNING, strlen(UART_ACC_WARNING), BLOCKING);
}

void uartSendLightWarning () {
	char UART_LIGHT_WARNING[64] = "Obstacle too near. \r\n";
	UART_Send(LPC_UART3, (uint32_t *)UART_LIGHT_WARNING, strlen(UART_LIGHT_WARNING), BLOCKING);
}

uint32_t getPClock (uint32_t timernum)
{
	uint32_t clkdlycnt;
	switch (timernum)
	{
	case 0:
		clkdlycnt = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_TIMER0);
		break;

	case 1:
		clkdlycnt = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_TIMER1);
		break;

	case 2:
		clkdlycnt = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_TIMER2);
		break;

	case 3:
		clkdlycnt = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_TIMER3);
		break;
	}
	return clkdlycnt;
}

uint32_t myTIM_ConverUSecToVal (uint32_t timernum, uint32_t usec)
{
	uint64_t clkdlycnt;

	// Get Pclock of timer
	clkdlycnt = (uint64_t) getPClock (timernum);

	clkdlycnt = (clkdlycnt * usec) / 1000000;
	return (uint32_t) clkdlycnt;
}

//Global Initialisation
static void init_all(void){
	init_i2c();
	init_ssp();
	init_GPIO();
	init_uart();

	pca9532_init();
	rgb_init();
	temp_init(&getMsTicks);
	acc_init();

	acc_init();
	acc_read(&x, &y, &z);
	xoff = 0 - x;

	led7seg_init();
	led7seg_setChar(0xFF, 1);

	light_init();
	light_enable();
	light_setRange(LIGHT_RANGE_4000);

	oled_init();
	oled_clearScreen(OLED_COLOR_BLACK);

	//interrupts
    NVIC_SetPriorityGrouping(5);
    NVIC_SetPriority (EINT0_IRQn, 0x01);
    NVIC_SetPriority (EINT3_IRQn, 0x04);
    NVIC_SetPriority (TIMER1_IRQn, 0x02);
    NVIC_SetPriority (TIMER0_IRQn, 0x03);
	NVIC_ClearPendingIRQ(EINT0_IRQn);
	NVIC_EnableIRQ(EINT0_IRQn);
	NVIC_EnableIRQ(TIMER0_IRQn);
	NVIC_EnableIRQ(TIMER1_IRQn);
}

void StationaryMode(){
	oled_clearScreen(OLED_COLOR_BLACK);
	oled_putString(5,5, (uint8_t *)"STATIONARY",OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	exceedTemp = 0;	exceedAcc = 0; wait = 0;
	//clearing
	pca9532_setLeds(0x0000, 0xFFFF);
	GPIO_ClearValue (0, (1<<26));
	GPIO_ClearValue( 2, 1 );

	while (currMode == Stationary) {
		if (MODE_TOGGLE == 1) {
			TIM_ResetCounter(LPC_TIM0);
			MODE_TOGGLE = 0;
			wait = 1;
			while (wait) {
				if (MODE_TOGGLE == 1) {
					MODE_TOGGLE = 0;
					currMode = Reverse;
				}
			}
			if (currMode == Stationary) {
				currMode = Forward;
			}
		}
	}
}

void ForwardMode(){
	int intTemp;
	double currTemp;
	static uint8_t chars[] = {0x24,0x7D,0xE0,0x70,0x39,0x32,0x22,0x3C,0x20,0x30,0x28,0x23,0xA6,0x61,0xA2,0xAA};
	char enterForward[64] = "Entering Forward mode.\r\n";
	float xg;
	uint8_t btnSw4;
	oled_clearScreen(OLED_COLOR_BLACK);
	oled_putString(5,5, (uint8_t *)"FORWARD",OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	UART_Send(LPC_UART3, enterForward, strlen(enterForward), BLOCKING);

	while (currMode == Forward) {
		btnSw4 = ((GPIO_ReadValue(1) >> 31) & 0x01);
		if (btnSw4 == 0) {
			exceedTemp = 0;
			exceedAcc = 0;
			oled_putString(5, 25, (uint8_t *) "Air bag", OLED_COLOR_BLACK, OLED_COLOR_BLACK);
			oled_putString(5, 35, (uint8_t *) "released", OLED_COLOR_BLACK, OLED_COLOR_BLACK);
			oled_putString(5, 45, (uint8_t *) "Temp. too", OLED_COLOR_BLACK, OLED_COLOR_BLACK);
			oled_putString(5, 55, (uint8_t *) "high", OLED_COLOR_BLACK, OLED_COLOR_BLACK);
		}
		if (oneSecond) {
			oneSecond = 0;

			//Accelerometer
			acc_read(&x, &y, &z);
			x = x + xoff;
			xg = x/16.0;

			//collision warning
			if((xg > ACC_THRESHOLD) && (btnSw4 == 1)){
				oled_putString(5, 25, (uint8_t *) "Air bag", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
				oled_putString(5, 35, (uint8_t *) "released", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
				exceedAcc = 1;
			}

			//Temperature Sensor
			intTemp = temp_read();
			currTemp = intTemp /10.0;
			if((currTemp > TEMP_HIGH_THRESHOLD) && (btnSw4 == 1)) {
				oled_putString(5, 45, (uint8_t *) "Temp. too", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
				oled_putString(5, 55, (uint8_t *) "high", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
				exceedTemp = 1;
			}

			//7 segment display
			led7seg_setChar(chars[currSeg],1);

			//updating every t = 5 A F
			sprintf(OLED_UPDATE, "X:%1.1fg T:%2.1f", xg, currTemp);
			sprintf(UART_UPDATE, "%03d_Temp_%2.2f_ACC_%05.2f\r\n",RATcount,currTemp,xg);
			if(currSeg == 5 || currSeg == 10 || currSeg ==15){
					oled_putString(5, 15, (uint8_t *) OLED_UPDATE, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			}
			if(currSeg == 15){
				RATcount++;
				if (exceedTemp) {
					uartSendTempWarning();
				}
				if (exceedAcc) {
					uartSendAccWarning();
				}
				UART_Send(LPC_UART3, (uint8_t *)UART_UPDATE, strlen(UART_UPDATE), BLOCKING);
			}

			//7seg
			if(currSeg < 15){
				currSeg ++;
			} else if (currSeg ==15) {
				currSeg =0;
			}
		}
		if(MODE_TOGGLE == 1){
			MODE_TOGGLE = 0;
			currMode = Stationary;
		}
	}
}

void ReverseMode(){
	uint8_t btnSw4;
	oled_clearScreen(OLED_COLOR_BLACK);
	oled_putString(5,5, (uint8_t *)"REVERSE", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	char enterReverse[64] = "Entering Reverse mode.\r\n";
	char obstacleWarning[64] = "Obstacle too near.\r\n";
	UART_Send(LPC_UART3, enterReverse, strlen(enterReverse), BLOCKING);

	while (currMode == Reverse) {
		btnSw4 = ((GPIO_ReadValue(1) >> 31) & 0x01);
		if (btnSw4 == 0){
			oled_putString(5, 15, (uint8_t *) "Obstacle too", OLED_COLOR_BLACK, OLED_COLOR_BLACK);
			oled_putString(5, 25, (uint8_t *) "near.", OLED_COLOR_BLACK, OLED_COLOR_BLACK);
		}
		if(oneSecond){
			oneSecond = 0;
			currLight = light_read();
			if((currLight > OBSTACE_NEAR_THRESHOLD) && (btnSw4 == 1)) {
				oled_putString(5, 15, (uint8_t *) "Obstacle too", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
				oled_putString(5, 25, (uint8_t *) "near.", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
				UART_Send(LPC_UART3, obstacleWarning, strlen(obstacleWarning), BLOCKING);
			}
			if (currLight <188) {pca9532_setLeds(0x0000, 0xFFFF);}
			if ((currLight >= 188) && (currLight < 375)) {pca9532_setLeds(0x8000, 0xFFFF);}
			if ((currLight >= 375) && (currLight < 566)) {pca9532_setLeds(0xC000, 0xFFFF);}
			if ((currLight >=566) && (currLight < 750)) {pca9532_setLeds(0xE000, 0xFFFF);}
			if ((currLight >= 750) && (currLight < 938)) {pca9532_setLeds(0xF000, 0xFFFF);}
			if ((currLight >= 938) && (currLight < 1125)) {pca9532_setLeds(0xF800, 0xFFFF);}
			if ((currLight >=1125) && (currLight < 1313)) {pca9532_setLeds(0xFC00, 0xFFFF);}
			if ((currLight >= 1313) && (currLight < 1500)) {pca9532_setLeds(0xFE00, 0xFFFF);}
			if ((currLight >= 1500) && (currLight < 1688)) {pca9532_setLeds(0xFF00, 0xFFFF);}
			if ((currLight >= 1688) && (currLight < 1875)) {pca9532_setLeds(0xFF80, 0xFFFF);}
			if ((currLight >= 1875) && (currLight < 2063)) {pca9532_setLeds(0xFFC0, 0xFFFF);}
			if ((currLight >= 2063) && (currLight < 2250)) {pca9532_setLeds(0xFFE0, 0xFFFF);}
			if ((currLight >= 2250) && (currLight < 2438)) {pca9532_setLeds(0xFFF0, 0xFFFF);}
			if ((currLight >= 2438) && (currLight < 2625)) {pca9532_setLeds(0xFFF8, 0xFFFF);}
			if ((currLight >= 2625) && (currLight < 2813)) {pca9532_setLeds(0xFFFC, 0xFFFF);}
			if ((currLight >= 2813) && (currLight < 3000)) {pca9532_setLeds(0xFFFE, 0xFFFF);}
			if (currLight >= 3000) {pca9532_setLeds(0xFFFF, 0xFFFF);}
		}

		if(MODE_TOGGLE == 1){
			MODE_TOGGLE = 0;
			currMode = Stationary;
		}
	}
}

int main (void) {
	init_all();
	my333Timer();
	myOneSecondTimer ();

    while (1){
    	if(currMode == Stationary){
    		StationaryMode();
    	}
    	if(currMode == Forward){
    		ForwardMode();
    	}
    	if(currMode == Reverse){
    		ReverseMode();
    	}

	}
}
