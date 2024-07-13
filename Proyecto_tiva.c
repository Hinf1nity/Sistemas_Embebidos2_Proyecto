//*****************************************************************************
//*****************************************************************************
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.c"

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif

//PF2 Derecha    PL0 y PL1
//PF3 Izquierda  PL2 y PL3

void config_pwm();
void config_uart();

char data[20]="0";
float vel1=0.8, vel2=0.9, vel3=1.0, vel=0.9;
uint8_t state=0;
volatile uint32_t width_i=500;
volatile uint32_t width_d=490;

int main(void)
{
	SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

	config_pwm();
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
	GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, 0x0F);
	GPIOPinWrite(GPIO_PORTL_BASE, 0x0F, 0x00);
	config_uart();

	while (1) {
		UARTgets(data, 20);
		state=atoi(data);
		if(state==0){
			;
		}
		else if(state==1){
			GPIOPinWrite(GPIO_PORTL_BASE, 0x0F, 0x05);
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, width_d*vel);
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, width_i*vel);
		}
		else if(state==2){
			GPIOPinWrite(GPIO_PORTL_BASE, 0x0F, 0x04);
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, width_d*vel);
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, width_i*vel);

		}
		else if(state==3){
			GPIOPinWrite(GPIO_PORTL_BASE, 0x0F, 0x01);
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, width_d*vel);
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, width_i*vel);

		}
		else if(state==4){
			GPIOPinWrite(GPIO_PORTL_BASE, 0x0F, 0x00);
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 0);
		}
		else if(state==5){
			vel=vel1;
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, width_d*vel);
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, width_i*vel);
		}
		else if(state==6){
			vel=vel2;
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, width_d*vel);
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, width_i*vel);
		}
		else if(state==7){
			vel=vel3;
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, width_d*vel);
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, width_i*vel);
		}
		if(state!=0){
			state=0;
		}
	}
}

void config_pwm(){
	//Enable the PWM0 peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, 0x0C);
	//Configuer the PWM function for this pin
	GPIOPinConfigure(GPIO_PF2_M0PWM2);
	GPIOPinConfigure(GPIO_PF3_M0PWM3);
	GPIOPinTypePWM(GPIO_PORTF_BASE, 0x0C);
	//Wait for the PWM0 module to be ready.
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
	//COnfigure PWM clock divisor. Use system clock / 8.
	PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_8);
	//Configure the PWM generator for count down mode with immediate updates to the parameters.
	PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	//Set the period. For a 30 KHz frequency, the period = 1/30000, or 33 microseconds.
	//For a 15 MHz clock, this translates to 500 clock ticks. Use this value to set the period.
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 500);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 0);
	//Start the timers in generator 1.
	PWMGenEnable(PWM0_BASE, PWM_GEN_1);
	//Enable the outputs.
	PWMOutputState(PWM0_BASE, (PWM_OUT_2_BIT | PWM_OUT_3_BIT), true);
}

void config_uart(){
	//Enable the peripherals used by this example.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	//Configure GPIO Pins for UART mode.
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, 0x03);
	//Initialize the UART for console I/O.
	UARTStdioConfig(0, 9600, 120000000);
}
