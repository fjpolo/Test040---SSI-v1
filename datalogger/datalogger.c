/*-------------------------------------------------------------------------------------------
 ********************************************************************************************
 *-------------------------------------------------------------------------------------------
 *
 *				DATA LOGGER DE VARIABLES AMBIENTALES INTERNAS
 *							CIMEC CONICET - UTN FRP
 *								     2016
 *
 *						Polo, Franco		fjpolo@frp.utn.edu.ar
 *						Burgos, Sergio		sergioburgos@frp.utn.edu.ar
 *						Bre, Facundo		facubre@cimec.santafe-conicet.gov.ar
 *
 *	datalogger.c
 *
 *	Descripción:
 *
 *  Desarrollo del firmware de la placa base del data logger, constando de:
 *
 *  - Periféricos I2C:
 *  	a) HR y Tbs		HIH9131		0b0100111		0x27
 *  	b) Ev			TSL2563		0b0101001		0x29
 *  	c) Va			ADS			0b1001000		0x48
 *  	d) Tg			LM92		0b1001011		0x51
 *  	e) RTC			DS1703		0b1101000		0x68
 *
 *  - Periféricos OneWire@PD6
 *  	a) Ts01			MAX31850	ROM_Addr		0x3B184D8803DC4C8C
 *  	b) Ts02			MAX31850	ROM_Addr		0x3B0D4D8803DC4C3C
 *  	c) Ts03			MAX31850	ROM_Addr		0x3B4D4D8803DC4C49
 *  	d) Ts04			MAX31850	ROM_Addr		0x3B234D8803DC4C99
 *  	e) Ts05			MAX31850	ROM_Addr		0x3B374D8803DC4C1E
 *  	f) Ts06			MAX31850	ROM_Addr
 *
 *  - IHM
 *  	a) RESET		!RST
 *  	b) SW_SD		PC6
 *  	c) SW_ON		PC5
 *  	d) SW_1			PC7
 *  	e) WAKE			PF2
 *  	f) LEDON		PE0
 *  	g) LED1			PE1
 *  	h) LED2			PE2
 *
 *  - SD
 *  	a) SD_IN		PA6
 *  	b) SD_RX		PA4
 *  	c) SD_TX		PA5
 *  	d) SD_CLK		PA2
 *  	e) SD_FSS		PA3
 *
 *--------------------------------------------------------------------------------------------
 *********************************************************************************************
 *-------------------------------------------------------------------------------------------*/

/*********************************************************************************************
 * INCLUDES
 ********************************************************************************************/
// standard C
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
// inc
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_i2c.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ssi.h"
// driverlib
#include "driverlib/fpu.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/hibernate.h"
#include "driverlib/i2c.h"
#include "driverlib/ssi.h"
// datalogger
#include "datalogger/datalogger.h"
#include "datalogger/delay.h"
// third_party
#include "fatfs/src/ff.h"
#include "fatfs/src/diskio.h"

/*****************************************************************************************************
 * Defines
 ****************************************************************************************************/
// SWITCHES
#define SW_PORT 	GPIO_PORTC_BASE
#define SW_ON 		GPIO_PIN_5
#define SW_SD 		GPIO_PIN_6
#define SW_1 		GPIO_PIN_7
// LEDS
#define LED_PORT 	GPIO_PORTE_BASE
#define LED_ON 		GPIO_PIN_0
#define LED_1 		GPIO_PIN_1
#define LED_2 		GPIO_PIN_2
// SD Card
#define SD_PORT		GPIO_PORTA_BASE
#define SD_IN		GPIO_PIN_6
#define SD_RX		GPIO_PIN_4
#define SD_TX		GPIO_PIN_5
#define SD_CLK		GPIO_PIN_2
#define SD_FSS		GPIO_PIN_3
// Timer0
#define TOGGLE_FREQUENCY 1
// Tg
#define SLAVE_ADDRESS_TG 0x4B
#define TG_REG_READ 0x00
#define TG_REG_LOWPOW 0x01
// RTC
#define SLAVE_ADDR_RTC 0x68
#define SEC 0x00
#define MIN 0x01
#define HRS 0x02
#define DAY 0x03
#define DATE 0x04
#define MONTH 0x05
#define YEAR 0x06
#define CNTRL 0x07
// Va ADC slave address
#define SLAVE_ADDR_VA 0x48
#define VA_REG_READ 0x00
// I2C3
#define GPIO_PD0_I2C3SCL        0x00030003
#define GPIO_PD1_I2C3SDA        0x00030403
// SSI1
#define SSI1_PORT				GPIO_PORTF_BASE
#define SSI1_RX					GPIO_PIN_0
#define SSI1_TX					GPIO_PIN_1
#define SSI1_CLK				GPIO_PIN_2
#define SSI1_CS					GPIO_PIN_3


/*********************************************************************************************
 * Functions
 * ******************************************************************************************/
extern void Hibernate_IRQHandler (void);

/*********************************************************************************************
 * Global variables
 * ******************************************************************************************/
extern unsigned long int _MSDELAY_BASE;
extern unsigned long int _USDELAY_BASE;
unsigned long ulPeriod;
extern unsigned long ulNVData[3];

/*********************************************************************************************
 * Initialize
 * ******************************************************************************************/
void Initialize(void){
	InitClock();
	InitGPIO();
	//InitTimer0();
	//InitGPIOInt();
	//InitSDCard();
	//InitHibernation();
	InitSPI1();
}

/*********************************************************************************************
 * InitClock
 * ******************************************************************************************/
void InitClock(void){
	// 400Mhz PLL
	// Half divider implicit
	// Sysdiv /5
	// Clock = 500/(5x2)= 40MHz
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	// Delay values
	_MSDELAY_BASE = (SysCtlClockGet()/3)/1000;
	_USDELAY_BASE = _MSDELAY_BASE/1000;

}
/*********************************************************************************************
 * InitGPIO
 * ******************************************************************************************/
void InitGPIO(void){
	//
	// IHM
	//
	//Entradas
	//Habilito el puerto C
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlDelay(10);
	//PC5,PC6PC7 como entradas
	GPIOPinTypeGPIOInput(SW_PORT, SW_ON|SW_SD|SW_1);
	//Enable pull-up resistor
	GPIOPadConfigSet(SW_PORT,SW_ON,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(SW_PORT,SW_SD,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(SW_PORT,SW_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	//
	//Salidas
	//Habilito el puerto E
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlDelay(10);
	//PE0,PE1 y PE2 salidas
	GPIOPinTypeGPIOOutput(LED_PORT, LED_ON|LED_1|LED_2);

}

/*********************************************************************************************
 * InitTimer0
 * ******************************************************************************************/
void InitTimer0(void){
	// Timer0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE,TIMER_CFG_A_PERIODIC);
	// Period
	ulPeriod=(SysCtlClockGet()/TOGGLE_FREQUENCY)/2;
	TimerLoadSet(TIMER0_BASE,TIMER_A,ulPeriod-1);
	// Enable interrupts
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
	IntMasterEnable();
}

/*********************************************************************************************
 * InitGPIOInt
 * ******************************************************************************************/
void InitGPIOInt(){
	//
	// Interrupt using GPIO
	//
	GPIOIntTypeSet(SW_PORT, SW_1, GPIO_RISING_EDGE );
	GPIOIntEnable(SW_PORT, SW_1);
	IntMasterEnable();
}

/*********************************************************************************************
 * InitHibernation
 * ******************************************************************************************/
void InitHibernation(void){
	// Enable Hibernate module
	SysCtlPeripheralEnable(SYSCTL_PERIPH_HIBERNATE);
	// Clock source
	HibernateEnableExpClk(SysCtlClockGet());
}

/*********************************************************************************************
 * DataLoggingON
 * ******************************************************************************************/
void DataLoggingON(unsigned long *Status){
	// Hibernate clock
	HibernateEnableExpClk(SysCtlClockGet());
	//Retencion de estado de los pines
	HibernateGPIORetentionEnable();
	//HibernateClockConfig(HIBERNATE_OSC_LOWDRIVE);
	SysCtlDelay(SysCtlClockGet()/10);
	//HibernateRTCTrimSet(0x7FFF);
	HibernateRTCEnable();
	//RTC
	HibernateRTCSet(0);
	//HibernateRTCMatch0Set(5);
	HibernateRTCMatchSet(0,5);
	//HibernateRTCMatch0Set(HibernateRTCGet() + 30);
	//Condiciones de wake
	HibernateWakeSet(HIBERNATE_WAKE_PIN | HIBERNATE_WAKE_RTC);
	// Recupero info
	HibernateDataSet(ulNVData, 2);
	//Interrupciones
	HibernateIntEnable(HIBERNATE_INT_RTC_MATCH_0 | HIBERNATE_INT_PIN_WAKE);
	// Clear any pending status.
	*Status = HibernateIntStatus(0);
	HibernateIntClear(*Status);
	//
	HibernateIntClear(HIBERNATE_INT_PIN_WAKE | HIBERNATE_INT_LOW_BAT | HIBERNATE_INT_RTC_MATCH_0);
	HibernateIntRegister(Hibernate_IRQHandler);
	//
	//Hibernamos
	//
	HibernateRequest();
}
/*********************************************************************************************
 * DataLoggingOFF
 * ******************************************************************************************/
void DataLoggingOFF(void){
	// Hibernation module disabled
	HibernateDisable();
}
/*********************************************************************************************
 * InitSDCard
 * ******************************************************************************************/
void InitSDCard(void){
	// Habilito el puerto A
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlDelay(10);
	// PA6
	GPIOPinTypeGPIOInput(SD_PORT, SD_IN);
	// Enable pull-up resistor
	GPIOPadConfigSet(SD_PORT, SD_IN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

}

/*********************************************************************************************
 * InitI2C3
 * ******************************************************************************************/
void InitI2C3(void){}

/*********************************************************************************************
 * InitSPI1
 * ******************************************************************************************/
void InitSPI1(void){
	// PF0 Rx
	// PF1 Tx Not used
	// PF2 Clk
	// PF3 Fss/CS
	//
	// Enable SSI1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlDelay(10);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
	SysCtlDelay(10);
	// Unlock PF0
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_0;
	HWREG(GPIO_PORTF_BASE + GPIO_O_AFSEL) |= GPIO_PIN_0;
	HWREG(GPIO_PORTF_BASE + GPIO_O_DEN) |= GPIO_PIN_0;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	//SSIDisable(SSI1_BASE);
	SysCtlDelay(10);
	// Set clock source
	SSIClockSourceSet(SSI1_BASE, SSI_CLOCK_SYSTEM);
	// GPIO config
	GPIOPinConfigure(GPIO_PF2_SSI1CLK);
	//GPIOPinConfigure(GPIO_PF3_SSI1FSS);
	GPIOPinConfigure(GPIO_PF0_SSI1RX);
	GPIOPinConfigure(GPIO_PF1_SSI1TX);
	GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);
	SSIConfigSetExpClk(SSI1_BASE, 400000, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000, 16);
	// CS/FSS as GPIO
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
	// Enable SSI1
	SSIEnable(SSI1_BASE);
}
