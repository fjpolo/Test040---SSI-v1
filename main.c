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
 *	main.c
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
// str2string
#define MAXDIGIT 10
// SSI1
#define SSI1_PORT				GPIO_PORTF_BASE
#define SSI1_RX					GPIO_PIN_0
#define SSI1_TX					GPIO_PIN_1
#define SSI1_CLK				GPIO_PIN_2
#define SSI1_CS					GPIO_PIN_3
#define GPIO_PF0_SSI1RX         0x00050002
#define GPIO_PF1_SSI1TX         0x00050402
#define GPIO_PF2_SSI1CLK        0x00050802
#define GPIO_PF3_SSI1FSS        0x00050C02

/*********************************************************************************************
 * Global variables
 * ******************************************************************************************/
//
int SWRead;
//
int SWRead, SD_Read;
// Estado del modulo hibernacion
unsigned long ulStatus;
unsigned long ulPeriod;
//Datos a mantener durante la hibernacion
//0: sensor_flag
//1: hibernate_flag
//2: Tg y RTC
unsigned long ulNVData[3] = { 1, 0, 0};
//
//static unsigned long g_ulDataRx, MSB,LSB, Sign;
static unsigned long Tg_Raw, Tg_MSB,Tg_LSB, Tg_Sign;
unsigned char sec,min,hour,day,date,month,year;
float Tg=0;
//static string Dias[7] = {'Lunes', 'Martes', 'Miercoles', 'Jueves', 'Viernes', 'Sabado', 'Domingo'};
// str2string
typedef struct
{
	unsigned int data[MAXDIGIT];
	unsigned int p;
	unsigned int mx;
}stStack;
stStack stack = {{0}, 0, MAXDIGIT};
//
// SD Card variables
//
FATFS FatFs;    /* Work area (file system object) for logical drive */
FIL fil;        /* File object */
//
FRESULT fr;     /* FatFs return code */
UINT br;    /* File read count */
//
DIR *dir;
static FILINFO fileInfo;
//
int i=0;
const char str_date[] = "Fecha";
const char str_hour[] = "Hora";
const char str_tg[] = "Temperatura de globo [ºC]";
const char str_hr[] = "Humedad relativa [%]";
const char str_tbs[] = "Temperatura de bulbo seco [ºC]";
const char str_ev[] = "Iluminancia [lux]";
const char str_va[] = "Velocidad de aire [m/s]";
const char newline[] = "\n\r\r";
const char comma[] = ";";
char dir_name[8];
char full_date[8];
char full_time[8];
char file_name[9];
char str[8];
int idir_name;
int ifile_name;
const char null[]="0";
int filesize;
uint16_t txdata=0xAA;
int HR_LSB = 0;
int HR_MSB = 0;
uint32_t pui32DataRx[2];
double t_bs;
double hr;
/*********************************************************************************************
 * Main loop
 *
 * - Testing HIH9131 using SSI1 module
 *
 ********************************************************************************************/

int main(void)
{

	//
	// Initialization
	Initialize();
	//
	while(1)
	{
		// TODO: Measurement Request command
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
		SSIDataPutNonBlocking(SSI1_BASE, 0xAA);
		SSIDataGet(SSI1_BASE, &txdata);
		delayUS(160);
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
		// TODO: Conversion time: 36.65mS
		delayMS(40);
		// TODO: Data Fetch
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
		SSIDataPutNonBlocking(SSI1_BASE, 0x0000);
		SSIDataGet(SSI1_BASE, &pui32DataRx[0]);
		SSIDataPutNonBlocking(SSI1_BASE, 0x0000);
		SSIDataGet(SSI1_BASE, &pui32DataRx[1]);
		delayUS(330);
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
		// hr
		hr = pui32DataRx[0] & 0x3FFF;
		hr = hr*100;
		hr = hr/16382;
		// tbs
		t_bs = pui32DataRx[1] & 0x3FFF;
		t_bs = t_bs*165;
		t_bs = t_bs/16382;
		t_bs = t_bs-40;
		//
		delayUS(310);
	}
}
