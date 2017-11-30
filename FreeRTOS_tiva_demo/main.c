/* FreeRTOS 8.2 Tiva Demo
 *
 * main.c
 *
 * Andy Kobyljanec
 *
 * This is a simple demonstration project of FreeRTOS 8.2 on the Tiva Launchpad
 * EK-TM4C1294XL.  TivaWare driverlib sourcecode is included.
 */

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "drivers/pinout.h"
#include "utils/uartstdio.h"


// TivaWare includes
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define LSM6DS3_ADDR		(0x6A)
#define TMP102_ADDR			(0x48)


// Global instance structure for the I2C master driver.
//tI2CMInstance g_sI2CInst;

// Demo Task declarations
void demoI2CTask(void *pvParameters);
//void demoSerialTask(void *pvParameters);

void I2C_Init(void)
{
	// The I2C2 peripheral must be enabled before use.
	// On GPIO PortL
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);

	//
	// Wait for the Peripheral to be ready for programming
	//
	while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL));

	//
	// Configure the pin muxing for I2C2 functions on port L0 and L1.
	// This step is not necessary if your part does not support pin muxing.
	//
	ROM_GPIOPinConfigure(GPIO_PL1_I2C2SCL);
	ROM_GPIOPinConfigure(GPIO_PL0_I2C2SDA);

	//
	// Select the I2C function for these pins.  This function will also
	// configure the GPIO pins pins for I2C operation, setting them to
	// open-drain operation with weak pull-ups.  Consult the data sheet
	// to see which functions are allocated per pin.
	//
	ROM_GPIOPinTypeI2C(GPIO_PORTL_BASE, GPIO_PIN_0);
	GPIOPinTypeI2CSCL(GPIO_PORTL_BASE, GPIO_PIN_1);

	//
	// Stop the Clock, Reset and Enable I2C Module
	// in Master Function
	//
	ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C2);
	ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);

	//
	// Wait for the Peripheral to be ready for programming
	//
	while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C2));

	//
	// Initialize and Configure the Master Module
	//
	ROM_I2CMasterInitExpClk(I2C2_BASE, SYSTEM_CLOCK, true);		//400kbps

	//
	// Enable Interrupts for Arbitration Lost, Stop, NAK, Clock Low
	// Timeout and Data.
	//
//	ROM_I2CMasterIntEnableEx(I2C2_BASE, (I2C_MASTER_INT_ARB_LOST |
//	I2C_MASTER_INT_STOP | I2C_MASTER_INT_NACK |
//	I2C_MASTER_INT_TIMEOUT | I2C_MASTER_INT_DATA));

	//
	// Enable the Interrupt in the NVIC from I2C Master
	//
//	ROM_IntEnable(INT_I2C2);

}


// Main function
int main(void)
{
    // Initialize system clock to 120 MHz
    uint32_t output_clock_rate_hz;
    output_clock_rate_hz = ROM_SysCtlClockFreqSet(
                               (SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                               SYSTEM_CLOCK);
    ASSERT(output_clock_rate_hz == SYSTEM_CLOCK);

    // Initialize the GPIO pins for the Launchpad
    PinoutSet(false, false);

    // Set up the UART which is connected to the virtual COM port
	UARTStdioConfig(0, 57600, SYSTEM_CLOCK);

	I2C_Init();

    // Create demo tasks
    xTaskCreate(demoI2CTask, (const portCHAR *)"I2C",
                configMINIMAL_STACK_SIZE, NULL, 1, NULL);

//    xTaskCreate(demoSerialTask, (const portCHAR *)"Serial",
//                configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    vTaskStartScheduler();
    return 0;
}


// Flash the LEDs on the launchpad
void demoI2CTask(void *pvParameters)
{
	uint8_t returned_data[2];
	uint16_t bit_shift;

//	bit_shift = ((returned_data[0] << 8) | returned_data[1]) >> 4 ;
//	temp = bit_shift * 0.0625;      //Multiplier value for temp sensor

    for (;;)
    {
//        SysCtlDelay(10);
        // Turn on LED 1
        LEDWrite(0x0F, 0x01);
        vTaskDelay(1000);

        // Turn on LED 2
        LEDWrite(0x0F, 0x02);
        vTaskDelay(1000);

        // Turn on LED 3
        LEDWrite(0x0F, 0x04);
        vTaskDelay(1000);

        // Turn on LED 4
        LEDWrite(0x0F, 0x08);
        vTaskDelay(1000);

        //writing
        ROM_I2CMasterSlaveAddrSet(I2C2_BASE, TMP102_ADDR, false);   //write to tmp sensor
        ROM_I2CMasterDataPut(I2C2_BASE, 0x01);
        ROM_I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);
        while(ROM_I2CMasterBusy(I2C2_BASE));

        //reading byte1
        ROM_I2CMasterSlaveAddrSet(I2C2_BASE, TMP102_ADDR, true);	//read from tmp sensor
        ROM_I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
        while(ROM_I2CMasterBusy(I2C2_BASE));
        returned_data[0] = ROM_I2CMasterDataGet(I2C2_BASE);
        UARTprintf("config reg1 is 0x%x\n", returned_data[0]);

        //reading byte2
        ROM_I2CMasterSlaveAddrSet(I2C2_BASE, TMP102_ADDR, true);    //read from tmp sensor
        ROM_I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        while(ROM_I2CMasterBusy(I2C2_BASE));
        returned_data[1] = ROM_I2CMasterDataGet(I2C2_BASE);
        UARTprintf("config reg2 is 0x%x\n\n", returned_data[1]);


//		bit_shift = ((returned_data[0] << 8) | returned_data[1]) >> 4 ;
//		temp = bit_shift * 0.0625;      //Multiplier value for temp sensor
    }
}


// Write text over the Stellaris debug interface UART port
//void demoSerialTask(void *pvParameters)
//{
//
//    for (;;)
//    {
//        UARTprintf("\r\nHello, world from FreeRTOS 9.0!");
//        vTaskDelay(5000 / portTICK_PERIOD_MS);
//    }
//}

/*  ASSERT() Error function
 *
 *  failed ASSERTS() from driverlib/debug.h are executed in this function
 */
void __error__(char *pcFilename, uint32_t ui32Line)
{
    // Place a breakpoint here to capture errors until logging routine is finished
    while (1)
    {
    }
}
