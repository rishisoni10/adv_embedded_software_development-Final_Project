/* APES Project_2 FreeRTOS and TivaWare test code
 *
 * main.c
 *
 * Rishi Soni & Snehal Sanghvi
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
#include "driverlib/adc.h"


// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define LSM6DS3_ADDR		(0x6B)
#define TMP102_ADDR			(0x48)
#define PEDOMETER           1
#define HEART_RATE          1

#undef HEART_RATE


// Global instance structure for the I2C master driver.
//tI2CMInstance g_sI2CInst;

uint32_t pulse_rate[1];

// Task declarations
void pedometerTask(void *pvParameters);
void heartbeatTask(void *pvParameters);
//void demoSerialTask(void *pvParameters);

void GPIO_Init(void)
{
    //PortB for GPIO r/w
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //PB2 = LO-; PB3 = LO+
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2|GPIO_PIN_3);

}


#ifdef HEART_RATE
//Setup the ADC Peripheral for the Pulse Sensor
void ADC_Init(void)
{
    SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlDelay(10);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);    //ADC Channel-0 Only

//    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_EIGHTH, 30);

    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.  Each ADC module has 4 programmable sequences, sequence 0
    // to sequence 3.
    //
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    //
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // information on the ADC sequences and steps, reference the datasheet.
    //
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
                             ADC_CTL_END);

    //
    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    //
    ADCIntClear(ADC0_BASE, 3);

    //
    // Since sample sequence 3 is now configured, it must be enabled.
    //
    ADCSequenceEnable(ADC0_BASE, 3);

}
#endif


#ifdef PEDOMETER
void I2C_Init(void)
{
	// The I2C2 peripheral must be enabled before use.
	// On GPIO PortL
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

	//
	// Wait for the Peripheral to be ready for programming
	//
	while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG));

	//
	// Configure the pin muxing for I2C2 functions on port L0 and L1.
	// This step is not necessary if your part does not support pin muxing.
	//
	ROM_GPIOPinConfigure(GPIO_PG0_I2C1SCL);
	ROM_GPIOPinConfigure(GPIO_PG1_I2C1SDA);
	//
	// Select the I2C function for these pins.  This function will also
	// configure the GPIO pins pins for I2C operation, setting them to
	// open-drain operation with weak pull-ups.  Consult the data sheet
	// to see which functions are allocated per pin.
	//
	ROM_GPIOPinTypeI2C(GPIO_PORTG_BASE, GPIO_PIN_1);
	GPIOPinTypeI2CSCL(GPIO_PORTG_BASE, GPIO_PIN_0);

	//
	// Stop the Clock, Reset and Enable I2C Module
	// in Master Function
	//
	ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C1);
	ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

	//
	// Wait for the Peripheral to be ready for programming
	//
	while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1));

	//
	// Initialize and Configure the Master Module
	//
	ROM_I2CMasterInitExpClk(I2C1_BASE, SYSTEM_CLOCK, true);		//400kbps

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
#endif


#ifdef PEDOMETER
// Flash the LEDs on the launchpad
void pedometerTask(void *pvParameters)
{
	uint8_t ctrl9_xl;
	uint8_t ctrl1_xl;
	uint8_t status;
	uint8_t outx_l_xl;
	uint8_t outx_h_xl;
	uint8_t outy_l_xl;
	uint8_t outy_h_xl;
	uint8_t outz_l_xl;
	uint8_t outz_h_xl;

    for (;;)
    {
//        SysCtlDelay(10);
        // Turn on LED 1
        /*LEDWrite(0x0F, 0x01);
        vTaskDelay(1000);
        */


        //accelerometer
        //writing 0x38 to CTRL9_XL(0x18)
        //--------------------------------------------------------
        ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, false);   //write to accelerometer
        ROM_I2CMasterDataPut(I2C1_BASE, 0x18);
        ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        while(ROM_I2CMasterBusy(I2C1_BASE));

        SysCtlDelay(100); //Delay by 1us

        ROM_I2CMasterDataPut(I2C1_BASE, 0x38);
        ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        while(ROM_I2CMasterBusy(I2C1_BASE));

        UARTprintf("Finished writing to the CTRL9_XL register.\n");

        //reading 0x38
        ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, false);   //write to accelerometer
        ROM_I2CMasterDataPut(I2C1_BASE, 0x18);
        ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        while(ROM_I2CMasterBusy(I2C1_BASE));

        SysCtlDelay(100); //Delay by 1us

        ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, true);    //read from status sensor
        ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
        while(ROM_I2CMasterBusy(I2C1_BASE));
        ctrl9_xl = ROM_I2CMasterDataGet(I2C1_BASE);
        //UARTprintf("CTRL9_XL is 0x%x\n", ctrl9_xl);


        //writing 0x60 to CTRL1_XL(0x10)
        //--------------------------------------------------------
        ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, false);   //write to accelerometer
        ROM_I2CMasterDataPut(I2C1_BASE, 0x10);
        ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        while(ROM_I2CMasterBusy(I2C1_BASE));

        SysCtlDelay(100); //Delay by 1us

        ROM_I2CMasterDataPut(I2C1_BASE, 0x60);
        ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        while(ROM_I2CMasterBusy(I2C1_BASE));

        UARTprintf("Finished writing to the CTRL1_XL register.\n");

        //reading 0x38
        ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, false);   //write to accelerometer
        ROM_I2CMasterDataPut(I2C1_BASE, 0x10);
        ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        while(ROM_I2CMasterBusy(I2C1_BASE));

        SysCtlDelay(100); //Delay by 1us

        ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, true);    //read from status sensor
        ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
        while(ROM_I2CMasterBusy(I2C1_BASE));
        ctrl1_xl = ROM_I2CMasterDataGet(I2C1_BASE);
        //UARTprintf("CTRL1_XL is 0x%x\n", ctrl1_xl);

        //read the STATUS register(0x1E)
        ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, false);   //write to accelerometer
        ROM_I2CMasterDataPut(I2C1_BASE, 0x1E);
        ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        while(ROM_I2CMasterBusy(I2C1_BASE));

        SysCtlDelay(100); //Delay by 1us

        ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, true);    //read from status sensor
        ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
        while(ROM_I2CMasterBusy(I2C1_BASE));
        status = ROM_I2CMasterDataGet(I2C1_BASE);
        //UARTprintf("STATUS is 0x%x\n", status);

        if(status & 0x01){
            UARTprintf("New accelerometer data available\n");

            //read the outx_l_xl register(0x28)
            ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, false);   //write to accelerometer
            ROM_I2CMasterDataPut(I2C1_BASE, 0x28);
            ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
            while(ROM_I2CMasterBusy(I2C1_BASE));

            SysCtlDelay(100); //Delay by 1us

            ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, true);    //read from status sensor
            ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
            while(ROM_I2CMasterBusy(I2C1_BASE));
            outx_l_xl = ROM_I2CMasterDataGet(I2C1_BASE);
            UARTprintf("outx_l_xl is 0x%x\n", outx_l_xl);

            //read the outx_h_xl register(0x29)
            ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, false);   //write to accelerometer
            ROM_I2CMasterDataPut(I2C1_BASE, 0x29);
            ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
            while(ROM_I2CMasterBusy(I2C1_BASE));

            SysCtlDelay(100); //Delay by 1us

            ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, true);    //read from status sensor
            ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
            while(ROM_I2CMasterBusy(I2C1_BASE));
            outx_l_xl = ROM_I2CMasterDataGet(I2C1_BASE);
            UARTprintf("outx_h_xl is 0x%x\n", outx_h_xl);

            //read the outy_l_xl register(0x2A)
            ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, false);   //write to accelerometer
            ROM_I2CMasterDataPut(I2C1_BASE, 0x2A);
            ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
            while(ROM_I2CMasterBusy(I2C1_BASE));

            SysCtlDelay(100); //Delay by 1us

            ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, true);    //read from status sensor
            ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
            while(ROM_I2CMasterBusy(I2C1_BASE));
            outy_l_xl = ROM_I2CMasterDataGet(I2C1_BASE);
            UARTprintf("outy_l_xl is 0x%x\n", outy_l_xl);

            //read the outy_h_xl register(0x2B)
            ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, false);   //write to accelerometer
            ROM_I2CMasterDataPut(I2C1_BASE, 0x2B);
            ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
            while(ROM_I2CMasterBusy(I2C1_BASE));

            SysCtlDelay(100); //Delay by 1us

            ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, true);    //read from status sensor
            ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
            while(ROM_I2CMasterBusy(I2C1_BASE));
            outy_h_xl = ROM_I2CMasterDataGet(I2C1_BASE);
            UARTprintf("outy_h_xl is 0x%x\n", outy_h_xl);

            //read the outz_l_xl register(0x2C)
            ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, false);   //write to accelerometer
            ROM_I2CMasterDataPut(I2C1_BASE, 0x2C);
            ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
            while(ROM_I2CMasterBusy(I2C1_BASE));

            SysCtlDelay(100); //Delay by 1us

            ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, true);    //read from status sensor
            ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
            while(ROM_I2CMasterBusy(I2C1_BASE));
            outz_l_xl = ROM_I2CMasterDataGet(I2C1_BASE);
            UARTprintf("outz_l_xl is 0x%x\n", outz_l_xl);

            //read the outz_h_xl register(0x2D)
            ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, false);   //write to accelerometer
            ROM_I2CMasterDataPut(I2C1_BASE, 0x2D);
            ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
            while(ROM_I2CMasterBusy(I2C1_BASE));

            SysCtlDelay(100); //Delay by 1us

            ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, true);    //read from status sensor
            ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
            while(ROM_I2CMasterBusy(I2C1_BASE));
            outz_h_xl = ROM_I2CMasterDataGet(I2C1_BASE);
            UARTprintf("outz_h_xl is 0x%x\n\n\n", outz_h_xl);

        }
    }
}
#endif


#ifdef HEART_RATE
// Read heartbeat digital values
void heartbeatTask(void *pvParameters)
{
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

        //Trigger ADC conversion
        ADCProcessorTrigger(ADC0_BASE, 3);

        //Wait for conversion to complete
        while(!ADCIntStatus(ADC0_BASE, 3, false));

        //Clear ADC interrupt flag
        ADCIntClear(ADC0_BASE, 3);

        //Read ADC value
        ADCSequenceDataGet(ADC0_BASE, 3, pulse_rate);

        //Display the AIN0 (PE3) digital value on UART
        UARTprintf("AIN0 = %4d\n", pulse_rate[0]);

        int32_t D0 = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_2);  //LO- Status
        int32_t D1 = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3);  //LO+ Status

        //Display the LO- & LO+ digital value on UART
        UARTprintf("LO- = %d  LO+ = %d\n", D0, D1);

        //Delay for some time
        SysCtlDelay(100);
    }
}
#endif


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

#ifdef  PEDOMETER
    I2C_Init();
#endif
#ifdef HEART_RATE
    GPIO_Init();
    ADC_Init();
#endif

#ifdef PEDOMETER
    // Create pedometer task
    xTaskCreate(pedometerTask, (const portCHAR *)"pedometer",
                configMINIMAL_STACK_SIZE, NULL, 1, NULL);
#endif
#ifdef HEART_RATE
    // Create heartbeat task
    xTaskCreate(heartbeatTask, (const portCHAR *)"heartbeat",
                configMINIMAL_STACK_SIZE, NULL, 1, NULL);
#endif

//    xTaskCreate(demoSerialTask, (const portCHAR *)"Serial",
//                configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    vTaskStartScheduler();
    return 0;
}
