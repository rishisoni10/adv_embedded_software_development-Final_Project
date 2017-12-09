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
#include "utils/lwiplib.h"
#include "utils/locator.h"

// TivaWare includes
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/comp.h"
#include "driverlib/i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"


// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define LSM6DS3_ADDR        (0x6B)
#define TMP102_ADDR         (0x48)
//#define PEDOMETER           1
#define PULSE               1
//#define SOCKET              1
#define ACCEL_RAW_VERBOSE   1
//#define EINVAL         0x0016
// #define ERANGE         0x0022

//#undef PULSE
#undef ACCEL_RAW_VERBOSE
#undef PULSE

uint32_t pulse_rate[1];
//uint32_t heart_rate;
uint32_t output_clock_rate_hz;
uint32_t GPIO_pin_a4;
volatile int isr_counter;
volatile uint32_t comp_out;
volatile uint8_t timer_isr_count = 0;
volatile uint8_t comp_isr_count = 0;
volatile uint32_t number_of_heartbeats = 0;

QueueHandle_t pedQueue;

// Task declarations
void pedometerTask(void *pvParameters);
void heartbeatTask(void *pvParameters);
void socketTask(void *pvParameters);
//void demoSerialTask(void *pvParameters);

//*****************************************************************************
//
// The interrupt handler for the Timer0A interrupt.
//
//*****************************************************************************
void Timer0AIntHandler(void)
{
    timer_isr_count++;
    UARTprintf("Timer Interrupt. ISR count:%d\n", timer_isr_count);

    // Disable the Timer0A interrupt.
    IntDisable(INT_TIMER0A);

    // Turn off Timer0A interrupt.
    TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Clear the timer interrupt flag.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    if(timer_isr_count == 2)
    {
        UARTprintf("Beats per min:%d", number_of_heartbeats);
        timer_isr_count = 0;
        number_of_heartbeats = 0;
        comp_isr_count = 0;
        TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        IntEnable(INT_TIMER0A);

    }
    else
    {
        number_of_heartbeats += comp_isr_count;
        TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        IntEnable(INT_TIMER0A);
    }
}


//*****************************************************************************
//
// Sets up the additional lwIP raw API services provided by the application.
//
//*****************************************************************************
void
SetupServices(void *pvArg)
{
    uint8_t pui8MAC[6];

    //
    // Setup the device locator service.
    //
    LocatorInit();
    lwIPLocalMACGet(pui8MAC);
    LocatorMACAddrSet(pui8MAC);

    LocatorAppTitleSet("DK-TM4C129X freertos_demo");
}


//*****************************************************************************
//
// Initializes the lwIP tasks.
//
//*****************************************************************************
uint32_t
lwIPTaskInit(void)
{
    uint32_t ui32User0, ui32User1;
    uint8_t pui8MAC[6];

    //
    // Get the MAC address from the user registers.
    //
    ROM_FlashUserGet(&ui32User0, &ui32User1);
    if((ui32User0 == 0xffffffff) || (ui32User1 == 0xffffffff))
    {
        return(1);
    }

    //
    // Convert the 24/24 split MAC address from NV ram into a 32/16 split MAC
    // address needed to program the hardware registers, then program the MAC
    // address into the Ethernet Controller registers.
    //
    pui8MAC[0] = ((ui32User0 >>  0) & 0xff);
    pui8MAC[1] = ((ui32User0 >>  8) & 0xff);
    pui8MAC[2] = ((ui32User0 >> 16) & 0xff);
    pui8MAC[3] = ((ui32User1 >>  0) & 0xff);
    pui8MAC[4] = ((ui32User1 >>  8) & 0xff);
    pui8MAC[5] = ((ui32User1 >> 16) & 0xff);

    //
    // Lower the priority of the Ethernet interrupt handler.  This is required
    // so that the interrupt handler can safely call the interrupt-safe
    // FreeRTOS functions (specifically to send messages to the queue).
    //
    ROM_IntPrioritySet(INT_EMAC0, 0xC0);

    //
    // Initialize lwIP.
    //
    lwIPInit(output_clock_rate_hz, pui8MAC, 0, 0, 0, IPADDR_USE_DHCP);

    //
    // Setup the remaining services inside the TCP/IP thread's context.
    //
    tcpip_callback(SetupServices, 0);

    //
    // Success.
    //
    return 0;
}



void PortAIntHandler(void){
    taskDISABLE_INTERRUPTS();
    ROM_GPIOIntDisable(GPIO_PORTA_BASE, GPIO_PIN_4);
    IntMasterDisable();
    ROM_GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_4);  // Clear interrupt flag
    isr_counter++;
    ROM_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_RISING_EDGE);
    IntMasterEnable();
    ROM_GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_4);
    taskENABLE_INTERRUPTS();
}

//Analog Comparator0 ISR
void COMP0_ISR(void)
{
    comp_isr_count++;
    UARTprintf("Comparator ISR count:%d\n", comp_isr_count);
    ComparatorIntDisable(COMP_BASE, 0);
    ComparatorIntClear(COMP_BASE, 0);
    comp_out = ComparatorValueGet(COMP_BASE, 0);
    ComparatorIntEnable(COMP_BASE, 0);
}

void UART_Init(void)
{

}

void GPIO_Init(void)
{
    //PortB for GPIO r/w
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
//    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //PB2 = LO-; PB3 = LO+
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2|GPIO_PIN_3);

    // Enable the GPIOA peripheral for INT1 signal coming from pedometer sensor
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Wait for the GPIOA module to be ready.
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    // Initialize the GPIO pin configuration.
    // Set pin A4 as input
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_4);

    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_4,
        GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  // Enable weak pullup resistor for PF4

    GPIOIntDisable(GPIO_PORTA_BASE, GPIO_PIN_4);        // Disable interrupt for PA4 (in case it was enabled)
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_4);      // Clear pending interrupts for PF4

    // Register the port-level interrupt handler. This handler is the first
    // level interrupt handler for all the pin interrupts.
    GPIOIntRegister(GPIO_PORTA_BASE, PortAIntHandler);

    // Make pin 4 rising edge triggered interrupts.
    ROM_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_RISING_EDGE);
    ROM_GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_4);     // Enable interrupt for PF4
}

#ifdef PULSE
void Timer_Init(void)
{
    // The Timer0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // Wait for the Timer0 module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));

    //Using System Clock of 120 MHz
    TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_SYSTEM);

    // Configure Timer0 as a 32-bit full-width periodic timer.
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    // Set the count time for the periodic timer (TimerA) = 30seconds
    TimerLoadSet(TIMER0_BASE, TIMER_A, output_clock_rate_hz/0.033);

    //Register timerA interrupt handler
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0AIntHandler);

}

void Comparator_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);        //enable PortC
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_6);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)); // Wait for the PortC GPIO to be ready.

    SysCtlPeripheralEnable(SYSCTL_PERIPH_COMP0);        //enable ACMP0
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_COMP0)); // Wait for the COMP module to be ready.

    GPIOPinTypeComparator(GPIO_PORTC_BASE, GPIO_PIN_6); //Assign Pin6 as ACMP0+
//    GPIOPinConfigure(GPIO_PD0_C0O);

    ComparatorConfigure(COMP_BASE, 0, COMP_TRIG_NONE | COMP_INT_RISE | COMP_ASRCP_REF | COMP_OUTPUT_NORMAL);
    SysCtlDelay(1000);

    ComparatorRefSet(COMP_BASE, COMP_REF_1_1V); //internal ref to OFF
    SysCtlDelay(1000);

    ComparatorIntRegister(COMP_BASE, 0, COMP0_ISR);
    SysCtlDelay(1000);

    ComparatorIntEnable(COMP_BASE, 0);
}

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
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // information on the ADC sequences and steps, reference the datasheet.
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
                             ADC_CTL_END);

    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    ADCIntClear(ADC0_BASE, 3);

    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);

}

void Peripheral_Int(void)
{
    // Configure the Timer0B interrupt for timer timeout.
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Enable the Timer0B interrupt on the processor (NVIC).
    IntEnable(INT_TIMER0A);
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
    ROM_I2CMasterInitExpClk(I2C1_BASE, SYSTEM_CLOCK, true);     //400kbps
}

void pedometerTask(void *pvParameters)
{
    uint8_t ctrl9_xl;
    uint8_t ctrl1_xl;
    uint8_t status;
#ifdef ACCEL_RAW_VERBOSE
    uint8_t outx_l_xl;
    uint8_t outx_h_xl;
    uint8_t outy_l_xl;
    uint8_t outy_h_xl;
    uint8_t outz_l_xl;
    uint8_t outz_h_xl;
#endif
    uint8_t step_counter_l;
    uint8_t step_counter_h;
    uint16_t step_counter;

    uint32_t data_pedQueue = 0;
    BaseType_t status_pedQueue;

    //instanting the message packet
    static message_t messg;
    task_id_t id = pedometer;
   for (;;)
    {
//        SysCtlDelay(10);
        // Turn on LED 1
        LEDWrite(0x0F, 0x01);
        vTaskDelay(1000);

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


        //writing 0x20 to CTRL1_XL(0x10) for pedometer functionality
        //--------------------------------------------------------
        ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, false);   //write to accelerometer
        ROM_I2CMasterDataPut(I2C1_BASE, 0x10);
        ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        while(ROM_I2CMasterBusy(I2C1_BASE));

        SysCtlDelay(100); //Delay by 1us

        ROM_I2CMasterDataPut(I2C1_BASE, 0x20);
        ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        while(ROM_I2CMasterBusy(I2C1_BASE));

        UARTprintf("Finished writing to the CTRL1_XL register.\n");
       //writing 0x3C to CTRL10_C(0x19)
        //--------------------------------------------------------
        ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, false);   //write to accelerometer
        ROM_I2CMasterDataPut(I2C1_BASE, 0x19);
        ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        while(ROM_I2CMasterBusy(I2C1_BASE));

        SysCtlDelay(100); //Delay by 1us

        ROM_I2CMasterDataPut(I2C1_BASE, 0x3C);
        ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        while(ROM_I2CMasterBusy(I2C1_BASE));

        UARTprintf("Finished writing to the CTRL10_C register.\n");

        //writing 0x40 to TAP_CFG(0x58) - Enabling pedometer algorithm
        //--------------------------------------------------------
        ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, false);   //write to accelerometer
        ROM_I2CMasterDataPut(I2C1_BASE, 0x58);
        ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        while(ROM_I2CMasterBusy(I2C1_BASE));

        SysCtlDelay(100); //Delay by 1us

        ROM_I2CMasterDataPut(I2C1_BASE, 0x40);
        ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        while(ROM_I2CMasterBusy(I2C1_BASE));

        UARTprintf("Finished writing to the TAP_CFG register.\n");


        //writing 0x80 to INT_CTRL1(0x0D)
        //--------------------------------------------------------
        ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, false);   //write to accelerometer
        ROM_I2CMasterDataPut(I2C1_BASE, 0x0D);
        ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        while(ROM_I2CMasterBusy(I2C1_BASE));

        SysCtlDelay(100); //Delay by 1us

        ROM_I2CMasterDataPut(I2C1_BASE, 0x80);
        ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        while(ROM_I2CMasterBusy(I2C1_BASE));

        UARTprintf("Finished writing to the INT_CTRL1 register.\n");

        //-------------------------------------------------------
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

            //macro for verbose accelerometer data on x,y,z axes
#ifdef ACCEL_RAW_VERBOSE
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
            UARTprintf("outz_h_xl is 0x%x\n", outz_h_xl);
#endif

            //read the step_counter_l register(0x4B)
            ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, false);   //write to accelerometer
            ROM_I2CMasterDataPut(I2C1_BASE, 0x4B);
            ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
            while(ROM_I2CMasterBusy(I2C1_BASE));

            SysCtlDelay(100); //Delay by 1us

            ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, true);    //read from status sensor
            ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
            while(ROM_I2CMasterBusy(I2C1_BASE));
            step_counter_l = ROM_I2CMasterDataGet(I2C1_BASE);
            //UARTprintf("step_counter_l is 0x%x\n", step_counter_l);

            //read the step_counter_h register(0x4C)
            ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, false);   //write to accelerometer
            ROM_I2CMasterDataPut(I2C1_BASE, 0x4C);
            ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
            while(ROM_I2CMasterBusy(I2C1_BASE));

            SysCtlDelay(100); //Delay by 1us

            ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, true);    //read from status sensor
            ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
            while(ROM_I2CMasterBusy(I2C1_BASE));
            step_counter_h = ROM_I2CMasterDataGet(I2C1_BASE);
            //UARTprintf("step_counter_h is 0x%x\n", step_counter_h);

            step_counter = (step_counter_h << 8) | step_counter_l;
            UARTprintf("step_counter is 0x%x\n", step_counter);

            UARTprintf("ISR count is %d\n\n", isr_counter);

            data_pedQueue = isr_counter;
            messg.data = data_pedQueue;
            messg.task_id = id;

            //sending the data to the socket task using queue
            status_pedQueue = xQueueSendToBack(pedQueue, &messg, 50);
            //UARTprintf("Step count sent is %d\n", data_pedQueue);
            if(status_pedQueue != pdPASS){
                UARTprintf("Could not send data to queue.\n");
            }

            //reading gpio pin
            GPIO_pin_a4 = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4);
            //UARTprintf("GPIO Pin A4 = %d\n\n\n", GPIO_pin_a4);
        }
    }
}
#endif

#ifdef PULSE
// Read heartbeat digital values
void heartbeatTask(void *pvParameters)
{
    // Enable Timer0A.
    static uint32_t heart_rate = 0;
    TimerEnable(TIMER0_BASE, TIMER_A);

    while (1)
    {
//        SysCtlDelay(10);
//         Turn on LED 1
//        LEDWrite(0x0F, 0x01);
//        vTaskDelay(1000);

        // Turn on LED 2
        LEDWrite(0x0F, 0x02);
        vTaskDelay(1000);
//
//        // Turn on LED 3
//        LEDWrite(0x0F, 0x04);
//        vTaskDelay(1000);
//
//        // Turn on LED 4
//        LEDWrite(0x0F, 0x08);
//        vTaskDelay(1000);

        /*
        //Trigger ADC conversion
        ADCProcessorTrigger(ADC0_BASE, 3);

        //Wait for conversion to complete
        while(!ADCIntStatus(ADC0_BASE, 3, false));

        //Clear ADC interrupt flag
        ADCIntClear(ADC0_BASE, 3);

//        int32_t D0 = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_2);  //LO- Status
        int32_t D0 = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1);  //LO- Status
        int32_t D1 = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_3);  //LO+ Status

        //Display the LO- & LO+ digital value on UART
        UARTprintf("LO- = 0x%x  LO+ = 0x%x\n", D0, D1);

        if(D0 == 0x04 || D1 == 0x08)
        {
            UARTprintf("NOT CONNECTED!\n");
        }

        else
        {
            //Read ADC value
            ADCSequenceDataGet(ADC0_BASE, 3, pulse_rate);
            if(pulse_rate[0] >= 2000)
            {
                heart_rate++;
            }

            //Display the AIN0 (PE3) digital value on UART
            UARTprintf("AIN0 = %d\n", pulse_rate[0]);
        }

        //Delay for some time
        SysCtlDelay(1000);
        */
//        UARTprintf("ACMP0 value is:%d\n", ComparatorValueGet(COMP_BASE, 0));
//        UARTprintf("ACMP0 trigger count is:%d\n", comp_isr_count);
    }
}
#endif

#ifdef SOCKET
void socketTask(void *pvParameters)
{
    BaseType_t rec_ped_status;

    //instanting the message packet
    static message_t recv_messg;

    for (;;)
    {
        //receive data from queue with a block of 100 ticks
        rec_ped_status = xQueueReceive(pedQueue, &recv_messg, 100);
        if(rec_ped_status == pdPASS){
            if(recv_messg.task_id == pedometer ){
                UARTprintf("Source: pedometer task \nStep count received from queue: %d\n\n", recv_messg.data);
            }
        }
    }
}
#endif

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
    output_clock_rate_hz = ROM_SysCtlClockFreqSet(
                               (SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                               SYSTEM_CLOCK);
    ASSERT(output_clock_rate_hz == SYSTEM_CLOCK);

    // Initialize the GPIO pins for the Launchpad
    PinoutSet(true, false);

    // Set up the UART which is connected to the virtual COM port
    UARTStdioConfig(0, 57600, SYSTEM_CLOCK);
    GPIO_Init();
    UART_Init();

#ifdef  PEDOMETER
    I2C_Init();
#endif
#ifdef PULSE
    ADC_Init();
    Timer_Init();
    Peripheral_Int();       //Enable interrupts of peripherals
    Comparator_Init();    
#endif

    //creating the pedometer queue
    pedQueue = xQueueCreate(10, sizeof(message_t));
#ifdef PEDOMETER
    // Create pedometer task
     xTaskCreate(pedometerTask, (const portCHAR *)"pedometer",
                  configMINIMAL_STACK_SIZE, NULL, 2, NULL);
#endif
#ifdef PULSE
    // Create heartbeat task
    xTaskCreate(heartbeatTask, (const portCHAR *)"heartbeat",
                configMINIMAL_STACK_SIZE, NULL, 2, NULL);
#endif

#ifdef SOCKET
    xTaskCreate(socketTask, (const portCHAR *)"socket",
                configMINIMAL_STACK_SIZE, NULL, 1, NULL);
#endif
    //
    // Enable processor interrupts.
    //
    //IntMasterEnable();

    //
    // Create the lwIP tasks.
    //
    if(lwIPTaskInit() != 0){
        while(1){

        }
    }

    //Start scheduler
    vTaskStartScheduler();
    return 0;
}


//FlashUserGet
//lwIPInit
