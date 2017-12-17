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
#include <string.h>
#include <stdio.h>
#include "utils/ustdlib.h"
#include "main.h"
#include "drivers/pinout.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"


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
#include "driverlib/hibernate.h"
#include "driverlib/timer.h"


// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define LSM6DS3_ADDR        (0x6B)
#define TMP102_ADDR         (0x48)
#define PEDOMETER           1
#define PULSE               1
#define SERIAL              1
#define ACCEL_RAW_VERBOSE   1
//#define UNIT                1
#define INIT                1

//#undef PEDOMETER
//#undef PULSE
//#undef SERIAL

volatile int total_tests;
volatile int pass_tests;
volatile int fail_tests;

#define SYSTICKHZ               100
#define SYSTICKMS               (1000 / SYSTICKHZ)
#define SYSTICK_INT_PRIORITY    0x80

#define CURRENT_HOUR    (14)
#define CURRENT_MIN     (30)
#define CURRENT_MON     (12)
#define CURRENT_DAY     (11)
#define CURRENT_YEAR    (2017)
#define STEP_BIT        1
#define PULSE_BIT       2


#undef ACCEL_RAW_VERBOSE

uint32_t output_clock_rate_hz;
uint32_t GPIO_pin_a4;
volatile int isr_counter;
volatile uint32_t comp_out;
volatile uint8_t timer_isr_count = 0;
volatile uint8_t comp_isr_count = 0;
volatile uint32_t bpm = 0;
volatile uint32_t bpm_cpy;
volatile uint32_t bpm_flag = 0;
volatile uint8_t stat;

uint32_t ui32NewIPAddress;

QueueHandle_t pedQueue;
QueueHandle_t pulseQueue;
QueueHandle_t sharedQueue1;
QueueHandle_t sharedQueue2;
static TaskHandle_t xTaskToNotify = NULL;

uint32_t g_ui32IPAddress;

volatile int uart_flag;
volatile int i2c_flag;

TaskHandle_t notifyHandle;

// Task declarations
void pedometerTask(void *pvParameters);
void pulseTask(void *pvParameters);
void serialTask(void *pvParameters);


/*
Integer to ASCII (Null terminated string)
*/
char * my_itoa(char *str, int32_t data){

    // If the number is '0'
    if (data == 0){
        *str++ = '0';
        *str-- = '\0';   //Adding null for end of string and resetting str to initial value
        return str;
    }

    int rem = 0;     //variable to store remainder
    uint8_t length = 0;         //Calculating length of the string

    //Dividing with the base to get the value of data in that base and storing it in the string
    while (data != 0){
        rem = data % 10;
        *str++ = (rem > 9)? (rem-10) + 'A' : rem + '0';             //Ternary for base values greater than 10.
        length++;
        data = data/10;
    }


    *str = '\0'; // Append null character for end of string

    // Reverse the string for final output as the loop gives us the last value first

    uint8_t j=0;         //Initializing counter for the loop
    int8_t temp;
    str = str - length;        //Resetting str to initial value
    for(j=0;j<length/2;j++){ //loop to reverse string
        temp=*(str+j);
        *(str+j) = *(str+length-j-1);
        *(str+length-j-1)=temp;
    }
    return str;
}


//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
#ifdef SERIAL
void
UARTIntHandler(void)
{
    taskDISABLE_INTERRUPTS();

    uint32_t ui32Status;
    //
    // Get the interrrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART3_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART3_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
    while(ROM_UARTCharsAvail(UART3_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
//        UARTprintf("%c", ROM_UARTCharGetNonBlocking(UART3_BASE));
        stat = ROM_UARTCharGetNonBlocking(UART3_BASE);

        if(stat == 'P')
        {

            xTaskNotifyFromISR(notifyHandle,
                                   PULSE_BIT,
                                   eSetBits,
                                   NULL);
        }
        if(stat == 'S')
        {
            xTaskNotifyFromISR(notifyHandle,STEP_BIT,
                                               eSetBits,
                                               NULL);
          }

        //ROM_UARTCharPutNonBlocking(UART3_BASE,
          //                         ROM_UARTCharGetNonBlocking(UART3_BASE));

        //
        // Blink the LED to show a character transfer is occuring.
        //
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);

        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        SysCtlDelay(output_clock_rate_hz / (1000 * 3));

        //
        // Turn off the LED
        //
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
    }
    taskENABLE_INTERRUPTS();

}
#endif


#ifdef UNIT
void
UARTIntHandler(void)
{
    taskDISABLE_INTERRUPTS();

    uint32_t ui32Status;
    //
    // Get the interrrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART3_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART3_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART3_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        //UARTprintf("Msg recv is %c", ROM_UARTCharGetNonBlocking(UART3_BASE));
        stat = ROM_UARTCharGetNonBlocking(UART3_BASE);
        if(stat == '4'){
            total_tests ++;
            pass_tests ++;
            uart_flag = 1;
        }
        else{
            total_tests ++;
            fail_tests ++;
        }

    }
    taskENABLE_INTERRUPTS();

}
#endif
//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        ROM_UARTCharPut(UART3_BASE, *pui8Buffer++);
    }
}


//*****************************************************************************
//
// The interrupt handler for the Timer0A interrupt.
// 30 second timer.
//*****************************************************************************
void Timer0AIntHandler(void)
{
    taskDISABLE_INTERRUPTS();
    timer_isr_count++;
//    UARTprintf("Timer Interrupt. ISR count:%d\n", timer_isr_count);

    // Disable the Timer0A interrupt.
    IntDisable(INT_TIMER0A);

    // Turn off Timer0A interrupt.
    TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Clear the timer interrupt flag.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    if(timer_isr_count == 2)
    {
//        UARTprintf("Beats per min:%d", bpm);
        bpm_cpy = bpm;
        timer_isr_count = 0;
        bpm = 0;
        comp_isr_count = 0;
        TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        IntEnable(INT_TIMER0A);

    }
    else
    {
        bpm += comp_isr_count;
        TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        IntEnable(INT_TIMER0A);
    }
    taskENABLE_INTERRUPTS();

}


void PortAIntHandler(void){
    taskDISABLE_INTERRUPTS();
    ROM_GPIOIntDisable(GPIO_PORTA_BASE, GPIO_PIN_6);
    IntMasterDisable();
    ROM_GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_6);  // Clear interrupt flag
    isr_counter++;
    ROM_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_RISING_EDGE);
    IntMasterEnable();
    ROM_GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_6);
    taskENABLE_INTERRUPTS();
}


//Analog Comparator0 ISR
void COMP0_ISR(void)
{
    taskDISABLE_INTERRUPTS();
    comp_isr_count++;
//    UARTprintf("Comparator ISR count:%d\n", comp_isr_count);
    ComparatorIntDisable(COMP_BASE, 0);
    ComparatorIntClear(COMP_BASE, 0);
    comp_out = ComparatorValueGet(COMP_BASE, 0);
    ComparatorIntEnable(COMP_BASE, 0);
    taskENABLE_INTERRUPTS();
}


// This function writes the requested date and time to the calendar logic of
// hibernation module.
void DateTimeSet(void)
{
    struct tm sTime;

    //
    // Get the latest date and time.  This is done here so that unchanged
    // parts of date and time can be written back as is.
    //
    HibernateCalendarGet(&sTime);

    //
    // Set the date and time values that are to be updated.
    //
    sTime.tm_min = CURRENT_MIN;
    sTime.tm_hour = CURRENT_HOUR;
    sTime.tm_mon = CURRENT_MON;
    sTime.tm_mday = CURRENT_DAY;
    sTime.tm_year = 100 + CURRENT_YEAR;

    //
    // Update the calendar logic of hibernation module with the requested data.
    //
    HibernateCalendarSet(&sTime);
}


//Hibernation module Init
void Hibernate_Init(void)
{
    //
    // Enable the hibernate module.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_HIBERNATE);
    if(HibernateIsActive())
    {
        //
        // Read the status bits to see what caused the wake.  Clear the wake
        // source so that the device can be put into hibernation again.
        //
        uint32_t ui32Status = HibernateIntStatus(0);
        HibernateIntClear(ui32Status);
    }

        //
        // Configure Hibernate module clock.
        //
        HibernateEnableExpClk(output_clock_rate_hz);

        //
        // Configure the module clock source.
        //
        HibernateClockConfig(HIBERNATE_OSC_LOWDRIVE);

        //
        // Enable RTC mode.
        //
        HibernateRTCEnable();

        //
        // Configure the hibernate module counter to 24-hour calendar mode.
        //
        HibernateCounterMode(HIBERNATE_COUNTER_24HR);

        DateTimeSet();

}





//UART setup
void UART_Init(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //
    // Set GPIO A4 and A5 as UART pins.
    //
    GPIOPinConfigure(GPIO_PA4_U3RX);
    GPIOPinConfigure(GPIO_PA5_U3TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    //
    // Configure the UART for 57600, 8-N-1 operation.
    //
    ROM_UARTConfigSetExpClk(UART3_BASE, output_clock_rate_hz, 57600,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    ROM_IntEnable(INT_UART3);
    ROM_UARTIntEnable(UART3_BASE, UART_INT_RX | UART_INT_RT);
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
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6);

    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6,
        GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  // Enable weak pullup resistor for PF4

    GPIOIntDisable(GPIO_PORTA_BASE, GPIO_PIN_6);        // Disable interrupt for PA4 (in case it was enabled)
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_6);      // Clear pending interrupts for PF4

    // Register the port-level interrupt handler. This handler is the first
    // level interrupt handler for all the pin interrupts.
    GPIOIntRegister(GPIO_PORTA_BASE, PortAIntHandler);

    // Make pin 4 rising edge triggered interrupts.
    ROM_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_RISING_EDGE);
    ROM_GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_6);     // Enable interrupt for PF4
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
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_6);  //PC6
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
    static int count = 0;
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
    BaseType_t status_recv_pulseQueue;
    BaseType_t status_send_pulseQueue;

    //instanting the message packet
   static message ped_msg;

   static message recv_pulse_msg;
   static message send_pulse_msg;

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

   for (;;)
    {
//        SysCtlDelay(10);
        // Turn on LED 1
        //LEDWrite(0x0F, 0x01);
        //vTaskDelay(1000);

        count++;

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
            //UARTprintf("step_counter is 0x%x\n", step_counter);

            //UARTprintf("ISR count is %d\n\n", isr_counter);


            if(count%15==0){
                send_pulse_msg.source_task = pedometer;
                send_pulse_msg.log_level = LOG_REQUEST;
                send_pulse_msg.request_type = PULSE_REQUEST;
                send_pulse_msg.msg_rqst_type = PED_DATA;
                send_pulse_msg.type = REQUEST_MESSAGE;
                send_pulse_msg.data = 32762;  //placeholder data
                //TODO : Figure out timestamps

                //sending the data to the socket task using queue
                status_send_pulseQueue = xQueueSendToBack(sharedQueue1, &send_pulse_msg, 50);
                if(status_send_pulseQueue != pdPASS){
                    UARTprintf("Could not send data to shared queue1.\n");
                }
                else{
                    //UARTprintf("Sent a data request to the PULSE task.\n");
                }

                //also send a copy of the request to the serial task
                status_send_pulseQueue = xQueueSendToBack(pedQueue, &send_pulse_msg, 50);
                if(status_send_pulseQueue != pdPASS){
                    UARTprintf("Could not send data to pedometer queue.\n");
                }
            }

            //memset(&send_pulse_msg, 0, sizeof(send_pulse_msg));

            status_recv_pulseQueue = xQueueReceive(sharedQueue2, &recv_pulse_msg, 50);
            if(status_recv_pulseQueue == pdPASS){
                if(recv_pulse_msg.source_task == pulse_rate ){
                    if(recv_pulse_msg.type == REQUEST_MESSAGE){
                        //UARTprintf("Obtained request for pedometer data\n");
                        //memset(&recv_pulse_msg, 0, sizeof(recv_pulse_msg));
                        recv_pulse_msg.data = data_pedQueue;
                        recv_pulse_msg.source_task = pedometer;
                        recv_pulse_msg.log_level = LOG_INFO_DATA;
                        recv_pulse_msg.request_type = NOT_REQUEST;
                        recv_pulse_msg.msg_rqst_type = PED_DATA;
                        recv_pulse_msg.type = RESPONSE_MESSAGE;
                        //TODO : Figure out timestamps

                        //sending the data to the socket task using queue
                        status_send_pulseQueue = xQueueSendToBack(sharedQueue1, &recv_pulse_msg, 50);
                        if(status_send_pulseQueue != pdPASS){
                            UARTprintf("Could not send response to shared queue1.\n");
                        }
                        else{
                            //UARTprintf("Sent a response to the PULSE task.\n");
                        }
                    }
                    else if(recv_pulse_msg.type == RESPONSE_MESSAGE){
                        //UARTprintf("Obtained a response from PULSE task and bpm is %d.\n", recv_pulse_msg.data);
                    }
                }
                //memset(&recv_pulse_msg, 0, sizeof(recv_pulse_msg));
            }

            data_pedQueue = step_counter;
            ped_msg.data = data_pedQueue;
            ped_msg.source_task = pedometer;
            ped_msg.log_level = LOG_INFO_DATA;
            ped_msg.request_type = NOT_REQUEST;
            ped_msg.msg_rqst_type = PED_DATA;
            ped_msg.type = LOG_MESSAGE;
            //TODO : Figure out timestamps


            //sending the data to the socket task using queue
            status_pedQueue = xQueueSendToBack(pedQueue, &ped_msg, 50);
            if(status_pedQueue != pdPASS){
                UARTprintf("Could not send data to pedometer queue.\n");
            }

            //memset(&ped_msg, 0, sizeof(ped_msg));

            //reading gpio pin
            GPIO_pin_a4 = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4);
            //UARTprintf("GPIO Pin A4 = %d\n\n\n", GPIO_pin_a4);
        }
    }
}
#endif

#ifdef PULSE
// Read heartbeat digital values
void pulseTask(void *pvParameters)
{
    //instanting the message packet
    static message pulse_msg;
    BaseType_t status_pulseQueue;

    static int count1 = 0;

    BaseType_t status_recv_pedQueue;
    BaseType_t status_send_pedQueue;

   static message recv_ped_msg;
   static message send_ped_msg;

    // Enable Timer0A.
    TimerEnable(TIMER0_BASE, TIMER_A);

    for (;;)
    {
        count1++;

        // Turn on LED 2
        //LEDWrite(0x0F, 0x02);
        //vTaskDelay(1000);

        //UARTprintf("Current BPM is:%d\n", bpm_cpy);

        if(count1%25==0){
            send_ped_msg.source_task = pulse_rate;
            send_ped_msg.log_level = LOG_REQUEST;
            send_ped_msg.request_type = PED_REQUEST;
            send_ped_msg.msg_rqst_type = PULSE_DATA;
            send_ped_msg.type = REQUEST_MESSAGE;
            send_ped_msg.data = 32761;  //placeholder data
            //TODO : Figure out timestamps

            //sending the data to the socket task using queue
            status_send_pedQueue = xQueueSendToBack(sharedQueue2, &send_ped_msg, 50);
            if(status_send_pedQueue != pdPASS){
                UARTprintf("Could not send data to shared queue2.\n");
            }
            else{
                //UARTprintf("Sent a data request to the PEDOMETER task.\n");
            }

            //also send a copy of the request to the serial task
            status_send_pedQueue = xQueueSendToBack(pulseQueue, &send_ped_msg, 50);
            if(status_send_pedQueue != pdPASS){
                UARTprintf("Could not send data to pulse queue.\n");
            }
        }

        //memset(&send_ped_msg, 0, sizeof(send_ped_msg));

        status_recv_pedQueue = xQueueReceive(sharedQueue1, &recv_ped_msg, 50);
        if(status_recv_pedQueue == pdPASS){
            if(recv_ped_msg.source_task == pedometer ){
                if(recv_ped_msg.type == REQUEST_MESSAGE){
                    UARTprintf("Obtained request for pulse rate data\n");
                    //memset(&recv_ped_msg, 0, sizeof(recv_ped_msg));
                    recv_ped_msg.data = bpm_cpy;
                    recv_ped_msg.source_task = pulse_rate;
                    recv_ped_msg.log_level = LOG_INFO_DATA;
                    recv_ped_msg.request_type = NOT_REQUEST;
                    recv_ped_msg.msg_rqst_type = PULSE_DATA;
                    recv_ped_msg.type = RESPONSE_MESSAGE;
                    //TODO : Figure out timestamps

                    //sending the data to the socket task using queue
                    status_send_pedQueue = xQueueSendToBack(sharedQueue2, &recv_ped_msg, 50);
                    if(status_send_pedQueue != pdPASS){
                        UARTprintf("Could not send response to shared queue2.\n");
                    }
                    else{
                        //UARTprintf("Sent a response to the PEDOMETER task.\n");
                    }
                }
                else if(recv_ped_msg.type == RESPONSE_MESSAGE){
                    //UARTprintf("Obtained a response from PEDOMETER task and step count is %d.\n", recv_ped_msg.data);
                }
            }
            //memset(&recv_ped_msg, 0, sizeof(recv_ped_msg));
        }

        pulse_msg.data = bpm_cpy;
        pulse_msg.source_task = pulse_rate;
        pulse_msg.log_level = LOG_INFO_DATA;
        pulse_msg.request_type = NOT_REQUEST;
        pulse_msg.msg_rqst_type = PULSE_DATA;
        pulse_msg.type = LOG_MESSAGE;
        //TODO : Figure out timestamps


        //sending the data to the socket task using queue
        status_pulseQueue = xQueueSendToBack(pulseQueue, &pulse_msg, 50);
        if(status_pulseQueue != pdPASS){
            UARTprintf("Could not send data to pulse queue.\n");
        }

        //memset(&pulse_msg, 0, sizeof(pulse_msg));

    }
}
#endif

void testTask(void *pvParameters){
    //instanting the message packet
    static message test_msg;
    BaseType_t status_testQueue;
    static int i = 0;
    for(;;){
        test_msg.source_task = test_t;
        test_msg.log_level = LOG_MODULE_STARTUP;
        test_msg.msg_rqst_type = FEATURE_ADDED;
        test_msg.request_type = NOT_REQUEST;
        test_msg.type = SYSTEM_INIT_MESSAGE;
        test_msg.data = 9999;

        i++;

        //sending the data to the socket task using queue
        if(i % 15 == 0){
            status_testQueue = xQueueSendToBack(pedQueue, &test_msg, 100);
            if(status_testQueue != pdPASS){
                //UARTprintf("Could not send feature data to pedometer queue.\n");
            }
        }
    }
}


void unit(void){
    int status = 0;

    //TEST1: UART loopback test
    UARTSend((uint8_t *)"4", 1);

    //TEST2: I2C sensor test
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

    //UARTprintf("Finished writing to the CTRL9_XL register.\n");


    //read the CTRL9_XL register(0x18)
    ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, false);   //write to accelerometer
    ROM_I2CMasterDataPut(I2C1_BASE, 0x18);
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(ROM_I2CMasterBusy(I2C1_BASE));

    SysCtlDelay(100); //Delay by 1us

    ROM_I2CMasterSlaveAddrSet(I2C1_BASE, LSM6DS3_ADDR, true);    //read from status sensor
    ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while(ROM_I2CMasterBusy(I2C1_BASE));
    status = ROM_I2CMasterDataGet(I2C1_BASE);

    if(status == 0x38){
        total_tests++;
        pass_tests++;
        UARTprintf("I2C interface read test: PASS\n");
    }
    else{
        total_tests++;
        fail_tests++;
        UARTprintf("I2C interface read test: FAIL\n");
    }

    if(uart_flag){
        UARTprintf("UART loopback test: PASS\n");
    }
    else
        UARTprintf("UART loopback test: FAIL\n");

    //final unit test status
    UARTprintf("Unit Test Status -\n");
    UARTprintf("-------------------------\n");
    UARTprintf("Pass tests: %d\n", pass_tests);
    UARTprintf("Fail tests: %d\n", fail_tests);
}


#ifdef SERIAL
void serialTask(void *pvParameters)
{
    uint8_t msg_len;
    char buffer[200];
    char cat[10];
    //struct tm curr_time;

#ifdef PEDOMETER
    BaseType_t rec_ped_status;
#endif
#ifdef PULSE
    BaseType_t rec_pulse_status;
#endif

    //instanting the message packet
    static message recv_msg;

    for (;;)
    {
#ifdef PEDOMETER
        //receive data from queue with a block of 50 ticks
        rec_ped_status = xQueueReceive(pedQueue, &recv_msg, 50);
        if(rec_ped_status == pdPASS){
            if(recv_msg.source_task == pedometer ){
                if(recv_msg.log_level == LOG_INFO_DATA && recv_msg.type == LOG_MESSAGE){
                    //UARTprintf("Source: pedometer task. Step count is %d\n", recv_msg.data);
                    memset(buffer, 0, sizeof(buffer));
                    strcpy(buffer, "Log_level: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.log_level));
                    strcat(buffer, "|Request_type: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.request_type));
                    strcat(buffer, "|Source_task: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.source_task));
                    strcat(buffer, "|Message_type: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.type));
                    strcat(buffer, "|Msg_rqst_type:  ");
                    strcat(buffer, my_itoa(cat,  recv_msg.msg_rqst_type));
                    strcat(buffer, "|Data: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.data));
                    strcat(buffer, "\n");

                    msg_len = strlen(buffer);
                    UARTSend((uint8_t*)buffer, msg_len);
                }
                else if(recv_msg.type == REQUEST_MESSAGE){
                    UARTprintf("A request message from pulse rate task to pedometer task.\n");

                    memset(buffer, 0, sizeof(buffer));
                    strcpy(buffer, "Log_level: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.log_level));
                    strcat(buffer, "|Request_type: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.request_type));
                    strcat(buffer, "|Source_task: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.source_task));
                    strcat(buffer, "|Message_type: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.type));
                    strcat(buffer, "|Msg_rqst_type:  ");
                    strcat(buffer, my_itoa(cat,  recv_msg.msg_rqst_type));
                    strcat(buffer, "|Data: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.data));
                    strcat(buffer, "\n");

                    msg_len = strlen(buffer);
                    UARTSend((uint8_t*)buffer, msg_len);
                }
            }
            else if(recv_msg.source_task == main_t ){
                if(recv_msg.msg_rqst_type == PED_STARTUP){
                      UARTprintf("Source: main task. Pedometer task is spawned\n");

                      memset(buffer, 0, sizeof(buffer));
                      strcpy(buffer, "Log_level: ");
                      strcat(buffer, my_itoa(cat,  recv_msg.log_level));
                      strcat(buffer, "|Request_type: ");
                      strcat(buffer, my_itoa(cat,  recv_msg.request_type));
                      strcat(buffer, "|Source_task: ");
                      strcat(buffer, my_itoa(cat,  recv_msg.source_task));
                      strcat(buffer, "|Message_type: ");
                      strcat(buffer, my_itoa(cat,  recv_msg.type));
                      strcat(buffer, "|Msg_rqst_type:  ");
                      strcat(buffer, my_itoa(cat,  recv_msg.msg_rqst_type));
                      strcat(buffer, "|Data: ");
                      strcat(buffer, my_itoa(cat,  recv_msg.data));
                      strcat(buffer, "\n");

                      msg_len = strlen(buffer);
                      UARTSend((uint8_t*)buffer, msg_len);
                  }
            }
            else if(recv_msg.source_task == test_t ){
                      UARTprintf("Source: main task. Feature task sent a msg.\n");

                      memset(buffer, 0, sizeof(buffer));
                      strcpy(buffer, "Log_level: ");
                      strcat(buffer, my_itoa(cat,  recv_msg.log_level));
                      strcat(buffer, "|Request_type: ");
                      strcat(buffer, my_itoa(cat,  recv_msg.request_type));
                      strcat(buffer, "|Source_task: ");
                      strcat(buffer, my_itoa(cat,  recv_msg.source_task));
                      strcat(buffer, "|Message_type: ");
                      strcat(buffer, my_itoa(cat,  recv_msg.type));
                      strcat(buffer, "|Msg_rqst_type:  ");
                      strcat(buffer, my_itoa(cat,  recv_msg.msg_rqst_type));
                      strcat(buffer, "|Data: ");
                      strcat(buffer, my_itoa(cat,  recv_msg.data));
                      strcat(buffer, "\n");

                      msg_len = strlen(buffer);
                      //UARTprintf("string is %s", buffer);
                      UARTSend((uint8_t*)buffer, msg_len);
            }
        }
#endif

        //memset(&recv_msg, 0, sizeof(recv_msg));

#ifdef PULSE
        rec_pulse_status = xQueueReceive(pulseQueue, &recv_msg, 50);
        if(rec_pulse_status == pdPASS){
            if(recv_msg.source_task == pulse_rate ){

                if(recv_msg.msg_rqst_type == PULSE_STARTUP){
                    UARTprintf("Source: main task. Pulse task is spawned\n");
                    memset(buffer, 0, sizeof(buffer));
                    strcpy(buffer, "Log_level: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.log_level));
                    strcat(buffer, "|Request_type: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.request_type));
                    strcat(buffer, "|Source_task: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.source_task));
                    strcat(buffer, "|Message_type: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.type));
                    strcat(buffer, "|Msg_rqst_type:  ");
                    strcat(buffer, my_itoa(cat,  recv_msg.msg_rqst_type));
                    strcat(buffer, "|Data: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.data));
                    strcat(buffer, "\n");

                    msg_len = strlen(buffer);
                    UARTSend((uint8_t*)buffer, msg_len);
                }
                else if(recv_msg.log_level == LOG_INFO_DATA){
                    //UARTprintf("Source: pulse task. BPM received from queue: %d\n\n", recv_msg.data);
                    memset(buffer, 0, sizeof(buffer));
                    strcpy(buffer, "Log_level: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.log_level));
                    strcat(buffer, "|Request_type: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.request_type));
                    strcat(buffer, "|Source_task: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.source_task));
                    strcat(buffer, "|Message_type: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.type));
                    strcat(buffer, "|Msg_rqst_type:  ");
                    strcat(buffer, my_itoa(cat,  recv_msg.msg_rqst_type));
                    strcat(buffer, "|Data: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.data));
                    strcat(buffer, "\n");

                    msg_len = strlen(buffer);
                    UARTSend((uint8_t*)buffer, msg_len);
                }
                else if(recv_msg.type == REQUEST_MESSAGE){
                    UARTprintf("A request message from pedometer task to pulse rate task.\n");
                    memset(buffer, 0, sizeof(buffer));
                    strcpy(buffer, "Log_level: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.log_level));
                    strcat(buffer, "|Request_type: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.request_type));
                    strcat(buffer, "|Source_task: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.source_task));
                    strcat(buffer, "|Message_type: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.type));
                    strcat(buffer, "|Msg_rqst_type:  ");
                    strcat(buffer, my_itoa(cat,  recv_msg.msg_rqst_type));
                    strcat(buffer, "|Data: ");
                    strcat(buffer, my_itoa(cat,  recv_msg.data));
                    strcat(buffer, "\n");

                    msg_len = strlen(buffer);
                    UARTSend((uint8_t*)buffer, msg_len);
                }
            }
            else if(recv_msg.source_task == main_t){
                if(recv_msg.msg_rqst_type == PULSE_STARTUP){
                      UARTprintf("Source: main task. Pulse task is spawned\n");

                      memset(buffer, 0, sizeof(buffer));
                      strcpy(buffer, "Log_level: ");
                      strcat(buffer, my_itoa(cat,  recv_msg.log_level));
                      strcat(buffer, "|Request_type: ");
                      strcat(buffer, my_itoa(cat,  recv_msg.request_type));
                      strcat(buffer, "|Source_task: ");
                      strcat(buffer, my_itoa(cat,  recv_msg.source_task));
                      strcat(buffer, "|Message_type: ");
                      strcat(buffer, my_itoa(cat,  recv_msg.type));
                      strcat(buffer, "|Msg_rqst_type:  ");
                      strcat(buffer, my_itoa(cat,  recv_msg.msg_rqst_type));
                      strcat(buffer, "|Data: ");
                      strcat(buffer, my_itoa(cat,  recv_msg.data));
                      strcat(buffer, "\n");

                      msg_len = strlen(buffer);
                      UARTSend((uint8_t*)buffer, msg_len);
                  }
            }
        }
#endif
    }
}
#endif


void notifyTask(void *pvParameters)
{
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );
    BaseType_t xResult;
    uint32_t ulNotifiedValue;

       for( ;; )
       {
          /* Wait to be notified of an interrupt. */
          xResult = xTaskNotifyWait( pdFALSE,    /* Don't clear bits on entry. */
                               0x03,        /* Clear all bits on exit. */
                               &ulNotifiedValue, /* Stores the notified value. */
                               xMaxBlockTime );

          if( xResult == pdPASS )
          {
             /* A notification was received.  See which bits were set. */
             if( ( ulNotifiedValue & STEP_BIT ) != 0 )
             {
                /* The TX ISR has set a bit. */
                //prvProcessTx();
                 UARTprintf("Steps exceeded!\n");
                 LEDWrite(0x0F, 0x01);
             }

             if( ( ulNotifiedValue & PULSE_BIT ) != 0 )
             {
                /* The RX ISR has set a bit. */
                //prvProcessRx();
                 UARTprintf("Pulse exceeded!\n");
                 LEDWrite(0x0F, 0x01);
             }
          }
          else
          {
             /* Did not receive a notification within the expected time. */

          }
       }


}
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
    static message main_msg;
#ifdef PEDOMETER
    BaseType_t status_pedQueue;
#endif
#ifdef PULSE
    BaseType_t status_pulseQueue;
#endif

    // Initialize system clock to 120 MHz
    output_clock_rate_hz = ROM_SysCtlClockFreqSet(
                               (SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                               SYSTEM_CLOCK);
    ASSERT(output_clock_rate_hz == SYSTEM_CLOCK);

    // Initialize the GPIO pins for the Launchpad
    PinoutSet(false, false);

    // Set up the UART which is connected to the virtual COM port
    UARTStdioConfig(0, 57600, SYSTEM_CLOCK);
    GPIO_Init();

    //
    // Enable the GPIO port and pin that is used for the on-board LED.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);

    UART_Init();

#ifdef  PEDOMETER
    I2C_Init();
#endif
#ifdef PULSE
    ADC_Init();
    Timer_Init();
    Peripheral_Int();       //Enable interrupts of peripherals
    Comparator_Init();
    Hibernate_Init();
#endif

    //creating the shared queues
    sharedQueue1 = xQueueCreate(10, sizeof(message));
    sharedQueue2 = xQueueCreate(10, sizeof(message));

#ifdef PEDOMETER
    //creating the pedometer queue
    pedQueue = xQueueCreate(10, sizeof(message));

    // Create pedometer task
    xTaskCreate(pedometerTask, (const portCHAR *)"pedometer",
                  configMINIMAL_STACK_SIZE, NULL, 2, NULL);


     main_msg.source_task = main_t;
     main_msg.log_level = LOG_MODULE_STARTUP;
     main_msg.msg_rqst_type = PED_STARTUP;
     main_msg.request_type = NOT_REQUEST;
     main_msg.type = SYSTEM_INIT_MESSAGE;
     main_msg.data = 32763;
     //TODO : Figure out timestamps

     //sending the data to the socket task using queue
     status_pedQueue = xQueueSendToBack(pedQueue, &main_msg, 50);
     if(status_pedQueue != pdPASS){
         UARTprintf("Could not send data to pedometer queue.\n");
     }


#endif
#ifdef PULSE
     //creating the pulse queue
     pulseQueue = xQueueCreate(10, sizeof(message));

    // Create heartbeat task
    xTaskCreate(pulseTask, (const portCHAR *)"pulse",
                configMINIMAL_STACK_SIZE, NULL, 2, NULL);


    main_msg.source_task = main_t;
    main_msg.log_level = LOG_MODULE_STARTUP;
    main_msg.msg_rqst_type = PULSE_STARTUP;
    main_msg.request_type = NOT_REQUEST;
    main_msg.type = SYSTEM_INIT_MESSAGE;
    main_msg.data = 32764;
    //TODO : Figure out timestamps

    //sending the data to the socket task using queue
    status_pulseQueue = xQueueSendToBack(pulseQueue, &main_msg, 50);
    if(status_pulseQueue != pdPASS){
        UARTprintf("Could not send data to pulse queue.\n");
    }


#endif

#ifdef SERIAL
    xTaskCreate(serialTask, (const portCHAR *)"serial",
                configMINIMAL_STACK_SIZE, NULL, 1, NULL);
#endif

    xTaskCreate(notifyTask, (const portCHAR *)"notify",
                configMINIMAL_STACK_SIZE, NULL, 1, &notifyHandle);

   // xTaskCreate(testTask, (const portCHAR *)"test",
                //configMINIMAL_STACK_SIZE, NULL, 1, NULL);

#ifdef UNIT
    unit();
#endif
    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();

    //Start scheduler
    vTaskStartScheduler();

    return 0;
}

