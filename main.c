/*
 *
 * 1. Program Device Configuration;
 *      - SMCLK, DCO at 1MHz
 *      - ACLK, external oscillator 32.768kHz
 *      - USCIA0 as UART, 9600, 8 bits Data, No parity, 1 Stop (Hardware Mode)
 *      - USCIB0 as 3 line SPI, Master Mode, 100kHz SCLK
 *      - P2.7 as output, is used as CS for ADS1118 ADC device
 *      - P8.1 as output,is used as RST of LCD
 *      - P2.3 as output,is used as RS of LCD
 *      - P2.6 as output,is used as CS of LCD
 *      - P6.5 as output, is used as switch of buzzer
 * 2. this project is used to measure the temperature by type-K thermocouple.
 *      - far-end temperature is measured by thermocouple, local temperature is measured by the internal sensor of ADS1118
 *      - ADC mode: inputs are AIN0/AIN1, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
 *      - temperature mode: DR=128sps, PULLUP on DOUT
 *      - Reference Tables are used to transform ADC code to temperature.
 *
 */

/*
 * ======== Standard MSP430 includes ========
 */
#include <msp430.h>
#include <driverlib.h>

#include "ADS1118.h"
#include "LCD_driver.h"
#include "led.h"
#include "uart.h"
#include "foam_cutter.h"

void System_Initial();  //Initialize system
void set_Thrtemp();     //set threshold temperature
void half_second();
void ADC_display();

/** new system functions for the F5529 ***/
void SysInit_F5529();
void GPIO_init(void);
void InitSPI();
void InitTimers();

// redefined for the F5529 BOOST board
#define BUZZ_ON     P6OUT &= ~BIT5; //set P4.2 low
#define BUZZ_OFF    P6OUT |= BIT5;  //set P4.2 high


/*
 * ======== global variable ========
 */

/* flag is used to transmit parameter between interrupt service function and main function.
 * the flag will be changed by ISR in InterruptVectors_init.c     ...\grace\InterruptVectors_init.c
 *
 * Bit0, Launchpad S2 is pushed
 * Bit1, Launchpad S1 is pushed
 * Bit2
 * Bit3, 1 second timer interrupt
 * Bit4, timer for ADC interrupts
 * Bit5, ADC input flag, 0 is internal temperature sensor, 1 is AIN0-AIN1
 * Bit6, make an inversion every half a second
 * Bit7, half a second interrupt
 * Bit8, for Fahrenheit display
 * Bit9, ADC channel flag, 0 for channel 0, 1 for channel 1.
 * BitA,
 */
volatile unsigned int  flag;		//global flag.
volatile unsigned char Thr_state;	// state for threshold temperature setting state machine.

unsigned int Thr_temp;	// Threshold temperature in degrees Centigrade
unsigned int set_temp;	// temporary for setting Threshold temperature
unsigned int num=0;		// temporary for setting Threshold temperature
		 int Act_temp;	// Actual temperature

//-----------------------------------------------------------------------------
int _system_pre_init(void)
{
	// Stop Watchdog timer
	WDT_A_hold(__MSP430_BASEADDRESS_WDT_A__);     // Stop WDT

	/*==================================*/
	/* Choose if segment initialization */
	/* should be done or not. */
	/* Return: 0 to omit initialization */
	/* 1 to run initialization */
	/*==================================*/
	return 1;
}

unsigned int t,s;

/*
 *  ======== main ========
 */
int main(int argc, char *argv[])
{
    /*** main system initialization
     *     UART, GPIO, WDT, CLOCK, System Registers
     */
    SysInit_F5529();
    delay(500);
    _enable_interrupt();        // enable interrupt
    System_Initial();           // initialize system.
    delay(500);                    // delay and wait the first conversion.

    t = tick_getTime();
    delay(100);
    s = tick_getTime();


    while(1)
    {
        // half a second interrupt.
        if(flag & BIT7)
        {
            half_second();
            blink_led1();
        }

        // read ADC result
        if(flag & BIT4)         //Read ADC result
        {
            ADC_display();
            blink_led2();
        }

   		if(flag & BIT0)				// S2 service, set the Threshold temperature
   		{
   			flag &= ~ BIT0;			// flag is reset
   			if(Thr_state != 0)
   			{
   				set_Thrtemp();		// set threshold temperature.
   			}
   			else
   				flag ^= BIT8;		// display temperature in Fahrenheit
   		}

   		if(flag & BIT1)				// if S1 is pushed, threshold temperature state machine will be changed
   		{
   			flag &= ~BIT1;			// flag is reset
   			if(Thr_state >= 3)		// if in state 3, change to state 0;
   			{
   				Thr_state = 0;
   				Thr_temp = set_temp;				// assign threshold temperature
   				LCD_display_number(0,3,Thr_temp);	// display threshold temperature
   			}
   			else					//else, Thr_state is changed to next state
   			{
   				Thr_state ++;
   			}
   		}

   		else
   		__no_operation();
    }
    
// never gets here..

}


/*
 * function name:System_Initial()
 * description: Initialize the system. include I/O, LCD and ADS1118.
 */
void System_Initial()
{
    flag  = 0;      //reset flag

    Thr_state = 0;  //threshold temperature setting state machine counter
    Thr_temp = 100; //configure threshold temperature to 100;
    Act_temp = 0;
    set_temp = Thr_temp; // set for future use in changing values

    // IO initial
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6);    // CS of LCD HIGH
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3);    // RS of LCD HIGH
    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN1);    // RST of LCD HIGH
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);    // CS of DS1118 HIGH
    GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN5);    // BUZZER OFF

    flag |= BIT8;   // Set temp to display Farenheit
    //flag |= BIT9; // set bit9 to use CH1 in the ads1118

    LCD_init();                     // LCD initial
    LCD_clear();                    // LCD clear
    LCD_display_string(0,"Th:\0");  // display "ADS1118"
    LCD_display_string(1,"Temp:        CH0\0"); // display threshold temp and actual temp;
    LCD_display_char(1,10,0xDF);
    LCD_display_char(1,11,'F');
    LCD_display_number(0,3,Thr_temp);// display threshold temp number

    ADS_Config(0);                  // set ADS1118 to convert local temperature, and start conversion.

}

#define GPIO_ALL    GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3| \
                    GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7

/*
 * This function drives all the I/O's as output-low, to avoid floating inputs
 * (which cause extra power to be consumed).  This setting is compatible with
 * TI FET target boards, the F5529 Launchpad, and F5529 Experimenters Board;
 * but may not be compatible with custom hardware, which may have components
 * attached to the I/Os that could be affected by these settings.  So if using
 * other boards, this function may need to be modified.
 */
void initPorts(void)
{
#ifdef __MSP430_HAS_PORT1_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT2_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT3_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT4_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT5_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT6_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT7_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT8_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT9_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORTJ_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_PJ, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_PJ, GPIO_ALL);
#endif
}

/* Configures the system clocks:
 * MCLK = SMCLK = DCO/FLL = mclkFreq (expected to be expressed in Hz)
 * ACLK = FLLref = REFO=32kHz
 *
 * XT2 is not configured here.  Instead, the USB API automatically starts XT2
 * when beginning USB communication, and optionally disables it during USB
 * suspend.  It's left running after the USB host is disconnected, at which
 * point you're free to disable it.  You need to configure the XT2 frequency
 * in the Descriptor Tool (currently set to 4MHz in this example, since that's
 * what the Launchpad uses).  See the Programmer's Guide for more information.
 */
void InitClocks(unsigned long mclkFreq)
{
    UCS_initClockSignal(
       UCS_FLLREF,
       UCS_REFOCLK_SELECT,
       UCS_CLOCK_DIVIDER_1);

    UCS_initClockSignal(
       UCS_ACLK,
       UCS_REFOCLK_SELECT,
       UCS_CLOCK_DIVIDER_1);

    UCS_initFLLSettle(
        mclkFreq/1000,
        mclkFreq/32768);
        //use REFO for FLL and ACLK
        UCSCTL3 = (UCSCTL3 & ~(SELREF_7)) | (SELREF__REFOCLK);
        UCSCTL4 = (UCSCTL4 & ~(SELA_7)) | (SELA__REFOCLK);

}


/****
 *      SysInit_F5529
 *      initialization of the chip - a MSP430F5529
 *          UART, GPIO, WDT, CLOCK, System Registers
 */

void SysInit_F5529()
{
    _disable_interrupt();

    // Set WatchDog Timer off, so doesn't fire during init
    WDTCTL = WDTPW + WDTHOLD;

    /* initialize Config for the MSP430 GPIO */
    GPIO_init();

    /* initialize Config for the MSP430 clock */
    InitClocks(8000000);

    /* initialize Config for the MSP430 USCI_B0 */
    InitSPI();

    InitTimers();

    tick_init();

    uart_init();

    // Enable global interrupt
    __enable_interrupt();

    // Set WatchDog Timer off
    WDTCTL = WDTPW + WDTHOLD;
}
void delay(unsigned int d)
{
    unsigned int t,s;

    t = tick_getTime();
    s = t + d;

    do
    {
        t = tick_getTime();
        __no_operation();
    } while (t<s);
}

void ADC_display()
{
    //float sensor;

    static signed int local_data = 0, far_data = 0;
    signed int temp;

    flag &= ~ BIT4;                 // flag is reset
    if (!(flag & BIT5))
    {
        local_data = ADS_Read(1);   //read local temperature data,and start a new convertion for far-end temperature sensor.
    }
    else
    {
        far_data = ADS_Read(0);     //read far-end temperature,and start a new convertion for local temperature sensor.
        temp = far_data + local_compensation(local_data);   // transform the local_data to compensation codes of far-end.

        temp = ADC_code2temp(temp); // transform the far-end thermocouple codes to temperature.

        if(flag & BIT8)             // display temperature in Fahrenheit
        {
            Act_temp = (signed int)(((temp * 9) / 5) + 320);
            LCD_display_temp(1,5,Act_temp);
            LCD_display_char(1,11,'F');
        }
        else
        {
            Act_temp = temp;
            LCD_display_temp(1,5,Act_temp);
            LCD_display_char(1,11,'C');
        }

    }

    if(flag & BIT9)
        LCD_display_char(1,15,'1');
    else
        LCD_display_char(1,15,'0');

}


/*
 * function name: half_second()
 * description: it is executed every half a second. it has three functions,
 * the first one is to compare the Actual temperature and threshold temperature, if Actual temperature is higher than threshold
 * temperature, buzzer will work
 * the second one is to flicker the threshold temperature bit which is being set.
 * the third one is to flicker the time bit which is being set.
 */

void half_second()
{
    int threshold_temp;
    flag &= ~ BIT7;


    // judge actual temperature is higher than threshold temperature. if higher, buzzer will work
    if(flag & BIT8) // check for Farenheit conversion
    {
        threshold_temp = 10*(((Thr_temp * 9)/5)+32);
    }
    else threshold_temp = (10*Thr_temp);

    if((Act_temp >= threshold_temp) && (flag & BIT6))
    {
        BUZZ_ON;
    }
    else
    {
        BUZZ_OFF;
    }

    //display threshold temperature setting
    if(Thr_state == 0x01)                       //threshold temperature state machine output.
    {
        if (flag & BIT6)
            LCD_display_char(0,4,' ');          //display blank space for half a second
        else
            LCD_display_number(0,3,set_temp);   //display hundred place for half a second
    }
    else if(Thr_state == 0x02)
    {
        if (flag & BIT6)
            LCD_display_char(0,5,' ');          //display blank space for half a second
        else
            LCD_display_number(0,3,set_temp);   //display decade for half a second
    }
    else if(Thr_state == 0x03)
    {
        if (flag & BIT6)
            LCD_display_char(0,6,' ');          //display blank space for half a second
        else
            LCD_display_number(0,3,set_temp);   //display unit's digit for half a second
    }
}

/*
 * function name:set_Thrtemp()
 * description: set the threshold temperature. the temporary is saved in variable set_temp.
 */
void set_Thrtemp()
{
    set_temp = Thr_temp;

    if (Thr_state == 0x01)
    {
        if (set_temp/100 == 9)
        {
            set_temp = set_temp-900;
        }
        else
        {
            set_temp += 100;
        }
    }
    else if (Thr_state == 0x02)
    {
        if (set_temp%100/10 == 9)
        {
            set_temp = set_temp-90;
        }
        else
        {
            set_temp += 10;
        }
    }
    else if (Thr_state == 0x03)
    {
        if (set_temp%10 == 9)
        {
            set_temp = set_temp-9;
        }
        else
        {
            set_temp ++;
        }
    }
    else
    __no_operation();

    Thr_temp = set_temp;
}




/*
 *  ======== GPIO_init ========
 *  Initialize MSP430 General Purpose Input Output Ports
 *
 *  Port pins must match funcionality of the ADS118 BOOST kit headers
 *
 *  Header P1
 *      Pin 1 = Vcc
 *      Pin 2 = Output to drive the Buzzer
 *      Pin 3,4,5 = no connection
 *      Pin 6 = not clear yet.. pulled high in circuit.. Vref ? (No.. error in board by TI - has no purpose)
 *      Pin 7 = SPI SCLK
 *      Pin 8 = SPI CS1
 *      Pin 9 = SW1, normal HI
 *      Pin 10 = SW2, normal HI
 *
 *  Header P2
 *      Pin 1 = GND
 *      Pin 2,3,4,5 = no connection
 *      Pin 6 = SPI SMO
 *      Pin 7 = SPI SMI
 *      Pin 8 = LCD CS
 *      Pin 9 = LCD RS
 *      Pin 10 = LCD RST
 *
 * --------------------------------------------
 * 	F5529 LaunchPad kit headers
 *  Header P1
 *      Pin 1 = Vcc
 * 		Pin 2 = P6.5 => out to buzzer
 * 		Pin 3 = P3.4
 * 		Pin 4 = P3.3
 * 		Pin 5 = P1.6
 * 		Pin 6 = P6.6 => unused - but held high by circuit -- set as Input
 * 		Pin 7 = P3.2 => SPI CLK
 * 		Pin 8 = 2.7 => SPI CS
 *      Pin 9 = P4.2 => SW1 detect - normal HI
 *      Pin 10 = P4.1 => SW2 detect - normal HI
 *
 *  Header P2
 *      Pin 1 = GND
 *      Pin 2 = P2.0
 *      Pin 3 = P2.2
 *      Pin 4 = P7.4
 *      Pin 5 = RST
 * 		Pin 6 = P3.0 => SPI SMO
 * 		Pin 7 = P3.1 => SPI SMI
 * 		Pin 8 = P2.6 => LCD CS
 * 		Pin 9 = P2.3 => LCD RS
 * 		Pin 10 = P8.1 => LCD RST
 *
 *  S1 --> P2.1
 *  S2 --> P1.1
 *  SW1 --> P4.2  on ADS118
 *  SW2 --> P4.1  on ADS118
 *
 *  LED1 P4.7
 *  LED2 P1.0
 *
 */
void GPIO_init()
{
    // Set all ports to output low
    initPorts();

    // P1.1 SW2 Input
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN1, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    // P2.1 SW1 input
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN1, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN1);

    // Buzzer off
    GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN5);

    // Clear all interrupt flags
    P1IFG = 0;



    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    //PMM_unlockLPM5();

}

void InitSPI()
{
    // Configure SPI pins

    // Set 3.2 to input Secondary Module Function, (UCB0CLK).
    GPIO_setAsPeripheralModuleFunctionOutputPin(
            GPIO_PORT_P3,
            GPIO_PIN2
            );

    // Configure Pins for UCB0TXD/UCB0SIMO, UCB0RXD/UCB0SOMI
    //Set P3.1 as Secondary Module Function Input.
    GPIO_setAsInputPin(GPIO_PORT_P3, GPIO_PIN1);
    GPIO_setAsPeripheralModuleFunctionOutputPin(
            GPIO_PORT_P3,
            GPIO_PIN1
            );


    // Configure Pins for UCB0TXD/UCB0SIMO, UCB0RXD/UCB0SOMI
    //Set P3.0 as Secondary Module Function Output.
    GPIO_setAsPeripheralModuleFunctionOutputPin(
            GPIO_PORT_P3,
            GPIO_PIN0
            );


    //Initialize Master
    USCI_B_SPI_initMasterParam param = {0};
    param.selectClockSource = USCI_B_SPI_CLOCKSOURCE_ACLK;
    param.clockSourceFrequency = UCS_getACLK();
    param.desiredSpiClock = 30000;
    param.msbFirst = USCI_B_SPI_MSB_FIRST;
    param.clockPhase = USCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
    param.clockPolarity = USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;
    USCI_B_SPI_initMaster(USCI_B0_BASE, &param);

    //Enable SPI module
    USCI_B_SPI_enable(USCI_B0_BASE);


    USCI_B_SPI_clearInterrupt (USCI_B0_BASE,
                               USCI_B_SPI_RECEIVE_INTERRUPT);

    //Wait for slave to initialize
    __delay_cycles(1000);

}


/***
 * Initialize Timer0 and Timer1
 *  both set to A3 mode
 */
void InitTimers(void)
{
    // Timer 0
    // needs to be running at 0.5 sec
    /*
     * TA0CCTL0, Capture/Compare Control Register 0
     *
     * CM_0 -- No Capture
     * CCIS_0 -- CCIxA
     * ~SCS -- Asynchronous Capture
     * ~SCCI -- Latched capture signal (read)
     * ~CAP -- Compare mode
     * OUTMOD_0 -- PWM output mode: 0 - OUT bit value
     *
     * Note: ~<BIT> indicates that <BIT> has value zero
     */
    TA0CCTL0 = CM_0 + CCIS_0 + OUTMOD_0 + CCIE;

    /* TA0CCR0, Timer_A Capture/Compare Register 0 */
    TA0CCR0 = 16384;  // 0.5 sec -- 32768 clock

    /*
     * TA0CTL, Timer_A0 Control Register
     *
     * TASSEL_1 -- ACLK
     * ID_0 -- Divider - /1
     * MC_1 -- Up Mode
     */
    TA0CTL = TASSEL_1 + ID_0 + MC_1;
    //---------------------------------------------

    //---------------------------------------------
    // Timer 1
    //  set for 0.1 sec to sample temp
    //
    /*
     * TA1CCTL0, Capture/Compare Control Register 0
     *
     * CM_0 -- No Capture
     * CCIS_0 -- CCIxA
     * ~SCS -- Asynchronous Capture
     * ~SCCI -- Latched capture signal (read)
     * ~CAP -- Compare mode
     * OUTMOD_0 -- PWM output mode: 0 - OUT bit value
     *
     * Note: ~<BIT> indicates that <BIT> has value zero
     */
    TA1CCTL0 = CM_0 + CCIS_0 + OUTMOD_0 + CCIE;

    /* TA1CCR0, Timer_A Capture/Compare Register 0 */
    TA1CCR0 = 3280;  // 0.1 sec -- 32768 clk

    /*
     * TA1CTL, Timer_A3 Control Register
     *
     * TASSEL_1 -- ACLK
     * ID_0 -- Divider - /1
     * MC_1 -- Up Mode
     */
    TA1CTL = TASSEL_1 + ID_0 + MC_1;
    //----------------------------------------------
}


//***************************************************************************
//***************************************************************************
//***************************************************************************
//
//               ISR routines
//
//***************************************************************************8
//***************************************************************************8
//***************************************************************************8

/*
 * Port 1 interrupt service routine to handle S2 button press
 *
 */
#if defined (__TI_COMPILER_VERSION__) || defined (__IAR_SYSTEMS_ICC__)
#pragma vector = PORT1_VECTOR
__interrupt
#elif defined(__GNUC__)
void Port_1_ISR(void) __attribute__((interrupt(PORT1_VECTOR)));
#else
#error Compiler not supported!
#endif
void Port_1_ISR(void)
{
    GPIO_disableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    if (GPIO_getInterruptStatus(GPIO_PORT_P1, GPIO_PIN1))      // Switch S2
    {
        flag |= BIT0;       // flag bit 1 is set
        GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    }

    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);
}

/*
 * Port 2 interrupt service routine to handle S1 button press
 *
 */
#if defined (__TI_COMPILER_VERSION__) || defined (__IAR_SYSTEMS_ICC__)
#pragma vector = PORT2_VECTOR
__interrupt
#elif defined(__GNUC__)
void Port_2_ISR(void) __attribute__((interrupt(PORT2_VECTOR)));
#else
#error Compiler not supported!
#endif
void Port_2_ISR(void)
{
    GPIO_disableInterrupt(GPIO_PORT_P2, GPIO_PIN1);

    if (GPIO_getInterruptStatus(GPIO_PORT_P2, GPIO_PIN1))      // Switch S1
    {
        flag |= BIT1;       // flag bit 1 is set
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN1);
    }

    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN1);
}


/*
 *  ======== Timer0_A0 Interrupt Service Routine ========
 *    set for 0.5 sec interval
 */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt
#elif defined(__GNUC__)
void TIMER0_A0_ISR(void) __attribute__((interrupt(TIMER0_A0_VECTOR)));
#endif
void TIMER0_A0_ISR(void)
{
    flag |= BIT7;
    flag ^= BIT6;
    if (!(flag & BIT6))
        flag |= BIT3;
}

/*
 *  ======== Timer1_A0 Interrupt Service Routine ========
 *   set for 0.10 sec interval
 */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt
#elif defined(__GNUC__)
void TIMER1_A0_ISR(void) __attribute__((interrupt(TIMER1_A0_VECTOR)));
#endif
void TIMER1_A0_ISR(void)
{
    flag |= BIT4;
    flag ^= BIT5;
}
