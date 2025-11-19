#include <msp430.h>

#include "myClocks.h"  // Required for the LCD
#include "myGpio.h"    // Required for the LCD
#include "myLcd.h"     // Required for the LCD
#include <driverlib.h> // Required for the LCD

#define RED_LED 0x0001       // P1.0 is the red LED
#define GREEN_LED 0x0080     // P9.7 is the green LED
#define STOP_WATCHDOG 0x5A80 // Stop the watchdog timer
#define ACLK 0x0100          // Timer_A ACLK source
#define UP 0x0010            // Timer_A Up mode
#define SMCLK 0x0200         // Timer_A SMCLK source
#define TIMER_OFF 0x30       // CLEAR these bits to stop Timer_A
#define TIMER_CLEAR 0x4      // Set this bit to clear the count of Timer_A
#define ENABLE_PINS 0xFFFE   // Required to use inputs and outputs

#define IR_LED BIT7 // P9.7 will be output for IR LED (it's also the green LED)

void ADC_SETUP(void);       // Used to setup ADC12 peripheral
void SwitchToLFXT(void);    // Switches ACLK to Low Frequency eXTernal crystal
                            // (LFXT) at 32.768kHz
void SMCLK_SetTo1MHz(void); // Switches SMCLK to DCO at 1MHz

volatile unsigned int ultrasonic_pulse_us;
// width of pulse from ultrasonic sensor, in microseconds

volatile unsigned int ir_on;
volatile unsigned int ir_off;
volatile int ir_state; // 0 for off, 1 for on. volatile

void main(void) {
  WDTCTL = WDTPW | WDTHOLD; // Stop WDT

  //**********************
  //* GPIO Setup
  //**********************

  PM5CTL0 = ENABLE_PINS; // Enable inputs and outputs

  //**********************
  //* LED Driver Setup
  //* Driver on pin P9.7
  //**********************
  P9DIR = IR_LED;     // P9.7 will be output for green LED
  P9OUT &= ~(IR_LED); // Drive LED off

  //**********************
  //* Clock Configuration
  //**********************
  SwitchToLFXT();
  SMCLK_SetTo1MHz();

  //******************************************************************
  //* Ultrasonic Sensor
  //* Trigger Output Setup
  //* Timer A 0 controls the trigger input to the ultrasonic sensor.
  //* Runs at 1 MHz in up mode, output mode 7.
  //* Sends a (at least) 10 us pulse every 60 ms (60,000 us).
  //* Outputs these pulses on P1.6.
  //******************************************************************
  TA0CCR0 = 60000;  // PWM period period is 60,000 us.
  TA0CCR1 = 100;    // PWM duty cycle is on for at least 10 us.
  TA0CCTL1 |= 0xE0; // Output Mode 7 (reset/set)

  TA0CTL = SMCLK | UP;   // Set SMCLK, UP MODE
  TA0CTL |= TIMER_CLEAR; // Timer starts at 0

  // GPIO Output
  P1SELC |= BIT6; // Set bits P1SEL1.6 and P1SEL0.6 simultaneously
  P1DIR |= BIT6;  // P1.6 is an output

  //******************************************************************
  //* Ultrasonic Sensor
  //* Echo Input Setup
  //* Pin P1.3 receives pulses from the ultrasonic sensor and generates
  //* interrupts.
  //* Timer A 2 measures the width of the echo pulse.
  //******************************************************************
  TA2CTL = SMCLK | MC__CONTINOUS; // Set SMCLK, CONTINUOUS MODE
  TA2CTL &= ~(TIMER_OFF);         // Timer starts as off
  TA2CTL |= TIMER_CLEAR;          // Timer starts at 0

  // GPIO Input
  P1IE = BIT3;      // Enable interrupt for P1.3
  P1IES &= ~(BIT3); // Start by looking for rising edges
  P1IFG = 0x00;     // Ensure no interrupts are pending

  //**********************
  //* Timer A 2 Setup
  //* The LCD display gets updated with the latest IR sensor reading from the
  // ADC
  //* whenever this timer interrupt goes off.
  //**********************
  // unsigned int clock_scalar = 0b111; // eigth scalar
  // unsigned int clock_count = 4096;
  // TA2EX0 = clock_scalar; // Sets the clock scalar value for Timer_2
  // TA2CCR0 = clock_count; // Sets value of Timer_2
  // TA2CTL = ACLK | UP;    // Set ACLK, UP MODE for Timer_2
  // TA2CCTL0 = CCIE;       // Enable interrupt for Timer_2

  //**********************
  //* Left and Right Motor Driver Setup
  //**********************
  // TODO set up motor GPIO pins

  //**********************
  //* Left and Right Motor PWM Output Setup
  //* Using Timer A X
  //**********************
  // refer to Lab09 for how to set up
  // TODO

  //**********************
  //* LCD Setup
  //**********************
  initGPIO();   // Initializes General Purpose Inputs and Outputs for LCD
  initClocks(); // Initialize clocks for LCD
  myLCD_init(); // Prepares LCD to receive commands

  //**********************
  //* ADC Setup
  //**********************
  ADC_SETUP();
  ADC12IER0 = ADC12IE0; // Enable ADC interrupt
  __enable_interrupt(); // Activate interrupts previously enabled

  ir_state = 1;
  P9OUT |= IR_LED;                  // turn on IR LED
  ADC12CTL0 = ADC12CTL0 | ADC12ENC; // Enable conversion
  ADC12CTL0 = ADC12CTL0 | ADC12SC;  // Start conversion

  while (1) {
  }
}

void ADC_SETUP(void) {
#define ADC12_SHT_16 0x0200      // 16 clock cycles for sample and hold
#define ADC12_SHT_128 0x0600     // 128 clock cycles for sample and hold
#define ADC12_SHT_512 0x0A00     // 128 clock cycles for sample and hold
#define ADC12_ON 0x0010          // Used to turn ADC12 peripheral on
#define ADC12_SHT_SRC_SEL 0x0200 // Selects source for sample & hold
#define ADC12_12BIT 0x0020       // Selects 12-bits of resolution
#define ADC12_P92 0x000A         // Use input P9.2 for analog input

  ADC12CTL0 = ADC12_SHT_512 | ADC12_ON; // Turn on, set sample & hold time
  ADC12CTL1 = ADC12_SHT_SRC_SEL;        // Specify sample & hold clock source
  ADC12CTL2 = ADC12_12BIT;              // 12-bit conversion results
  ADC12MCTL0 = ADC12_P92;               // P9.2 is analog input
}

//***********************************************************************
//* Port 1 Interrupt Service Routine
//* Measures pulse width from the ultrasonic sensor's echo pin
//* Using Timer A 2
//***********************************************************************
#pragma vector = PORT1_VECTOR
__interrupt void Port_1(void) {
  if (P1IV == 0x8) { // check that port P1.3 is the source of the interrupt
    if (!(P1IES & BIT3)) {     // Rising edge detected
      TA2CTL |= MC__CONTINOUS; // start the timer
      P1IES |= BIT3;           // searching for falling edge next
    } else {
      TA2CTL &= ~(TIMER_OFF);     // Turn timer off
      ultrasonic_pulse_us = TA2R; // Save reading
      TA2CTL |= TIMER_CLEAR;      // clear timer
      P1IES &= ~(BIT3);           // searching for rising edge next
    }
  }
}

//************************************************************************
//* ADC12 Interrupt Service Routine
//************************************************************************
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void) {
  // interrupt flag is cleared by accessing this value
  int adc_reading = ADC12MEM0;

  if (ir_state == 1) {
    // ir_on_sum += adc_reading;
    // ir_on_sum -= ir_on_value[num_readings];
    // ir_on_value[num_readings] = adc_reading;
    // num_readings++;
    // num_readings = num_readings % NUM_AVG;

    ir_on = adc_reading;

    P9OUT &= ~(IR_LED); // Turn LED off
    ir_state = 0;
  } else {
    ir_off = adc_reading;

    P9OUT |= IR_LED; // Turn LED on
    ir_state = 1;
  }
  ADC12CTL0 = ADC12CTL0 | ADC12ENC; // Enable conversion
  ADC12CTL0 = ADC12CTL0 | ADC12SC;  // Start conversion
}

/**
 * Timer A 2 Interrupt
 * Updates the LCD display with the latest IR sensor reading from the ADC
 * every time the timer goes off.
 */
#pragma vector = TIMER2_A0_VECTOR
__interrupt void Timer2_ISR(void) {

  unsigned long dist = ir_on - ir_off;

  // dist = ((5600 / dist) + 1.52) * 100;

  if (ir_on > ir_off) {
    myLCD_showSymbol(LCD_CLEAR, LCD_NEG, 0);
    myLCD_displayNumber(dist);
  } else {
    myLCD_displayNumber(dist);
    myLCD_showSymbol(LCD_UPDATE, LCD_NEG, 0);
  }
  myLCD_showSymbol(LCD_UPDATE, LCD_A4DP, 0);
}

#define SELA_MASK 0x0300
//***********************************************************************************************
void SwitchToLFXT(void)
// Switches ACLK to Low Frequency eXTernal crystal (LFXT) at 32.768kHz
//***********************************************************************************************
{
  CSCTL0_H = CSKEY_H;       // Unlock Clock Select (CS) registers
  PJSEL0 |= BIT4 | BIT5;    // Configure PJ.4 and PJ.5 as LFXIN/LFXOUT
  PJSEL1 &= ~(BIT4 | BIT5); // Now, they cannot be used for inputs/outputs
  CSCTL4 &= ~LFXTOFF; // Turn on 32.76kHz Low Frequency eXTernal (LFXT) signal
  // for lowest power mode. THis will require a little
  // more time for the LFXT to be stable before it can be
  // connected to the ACLK signal
  do // Wait for the 32.768kHz LFXT signal to stabilize
  {
    CSCTL5 &= ~LFXTOFFG; // Clear LFXT fault flag
    SFRIFG1 &= ~OFIFG;   // Clear global Oscillator Fault Interrupt FlaG (OFIFG)
  } while (SFRIFG1 & OFIFG); // Loop until stable and OFIFG goes LO
  CSCTL1 = (CSCTL1 & ~SELA_MASK) |
           SELA__LFXTCLK; // Use 32.768kHz LFXT signal as input to ACLK
  CSCTL0_H = 0;           // Lock Clock Select (CS) registers
}
//***********************************************************************************************
void SMCLK_SetTo1MHz(void) // Switches SMCLK to DCO at 1MHz
//***********************************************************************************************
{
#define DIVS_7 0x0070

  CSCTL0_H = CSKEY_H; // unlock CS registers

  CSCTL1 = (CSCTL1 & ~(DCORSEL | DCOFSEL_7)) |
           DCOFSEL_0; // The digitally controlled oscillator (DCO)
  // is an internal high frequency signal that
  // can be mapped to the SMCLK for Timer A.
  // This command ensures DCO is at 1MHz

  CSCTL2 =
      (CSCTL2 & ~SELS_3) | SELS__DCOCLK; // Route SMCLK = DCO (don’t touch ACLK)

  CSCTL3 = (CSCTL3 & ~DIVS_7) | DIVS__1; // DCO is 1MHz, so SMCLK will be 1MHz

  SFRIFG1 &= ~OFIFG; // Clear oscillator fault flags

  CSCTL0_H = 0; // lock CS registers
}
