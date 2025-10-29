#include <msp430.h>

#define STOP_WATCHDOG 0x5A80 // Stop the watchdog timer
#define ENABLE_PINS 0xFFFE   // Required to use inputs and outputs

#define GREEN_LED 0x0080 // P9.7 is the green LED

#define BUTTON1 0x02 // P1.1 is button one
#define BUTTON2 0x04 // P1.2 is button two

#define TIMER_OFF 0x30 // CLEAR these bits to stop Timer_A

#define M1EN1 BIT7 // motor pin EN1 is connected to P1.7
#define M1EN2 BIT5 // motor pin EN2 is connected to P2.5
#define M1PWM BIT6 // motor pin PWM is connected to P1.6

void SMCLK_SetTo1MHz(void); // Switches SMCLK to DCO at 1MHz
void SwitchToLFXT(void);    // Switches ACLK to Low Frequency eXTernal crystal
                            // (LFXT) at 32.768kHz

volatile int dutycycle_direction = 1; // +1 for increasing, -1 for decreasing
#define CLOCK_SCALAR 8
#define RAMP_TIME 10
// seconds
#define RESOLUTION 50
// the number of steps for the pwm duty cycle to go from 0% to 100%
// over the course of 10 seconds
// note that since the CCR0 controlling the PWM period is 100 (microseconds),
// the highest resolution possible is 100.
// also be careful of off-by-one errors.

main() {
  WDTCTL = STOP_WATCHDOG; // Stop the watchdog timer
  PM5CTL0 = ENABLE_PINS;  // Required to use inputs and outputs

  P9DIR |= GREEN_LED;    // P9.7 will be output for green LED
  P9OUT &= ~(GREEN_LED); // Drive LED off

  //**********************
  //* Motor Driver Setup
  //**********************
  P1DIR |= M1EN1;
  P2DIR |= M1EN2;

  P1OUT &= ~M1EN1;
  P2OUT |= M1EN2;

  //**********************
  //* PWM Output Setup
  //* Using Timer A 0
  //**********************
  SMCLK_SetTo1MHz();               // 1 MHz freqency = 1 us period
  TA0CTL = TASSEL__SMCLK | MC__UP; // Set SMCLK, UP MODE

  TA0CCR0 = 99;
  // Timer fires every 100 ticks, so PWM period period is 100 us (10kHz).
  TA0CCR1 = 0;      // PWM duty cycle is 0% initially.
  TA0CCTL1 |= 0xE0; // Output Mode 7 (reset/set) for PWM

  TA0CTL |= TACLR;        // Timer starts at 0
  TA0CTL &= ~(TIMER_OFF); // Timer starts as off

  // Tie P1.6 to PWM output
  P1SELC |= BIT6; // Set bits P1SEL1.6 and P1SEL0.1 simultaneously
  P1DIR |= BIT6;  // P1.6 is an output

  //***********************************************************************
  //* Incrementor Setup
  //* Using Timer A 1
  //* Timer goes off at certain intervals to periodically
  //* increment the PWM signal's duty cycle over the course of 10 seconds.
  //***********************************************************************
  SwitchToLFXT();
  TA1CTL = TASSEL__ACLK | MC__UP; // Set ACLK , UP mode

  TA1EX0 |= CLOCK_SCALAR - 1; // prescalar of 8 (we subtract 1 because of how
                              // the register values are defined)

  /*
  For some reason, the formula is not working, but TA1CCR0 should equal:
  (RAMP_TIME/RESOLUTION) * 1000000 / (CLOCK_SCALAR * 30.6)
  */
  TA1CCR0 = 817;
  // Timer goes off RESOLUTION times in RAMP_TIME seconds, accounting for a
  // clock scalar of CLOCK_SCALAR.

  TA1CTL |= TACLR;        // Timer starts at 0
  TA1CTL &= ~(TIMER_OFF); // Timer starts as off
  TA1CCTL0 = CCIE;        // Enable interrupt for Timer_1

  //*************************************************
  //* Button Setup
  //* Button 1 (P1.1) is used to turn the motor on.
  //* Button 2 (P1.2) is used to turn the motor off.
  //* These buttons are only enabled after the motor is completely spun up or
  // down.
  //*************************************************
  P1OUT |= BUTTON1 | BUTTON2; // P1.1 and P1.2 will be an input
  P1REN |= BUTTON1 | BUTTON2; // with a pull-up resistor

  P1IE |= BUTTON1; // Enable interrupt for button 1 (but not button 2)
  P1IES &= ~(BUTTON1 | BUTTON2); // look for rising edges
  P1IFG = 0x00;                  // Ensure no interrupts are pending

  __enable_interrupt();

  while (1)
    ;
}

//***********************************************************************
//* Port 1 Interrupt Service Routine
//* Buttons 1 and 2
//***********************************************************************
#pragma vector = PORT1_VECTOR
__interrupt void Port_1(void) {
  int buttonInterupt = P1IV;
  if (buttonInterupt == 0x4) { // check that P1.1 is the source of the interrupt
    // toggle the direction the motor spins
    P1OUT ^= M1EN1;
    P2OUT ^= M1EN2;
    // set motor speed to increase
    dutycycle_direction = 1;
    // make sure motor speed is currently at 0%
    TA0CCR1 = 0;

    TA0CTL |= MC__UP; // enable PWM output

    TA1CTL |= TACLR;  // clear pwm increment timer
    TA1CTL |= MC__UP; // start pwm increment timer

    P1IE &= ~(BUTTON1); // disable interrupt for button 1
  } else if (buttonInterupt ==
             0x6) { // check that P1.2 was the source of the interrupt
    // set motor speed to decrease
    dutycycle_direction = -1;
    // make sure motor speed is currently 100%
    TA0CCR1 = 100;

    TA0CTL |= MC__UP; // enable PWM output

    TA1CTL |= TACLR;  // clear pwm increment timer
    TA1CTL |= MC__UP; // start pwm increment timer

    P1IE &= ~(BUTTON2); // disable interrupt for button 2
  }
}

//************************************************************************
// Timer A 1 Interrupt Service Routine
// Updating PWM duty cycle
//************************************************************************
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_ISR(void) {
  P9OUT ^= GREEN_LED;

  TA0CCR1 += dutycycle_direction * 100 / RESOLUTION;
  if (TA0CCR1 <= 0 || TA0CCR1 >= 100) {
    TA1CTL &= ~(TIMER_OFF); // motor has reached final speed.  stop the timer
    // enable a button
    if (dutycycle_direction == -1) {
      P1IFG =
          0; // clear all interrupts before re-enabling button because bounce
      P1IE |= BUTTON1;
    } else if (dutycycle_direction == 1) {
      P1IFG =
          0; // clear all interrupts before re-enabling button because bounce
      P1IE |= BUTTON2;
    }
  }
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
