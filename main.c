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

#define US_INDICATOR                                                           \
  BIT1 // P4.1 is used as an indicator for the ultrasonic sensor

#define IR_INDICATOR BIT0 // P4.0 is used as an LED indicator for IR sensor

#define ADC12_SHT_16 0x0200      // 16 clock cycles for sample and hold
#define ADC12_SHT_128 0x0600     // 128 clock cycles for sample and hold
#define ADC12_SHT_512 0x0A00     // 128 clock cycles for sample and hold
#define ADC12_ON 0x0010          // Used to turn ADC12 peripheral on
#define ADC12_SHT_SRC_SEL 0x0200 // Selects source for sample & hold
#define ADC12PDIV_64 0x6000      // Sets clock source prescalar to divide by 64
#define ADC12_12BIT 0x0020       // Selects 12-bits of resolution
#define ADC12_P92 0x000A         // Use input P9.2 for analog input
#define ADC12_P84 0b00111        // Use input P8.4 for analog input

#define PID_INDICATOR                                                          \
  BIT0              // P9.0 is used for seeing when the control loop is running
#define IR_LED BIT3 // P4.3 will be output for IR LED
#define IR_PHOTO_FAR ADC12_P84  // P8.4 is used as the ADC input
#define IR_PHOTO_NEAR ADC12_P92 // P9.2 is used as the ADC input

#define IR_NEAR_SETPOINT // the PID setpoint for the value of the near-range IR
                         // photodiode reading
#define IR_FAR_SETPOINT  // the PID setpoint for the value of the far-range IR
                         // photodiode reading

#define MR_EN1 BIT6 // P2.6
#define MR_EN2 BIT7 // P2.7
#define MR_PWM BIT3 // P3.3

#define ML_EN1 BIT4 // P2.4
#define ML_EN2 BIT5 // P2.5
#define ML_PWM BIT7 // P4.7

void ADC_SETUP(void);       // Used to setup ADC12 peripheral
void SwitchToLFXT(void);    // Switches ACLK to Low Frequency eXTernal crystal
                            // (LFXT) at 32.768kHz
void SMCLK_SetTo1MHz(void); // Switches SMCLK to DCO at 1MHz

void ULTRASONIC_SETUP(void); // set up pins and timers for ultrasonic sensor
void MOTOR_SETUP(void);      // sets up pins and timer for motor outputs

void set_ir_led(bool); // turn IR LED on (true) or off (false)
void read_ir(void); // pulses IR LED, and sets volatile values from ADC outputs
void set_left_motor(
    int); // set left motor to given direction and PWM, value -100 to 100
void set_right_motor(
    int); // set right motor to given direction and PWM, value -100 to 100

volatile unsigned int ultrasonic_pulse_us;
// width of pulse from ultrasonic sensor, in microseconds
// Low-pass filtering the ultrasonic sensor
#define US_FILTER_POWER 2
#define US_FILTER_SIZE (1 << US_FILTER_POWER)
volatile unsigned int ultrasonic_filter[US_FILTER_SIZE];
volatile unsigned long us_filter_sum;
volatile unsigned char us_filter_idx;

volatile unsigned int ir_near_on;  // ADC register value when the IR LED is on,
                                   // from the near-range IR photodiode
volatile unsigned int ir_near_off; // ADC register value when the IR LED is off,
                                   // from the near-range IR photodiode
volatile unsigned int ir_far_on;   // ADC register value when the IR LED is on,
                                   // from the far-range IR photodiode
volatile unsigned int ir_far_off;  // ADC register value when the IR LED is off,
                                   // from the far-range IR photodiode
volatile int ir_state;             // 0 for off, 1 for on. volatile
volatile bool ir_measure_pending;  // used for waiting until the ADC is done
                                   // measuring IR intensity

volatile int
    forward; // a value from -100 to 100, determing the overall forward speed
volatile int steer; // a value from -100 (left) to 100 (right) determining the
                    // steering intensity

#define PID_LOOP_MS 62.5f // the period of the PID control loop.
// period is based off of the configuration of Timer A 3
typedef struct {
  float k_p;              // Proportional gain
  float k_i;              // Integral gain
  float k_d;              // Derivative gain
  float prev_measurement; // Previous process variable measurement
  float integral;         // sum of the previous error values
  float setpoint;         // target value
} pid_controller_t;

pid_controller_t ir_pid = {.k_p = 120000.0f,
                           .k_i = 0.0f,
                           .k_d = 0.00f,
                           .prev_measurement = 0,
                           .integral = 0,
                           .setpoint = 0.0005f};

pid_controller_t us_pid = {.k_p = -100.0f / 4000.0f,
                           .k_i = 0.0f,
                           .k_d = 0.0f,
                           .prev_measurement = 0,
                           .integral = 0,
                           .setpoint = 580};

void main(void) {
  WDTCTL = WDTPW | WDTHOLD; // Stop WDT

  //**********************
  //* GPIO Setup
  //**********************
  PM5CTL0 = ENABLE_PINS; // Enable inputs and outputs

  //**********************
  //* LED Driver Setup
  //**********************
  P4DIR = IR_LED;    // P4.3 is output for IR LED
  set_ir_led(false); // Drive LED off

  //**********************
  //* ADC Setup
  //* Controlls the IR LED and photodiode
  // ADC interrupt
  //**********************
  ADC_SETUP();
  ADC12IER0 |= ADC12IE1; // Enable ADC interrupt

  //**********************
  //* Clock Configuration
  //**********************
  SwitchToLFXT();
  SMCLK_SetTo1MHz();

  //**********************
  //* Ultrasonic Sensor Setup
  //**********************
  ULTRASONIC_SETUP();
  int i = 0;
  while (i < US_FILTER_SIZE) {
    ultrasonic_filter[i] = 0;
    i++;
  }
  us_filter_sum = 0;
  us_filter_idx = 0;

  //**********************
  //* Motor Setup
  //**********************
  MOTOR_SETUP();

  //**********************
  //* Timer A 3 Setup
  //* This is basically the main loop, runs periodically.
  //**********************
  unsigned int clock_scalar = 0b000;
  unsigned int clock_count = 2048;
  TA3EX0 = clock_scalar; // Sets the clock scalar value for Timer_3
  TA3CCR0 = clock_count; // Sets value of Timer_3
  TA3CTL = ACLK | UP;    // Set ACLK, UP MODE for Timer_3
  TA3CCTL0 = CCIE;       // Enable interrupt for Timer_3

  forward = 0;
  steer = 0;

  // DEBUG LEDS

  P9DIR |= PID_INDICATOR;
  P9OUT &= ~(PID_INDICATOR);

  P4DIR |= US_INDICATOR;
  P4OUT &= ~(US_INDICATOR);

  P4DIR |= IR_INDICATOR;
  P4OUT &= ~(IR_INDICATOR);

  __enable_interrupt(); // Activate interrupts previously enabled
  while (1) {
    read_ir();

    // Turn LEDs on if we are too close
    if ((us_filter_sum / US_FILTER_SIZE) < us_pid.setpoint) {
      P4OUT |= US_INDICATOR;
    } else {
      P4OUT &= ~(US_INDICATOR);
    }

    if (ir_far_on - ir_far_off > 1.0f / ir_pid.setpoint) {
      P4OUT |= IR_INDICATOR;
    } else {
      P4OUT &= ~(IR_INDICATOR);
    }

    set_left_motor(forward - steer);
    set_right_motor(forward + steer);
  }
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

      us_filter_sum -= ultrasonic_filter[us_filter_idx];
      us_filter_sum += ultrasonic_pulse_us;
      ultrasonic_filter[us_filter_idx] = ultrasonic_pulse_us;
      us_filter_idx++;
      us_filter_idx %= US_FILTER_SIZE;
    }
  }
}

//************************************************************************
//* ADC12 Interrupt Service Routine
//************************************************************************
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void) {
  // interrupt flag is cleared by accessing this value
  int adc0_reading = ADC12MEM0;
  int adc1_reading = ADC12MEM1;

  if (ir_state == 1) {
    ir_near_on = adc0_reading;
    ir_far_on = adc1_reading;

    set_ir_led(false);
    ir_state = 0;

    ADC12CTL0 |= ADC12ENC; // Enable conversion
    ADC12CTL0 |= ADC12SC;  // Start conversion
  } else {
    ir_near_off = adc0_reading;
    ir_far_off = adc1_reading;

    set_ir_led(true);
    ir_state = 1;
    ir_measure_pending = false;
  }
}

void read_ir(void) {
  ir_state = 1;
  set_ir_led(true); // turn on IR LED
  ir_measure_pending = true;
  ADC12CTL0 |= ADC12ENC; // Enable conversion
  ADC12CTL0 |= ADC12SC;  // Start conversion
  while (ir_measure_pending) {
    return;
    continue;
  }
  return;
}

/**
 * Timer A 3 Interrupt
 * Reads sensor values and calculates control outputs
 */
#pragma vector = TIMER3_A0_VECTOR
__interrupt void Timer3_ISR(void) {
  // read IR sensor values
  // determine steer
  unsigned int filtered_ultrasonic_value = us_filter_sum >> US_FILTER_POWER;
  if (filtered_ultrasonic_value > 0x7FFF) {
    filtered_ultrasonic_value = 0x7FFF;
  }

  float us_error = us_pid.setpoint - filtered_ultrasonic_value;

  float us_p = us_pid.k_p * us_error;
  float us_d =
      us_pid.k_d * (filtered_ultrasonic_value - us_pid.prev_measurement);

  float us_output = us_p + us_d;
  us_pid.prev_measurement = filtered_ultrasonic_value;

  if (us_output > 100) {
    us_output = 100;
  }
  if (us_output < -100) {
    us_output = -100;
  }
  forward = us_output;

  // read ultrasonic sensor
  // determine forward speed
  float ir_value = (float)ir_far_on - (float)ir_far_off;
  ir_value = 1 / ir_value;
  float ir_error = ir_pid.setpoint - ir_value;

  float ir_p = ir_pid.k_p * ir_error;

  float ir_d = ir_pid.k_d * (ir_value - ir_pid.prev_measurement);

  float ir_output = ir_p + ir_d;

  if (ir_output > 100) {
    ir_output = 100;
  }
  if (ir_output < -100) {
    ir_output = -100;
  }
  steer = ir_output;
  ir_pid.prev_measurement = ir_value;

  P9OUT ^= PID_INDICATOR;
}

void ULTRASONIC_SETUP(void) {
  //******************************************************************
  //* Ultrasonic Sensor
  //* Trigger Output Setup
  //* Timer A 0 controls the trigger input to the ultrasonic sensor.
  //* Runs in up mode, output mode 7.
  //* Sends a pulse that is (at least) 10 us wide.
  //* Sends a pulse (at most) every 60 us, possibly longer.
  //* Outputs these pulses on P1.6.
  //******************************************************************
  TA0CCR0 = 3279;   // PWM period period, in units of scaled ACLK clock ticks.
  TA0CCR1 = 1;      // Duration for which output is HI, in units of scaled ACLK
                    // clock ticks.
  TA0CCTL1 |= 0xE0; // Output Mode 7 (reset/set)

  TA0CTL = ACLK | UP;    // Set ACLK, UP MODE
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
}

void ADC_SETUP(void) {
  ADC12CTL0 |= ADC12_SHT_128 | ADC12_ON; // Turn on, set sample & hold time
  ADC12CTL0 |= ADC12MSC;
  ADC12CTL1 |= ADC12_SHT_SRC_SEL; // Specify sample & hold clock source
  ADC12CTL2 |= ADC12_12BIT;       // 12-bit conversion results

  ADC12MCTL0 |= IR_PHOTO_NEAR; // P9.2 is analog input to ADC12MEM0 (near)
  ADC12MCTL1 |= IR_PHOTO_FAR;  // P8.4 is analog input to ADC12MEM1 (far)

  ADC12CTL1 |= 0b010;     // set to sequence-of-channels (autoscan) mode
  ADC12MCTL1 |= ADC12EOS; // MEM1 is the last conversion in the sequence
}

void MOTOR_SETUP(void) {
  //**********************
  //* Left and Right Motor PWM Output Setup
  //* Using Timer A 1
  //**********************
  TA1CTL = TASSEL__SMCLK | MC__UP; // Set SMCLK, UP MODE
                                   // 1 MHz freqency = 1 us period

  TA1CCR0 = 99;
  // Timer fires every 100 ticks, so PWM period period is 100 us (10kHz).

  TA1CCR1 = 0;      // PWM duty cycle is 0% initially.
  TA1CCTL1 |= 0xE0; // Output Mode 7 (reset/set) for PWM

  TA1CCR2 = 0;      // PWM duty cycle is 0% initially.
  TA1CCTL2 |= 0xE0; // Output Mode 7 (reset/set) for PWM

  TA1CTL |= TACLR;        // Timer starts at 0
  TA1CTL &= ~(TIMER_OFF); // Timer starts as off
  TA1CTL |= MC__UP;       // Turn on timer to enable PWM output

  // Tie P4.7 to PWM output
  P4SELC |= BIT7;
  P4DIR |= ML_PWM;

  // Tie P3.3 to PWM output
  P3SEL1 |= BIT3;
  P3DIR |= MR_PWM;

  //**********************
  //* Left and Right Motor Input Pin Setup
  //**********************
  // left motor enable pins configured as outputs
  P2DIR |= ML_EN1;
  P2DIR |= ML_EN2;

  // right motor enable pins configured as outputs
  P2DIR |= MR_EN1;
  P2DIR |= MR_EN2;

  set_left_motor(0);
  set_right_motor(0);
}

void set_ir_led(bool state) {
  if (!state) {
    P4OUT &= ~(IR_LED); // Drive LED off
  } else {
    P4OUT |= IR_LED; // Turn LED on
  }
}

void set_left_motor(int speed) {
  P2OUT |= ML_EN1;
  P2OUT &= ~(ML_EN2);
  if (speed < 0) {
    P2OUT &= ~(ML_EN1);
    P2OUT |= ML_EN2;
    speed *= -1;
  }
  TA1CCR2 = speed;
}

void set_right_motor(int speed) {
  P2OUT |= MR_EN1;
  P2OUT &= ~(MR_EN2);
  if (speed < 0) {
    P2OUT &= ~(MR_EN1);
    P2OUT |= MR_EN2;
    speed *= -1;
  }
  TA1CCR1 = speed;
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
