#include <msp430.h>


#define VL53L4CXADDR 0x52         // address of sensor is 52hex



int VL53L4CX_setup() {
    // Setup I2C
    UCB0CTLW0 = UCSWRST;                // put eUSCI_B in reset state

    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK;      // I2C mode | master mode | SMCLK
    UCB0BRW = 0x02;                       // baud rate = SMCLK / 1
    UCB0CTLW1 |= UCASTP_2;              // automatic STOP assertion
    UCB0TBCNT = 0x07;                   // TX 2 bytes of data

    P1SEL0 |= (BIT6 | BIT7);               // configure I2C pins (device specific)
    P1SEL1 &= ~(BIT6 | BIT7);


    UCB0CTLW0 &= ~UCSWRST;               // eUSCI_B in operational state
    //UCB0IE |= UCTXIE;                   // enable TX-interrupt
    return 0;
}

// Test I2C communication with sensor
int verify_VL53L4CX() {    
    // Disable I2C interupts
    int tmp_interupt = UCB0IE;
    UCB0IE = 0;

    // Send I2C command to read module ID
    int reg = 0x010F;              // Read register

    int read_value = 0;

    UCB0I2CSA = VL53L4CXADDR;


    UCB0CTLW0 |= UCTR | UCTXSTT;    // Set transmit and send start
    //UCB0CTLW0 |= UCTXSTT;

    while (!(UCB0IFG & UCTXIFG));   // Wait for tx buf ready
    UCB0TXBUF = reg >> 8;           // Put data in tx buf
    while (!(UCB0IFG & UCTXIFG));   // Wait for tx buf ready
    UCB0TXBUF = reg;                // Put data in tx buf
    

    while (!(UCTXIFG0 & UCB0IFG));  // Wait for transmission


    UCB0CTLW0 &= ~UCTR;             // Set to recieve mode
    UCB0CTLW0 |= UCTXSTT;           // Send start

    while (!(UCTXIFG0 & UCB0IFG));  // Wait start to transmit


    while(!(UCRXIFG0 & UCB0IFG));
    read_value = UCB0RXBUF;

    UCB0CTLW0 |= UCTXSTP;           // Send stop


    // Reset I2C interupts
    UCB0IE = tmp_interupt;
    
    if (read_value == 0xEB) {
        return 0;
    }
    return 1;
}



