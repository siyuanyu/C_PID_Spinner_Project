#include <plib.h>
#include "NU32.h"

#define DESIRED_BAUDRATE_NU32 230400 // Baudrate for RS232

// Private Buffers
char NU32_RS232OutBuffer[32]; // Buffer for sprintf in serial tx

/* Perform startup routines:
 * Make NU32LED1 and NU32LED2 pins outputs (NU32USER is by default an input)
 * Initialize the serial ports - UART1 (no interrupt) and UART4 (with interrupt)
 */
void NU32_Startup() {
  // set to maximum performance and enable all interrupts
  SYSTEMConfig(SYS_FREQ, SYS_CFG_ALL);
  INTEnableSystemMultiVectoredInt();
  // disable JTAG to get A4 and A5 back
  DDPCONbits.JTAGEN = 0;

  TRISACLR = 0x0030; // Make A5 and A4 outputs (L2 and L1 on the silkscreen)
  NU32LED1 = 1; // L1 is off
  NU32LED2 = 0; // L2 is on

  // turn on UART1 without an interrupt
  U1MODEbits.BRGH = 0; // set baudrate to DESIRED_BAUDRATE_NU32
  U1BRG = ((SYS_FREQ / DESIRED_BAUDRATE_NU32) / 16) - 1;
  // 8 bit, no parity bit, and 1 stop bit (8N1 setup)
  U1MODEbits.PDSEL = 0;
  U1MODEbits.STSEL = 0;
  // configure TX & RX pins as output & input pins
  U1STAbits.UTXEN = 1;
  U1STAbits.URXEN = 1;
  // configure using RTS and CTS
  U1MODEbits.UEN = 2;
  U1MODEbits.ON = 1;
}

// Enable UART1 interrupt, so don't use NU32_ReadUART1 anymore
void NU32_EnableUART1Interrupt(void) {
  // turn off the module to change the settings
  U1MODEbits.ON = 0;

  U1MODEbits.BRGH = 0; // set baudrate to DESIRED_BAUDRATE_NU32
  U1BRG = ((SYS_FREQ / DESIRED_BAUDRATE_NU32) / 16) - 1;
  // 8 bit, no parity bit, and 1 stop bit (8N1 setup)
  U1MODEbits.PDSEL = 0;
  U1MODEbits.STSEL = 0;
  // configure TX & RX pins as output & input pins
  U1STAbits.UTXEN = 1;
  U1STAbits.URXEN = 1;
  // configure using RTS and CTS
  U1MODEbits.UEN = 2;

  // Clear the RX interrupt Flag
  IFS0bits.U1RXIF = 0;

  // Configure UART1 RX Interrupt
  // configure RX to interrupt whenever a character arrives
  U1STAbits.URXISEL = 0;
  IPC6bits.U1IP = 2;
  IPC6bits.U1IS = 0;
  IEC0bits.U1RXIE = 1;
  
  // turn on the UART
  U1MODEbits.ON = 1;
}

// Disable UART1 interrupt, so you can use NU32_ReadUART1 again
void NU32_DisableUART1Interrupt(void) {
  // turn off the module to change the settings
  U1MODEbits.ON = 0;

  // Configure UART1 RX Interrupt to off
  IEC0bits.U1RXIE = 0;

  // turn on UART1 without an interrupt
  U1MODEbits.BRGH = 0; // set baudrate to DESIRED_BAUDRATE_NU32
  U1BRG = ((SYS_FREQ / DESIRED_BAUDRATE_NU32) / 16) - 1;
  // 8 bit, no parity bit, and 1 stop bit (8N1 setup)
  U1MODEbits.PDSEL = 0;
  U1MODEbits.STSEL = 0;
  // configure TX & RX pins as output & input pins
  U1STAbits.UTXEN = 1;
  U1STAbits.URXEN = 1;
  // configure using RTS and CTS
  U1MODEbits.UEN = 2;
  U1MODEbits.ON = 1;
}

/* Read from UART1
 * block other functions until you get a '\r' or '\n'
 * send the pointer to your char array and the number of elements in the array
 */
void NU32_ReadUART1(char * message, int maxLength) {
  char data;
  int complete = 0, num_bytes = 0;
  // loop until you get a '\r' or '\n'
  while (!complete) {
    if (U1STAbits.URXDA) {
      data = U1RXREG;
      if ((data == '\n') || (data == '\r')) {
        complete = 1;
      } else {
        message[num_bytes] = data;
        num_bytes++;
        // roll over if the array is too small
        if (num_bytes >= maxLength) {
          num_bytes = 0;
        }
      }
    }
  }
  // end the string
  message[num_bytes] = '\0';
}

// Write a charater array using UART1
void NU32_WriteUART1(const char *string) {
  WriteString(UART1, string);
}

// Write a string over the serial port
void WriteString(UART_MODULE id, const char *string) {
  while (*string != '\0') {
    while (!UARTTransmitterIsReady(id));
    UARTSendDataByte(id, (char) *string);
    string++;
    while (!UARTTransmissionHasCompleted(id));
  }
}

// Put a character over the serial port, called by WriteString
void PutCharacter(UART_MODULE id, const char character) {
  while (!UARTTransmitterIsReady(id));
  UARTSendDataByte(id, character);
  while (!UARTTransmissionHasCompleted(id));
}
