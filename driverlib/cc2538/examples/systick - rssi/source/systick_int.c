/******************************************************************************
*  Filename:       systick_int.c
*  Revised:        $Date: 2013-04-17 10:40:35 +0200 (Wed, 17 Apr 2013) $
*  Revision:       $Revision: 9797 $
*
*  Description:    Example demonstrating how to configure the systick 
*                  interrupt.
*
*  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include "hw_memmap.h"
#include "gpio.h"
#include "hw_ioc.h"
#include "ioc.h"
#include "hw_ints.h"
#include "interrupt.h"
#include "sys_ctrl.h"
#include "systick.h"
#include "uartstdio.h"

#include "hw_sys_ctrl.h"
#include "hw_rfcore_ffsm.h"
#include "hw_rfcore_sfr.h"
#include "hw_rfcore_xreg.h"

// Define for UART0
#define EXAMPLE_PIN_UART_RXD            GPIO_PIN_0 
#define EXAMPLE_PIN_UART_TXD            GPIO_PIN_1 
#define EXAMPLE_GPIO_UART_BASE          GPIO_A_BASE

// RF OPCODES
#define CC2538_RF_CSP_OP_ISRXON                0xE3
#define CC2538_RF_CSP_OP_ISTXON                0xE9
#define CC2538_RF_CSP_OP_ISTXONCCA             0xEA
#define CC2538_RF_CSP_OP_ISRFOFF               0xEF
#define CC2538_RF_CSP_OP_ISFLUSHRX             0xED
#define CC2538_RF_CSP_OP_ISFLUSHTX             0xEE

// Define RF
#define CC2538_RF_CHANNEL               11
#define CC2538_RF_CHANNEL_MIN           11
#define CC2538_RF_CHANNEL_SPACING       5
#define CC2538_RF_MAX_PACKET_LEN        127
#define RSSI_OFFSET    73
#define RFCORE_XREG_RSSI_RSSI_VAL 0x000000FF /**< RSSI estimate */
#define ANA_REGS_IVCTRL       0x400D6004 // Controls bias currents. User guide page 703

// Define trigger
#define TRIGGER_GPIO_BASE               GPIO_D_BASE
#define TRIGGER_GPIO_PIN                GPIO_PIN_3
#define TRIGGER_INT_GPIO                INT_GPIOD

// Define debug LED/sync LED at PC4-PC7(RED-GREEN-BLUE-YELLOW) 
#define LEDS_RED GPIO_PIN_4
#define LEDS_GREEN GPIO_PIN_5
#define LEDS_BLUE GPIO_PIN_6
#define LEDS_YELLOW GPIO_PIN_7
#define LEDS_PORT_BASE GPIO_C_BASE

// Define debug pin sync at PA2/DIO4
#define DEBUG_PORT_BASE GPIO_A_BASE
#define DEBUG_PIN GPIO_PIN_2
/*#define DEBUG_LED() do { \
  static uint8_t dio_state = 0; \
  dio_state = !dio_state; \
  if(dio_state) { \
    GPIOPinWrite(DEBUG_PORT_BASE, LEDS_RED, LEDS_RED); \
  } else { \
    GPIOPinWrite(DEBUG_PORT_BASE, LEDS_RED, 0); \
  } \
} while(0);*/

// Counter to count the number of interrupts that have been called.
volatile uint32_t channel = CC2538_RF_CHANNEL;
//volatile int16_t rssi = 0;

// This function sets up UART0 to be used for a console to display information
void
InitConsole(void)
{
    //
    // Map UART signals to the correct GPIO pins and configure them as
    // hardware controlled.
    //
    IOCPinConfigPeriphOutput(EXAMPLE_GPIO_UART_BASE, EXAMPLE_PIN_UART_TXD, 
                             IOC_MUX_OUT_SEL_UART0_TXD);
    GPIOPinTypeUARTOutput(EXAMPLE_GPIO_UART_BASE, EXAMPLE_PIN_UART_TXD);
    
    IOCPinConfigPeriphInput(EXAMPLE_GPIO_UART_BASE, EXAMPLE_PIN_UART_RXD, 
                            IOC_UARTRXD_UART0);
    GPIOPinTypeUARTInput(EXAMPLE_GPIO_UART_BASE, EXAMPLE_PIN_UART_RXD);
     
    //
    // Initialize the UART (UART0) for console I/O.
    //
    UARTStdioInit(0);
}

// Initial RF for read out rssi.
void
set_channel(uint8_t channel)
{
  HWREG(RFCORE_XREG_FREQCTRL) = CC2538_RF_CHANNEL_MIN +
    (channel - CC2538_RF_CHANNEL_MIN) * CC2538_RF_CHANNEL_SPACING;
}
void
RF_init(void)
{
/* Enable clock for the RF Core while Running, in Sleep and Deep Sleep */
  HWREG(SYS_CTRL_RCGCRFC) = 1; 
  HWREG(SYS_CTRL_SCGCRFC) = 1; 
  HWREG(SYS_CTRL_DCGCRFC) = 1; 

  /*
   * Changes from default values
   * See User Guide, section "Register Settings Update"
   */
  HWREG(RFCORE_XREG_TXFILTCFG) = 0x09;    /** TX anti-aliasing filter bandwidth */
  HWREG(RFCORE_XREG_AGCCTRL1) = 0x15;     /** AGC target value */
  HWREG(ANA_REGS_IVCTRL) = 0x0B;          /** Bias currents */

  /*
   * Defaults:
   * Auto CRC; Append RSSI, CRC-OK and Corr. Val.; CRC calculation;
   * RX and TX modes with FIFOs
   */
  HWREG(RFCORE_XREG_FRMCTRL0) = RFCORE_XREG_FRMCTRL0_AUTOCRC;

#if CC2538_RF_AUTOACK
  HWREG(RFCORE_XREG_FRMCTRL0) |= RFCORE_XREG_FRMCTRL0_AUTOACK;
#endif

  /* Disable source address matching and autopend */
  HWREG(RFCORE_XREG_SRCMATCH) = 0;

  /* MAX FIFOP threshold */
  HWREG(RFCORE_XREG_FIFOPCTRL) = CC2538_RF_MAX_PACKET_LEN;

  /* Set TX Power */
//  HWREG(RFCORE_XREG_TXPOWER) = CC2538_RF_TX_POWER;

  set_channel(channel);

  /* Acknowledge all RF Error interrupts */
  HWREG(RFCORE_XREG_RFERRM) = RFCORE_XREG_RFERRM_RFERRM_M;
  //nvic_interrupt_enable(NVIC_INT_RF_ERR);
}

int8_t get_rssi(void)
{
  int8_t rssi;
  /* If we are off, turn on first */
  if((HWREG(RFCORE_XREG_FSMSTAT0) & RFCORE_XREG_FSMSTAT0_FSM_FFCTRL_STATE_M) == 0) {
    HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISFLUSHRX;
    HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISFLUSHRX;
    HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISRXON;
  }

  /* Wait on RSSI_VALID */
  while((HWREG(RFCORE_XREG_RSSISTAT) & RFCORE_XREG_RSSISTAT_RSSI_VALID) == 0);

  rssi = (int8_t)(HWREG(RFCORE_XREG_RSSI) & RFCORE_XREG_RSSI_RSSI_VAL) - RSSI_OFFSET;
  rssi = rssi +100;
  if(rssi <0) rssi =0;
  return rssi;
}

// Debug LED/sync
void
DebugInit(void)
{
    // Set GPIO pins as output low
    GPIOPinTypeGPIOOutput(DEBUG_PORT_BASE, DEBUG_PIN);
    GPIOPinWrite(DEBUG_PORT_BASE, DEBUG_PIN, 0);
}
void
LEDS_init(void)
{
  // Set GPIO pins as output low, LEDs off
  GPIOPinTypeGPIOOutput(LEDS_PORT_BASE, LEDS_RED|LEDS_GREEN|LEDS_BLUE|LEDS_YELLOW);
  GPIOPinWrite(LEDS_PORT_BASE, LEDS_RED|LEDS_GREEN|LEDS_BLUE|LEDS_YELLOW, 0);
}

// The interrupt handler for the for GPIO D interrupt.
void
GPIODIntHandler(void)
{
  GPIOPinWrite(LEDS_PORT_BASE, LEDS_RED, LEDS_RED);
  GPIOPinWrite(DEBUG_PORT_BASE, DEBUG_PIN, 0);
  
  // Simple debounce function wait for button de-press
//  while(!GPIOPinRead(TRIGGER_GPIO_BASE, TRIGGER_GPIO_PIN))  {}
  
  // Acknowledge the GPIO  - Pin n interrupt by clearing the interrupt flag.
  GPIOPinIntClear(TRIGGER_GPIO_BASE, TRIGGER_GPIO_PIN);
  
  //DEBUG_LED();
  UARTprintf("%d %d\n", channel, get_rssi());
  
  GPIOPinWrite(LEDS_PORT_BASE, LEDS_RED, 0);
  GPIOPinWrite(DEBUG_PORT_BASE, DEBUG_PIN, DEBUG_PIN);
}

// Trigger at pin PD3
void
TriggerInit(void) 
{
  // Set Port D Pin 3 as an input, push-pull mode.
  GPIOPinTypeGPIOInput(TRIGGER_GPIO_BASE, TRIGGER_GPIO_PIN);
  IOCPadConfigSet(TRIGGER_GPIO_BASE, TRIGGER_GPIO_PIN, IOC_OVERRIDE_PUE);

  // Set the type of interrupt for PD3, falling edge.
  GPIOIntTypeSet(TRIGGER_GPIO_BASE, TRIGGER_GPIO_PIN, GPIO_FALLING_EDGE);
 
  // Enable the GPIO interrupt.
  GPIOPinIntEnable(TRIGGER_GPIO_BASE, TRIGGER_GPIO_PIN);
  
  // Enable the GPIOD interrupt on the processor (NVIC).
  IntEnable(TRIGGER_INT_GPIO);
  
}

// Configure the SysTick and SysTick interrupt with a period of 1 second.
int
main(void)
{
    // Set the clocking to run directly from the external crystal/oscillator.
    // (no ext 32k osc, no internal osc)
    SysCtrlClockSet(false, false, SYS_CTRL_SYSDIV_32MHZ);

    // Set IO clock to the same as system clock
    SysCtrlIOClockSet(SYS_CTRL_SYSDIV_32MHZ);
    
    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for Systick operation.
    InitConsole();

    // Display the setup on the console.
    UARTprintf("SBAN RSSI live stream\n");

    // Set up RF
    RF_init();
    
    //Debug
    DebugInit();
    
    //LEDs init
    LEDS_init();
    
    // Use small interrupt map
    IntAltMapEnable();

    // Init trigger function
    TriggerInit();
    
    // Enable interrupts to the processor.
    IntMasterEnable();
    
    // Loop forever while the SysTick runs.
    while(1)
    {
      //UARTprintf("RSSI %d %d\n", channel, get_rssi());
    }
}
