//*****************************************************************************
//  MSP-FET430P140 Demo - Bootstrap Loader (BSL) Replicator
//
//  Description: This program implements the BSL protocol used to download
//  a MSP430 program to a target MSP430 device. The program is stored locally
//  in Flash in array CodeArray[]. Pressing SW1 downloads the program to the
//  target device and pressing SW2 starts program execution on the target
//  device.
//  ACLK = LFXT1 = 32768Hz,  MCLK = SMCLK = BRCLK = DCOCLK = 1048576Hz
//  Baud rate divider with 1048576Hz = 1048576/9600 ~109.23 (06Dh)
//  //* An external 32kHz watch crystal btw XIN & XOUT is required for ACLK *//
//
//
//                    MSP430F149
//                -----------------
//            /|\|              XIN|-
//             | |                 | 32768Hz
//              --|RST          XOUT|-
//               |                 |
//               |       P3.4/UTXD0|------------> (BOOTST pin 3)
//               |                 | 9600 - 8N1
//               |       P3.5/URXD0|<------------ (BOOTST pin 1)
//               |                 |
//               |             P1.5|--> RST/NMI   (BOOTST pin 4)
//               |             P1.6|--> TEST      (BOOTST pin 7)
//  BSL PATCH -->|P1.1         P1.7|--> TCK       (BOOTST pin 2)
//    BSL SEQ -->|P1.2             |
//        SW1 -->|P1.3         P1.0|--> LED (DCO Calibration/Timer)
//        SW2 -->|P1.4             |
//               |             P5.4|--> Status LED 1 (BSL ACK/BSL RX MSG DONE)
//               |             P5.3|--> Status LED 2 (BSL TX MSG)
//               |             P5.2|--> Status LED 3 (BSL NACK/FAILURE)
//               |             P5.1|--> Status LED 4 (SUCCESS)
//
//
//    NOTE: 1. Grounding pin P1.2 selects BSL entry sequence for MSP430
//             devices having shared JTAG pins.
//          2. Connecting pin P1.2 to Vcc selects BSL entry sequence for
//             MSP430 devices having dedicated JTAG pins.
//
//
//  G. Morton
//  Texas Instruments Inc.
//  April 2005
//  Built with IAR Embedded Workbench Version: 3.21A
//*****************************************************************************

#include  <msp430x14x.h>


//
// Events
//
typedef enum
{
  EVENT_SW1     = 0x0001,                   // Pushbutton 1 event
  EVENT_SW2     = 0x0002                    // Pushbutton 2 event
} Event;


//
// BSL Message Structure
//
typedef struct _bslMsg
{
  unsigned char reply;                      // BSL reply type - ACK, MSG, None
  unsigned char hdr[8];                     // Header
  unsigned char data[256];                  // Data
  unsigned char chksum[2];                  // Checksum
} BslMsg;


//
// ERRORS
//
#define TIMEOUT_ERROR   -101                // Timeout error
#define NACK_ERROR      -102                // NACK error

//
// External, momentary pushbutton switches
//
#define SW1             0x08                // P1.3
#define SW2             0x10                // P1.4

//
// DCO frequency
//
#define DCO_FREQ       1048576              // 1048576 MHz

//
// Timer Values
//
#define TIMERLEDPORT    P1OUT               // Timer Status LED port
#define TIMER_LED       0x01                // Timer Status LED pin
#define DELAY_10_MSEC   10486               // ~10 msec delay
#define DELAY_20_MSEC   20972               // ~20 msec delay
#define BSL_DELAY_CNT   1259                // ~1.2 msec
#define CHAR_DELAY      1259                // ~1.2 msec
#define SYNC_DELAY      50000               // Wait for SYNC ACK reply delay
#define ACK_DELAY       DELAY_10_MSEC       // Wait for ACK/NACK reply delay
#define MSG_DELAY       DELAY_10_MSEC       // Wait for msg reply delay
#define SW_DELAY        DELAY_10_MSEC       // Switch debounce delay
#define TIMEOUT_LOOP    100                 // Number of delay loops

//
// BSL Connections
//
#define BSLPATCHIN      P1IN                // BSL patch option port
#define BSL_PATCH_PIN   0x02                // BSL patch option pin
#define BSLSEQIN        P1IN                // BSL entry sequence option port
#define BSL_SEQ_PIN     0x04                // BSL entry sequence option pin
#define BSLOUT          P1OUT               // BSL port output
#define BSLDIR          P1DIR               // BSL port direction
#define RESET_NMI_PORT  BSLOUT              // RST/NMI port output
#define TEST_TCK_PORT   BSLOUT              // TEST/TCK port output
#define RESET_NMI_PIN   0x20                // RST/NMI pin (P1.5)
#define TEST_PIN        0x40                // TEST pin    (P1.6)
#define TCK_PIN         0x80                // TCK pin     (P1.7)
#define BSL_PINS        (RESET_NMI_PIN+TEST_PIN+TCK_PIN)

//
// Status LEDs
//
#define DCO_STATUS_LED  0x01
#define LED1            0x10
#define LED2            0x08
#define LED3            0x04
#define LED4            0x02
#define STATUS_LEDS     (LED1+LED2+LED3+LED4)
#define LEDPORTOUT      P5OUT
#define LEDPORTDIR      P5DIR
#define DCOLEDPORTOUT   P1OUT
#define DCOLEDPORTDIR   P1DIR

//
// State Flags
//
unsigned char flags = 0;
#define TIMER_FLAG      0x01                // Timer flag
#define ACK_FLAG        0x02                // ACK flag
#define NACK_FLAG       0x04                // NACK flag
#define REPLY_FLAG      0x08                // REPLY flag
#define DONE_FLAG       0x10                // DONE flag
#define DOWNLOAD_FLAG   0x20                // DOWNLOAD flag
#define PATCH_FLAG      0x80                // PATCH flag
#define BSL_FLAGS       (ACK_FLAG + NACK_FLAG + REPLY_FLAG + DONE_FLAG)

//
// BSL Characters
//
#define SYNC_CHAR       0x80                // SYNC char to BSL
#define ACK_CHAR        0x90                // ACK char from BSL
#define NACK_CHAR       0xA0                // NACK char from BSL

//
// BSL Message Reply Types
//
#define BSL_ACK_REPLY   0x01                // Wait for ACK from BSL
#define BSL_MSG_REPLY   0x02                // Wait for msg from BSL

//
// Block size for downloads to target BSL
//
#define TX_BLK_SIZE     0x10                // BSL Tx data block size


//
// Function prototypes
//
void configUart0(void);
void configTimer_A(void);
void setDCO(unsigned long freq);
void delay(unsigned int val);
void timer(unsigned int val);
void cancelTimer(void);
int  strncompare(const char* str1, const char* str2, unsigned char len);


//
// BSL Function Prototypes
//
void bslEntrySeq(void);
void bslChecksum(BslMsg* pMsg);
int  bslTxMsg(BslMsg* pMsg);
int  bslTxSync(void);
void bslSendChar(unsigned char val);
int  bslWaitForReply(unsigned char type);
void bslStateReset(void);
int  bslDownloadCode(unsigned int* pCodeArray, unsigned char patchFlag);
int  bslDownloadProgram(void);
int  bslInstallPatch(void);


//
// BSL Message Function Prototypes
//
void bslMsgRxDataBlk(BslMsg* pMsg, int addr, int len, unsigned int* pData);
void bslMsgRxPassword(BslMsg* pMsg);
void bslMsgEraseSeg(BslMsg* pMsg, int addr, int len);
void bslMsgErase(BslMsg* pMsg);
void bslMsgEraseChk(BslMsg* pMsg, int addr, int len);
void bslMsgBaudRate(BslMsg* pMsg, int dco, int bcs, int baudRate);
void bslMsgLoadPC(BslMsg* pMsg, int addr);
void bslMsgTxDataBlk(BslMsg* pMsg, int addr, int bytes);
void bslMsgVer(BslMsg* pMsg);


//
// Global Variables
//
extern unsigned int PatchArray[];           // Holds the BSL patch for
                                            // version 1.10

extern unsigned int CodeArray[];            // Holds the target program code

unsigned char PatchFlag = 0;                // Patch flag

unsigned int eventFlag = 0;                 // Event Flag

unsigned int StartAddr = 0;                 // Start Address

BslMsg bslMsg;                              // BSL message

char BslRxMsg[256];                         // Stores message from target BSL
unsigned char bslRxIndex = 0;               // BSL Rx msg array index
unsigned char bslReplyLen = 0;              // Length of msg from target BSL

//
// BSL Password - initialized to default password for erased device
//
unsigned char BslPassword[32] =
{
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};


//
//  Low-level System Initialization - called prior to main()
//
int __low_level_init(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  return (1);                               // Force initialization of RAM
  //  return (0);                           // Skip initialization of RAM
}


//
// Set DCO Frequency
//
void setDCO(unsigned long freq)
{
  unsigned char old_BCSCTL1;
  unsigned int old_TACCTL2;
  unsigned int old_TACTL;
  unsigned int clkCnt;
  unsigned int numDcoClks;
  unsigned int prevCnt = 0;

  // PUC value for DCOx = 3
  // PUC value for RSELx = 4

  old_BCSCTL1 = BCSCTL1;                    // Save current BCSCTL1 setting
  old_TACCTL2 = TACCTL2;                    // Save current TACCTL2 setting
  old_TACTL   = TACTL;                      // Save current TACTL setting


  // Basic Clock System Control Register 1
  BCSCTL1 |= DIVA_3;                        // ACLK = LFXT1CLK/8 = 4096 Hz

  numDcoClks = freq/4096;                   // Number of DCO clocks in one
                                            // ACLK/8 period

  // Timer_A Capture/Compare Control Register
  TACCTL2 = CM_1 + CCIS_1 + CAP;            // Capture on rising Edge
                                            // Capture input is CCI2B = ACLK
                                            // Async capture
                                            // Capture mode
                                            // Output mode is OUT bit
                                            // Interrupt disabled
                                            // OUT bit is 0
                                            // Clear capture overflow bit (COV)
                                            // Clear interrupt flag (CCIFG)

  // Timer A Control Register
  TACTL = TASSEL_2 + MC_2 + TACLR;          // Clk src is SMCLK
                                            // Input clock divider is 1
                                            // Continuous mode
                                            // Reset
                                            // Interrupt is disabled
                                            // Clear interrupt flag (TAIFG)

  while(1)
  {
    while( !(TACCTL2 & CCIFG) );            // Wait for capture event

    TACCTL2 &= ~CCIFG;                      // Capture occured, clear flag

    clkCnt = TACCR2 - prevCnt;              // Num of clks since last capture

    prevCnt = TACCR2;                       // Save current clock count

    if( (numDcoClks <= (clkCnt + 2)) && (numDcoClks >= (clkCnt - 2))  )
    {
      DCOLEDPORTOUT |= DCO_STATUS_LED;      // Set DCO Status LED
      break;
    }
    else if( clkCnt > numDcoClks )          // DCO is too fast, slow it down
    {
      DCOCTL--;

      if( DCOCTL == 0xFF )
      {
        if( BCSCTL1 & 0x07 )
        {
          BCSCTL1--;                        // DCO role under?, dec RSEL
        }
        else
        {
          break;                            // Error condition, break loop
        }
      }
    }
    else                                    // DCO is too slow, speed it up
    {
      DCOCTL++;

      if( DCOCTL == 0x00 )
      {
        if( (BCSCTL1 & 0x07) != 0x07 )
        {
          BCSCTL1++;                        // DCO role over? higher RSEL
        }
        else
        {
          break;                            // Error condition, break loop
        }
      }
    }
  }

  // Stop Timer_A
  TACTL = 0;
  TACCTL2 = 0;

  // Restore original values
  BCSCTL1 = old_BCSCTL1;
  TACCTL2 = old_TACCTL2;
  TACTL = old_TACTL;
}


//
//  System Initialization
//
void sysInit(void)
{
  BSLOUT &= ~BSL_PINS;                      // Clear BSL outputs
  BSLDIR |= BSL_PINS;                       // Set BSL outputs

  DCOLEDPORTOUT &= ~DCO_STATUS_LED;         // Clear DCO Status LED
  DCOLEDPORTDIR |= DCO_STATUS_LED;          // Set DCO Status LED output

  LEDPORTOUT &= ~STATUS_LEDS;               // Clear Status LED outputs
  LEDPORTDIR |= STATUS_LEDS;                // Set Status LED outputs

  P1IES |= (SW1+SW2);                       // Set P1.4,3 for 1->0 interrupt
  P1IFG  = 0x00;                            // Clear P1 interrupts
  P1IE  |= (SW1+SW2);                       // Enable P1.4,3 interrupts

  setDCO(DCO_FREQ);                         // Set DCO frequency

  configUart0();                            // Configure UART0

  configTimer_A();                          // Configure Timer_A
}


//
// Configure USART0 for UART mode
//
void configUart0(void)
{
  P3SEL |= 0x30;                            // P3.4,5 = UTXD0/URXD0
  ME1 |= UTXE0 + URXE0;                     // Enable USART0 TXD/RXD
  U0CTL |= (CHAR + PENA + PEV + SWRST);     // 8-bit char, even parity, reset
  U0TCTL |= SSEL1;                          // BRCLK = SMCLK
  U0BR0 = 0x6d;                             // 9600 from 1MHz
  U0BR1 = 0x00;                             //
  U0MCTL = 0x03;                            // Modulation
  U0CTL &= ~SWRST;                          // Initialize USART state machine
  IE1 |= URXIE0;                            // Enable USART0 RX interrupt
}


//
// Configure Timer_A
//
void configTimer_A(void)
{
  TACCTL1 = 0;
  TACCTL2 = 0;
  TACTL = TASSEL_2 + MC_2 + TACLR;          // SMCLK, continuous mode, clear
}


//
// Main Program Loop
//
void main(void)
{
  int retVal;

  // Initialize System
  sysInit();

  // Event Loop
  for(;;)
  {
    if( eventFlag == 0 )                    // No events to handle?
    {
      // Enter low-power mode 0 with interrupts enabled
      _BIS_SR(LPM0_bits + GIE);
    }

    if( eventFlag & EVENT_SW1 )
    {
      eventFlag &= ~EVENT_SW1;              // Clear event flag

      LEDPORTOUT &= ~(LED3+LED4);           // Ensure LED3 and LED4 are off

      retVal = 0;                           // Clear return value variable

      bslEntrySeq();                        // BSL entry sequence

      if( BSLPATCHIN & BSL_PATCH_PIN )      // Install patch option selected?
      {
        flags |= PATCH_FLAG;                // Set patch flag
        retVal = bslInstallPatch();         // Install BSL patch
      }
      else
      {
        flags &= ~PATCH_FLAG;               // Clear patch flag
      }

      if( retVal == 0 )
      {
        retVal = bslDownloadProgram();      // Download program to target BSL
      }

      if( retVal < 0 )                      // Check result
      {
        LEDPORTOUT |= LED3;                 // Set LED3 to indicate an error
      }
      else
      {
        LEDPORTOUT |= LED4;                 // Set LED4 to indicate success
      }
    }

    if( eventFlag & EVENT_SW2 )
    {
      eventFlag &= ~EVENT_SW2;              // Clear event flag

      LEDPORTOUT &= ~(LED3+LED4);           // Ensure LED3 and LED4 are off

      bslMsgLoadPC(&bslMsg, StartAddr);     // Jump to start address msg

      if( bslTxMsg(&bslMsg) < 0 )           // Send msg to BSL
      {
        LEDPORTOUT |= LED3;                 // Set LED3 to indicate an error
      }
      else
      {
        LEDPORTOUT |= LED4;                 // Set LED4 to indicate success
      }
    }
  }
}


//
// Port 1 Interrupt Service Routine
//
#pragma vector=PORT1_VECTOR
__interrupt void port1_ISR(void)
{
  P1IFG = 0;                                // Clear P1 interrupt flags
  if( !(TACCTL2 & CCIE) )                   // Is timer CCR2 NOT active?
  {
    TACCR2 = TAR + SW_DELAY;                // Set timer delay
    TACCTL2 &= ~CCIFG;                      // Clear TACCR2 interrupt flag
    TACCTL2 |= CCIE;                        // Enable TACCR2 interrupt
  }
}


//
// Timer_A3 Interrupt Vector (TAIV) Handler
//
#pragma vector=TIMERA1_VECTOR
__interrupt void Timer_A(void)
{
  unsigned char pin;
  static unsigned char prevPin;

  switch( TAIV )
  {
    case  2:                                // TACCR1 CCIFG
      TACCTL1 &= ~CCIE;                     // Disable TACCR1 interrupt
      flags |= TIMER_FLAG;                  // Set Timer flag
      LPM0_EXIT;                            // Exit LPM0
      TIMERLEDPORT &= ~TIMER_LED;           // Clear LED
      break;
    case  4:                                // TACCR2 CCIFG
      TACCTL2 &= ~CCIE;                     // Disable TACCR2 interrupt
      LPM0_EXIT;                            // Exit LPM0
      pin = ~P1IN;                          // Read INVERTED port 1 pin state
      if( (pin & SW1) && !(prevPin & SW1) ) // Was SW1 off and now active?
      {
        eventFlag |= EVENT_SW1;             // SW1 is active
        P1IES &= ~SW1;                      // Set P1.3 0->1 edge trigger
      }
      else if( !(pin & SW1) && (prevPin & SW1) ) // Was SW1 active and now off?
      {
        P1IES |=  SW1;                      // Set P1.3 1->0 edge trigger
      }

      if( (pin & SW2) && !(prevPin & SW2) ) // Was SW2 off and now active?
      {
        eventFlag |= EVENT_SW2;             // SW2 is active
        P1IES &= ~SW2;                      // Set P1.4 0->1 edge trigger
      }
      else if( !(pin & SW2) && (prevPin & SW2) ) // Was SW1 active and now off?
      {
        P1IES |=  SW2;                      // Set P1.4 1->0 edge trigger
      }

      prevPin = pin;                        // Save pin state
      P1IFG = 0;                            // Clear P1 interrupt flags
      P1IE |= (SW1+SW2);                    // Enable P1.4,3 interrupts
      break;
    case 10:                                // TAIFG (not used)
      break;
  }
}


//
// UART0 RX Interrupt Service Routine
//
#pragma vector=UART0RX_VECTOR
__interrupt void usart0_rx(void)
{
  // Save character received from BSL
  BslRxMsg[bslRxIndex++] = RXBUF0;

  if( flags & REPLY_FLAG )                  // Is BSL Rx msg state active?
  {
    if( bslRxIndex == 4 )
    {
      bslReplyLen = RXBUF0 + 6;             // Save length of reply message
    }
    else if( bslRxIndex == bslReplyLen )
    {
      if( flags & REPLY_FLAG )
      {
        flags &= ~REPLY_FLAG;               // Clear REPLY flag
        flags |= DONE_FLAG;                 // Set DONE flag
        LEDPORTOUT &= ~(LED3);              // Clear LED3
        LEDPORTOUT |= LED1;                 // Set LED1
      }
    }
  }
  else                                      // Not in BSL Rx reply msg state
  {
    if( RXBUF0 == ACK_CHAR )                // Check for ACK from BSL
    {
      LEDPORTOUT &= ~(LED3);                // Clear LED3
      LEDPORTOUT |= LED1;                   // Set LED1
      flags |= ACK_FLAG;                    // Set ACK flag
      cancelTimer();                        // Ensure timer in cancelled
    }
    else if( RXBUF0 == NACK_CHAR )          // Check for NACK from BSL
    {
      LEDPORTOUT &= ~(LED1);                // Clear LED1
      LEDPORTOUT |= LED3;                   // Set LED3
      flags |= NACK_FLAG;                   // Set NACK flag
      cancelTimer();                        // Ensure timer in cancelled
    }
    else if( bslRxIndex == 1 )
    {
      if( RXBUF0 == SYNC_CHAR )             // Is start of BSL Rx message?
      {
        flags |= REPLY_FLAG;                // Set REPLY flag
        LEDPORTOUT &= ~(LED1+LED3);         // Clear LED1 and LED3
      }
    }
  }

  LPM0_EXIT;                                // Exit LPM0
}


//
// Compare strings
//
int strncompare(const char* str1, const char* str2, unsigned char len)
{
  unsigned char x;

  for(x = 0; x < len; x++)
  {
    if( str1[x] > str2[x] )
    {
      return 1;
    }

    if( str1[x] < str2[x] )
    {
      return -1;
    }
  }

  return 0;
}


//
// Wait for reply from target BSL
//
int bslWaitForReply(unsigned char type)
{
  unsigned int delayVal;
  unsigned char replyFlag;
  int retVal = 0;
  int timeout = TIMEOUT_LOOP;

  // BSL reply type
  switch(type)
  {
  case BSL_ACK_REPLY:
    delayVal = ACK_DELAY;
    replyFlag = ACK_FLAG;
    break;
  case BSL_MSG_REPLY:
    delayVal = MSG_DELAY;
    replyFlag = DONE_FLAG;
    break;
  };

  do
  {
    delay(delayVal);

    // Check for NACK from BSL
    if( flags & NACK_FLAG )
    {
      retVal = NACK_ERROR;
      break;
    }

  } while( !(flags & replyFlag) && --timeout );

  // Check for timeout error
  if( timeout == 0 )
  {
    retVal = TIMEOUT_ERROR;
  }

  return retVal;
}


//
// Delay using Timer_A CCR1
//
void delay(unsigned int val)
{
  TIMERLEDPORT |= TIMER_LED;                // Set LED
  flags &= ~TIMER_FLAG;                     // Clear Timer flag
  TACCR1 = TAR + val;                       // Delay
  TACCTL1 &= ~CCIFG;                        // Clear TACCR1 interrupt flag
  TACCTL1 |= CCIE;                          // Enable TACCR1 interrupt

  while( !(flags & TIMER_FLAG) )            // Wait for Timer_A to set flag
  {
    _BIS_SR(LPM0_bits);                     // Enter LPM0
  }
}


//
// Set timer using Timer_A CCR1
//
void timer(unsigned int val)
{
  TIMERLEDPORT |= TIMER_LED;                // Set LED
  flags &= ~TIMER_FLAG;                     // Clear Timer flag
  TACCR1 = TAR + val;                       // Set timeout delay
  TACCTL1 &= ~CCIFG;                        // Clear TACCR1 interrupt flag
  TACCTL1 |= CCIE;                          // Enable TACCR1 interrupt
}


//
// Cancel Timer
//
void cancelTimer(void)
{
  TACCTL1 &= ~(CCIE + CCIFG);               // Disable TACCR1 interrupt
  flags |= TIMER_FLAG;                      // Set Timer flag
  TIMERLEDPORT &= ~TIMER_LED;               // Clear LED
}


//
// Send character to BSL
//
void bslSendChar(unsigned char val)
{
  while( !(IFG1 & UTXIFG0) );               // USART0 TX buffer ready?
  TXBUF0 = val;                             // Copy val to TXBUF0
}


//
// Transmit message to target BSL
//
int bslTxMsg(BslMsg* pMsg)
{
  char x;
  char len;
  int retVal = 0;

  // Set LED2
  LEDPORTOUT |= LED2;

  // Send Sync charater to target BSL
  if( (retVal = bslTxSync()) < 0 )
  {
    return retVal;
  }

  // Clear flags
  flags &= ~BSL_FLAGS;

  // Reset BSL Rx message index
  bslRxIndex = 0;

  // Reset reply message length
  bslReplyLen = 0;

  // Calculate checksum
  bslChecksum(pMsg);

  // Transmit BSL header
  for(x = 0; x < sizeof(pMsg->hdr); x++)
  {
    bslSendChar(pMsg->hdr[x]);
  }

  // Transmit BSL data
  len = pMsg->hdr[2] - 4;
  for(x = 0; x < len; x++)
  {
    bslSendChar(pMsg->data[x]);
  }

  // Transmit checksum
  for(x = 0; x < sizeof(pMsg->chksum); x++)
  {
    bslSendChar(pMsg->chksum[x]);
  }

  // Check for BSL reply type
  switch(pMsg->reply)
  {
  case BSL_ACK_REPLY:
    retVal = bslWaitForReply(BSL_ACK_REPLY);
    break;
  case BSL_MSG_REPLY:
    retVal = bslWaitForReply(BSL_MSG_REPLY);
    break;
  };

  // Clear LED2
  LEDPORTOUT &= ~LED2;

  return retVal;
}


//
// Send SYNC character to BSL
//
int bslTxSync(void)
{
  int retVal = 0;

  // Clear ACK flag
  flags &= ~ACK_FLAG;

  // Set timer
  timer(SYNC_DELAY);

  // Send SYNC charcter to target BSL
  bslSendChar(SYNC_CHAR);

  // Wait for ACK from BSL or Timer
  while( !( (flags & ACK_FLAG) || (flags & TIMER_FLAG) ) );

  if( !(flags & ACK_FLAG) )
  {
    retVal = TIMEOUT_ERROR;
  }
  else
  {
    // Delay after sending SYNC character to BSL
    delay(CHAR_DELAY);
  }

  return retVal;
}


//
// Calculate checksum
//
void bslChecksum(BslMsg* pMsg)
{
  char x;
  char len;

  // Clear checksum
  pMsg->chksum[0] = 0;
  pMsg->chksum[1] = 0;

  // Calculate checksum over header
  for(x = 0; x < sizeof(pMsg->hdr)/2; x++)
  {
    pMsg->chksum[0] ^= pMsg->hdr[(2*x)];
    pMsg->chksum[1] ^= pMsg->hdr[(2*x)+1];
  }

  // Calculate checksum over data
  // hdr[2] contains length
  len = (pMsg->hdr[2] - 4)/2;
  for(x = 0; x < len; x++)
  {
    pMsg->chksum[0] ^= pMsg->data[(2*x)];
    pMsg->chksum[1] ^= pMsg->data[(2*x)+1];
  }

  // Invert
  pMsg->chksum[0] = ~pMsg->chksum[0];
  pMsg->chksum[1] = ~pMsg->chksum[1];
}


//
// Reset BSL state
//
void bslStateReset(void)
{
  LEDPORTOUT &= ~STATUS_LEDS;               // Clear status LEDs
  bslRxIndex = 0;                           // Reset BSL Rx message index
  bslReplyLen = 0;                          // Reset reply message length
  flags = 0;                                // Clear BSL flags
  configTimer_A();                          // Reset Timer_A
}


//
// Generate BSL entry sequence for MSP430's with shared JTAG pins
//
void bslEntrySeq(void)
{
  bslStateReset();                          // Reset BSL state

  if( BSLSEQIN & BSL_SEQ_PIN )
  {
    // BSL Entry Sequence - MSP430's with dedicated JTAG pins
    TEST_TCK_PORT |= TCK_PIN;               // Set TCK pin high
    delay(BSL_DELAY_CNT);                   // Delay
    RESET_NMI_PORT &= ~RESET_NMI_PIN;       // Set RST/NMI low
    delay(BSL_DELAY_CNT);                   // Delay
    TEST_TCK_PORT &= ~TCK_PIN;              // Set TCK pin low
    delay(BSL_DELAY_CNT);                   // Delay
    TEST_TCK_PORT |= TCK_PIN;               // Set TCK pin high
    delay(BSL_DELAY_CNT);                   // Delay
    TEST_TCK_PORT &= ~TCK_PIN;              // Set TCK pin low
    delay(BSL_DELAY_CNT);                   // Delay
    RESET_NMI_PORT |= RESET_NMI_PIN;        // Set RST/NMI pin high
    delay(BSL_DELAY_CNT);                   // Delay
    TEST_TCK_PORT |= TCK_PIN;               // Set TCK pin high
  }
  else
  {
    // BSL Entry Sequence - MSP430's with shared JTAG pins
    RESET_NMI_PORT &= ~RESET_NMI_PIN;       // Set RST/NMI pin low
    delay(BSL_DELAY_CNT);                   // Delay
    TEST_TCK_PORT  &= ~TEST_PIN;            // Set TEST pin low
    delay(BSL_DELAY_CNT);                   // Delay
    TEST_TCK_PORT  |= TEST_PIN;             // Set TEST pin high
    delay(BSL_DELAY_CNT);                   // Delay
    TEST_TCK_PORT  &= ~TEST_PIN;            // Set TEST pin low
    delay(BSL_DELAY_CNT);                   // Delay
    TEST_TCK_PORT  |= TEST_PIN;             // Set TEST pin high
    delay(BSL_DELAY_CNT);                   // Delay
    RESET_NMI_PORT |= RESET_NMI_PIN;        // Set RST/NMI pin high
    delay(BSL_DELAY_CNT);                   // Delay
    TEST_TCK_PORT  &= ~TEST_PIN;            // Set TEST pin low
  }
}


//
// Create BSL Version Command
//
void bslMsgVer(BslMsg* pMsg)
{
  pMsg->hdr[0] = SYNC_CHAR;                 // Sync character
  pMsg->hdr[1] = 0x1e;                      // BSL command identifier
  pMsg->hdr[2] = 0x04;                      // Length
  pMsg->hdr[3] = 0x04;                      // Length (repeated)
  pMsg->hdr[4] = 0x00;                      // Dummy data (required)
  pMsg->hdr[5] = 0x00;                      // Dummy data (required)
  pMsg->hdr[6] = 0x00;                      // Dummy data (required)
  pMsg->hdr[7] = 0x00;                      // Dummy data (required)

  pMsg->reply = BSL_MSG_REPLY;              // Set BSL reply type
}


//
// Create BSL Mass Erase Command
//
void bslMsgErase(BslMsg* pMsg)
{
  pMsg->hdr[0] = SYNC_CHAR;                 // Sync character
  pMsg->hdr[1] = 0x18;                      // BSL command identifier
  pMsg->hdr[2] = 0x04;                      // Length
  pMsg->hdr[3] = 0x04;                      // Length (repeated)
  pMsg->hdr[4] = 0x00;                      // Dummy data (required)
  pMsg->hdr[5] = 0x00;                      // Dummy data (required)
  pMsg->hdr[6] = 0x00;                      // Dummy data (required)
  pMsg->hdr[7] = 0x00;                      // Dummy data (required)

  pMsg->reply = BSL_ACK_REPLY;              // Set BSL reply type
}


//
// Create BSL Check Erase Command
//
void bslMsgEraseChk(BslMsg* pMsg, int addr, int len)
{
  pMsg->hdr[0] = SYNC_CHAR;                 // Sync character
  pMsg->hdr[1] = 0x1C;                      // BSL command identifier
  pMsg->hdr[2] = 0x04;                      // Length
  pMsg->hdr[3] = 0x04;                      // Length (repeated)
  pMsg->hdr[4] = (char)(addr & 0x00ff);     // Address (low byte)
  pMsg->hdr[5] = (char)(addr >> 8);         // Address (high byte)
  pMsg->hdr[6] = (char)(len & 0x00ff);      // Length (low byte)
  pMsg->hdr[7] = (char)(len >> 8);          // Length (high byte)

  pMsg->reply = BSL_ACK_REPLY;              // Set BSL reply type
}


//
// Create BSL TX Data Block Command
//
void bslMsgTxDataBlk(BslMsg* pMsg, int addr, int bytes)
{
  pMsg->hdr[0] = SYNC_CHAR;                 // Sync character
  pMsg->hdr[1] = 0x14;                      // BSL command identifier
  pMsg->hdr[2] = 0x04;                      // Length
  pMsg->hdr[3] = 0x04;                      // Length (repeated)
  pMsg->hdr[4] = (char)(addr & 0x00ff);     // Address (low byte)
  pMsg->hdr[5] = (char)(addr >> 8);         // Address (high byte)
  pMsg->hdr[6] = (char)(bytes & 0x00ff);    // Length (250 max)
  pMsg->hdr[7] = 0x00;                      // Set to 0

  pMsg->reply = BSL_MSG_REPLY;              // Set BSL reply type
}


//
// Create BSL RX Password Command
//
void bslMsgRxPassword(BslMsg* pMsg)
{
  int x;

  pMsg->hdr[0] = SYNC_CHAR;                 // Sync character
  pMsg->hdr[1] = 0x10;                      // BSL command identifier
  pMsg->hdr[2] = 0x24;                      // Length
  pMsg->hdr[3] = 0x24;                      // Length (repeated)
  pMsg->hdr[4] = 0x00;                      // Dummy data (required)
  pMsg->hdr[5] = 0x00;                      // Dummy data (required)
  pMsg->hdr[6] = 0x00;                      // Dummy data (required)
  pMsg->hdr[7] = 0x00;                      // Dummy data (required)

  for(x = 0; x < 32; x++)
  {
    pMsg->data[x] = BslPassword[x];         // Copy BSL password
  }

  pMsg->reply = BSL_ACK_REPLY;              // Set BSL reply type
}


//
// Create BSL RX Data Block Command
//
void bslMsgRxDataBlk(BslMsg* pMsg, int addr, int len, unsigned int* pData)
{
  int x;
  unsigned char* pCode;

  if( pData == 0 )
  {
    pCode = (unsigned char*)CodeArray;
  }
  else
  {
    pCode = (unsigned char*)pData;
  }

  pMsg->hdr[0] = SYNC_CHAR;                 // Sync character
  pMsg->hdr[1] = 0x12;                      // BSL command identifier
  pMsg->hdr[2] = len+4;                     // Length
  pMsg->hdr[3] = len+4;                     // Length (repeated)
  pMsg->hdr[4] = (char)(addr & 0x00ff);     // Address (low byte)
  pMsg->hdr[5] = (char)(addr >> 8);         // Address (high byte)
  pMsg->hdr[6] = (char)(len & 0x00ff);      // Length (250 max)
  pMsg->hdr[7] = 0x00;                      // Set to 0

  for(x = 0; x < len; x++)
  {
    pMsg->data[x] = *(pCode + x);           // Copy data
  }

  pMsg->reply = BSL_ACK_REPLY;              // Set BSL reply type
}


//
// Create BSL Erase Segment Command
//
void bslMsgEraseSeg(BslMsg* pMsg, int addr, int len)
{
  pMsg->hdr[0] = SYNC_CHAR;                 // Sync character
  pMsg->hdr[1] = 0x16;                      // BSL command identifier
  pMsg->hdr[2] = 0x04;                      // Length
  pMsg->hdr[3] = 0x04;                      // Length (repeated)
  pMsg->hdr[4] = (char)(addr & 0x00ff);     // Address (low byte)
  pMsg->hdr[5] = (char)(addr >> 8);         // Address (high byte)
  pMsg->hdr[6] = (char)(len & 0x00ff);      // Length (low byte)
  pMsg->hdr[7] = (char)(len >> 8);          // Length (high byte)

  pMsg->reply = BSL_ACK_REPLY;              // Set BSL reply type
}


//
// Create BSL Load PC Command
//
void bslMsgLoadPC(BslMsg* pMsg, int addr)
{
  pMsg->hdr[0] = SYNC_CHAR;                 // Sync character
  pMsg->hdr[1] = 0x1A;                      // BSL command identifier
  pMsg->hdr[2] = 0x04;                      // Length
  pMsg->hdr[3] = 0x04;                      // Length (repeated)
  pMsg->hdr[4] = (char)(addr & 0x00ff);     // Address (low byte)
  pMsg->hdr[5] = (char)(addr >> 8);         // Address (high byte)
  pMsg->hdr[6] = 0x00;                      // Dummy, set to 0
  pMsg->hdr[7] = 0x00;                      // Dummy, set to 0

  pMsg->reply = BSL_ACK_REPLY;              // Set BSL reply type
}


//
// Create BSL Change Baud Rate Command
//
void bslMsgBaudRate(BslMsg* pMsg, int dco, int bcs, int baudRate)
{
  pMsg->hdr[0] = SYNC_CHAR;                 // Sync character
  pMsg->hdr[1] = 0x20;                      // BSL command identifier
  pMsg->hdr[2] = 0x04;                      // Length
  pMsg->hdr[3] = 0x04;                      // Length (repeated)
  pMsg->hdr[4] = (char)(dco & 0x00ff);      // DCOCTL (1xx), SCFI0 (4xx)
  pMsg->hdr[5] = (char)(bcs & 0x00ff);      // BCSCTL1 (1xx), SCFI1 (4xx)
  pMsg->hdr[6] = (char)(baudRate & 0x0003); // Baud Rate (0-9600, 1-19200, 2-38400)
  pMsg->hdr[7] = 0x00;                      // Dummy, set to 0

  pMsg->reply = BSL_ACK_REPLY;              // Set BSL reply type
}


//
// Download program to target MSP430 via BSL
//
int bslDownloadProgram(void)
{
  int retVal;

  // Erase Flash
  bslMsgErase(&bslMsg);
  if( (retVal = bslTxMsg(&bslMsg)) < 0 )
  {
    return -2;
  }

  // Send password
  bslMsgRxPassword(&bslMsg);
  if( (retVal = bslTxMsg(&bslMsg)) < 0 )
  {
    return -3;
  }

  // Download code stored in array CodeArray[] to target
  if( (retVal = bslDownloadCode(&CodeArray[0], (flags & PATCH_FLAG))) < 0 )
  {
    return -4;
  }

  return retVal;
}


//
// Install BSL patch for MSP430 devices with BSL version 1.10
//
int bslInstallPatch(void)
{
  int retVal;

  // Erase Flash
  bslMsgErase(&bslMsg);
  if( (retVal = bslTxMsg(&bslMsg)) < 0 )
  {
    return -1;
  }

  // Send password
  bslMsgRxPassword(&bslMsg);
  if( (retVal = bslTxMsg(&bslMsg)) < 0 )
  {
    return -2;
  }

  // Load PC with 0xc22 (initialize stack pointer to safe address)
  bslMsgLoadPC(&bslMsg, 0xc22);
  if( (retVal = bslTxMsg(&bslMsg)) < 0 )
  {
    return -3;
  }

  // Send password again
  bslMsgRxPassword(&bslMsg);
  if( (retVal = bslTxMsg(&bslMsg)) < 0 )
  {
    return -4;
  }

  // Download code stored in array PatchArray[] to target
  if( (retVal = bslDownloadCode(&PatchArray[0], 0)) < 0 )
  {
    return -5;
  }

  return retVal;
}


//
// Download code to target. Code to download to target is stored locally in
// Flash in a special format.
//
int bslDownloadCode(unsigned int* pCodeArray, unsigned char patchFlag)
{
  int addr;
  int words;
  int section;
  int bytes;
  int blockSize;
  int index = 1;

  // Load one section at a time
  for(section = 0; section < *pCodeArray; section++)
  {
    addr = *(pCodeArray + index++);         // Section load address

    if( section == 0 )
    {
      StartAddr = addr;                     // Save start address
    }

    words = *(pCodeArray + index++);        // Number of words in section
    bytes = words * 2;

    // Set block size
    if( bytes >= TX_BLK_SIZE )
    {
      blockSize = TX_BLK_SIZE;
    }
    else
    {
      blockSize = bytes;
    }

    do
    {
      if( patchFlag != 0 )
      {
        // Load PC with start address of patch
        bslMsgLoadPC(&bslMsg, PatchArray[1]);
        if( bslTxMsg(&bslMsg) < 0 )
        {
          return -1;
        }
      }

      // Write data block to target device
      bslMsgRxDataBlk(&bslMsg, addr, blockSize, (pCodeArray + index));
      if( bslTxMsg(&bslMsg) < 0 )
      {
        return -2;
      }

      if( patchFlag != 0 )
      {
        // Load PC with start address of patch
        bslMsgLoadPC(&bslMsg, PatchArray[1]);
        if( bslTxMsg(&bslMsg) < 0 )
        {
          return -3;
        }
      }

      // Read data block from target device
      bslMsgTxDataBlk(&bslMsg, addr, blockSize);
      if( bslTxMsg(&bslMsg) < 0 )
      {
        return -4;
      }

      // Verify data written matches data read
      if( strncompare((const char*)(pCodeArray + index), (const char*)&BslRxMsg[4], blockSize) )
      {
        // Data read does not match data written
        return -5;
      }

      if( bytes >= TX_BLK_SIZE )
      {
        bytes -= TX_BLK_SIZE;
        addr += TX_BLK_SIZE;
        index += TX_BLK_SIZE/2;
      }
      else
      {
        blockSize = bytes;
        index += bytes/2;
        bytes = 0;
      }

    } while(bytes > 0);
  }

  return 0;
}
