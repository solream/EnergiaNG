/****************************************************************
Copyright (c) 2015-2017 Texas Instruments Incorporated

All rights reserved not granted herein.

Limited License.

Texas Instruments Incorporated grants a world-wide, royalty-free,
non-exclusive license under copyrights and patents it now or
hereafter owns or controls to make, have made, use, import,
offer to sell and sell ("Utilize") this software subject to
the terms herein.  With respect to the foregoing patent license,
such license is granted  solely to the extent that any such patent
is necessary to Utilize the software alone.  The patent license
shall not apply to any combinations which include this software,
other than combinations with devices manufactured by or for TI
(“TI Devices”). No hardware patent is licensed hereunder.

Redistributions must preserve existing copyright notices and
reproduce this license (including the above copyright notice
and the disclaimer and (if applicable) source code license
limitations below) in the documentation and/or other materials
provided with the distribution

Redistribution and use in binary form, without modification,
are permitted provided that the following conditions are met:

*	No reverse engineering, decompilation, or disassembly of
this software is permitted with respect to any software
provided in binary form.

*	any redistribution and use are licensed by TI for use only
with TI Devices.

*	Nothing shall obligate TI to provide you with source code
for the software licensed and provided to you in object code.

If software source code is provided to you, modification and
redistribution of the source code are permitted provided that
the following conditions are met:

*	any redistribution and use of the source code, including any
resulting derivative works, are licensed by TI for use only
with TI Devices.

*	any redistribution and use of any object code compiled from
the source code and any resulting derivative works, are licensed
by TI for use only with TI Devices.

Neither the name of Texas Instruments Incorporated nor the names
of its suppliers may be used to endorse or promote products derived
from this software without specific prior written permission.

DISCLAIMER.

THIS SOFTWARE IS PROVIDED BY TI AND TI’S LICENSORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI’S LICENSORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*----------------------------------------------------------------
* 
* Project: MSP430 Bootstrap Loader Demonstration Program
*
* File       : ssp.C
*
* History:
*
* 08/01 FRGR Implemented function comChangeBaudrate()
* 06/04 UPSF Added return no error at comChangeBaudrate()
****************************************************************/

#include <string.h>
#include <stdio.h>
#include <windows.h>
#include "ssp.h"

/* Global Constants: */

/* Size of internal WINDOWS-Comm-Buffer: */
#define QUEUE_SIZE       512 

#define MAX_FRAME_COUNT   16
#define MAX_ERR_COUNT      5

/* Global Variables: */
const unsigned short protocolMode= MODE_BSL;
HANDLE       hComPort;    /* COM-Port Handle             */
DCB          comDCB;      /* COM-Port Control-Settings   */
COMSTAT      comState;    /* COM-Port Status-Information */
COMMTIMEOUTS orgTimeouts; /* Original COM-Port Time-out  */

/* Time in milliseconds until a timeout occurs: */
DWORD timeout      = DEFAULT_TIMEOUT; 
/* Factor by which the timeout after sending a frame is prolonged: */
int prolongFactor= DEFAULT_PROLONG; 

/* Variable to save the latest error (used by comGetLastError): */
int lastError;     

BYTE seqNo, reqNo, txPtr, rxPtr;
BYTE rxFrame[MAX_FRAME_SIZE];

DWORD nakDelay; /* Delay before DATA_NAK will be send */

/***************************************************************/
DWORD calcTimeout(DWORD startTime) /* exported! */
/* Calculates the difference between startTime and the acutal
 * windows time (in milliseconds).
 */
{
  return((DWORD)(GetTickCount() - startTime));
}

/*-------------------------------------------------------------*/
void delay(DWORD time) /* exported! */
/* Delays the execution by a given time in ms.
 */
{ 
#ifndef WIN32
  DWORD startTime= GetTickCount();
  while (calcTimeout(startTime) < time);
#else
  Sleep(time);
#endif
}

/*-------------------------------------------------------------*/
WORD calcChecksum(BYTE data[], WORD length)
/* Calculates a checksum of "data".
 */
{
  WORD* i_data;
  WORD checksum= 0;
  BYTE i= 0;

  i_data= (WORD*)data;

  for (i= 0; i < length/2; i++)
  { 
    checksum^= i_data[i];    /* xor-ing   */
  }
  return(checksum ^ 0xffff); /* inverting */
}

/*-------------------------------------------------------------*/
int comWaitForData(int count, DWORD timeout) /* exported! */
/* Waits until a given number (count) of bytes was received or a
 * given time (timeout) has passed.
 */
{
  DWORD errors;
  int rxCount= 0;
  DWORD startTime= GetTickCount();
  do
  { 
    ClearCommError(hComPort, &errors, &comState);
  } while (((rxCount= comState.cbInQue) < count) && 
           (calcTimeout(startTime) <= timeout));

  return(rxCount);
}

/*-------------------------------------------------------------*/
int comRxHeader(BYTE *rxHeader, BYTE *rxNum, 
                DWORD timeout)
{
  BYTE Hdr;
  DWORD dwRead;

  if (comWaitForData(1, timeout) >= 1)
  { 
    ReadFile(hComPort, &Hdr, 1, &dwRead, NULL);
    *rxHeader= Hdr & 0xf0;
    *rxNum   = Hdr & 0x0f;

    if (protocolMode == MODE_BSL)
    { reqNo= 0;
      seqNo= 0;
      *rxNum= 0; 
    }

    return(ERR_NONE);
  }
  else
  { 
    *rxHeader= 0; 
    *rxNum= 0;
    return(lastError= ERR_RX_HDR_TIMEOUT);
  }
}

/*-------------------------------------------------------------*/
void comTxHeader(const BYTE txHeader)
{
  DWORD dwWrite;
  BYTE Hdr= txHeader;

  WriteFile(hComPort, &Hdr, 1, &dwWrite, NULL);
}

/***************************************************************/
int comGetLastError()
/* Returns the error code generated by the last function call to
 * a SERCOMM-Function.  If this function returned without errors, 
 * comGetLastError will return zero (errNoError) as well.
 */
{ return(lastError); }

/***************************************************************/
int comInit(LPCSTR lpszDevice, DWORD aTimeout, int aProlongFactor)
/* Tries to open the serial port given in 'lpszDevice' and
 * initialises the port and global variables.
 * The timeout and the number of allowed errors is multiplied by
 * 'aProlongFactor' after transmission of a command to give
 * plenty of time to the micro controller to finish the command.
 * Returns zero if the function is successful.
 */
{
  COMMTIMEOUTS timeouts;
  DWORD dwCommEvents;

  /* Init. global variables: */

  seqNo= 0; 
  reqNo= 0;
  rxPtr= 0;
  txPtr= 0;

  timeout= aTimeout;
  prolongFactor= aProlongFactor;
  
  hComPort= CreateFile(lpszDevice, GENERIC_READ | GENERIC_WRITE, 
                       0, 0, OPEN_EXISTING, 0, 0);
  /* In this application the serial port is used in 
   * nonoverlapped mode! 
   */
  if (hComPort == INVALID_HANDLE_VALUE)
  {   
    hComPort= 0;
    return (lastError= ERR_OPEN_COMM); /* Error! */
  }
  if (SetupComm(hComPort, QUEUE_SIZE, QUEUE_SIZE) == 0)
  { 
    CloseHandle(hComPort);
   hComPort= 0;
    return (lastError= ERR_OPEN_COMM); /* Error! */
  }

  /* Save original timeout values: */
  GetCommTimeouts(hComPort, &orgTimeouts);
  /* Set Windows timeout values (disable build-in timeouts): */
  timeouts.ReadIntervalTimeout= 0;
  timeouts.ReadTotalTimeoutMultiplier= 0;
  timeouts.ReadTotalTimeoutConstant= 0;
  timeouts.WriteTotalTimeoutMultiplier= 0;
  timeouts.WriteTotalTimeoutConstant= 0;
  if (!SetCommTimeouts(hComPort, &timeouts))
  { 
    CloseHandle(hComPort);
   hComPort= 0;
    return (lastError= ERR_OPEN_COMM); /* Error! */
  }

  dwCommEvents= EV_RXCHAR | EV_TXEMPTY | EV_RXFLAG | EV_ERR;
  SetCommMask(hComPort, dwCommEvents);


  /* Get state and modify it: */
  if (!GetCommState(hComPort, &comDCB))
  { 
    CloseHandle(hComPort);
   hComPort= 0;
    return (lastError= ERR_OPEN_COMM); /* Error! */
  }

  comDCB.BaudRate    = CBR_9600; /* Startup-Baudrate: 9,6kBaud */
  comDCB.ByteSize    = 8;
  nakDelay= (DWORD)((11*MAX_FRAME_SIZE)/9.6);

  comDCB.Parity      = EVENPARITY;   
  comDCB.StopBits    = ONESTOPBIT;
  comDCB.fBinary     = TRUE; /* Enable Binary Transmission */
  comDCB.fParity     = TRUE; /* Enable Parity Check        */
  comDCB.ErrorChar   = (char)0xff; 
  /* Char. w/ Parity-Err are replaced with 0xff 
   *(if fErrorChar is set to TRUE) 
   */
  comDCB.fRtsControl = RTS_CONTROL_ENABLE; /* For power supply */
  comDCB.fDtrControl = DTR_CONTROL_ENABLE; /* For power supply */
                       
  comDCB.fOutxCtsFlow= FALSE;        comDCB.fOutxDsrFlow= FALSE;        
  comDCB.fOutX       = FALSE;        comDCB.fInX        = FALSE;
  comDCB.fNull       = FALSE;

  comDCB.fErrorChar  = FALSE; 

  /* Assign new state: */
  if (!SetCommState(hComPort, &comDCB))
  { 
    CloseHandle(hComPort);
   hComPort= 0;
    return(lastError= ERR_SET_COMM_STATE); /* Error! */
  }

  /* Clear buffers: */
  PurgeComm(hComPort, PURGE_TXCLEAR | PURGE_TXABORT);
  PurgeComm(hComPort, PURGE_RXCLEAR | PURGE_RXABORT);

  return(lastError= 0);
} /* comInit */

/***************************************************************/
DWORD comGetBaudrate()
/* Returns Baudrate of the used serial port
 */
{
   return(comDCB.BaudRate);
}

int comChangeBaudrate(DWORD Baud)
/* Changes Baudrate of the used serial port
 */
{
   comDCB.BaudRate = Baud;
   if (!SetCommState(hComPort, &comDCB))
   { 
      CloseHandle(hComPort);
      hComPort= 0;
      return(lastError= ERR_SET_COMM_STATE); /* Error! */
   }
    return(ERR_NONE);
}

/***************************************************************/
int comDone()
/* Closes the used serial port. 
 * This function must be called at the end of a program,
 * otherwise the serial port might not be released and can not be
 * used in other programs.
 * Returns zero if the function is successful.
 */
{
  DWORD errors;
  DWORD startTime= GetTickCount();
  /* Wait until data is transmitted, but not too long... (Timeout-Time) */
  do
  { 
    ClearCommError(hComPort, &errors, &comState);
  } while ((comState.cbOutQue > 0) && 
           (calcTimeout(startTime) < timeout));

  /* Clear buffers: */
  PurgeComm(hComPort, PURGE_TXCLEAR | PURGE_TXABORT);
  PurgeComm(hComPort, PURGE_RXCLEAR | PURGE_RXABORT);
  /* Restore original timeout values: */
  SetCommTimeouts(hComPort, &orgTimeouts);
  /* Close COM-Port: */
  if (!CloseHandle(hComPort))
    return(lastError= ERR_CLOSE_COMM); /* Error! */
  else
    return(lastError= ERR_NONE);
} /* comDone */


/***************************************************************/

/*-------------------------------------------------------------*/
int comRxFrame(BYTE *rxHeader, BYTE *rxNum)
{
  DWORD dwRead;
  WORD checksum;
  BYTE* rxLength;
  WORD rxLengthCRC;

  rxFrame[0]= DATA_FRAME | *rxNum;

  if (comWaitForData(3, timeout) >= 3)
  {
    ReadFile(hComPort, &rxFrame[1], 3, &dwRead, NULL);
    
    if ((rxFrame[1] == 0) && (rxFrame[2] == rxFrame[3]))
    {
      rxLength= &rxFrame[2];      /* Pointer to rxFrame[2]   */
      rxLengthCRC= *rxLength + 2; /* Add CRC-Bytes to length */
        
      if (comWaitForData(rxLengthCRC, timeout) >= rxLengthCRC)
      {
        ReadFile(hComPort, &rxFrame[4], rxLengthCRC, &dwRead, NULL);

        /* Check received frame: */
        checksum= calcChecksum(rxFrame, (WORD)(*rxLength+4)); 
                  /* rxLength+4: Length with header but w/o CRC */

        if ((rxFrame[*rxLength+4] == (BYTE)checksum) && 
            (rxFrame[*rxLength+5] == (BYTE)(checksum >> 8)))
        {
          return(ERR_NONE); 
          /* Frame received correctly (=> send next frame) */
        } /* if (Checksum correct?)        */
      } /* if (Data: no timeout?)          */
    } /* if (Add. header info. correct?)   */
  } /* if (Add. header info.: no timeout?) */

  return(ERR_COM); /* Frame has errors! */
}  /* comRxFrame */

/*-------------------------------------------------------------*/
int comTxRx(BYTE cmd, BYTE dataOut[], BYTE length)
/* Sends the command cmd with the data given in dataOut to the
 * microcontroller and expects either an acknowledge or a frame
 * with result from the microcontroller.  The results are stored
 * in dataIn (if not a NULL pointer is passed).
 * In this routine all the necessary protocol stuff is handled.
 * Returns zero if the function was successful.
 */
{
  DWORD dwWrite;
  DWORD errors;
  BYTE txFrame[MAX_FRAME_SIZE];
  WORD checksum= 0;
  int k= 0;
  int errCtr= 0;
  int resendCtr= 0;
  BYTE rxHeader= 0;
  BYTE rxNum= 0;
  int resentFrame= 0;
  int pollCtr= 0;

  /* Transmitting part ----------------------------------------*/
  /* Prepare data for transmit */
  if ((length % 2) != 0)
  { /* Fill with one byte to have even number of bytes to send */
    if (protocolMode == MODE_BSL)
      dataOut[length++]= 0xFF; // fill with 0xFF
    else                    
      dataOut[length++]= 0;    // fill with zero 
  }

  txFrame[0]= DATA_FRAME | seqNo;
  txFrame[1]= cmd; 
  txFrame[2]= length;
  txFrame[3]= length;

  reqNo= (seqNo + 1) % MAX_FRAME_COUNT;

  memcpy(&txFrame[4], dataOut, length);

  checksum= calcChecksum(txFrame, (WORD)(length+4));
  txFrame[length+4]= (BYTE)(checksum);
  txFrame[length+5]= (BYTE)(checksum >> 8);

  {
    WORD accessAddr= (0x0212 + (checksum^0xffff)) & 0xfffe;
                     /* 0x0212: Address of wCHKSUM */
    if (BSLMemAccessWarning && (accessAddr < BSL_CRITICAL_ADDR))
    {
      printf("WARNING: This command might change data "
             "at address %x or %x!\n", 
             accessAddr, accessAddr + 1);
    }
  }

  /* Transmit data: */
  k= 0;

  /* Clear receiving queue: */
  PurgeComm(hComPort, PURGE_RXCLEAR | PURGE_RXABORT);
  do
  {
    WriteFile(hComPort, &txFrame[k++], 1, &dwWrite, NULL);
    
    ClearCommError(hComPort, &errors, &comState);
  } while ((k < length + 6) && (comState.cbInQue == 0));
  /* Check after each transmitted character, 
   * if microcontroller did send a character (probably a NAK!).
   */

  /* Receiving part -------------------------------------------*/
  rxFrame[2]= 0;
  rxFrame[3]= 0; /* Set lengths of received data to 0! */

  do
  {
    lastError= 0; /* Clear last error */
    if (comRxHeader(&rxHeader, &rxNum, timeout*prolongFactor) == 0) 
        /* prolong timeout to allow execution of sent command */
    { /* => Header received */
      do
      {
        resentFrame= 0;
        switch (rxHeader)
        { case DATA_ACK:
          if (rxNum == reqNo)
            { seqNo= reqNo;
              return(lastError= ERR_NONE); 
              /* Acknowledge received correctly => next frame */
            }
          break; /* case DATA_ACK */

          case DATA_NAK:
            return(lastError= ERR_RX_NAK);
        break; /* case DATA_NAK */

          case DATA_FRAME:
            if (rxNum == reqNo)
              if (comRxFrame(&rxHeader, &rxNum) == 0)
                return(lastError= ERR_NONE);
          break; /* case DATA_FRAME */

          case CMD_FAILED:
            /* Frame ok, but command failed. */
            return(lastError= ERR_CMD_FAILED); 
          break; /* case CMD_FAILED */

          default:
            ;
        } /* switch */
        
          errCtr= MAX_ERR_COUNT;
      } while ((resentFrame == 0) && (errCtr < MAX_ERR_COUNT));
    } /* if (comRxHeader) */
    else
    { /* => Timeout while receiving header */
        errCtr= MAX_ERR_COUNT;
    } /* else (comRxHeader) */
  } while (errCtr < MAX_ERR_COUNT);

  if (lastError == ERR_CMD_NOT_COMPLETED)
  { /* Accept QUERY_RESPONSE as real ACK and correct Seq.-No.: */
    seqNo= reqNo;
  }

  if (lastError == ERR_NONE)
    return(lastError= ERR_COM);
  else
    return(lastError);
} /* comTxRx */


/* EOF */
