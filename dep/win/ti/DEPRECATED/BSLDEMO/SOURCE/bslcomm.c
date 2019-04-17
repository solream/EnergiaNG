


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
*
*****************************************************************
* 
* Project: MSP430 Bootstrap Loader Demonstration Program
*
* File:    BSLCOMM.C
*
* History:
*   Version 1.00 (05/2000)
*   Version 1.11 (09/2000)
*     - Added handling of frames with odd starting address
*       (This is required for bootstrap loaders with word
*       programming algorithm! > BSL-Version >= 1.30)
*     - Usage of BSL_CRITICAL_ADDR for warnings about memory 
*       accesses due to "BSL checksum bug".
*   Version is controlled by master file (11/00)
*     - 11/00 FRGR: Added delays in bslReset() routine to meet also
*       critical customer designs (considering MK's hint).
*
****************************************************************/

#include <windows.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>

#include "bslcomm.h"
#include "ssp.c"


#define BSL_SYNC 0x80

/* 1: Warning, if access to memory below 0x1000 is possible.
 *    This can happen due to an error in the first version(s) of
 *    the bootstrap loader code in combination with specific
 *    checksum values.
 * 0: No Warning.
 */
int BSLMemAccessWarning= 0; /* Default: no warning. */

/*-------------------------------------------------------------*/
void SetRSTpin(BOOL level)
/* Controls RST/NMI pin (0: GND; 1: VCC) */
{
  //if (level == TRUE)
  //  comDCB.fDtrControl = DTR_CONTROL_ENABLE;
  //else
  //  comDCB.fDtrControl = DTR_CONTROL_DISABLE;
  comDCB.fDtrControl = level ? DTR_CONTROL_ENABLE : DTR_CONTROL_DISABLE;

  SetCommState(hComPort, &comDCB);
} /* SetRSTpin */

void SetTESTpin(BOOL level)
/* Controls TEST pin (0: VCC; 1: GND) */
{
  //if (level == TRUE)
  //  comDCB.fRtsControl = RTS_CONTROL_ENABLE;
  //else
  //  comDCB.fRtsControl = RTS_CONTROL_DISABLE;
  comDCB.fRtsControl = level ? RTS_CONTROL_ENABLE : RTS_CONTROL_DISABLE;

  SetCommState(hComPort, &comDCB);
} /* SetTESTpin */

/*-------------------------------------------------------------*/
void bslReset(BOOL invokeBSL)
/* Applies BSL entry sequence on RST/NMI and TEST/VPP pins
 * Parameters: invokeBSL = TRUE:  complete sequence
 *             invokeBSL = FALSE: only RST/NMI pin accessed
 *
 * RST is inverted twice on boot loader hardware
 * TEST is inverted (only once)
 * Need positive voltage on DTR, RTS for power-supply of hardware
 */
{
  /* To charge capacitor on boot loader hardware: */
  SetRSTpin(1); 
  SetTESTpin(1);
  delay(250);

  if (invokeBSL)
  {
    SetRSTpin(0);    /* RST  pin: GND */
    SetTESTpin(1); /* TEST pin: GND */   delay(10);   /* delays added to meet also */
    SetTESTpin(0); /* TEST pin: Vcc */   delay(10);   /* critical layout and/or    */
    SetTESTpin(1); /* TEST pin: GND */   delay(10);   /* dimensioning problems     */
    SetTESTpin(0); /* TEST pin: Vcc */   delay(10);   /* (poked by MK)             */
    SetRSTpin (1); /* RST  pin: Vcc */   delay(10);
    SetTESTpin(1); /* TEST pin: GND */
  }
  else
  {  
    SetRSTpin(0);    /* RST  pin: GND */
    delay(10);       /* delays */
    SetRSTpin(1);    /* RST  pin: Vcc */
  }
  /* Give MSP430's oscillator time to stabilize: */
  delay(250); 

  /* Clear buffers: */
  PurgeComm(hComPort, PURGE_TXCLEAR);
  PurgeComm(hComPort, PURGE_RXCLEAR); 
} /* bslReset */

/*-------------------------------------------------------------*/
int bslSync()
/* Transmits Synchronization character and expects to
 * receive Acknowledge character
 * Return == 0: OK
 * Return == 1: Sync. failed.
 */
{
  BYTE  ch;
  int rxCount, loopcnt;
  const BYTE cLoopOut = 3; /* Max. trials to get synchronization */
  DWORD NrTx;
  DWORD NrRx;
    
  for (loopcnt=0; loopcnt < cLoopOut; loopcnt++)
  {
    PurgeComm(hComPort, PURGE_RXCLEAR); /* Clear receiving queue */

    /* Send synchronization byte: */
    ch = BSL_SYNC;
    WriteFile(hComPort, &ch, 1, &NrTx, NULL);

    /* Wait for 1 byte; Timeout: 100ms */
    rxCount= comWaitForData(1, 100); 
    if (rxCount > 0) 
    {
      ReadFile(hComPort, &ch, 1, &NrRx, NULL); 
      if (ch == DATA_ACK) 
      { return(ERR_NONE); } /* Sync. successful */
    }
  } /* for (loopcount) */
  
  return(ERR_BSL_SYNC); /* Sync. failed */
} /* bslSync */

/*-------------------------------------------------------------*/
int bslTxRx(BYTE cmd, unsigned long addr, WORD len, 
            BYTE* blkout, BYTE* blkin)
/* Transmits a command (cmd) with its parameters: 
 * start-address (addr), length (len) and additional 
 * data (blkout) to boot loader. 
 * Parameters return by boot loader are passed via blkin.
 * Return == 0: OK
 * Return != 0: Error!
 */
{
    BYTE dataOut[MAX_FRAME_SIZE];
    int error;
    WORD length= 4;

    if (cmd == BSL_TXBLK)
    {
      /* Align to even start address */
      if ((addr % 2) != 0)
      {
        /* Decrement address and               */
        addr--;
        /* fill first byte of blkout with 0xFF */
        memmove(&blkout[1], &blkout[0], len);
        blkout[0]= 0xFF; 
        len++;
      }
      /* Make sure that len is even */
      if ((len % 2) != 0)
      { 
        /* Inc. len and fill last byte of blkout with 0xFF */
        blkout[(len++)]= 0xFF;
      }
    }

    if (cmd == BSL_RXBLK)
    {
      /* Align to even start address */
      if ((addr % 2) != 0)
      {
        /* Decrement address but       */
        addr--;
        /* request an additional byte. */
        len++;
      }
      /* Make sure that len is even */
      if ((len % 2) != 0)
      { 
        len++;
      }
    }

    if ((cmd == BSL_TXBLK) || (cmd == BSL_TXPWORD)) 
    {
      length = len + 4; 
    }

    /* Add necessary information data to frame: */
    dataOut[0] =  (BYTE)( addr       & 0x00ff);
    dataOut[1] =  (BYTE)((addr >> 8) & 0x00ff);
    dataOut[2] =  (BYTE)( len        & 0x00ff);
    dataOut[3] =  (BYTE)((len  >> 8) & 0x00ff);
    
    if (blkout != NULL)
    { /* Copy data out of blkout into frame: */
      memcpy(&dataOut[4], blkout, len);
    }
    
    if (bslSync() != ERR_NONE) 
    { 
      return(ERR_BSL_SYNC); 
    }

    /* Send frame: */
    error = comTxRx(cmd, dataOut, (BYTE)length);

    if (blkin != NULL)
    { /* Copy received data out of frame buffer into blkin: */
      memcpy(blkin, &rxFrame[4], rxFrame[2]);
    }

    return (error);
}

/* EOF */
