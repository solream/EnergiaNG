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

*****************************************************************
* 
* Project: MSP430 Bootstrap Loader Demonstration Program
*
* File:    BSLCOMM.H
*
* History:
*   Version 1.00 (05/2000)
*   Version 1.11 (09/2000)
*     - Added definition of BSL_CRITICAL_ADDR.
*   Version 1.12 (02/2001 FRGR)
*     - Added definition of (not released) BSL command
*       "Erase Check" BSL_ECHECK
*     
****************************************************************/

#ifndef BSLComm__H
#define BSLComm__H

#include "ssp.h"

/* Transmit password to boot loader: */
#define BSL_TXPWORD 0x10 
/* Transmit block    to boot loader: */
#define BSL_TXBLK   0x12 
/* Receive  block  from boot loader: */
#define BSL_RXBLK   0x14 
/* Erase one segment:                */
#define BSL_ERASE   0x16 
/* Erase complete FLASH memory:      */
#define BSL_MERAS   0x18 
/* Load PC and start execution:      */
#define BSL_LOADPC  0x1A 
/* Erase Check Fast:                 */
#define BSL_ECHECK  0x1C 
/* Receive ID:                       */
#define BSL_RXID    0x1E
/* Change Speed:                     */
#define BSL_SPEED   0x20  
/* Setg Memory Offset (for devices with >64k Mem  */
#define BSL_MEMOFFSET 0x21  
   

/* Bootstrap loader syncronization error: */
#define ERR_BSL_SYNC      99

/* Upper limit of address range that might be modified by 
 * "BSL checksum bug".
 */
#define BSL_CRITICAL_ADDR 0x0A00

#ifdef __cplusplus
extern "C" {
#endif

extern int BSLMemAccessWarning;

/*-------------------------------------------------------------*/
void bslReset(BOOL invokeBSL);
/* Applies BSL entry sequence on RST/NMI and TEST/VPP pins
 * Parameters: invokeBSL = TRUE:  complete sequence
 *             invokeBSL = FALSE: only RST/NMI pin accessed
 */

/*-------------------------------------------------------------*/
int bslSync();
/* Transmits Synchronization character and expects to
 * receive Acknowledge character
 * Return == 0: OK
 * Return == 1: Sync. failed.
 */

/*-------------------------------------------------------------*/
int bslTxRx(BYTE cmd, unsigned long addr, WORD len, 
            BYTE blkout[], BYTE blkin[]);
/* Transmits a command (cmd) with its parameters: 
 * start-address (addr), length (len) and additional 
 * data (blkout) to boot loader. 
 * Parameters return by boot loader are passed via blkin.
 * Return == 0: OK
 * Return != 0: Error!
 */

#ifdef __cplusplus
}
#endif

#endif

/* EOF */