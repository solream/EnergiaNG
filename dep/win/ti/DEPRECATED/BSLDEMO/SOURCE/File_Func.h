/*========================================================================*\
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
/*========================================================================*\
|                                                                          |
| FileFunc.h                                                               |
|                                                                          |
|                                                                          |
|--------------------------------------------------------------------------|
| Project:              MSP430 JTAG interfcae                              |
| Developed using:      MS Visual C++ 5.0                                  |
|--------------------------------------------------------------------------|
| Version:              0.00                                               |
| Initial Version:      20 / 02 / 02                                       |
| Last Change:                                                             |
|--------------------------------------------------------------------------|
| Version history:                                                         |
|                                                                          |
|--------------------------------------------------------------------------|
| Designed 2002 by Texas Instruments                                       |
\*========================================================================*/

/*------------------------------------------------------------------------*\
| Remarks:                                                                 |
|                                                                          |
\*------------------------------------------------------------------------*/

#ifndef _File_Func_H_
#define _File_Func_H_

#include "MSP430.h"
#include "TI_TXT_Files.h"

#define MAX_LINE_SIZE 1024
#define MAX_MEM_SIZE 65536

#define LASTPERIPHERALADDR 0x1ff

extern const long ramStart;
extern const long ramEnd;
extern const long ram2Start;
extern const long ram2End;
extern const long infoStart;
extern const long infoEnd;
extern const long mainStart;
extern const long mainEnd;


typedef BYTE BYTEARRAY[];
typedef BYTEARRAY *LPBYTEARRAY;

struct downloadSegment
{
  struct downloadSegment* next;
  BYTE* data;
  long startAddress;
  long size;
};

extern struct downloadSegment* ramSegmentList;
extern struct downloadSegment* flashSegmentList;

//-- FreeSegBuffer -----------------------------------------------------------
// Free Allocated Buffer for Flash and Ram Data
// Arguments: 
// Result:    
void FreeSegBuffer(void);

STATUS_T MSP430_ReadOutFile(LONG wStart, LONG wLength, 
                  LPTSTR lpszFileName, LONG iFileType);

#endif /* _File_Func_H_ */
