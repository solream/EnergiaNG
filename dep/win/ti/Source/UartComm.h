/*
==UartComm.h==

Copyright (c) 2015�2017 Texas Instruments Incorporated

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
(�TI Devices�). No hardware patent is licensed hereunder.

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

THIS SOFTWARE IS PROVIDED BY TI AND TI�S LICENSORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI�S LICENSORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

#include "stdafx.h"

class UartComm : public CommInterface, public BslResponseHandler
{
public:

	UartComm(ModeParams* modeParams, Util * util);
	~UartComm();

	/***********************************************************************************
	*Function:    init
	*Description: do the initialization of the communication protocol based
	*             on the parameters of the given command
	*Parameters:  ModeParams modeParams: parameters from interpreter
	*Returns:     -
	***********************************************************************************/
	void init(ModeParams* modeParams) OVERRIDE;

	/***********************************************************************************
	*Function:    transmitBuffer
	*Description: send the BSL command
	*Parameters:  std::vector<uint8_t>* txBuffer: prepared buffer from Target class
	*Returns:     -
	***********************************************************************************/
	void transmitBuffer(std::vector<uint8_t>* txBuffer) OVERRIDE;

	/***********************************************************************************
	*Function:	  receiveBuffer
	*Description: receive byte(s) response from target device
	*Parameters:  int32_t size : number of bytes that should be received by the
	*                            scripter
	*             (-1) means only one byte response without wrapped in BSL data packet
	*             (0) means no byte response
	*			  (n>0) means n bytes wrapped in BSL data packet
	*Returns:     std::vector<uint8_t>*
	***********************************************************************************/
	std::vector<uint8_t>*  receiveBuffer(int32_t size) OVERRIDE;
	
	/***********************************************************************************
	*Function:    changeBaudRate
	*Description: change baud rate with the given baudRate on the serial communication
	*Parameters:  uint32_t baudRate: new baudRate
	*Returns:	  -
	***********************************************************************************/
	void changeBaudRate(uint32_t baudRate) OVERRIDE;

	/***********************************************************************************
	*Function:	  close
	*Description: close communication port
	*Parameters:  -
	*Returns:     -
	***********************************************************************************/
	void close() OVERRIDE;

	/***********************************************************************************
	*Function:	  resetBytesWritten
	*Description: reset bytes written in comm port
	*Parameters:  -
	*Returns:     -
	***********************************************************************************/
	void resetBytesWritten() OVERRIDE;

	/***********************************************************************************
	*Function:	  retBytesWritten
	*Description: return number of bytes sent including the additional bytes
	*Parameters:  -
	*Returns:     uint32_t
	***********************************************************************************/
	uint32_t retBytesWritten() OVERRIDE;

	/***********************************************************************************
	*Function:	  resetBytesWritten
	*Description: reset bytes written in comm port
	*Parameters:  -
	*Returns:     -
	***********************************************************************************/
	void resetBytesSent() OVERRIDE;

	/***********************************************************************************
	*Function:	  retBytesWritten
	*Description: return number of bytes sent including the additional bytes
	*Parameters:  -
	*Returns:     uint32_t
	***********************************************************************************/
	uint32_t retBytesSent() OVERRIDE;

	/***********************************************************************************
	*Function:	  retBytesRead
	*Description: return number of bytes received without the additional bytes
	*Parameters:  -
	*Returns:     uint32_t
	***********************************************************************************/
	uint32_t retBytesRead() OVERRIDE;

	/***********************************************************************************
	*Function:	  retAck
	*Description: return received ACK byte
	*Parameters:  -
	*Returns:     uint8_t
	***********************************************************************************/
	uint8_t retAck() OVERRIDE;

	/***********************************************************************************
	*Function:	  retLastRxBufferIdx
	*Description: return last index on the rxBuffer
	*Parameters:  -
	*Returns:	  uint32_t
	***********************************************************************************/
	uint32_t retLastRxBufferIdx() OVERRIDE;

	/***********************************************************************************
	*Function:	  retPtrRxBuffer
	*Description: return the pointer of rxBuffer
	*Parameters:  -
	*Returns:	  std::vector<uint8_t>*
	***********************************************************************************/
	std::vector<uint8_t>* retPtrRxBuffer() OVERRIDE;

	/***********************************************************************************
	*Function:	  retPtrTxBuffer
	*Description: return the pointer of txBuffer
	*Parameters:  -
	*Returns:	  std::vector<uint8_t>*
	***********************************************************************************/
	std::vector<uint8_t>* retPtrTxBuffer() OVERRIDE;

	/***********************************************************************************
	*Function:	  resetIdxAcc
	*Description: reset index calculation for the rxBuffer for reading memory process
	*Parameters:  -
	*Returns:	  -
	***********************************************************************************/
	void resetIdxAcc() OVERRIDE;

	/***********************************************************************************
	*Function:	  processResponse
	*Description: calling processResponse from the inhereted class BslResponseHandler
	*Parameters:  uint8_t ack: received ACK byte
	std::vector<uint8_t> *response: pointer to rxBuffer
	*Returns:	  -
	***********************************************************************************/
	void processResponse(uint8_t ack, std::vector<uint8_t> *response) OVERRIDE;

	/***********************************************************************************
	*Function:	  retErrorAck
	*Description: calling retErrorAck from the inhereted class BslResponseHandler
	*Parameters:  -
	*Returns:	  bool
	***********************************************************************************/
	bool retErrorAck() OVERRIDE;

	/***********************************************************************************
	*Function:	  retErrorStatus
	*Description: calling retErrorStatus from the inhereted class BslResponseHandler
	*Parameters:  -
	*Returns:	  bool
	***********************************************************************************/
	bool retErrorStatus() OVERRIDE;

	/***********************************************************************************
	*Function:	  retErrorAckMessage
	*Description: calling retErrorAckMessage from the inhereted class BslResponseHandler
	*Parameters:  -
	*Returns:	  std::string
	***********************************************************************************/
	std::string retErrorAckMessage() OVERRIDE;

	/***********************************************************************************
	*Function:	  retErrorStatusMessage
	*Description: calling retErrorStatusMessage from the inhereted class
	BslResponseHandler
	*Parameters:  -
	*Returns:	  std::string
	***********************************************************************************/
	std::string retErrorStatusMessage() OVERRIDE;

	/***********************************************************************************
	*Function:	  retCrcCheckResult
	*Description: calling retCrcCheckResult from the inhereted class BslResponseHandler
	*Parameters:  -
	*Returns:	  std::string
	***********************************************************************************/
	bool retCrcCheckResult()OVERRIDE;

	/***********************************************************************************
	*Function:	  translateResponse
	*Description: calling translateResponse from the inhereted class BslResponseHandler
	*Parameters:  -
	*Returns:	  std::string
	***********************************************************************************/
	std::string translateResponse(CommandParams *cmdParams, std::vector<uint8_t>* buffer, uint8_t idx) OVERRIDE;

   	/***********************************************************************************
	*Class:		  UartComm
	*Function:	  resetIdxRead
	*Description: reset all indexes that are used in the receiving process
	*Parameters:  none
	*Returns:	  none
	***********************************************************************************/
	void resetIdxRead();

private:
	
	/***********************************************************************************
	* Class variables and constants
	***********************************************************************************/
	boost::asio::io_service io_service;
	boost::asio::serial_port port;
	boost::asio::deadline_timer timer;

	std::vector<uint8_t> rxUartBuffer;

	uint32_t actualBaudRate;
	static const uint32_t INVOKE_430_BSL_ADD = 1;
	static const uint32_t INVOKE_MSPFET_P432_BSL_UART = 9602;
	static const uint32_t INVOKE_MSPFET_P432_BSL_I2C_ADD = 2;
	
	void sendByte(uint8_t *data);

	bool timerEvent;
	bool readEvent;
	bool cancelled;
  	bool waitAck;
	bool waitHeader;
	bool waitData;

	uint32_t actSize;
	uint32_t expSize;
	uint32_t bytesReceived;

	/***********************************************************************************
	*Function:	  fsmReceiveBuffer
	*Description: handles the process in receiving bytes
	*Parameters:  -
	*Returns:	  bool
	***********************************************************************************/
	bool fsmReceiveBuffer(int32_t size);

	/***********************************************************************************
	*Function:	  setTimer
	*Description: set deadline timer with delay input in the parameter
	*Parameters:  -
	*Returns:	  uint32_t
	***********************************************************************************/
	void setTimer(uint32_t duration);

	/***********************************************************************************
	*Function:	  onTimer
	*Description: set deadline timer on
	*Parameters:  const boost::system::error_code& ec : error message
	*Returns:	  -
	***********************************************************************************/
	void onTimer(const boost::system::error_code& ec);

	/***********************************************************************************
	*Function:	  startRead
	*Description: set deadline timer on
	*Parameters:  const boost::system::error_code& ec : error message
	*Returns:	  uint8_t* buf : pointer to the buffer where data will be stored
	*             size_t numBytes : number of bytes should be received
	***********************************************************************************/
	void startReadAck(uint8_t* buf, size_t numBytes);

	/***********************************************************************************
	*Function:	  startRead
	*Description: set deadline timer on
	*Parameters:  const boost::system::error_code& ec : error message
	*Returns:	  uint8_t* buf : pointer to the buffer where data will be stored
	*             size_t numBytes : number of bytes should be received
	***********************************************************************************/
	void startReadBuf(uint16_t offset, size_t numBytes);

	/***********************************************************************************
	*Function:	  onRead
	*Description: set read process on
	*Parameters:  const boost::system::error_code& ec : error message
	*             size_t numBytes : number of received bytes
	*Returns:	  -
	***********************************************************************************/
	void onRead(const boost::system::error_code& ec, size_t numBytes);

	/***********************************************************************************
	*Function:	  isOpen
	*Description: check status of comm port
	*Parameters:  none
	*Returns:	  (true) port is accessible
	*             (false) port is not accessible
	***********************************************************************************/
	bool isOpen() const;

	/***********************************************************************************
	*Function:	  cancel
	*Description: cancel respective process (async_read or timer) when certain condition
	*             obtained
	*Parameters:  -
	*Returns:	  -
	***********************************************************************************/
	void cancel();

	/***********************************************************************************
	*Function:	  transmitP4xxInitialization
	*Description: transmit the initialization byte to invoke UART BSL 
	*Parameters:  -
	*Returns:	  -
	***********************************************************************************/
	void transmitP4xxInitialization();

	/***********************************************************************************
	*Function:	  changeNonStandardBaudRate
	*Description: To change the baud rate speed into non standard value that requires
	*             to trigger MSP-BSL "Rocket" or MSP-FET, the function in Ubuntu
	*             and Mac OS X requires a low-level function call
	*Parameters:  uint32_t baudRate : value of audrate speed
	*Returns:	  -
	***********************************************************************************/
	void changeNonStandardBaudRate(uint32_t baudRate);
};


