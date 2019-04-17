#!/bin/bash

HEX=$1 # path to hex 
PORT=$2 # com port
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
OUT_FILE="$SCRIPT_DIR/bsl-config.conf"

echo -e "MODE FRxx UART 9600 ${PORT} PARITY" >  ${OUT_FILE}
echo -e "RX_PASSWORD ${SCRIPT_DIR}/pass32_wrong.txt" >> ${OUT_FILE}
echo -e "DELAY 2000" >> ${OUT_FILE}
echo -e "RX_PASSWORD ${SCRIPT_DIR}/pass32_default.txt" >> ${OUT_FILE}
echo -e "RX_DATA_BLOCK ${HEX}" >> ${OUT_FILE}
echo -e "SET_PC 0x4400" >> ${OUT_FILE}

