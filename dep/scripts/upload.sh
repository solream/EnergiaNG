#!/bin/bash
BUILD_PATH=$1
SKETCH_NAME=$2
PORT=$3
HEX_PATH=${BUILD_PATH}/${SKETCH_NAME}.hex
SOIL_DIR=""
PYTHON=python

exec > upload.log                                                                    
exec 2>&1

if [ "${OSTYPE//[0-9.]/}" == "darwin" ]
then
	echo "build: ${BUILD_PATH}, sketch: ${SKETCH_NAME}, port: ${PORT}, HEX: ${HEX_PATH}, OS: MAC"
	SOIL_DIR="./soil/mac"
else
	echo "build: ${BUILD_PATH}, sketch: ${SKETCH_NAME}, port: ${PORT}, HEX: ${HEX_PATH}, OS: LINUX"
	SOIL_DIR="./soil/linux"
fi

echo "SOIL in ${SOIL_DIR}" 

cd ${SOIL_DIR}

echo "GPIO OFF"
${PYTHON} switch_OFF_cp2102N_gpio5.py


echo "INVOKE BSL"
./BSL_invoke 


echo "GPIO ON"
${PYTHON} switch_OFF_cp2102N_gpio5.py


echo "GENERATE CONFIG"
./BSL_config ${HEX_PATH} ${PORT}

echo "===============CONFIG==============="
echo "$(cat ./bsl_config.conf)"
echo "=================END================"

echo "Run scripter"
./BSL_scripter ./bsl_config.conf

