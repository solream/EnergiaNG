#!/bin/bash -e

cd linux/work

./arduino --board arduino:avr:uno --verify examples/01.Basics/Blink/Blink.ino

for s in `find examples/0{1,2,3,4,5,6,7,8}* -name '*.ino' -not -name '*MultiSerial*'`
do
	echo arduino:avr:uno $s
	./arduino-builder -hardware ./hardware -tools ./hardware/tools/avr -tools ./tools-builder -libraries ./libraries -fqbn arduino:avr:uno $s
	echo
done
