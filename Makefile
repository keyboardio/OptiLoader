process-images:
	cp images/atmega32u4_factory.hex atmega32u4.h
	cp images/attiny88_factory.hex attiny88.h
	perl -pi -e's|^([:0-9A-Z]*)|"$$1\\n"|i' atmega32u4.h attiny88.h
build:
	/usr/local/arduino/arduino-builder \
	-compile \
	-hardware /usr/local/arduino/hardware \
	-hardware /home/jesse/Arduino/hardware \
	-tools /usr/local/arduino/tools-builder \
	-tools /usr/local/arduino/hardware/tools/avr \
	-built-in-libraries /usr/local/arduino/libraries \
	-libraries /home/jesse/Arduino/libraries \
	-fqbn=mighty-1284p-1.6.3:avr:avr_developers \
	-ide-version=10800 \
	-prefs=build.warn_data_percentage=75 \
	-prefs=runtime.tools.avrdude.path=/usr/local/arduino/hardware/tools/avr \
	-prefs=runtime.tools.arduinoOTA.path=/usr/local/arduino/hardware/tools/avr \
	-prefs=runtime.tools.avr-gcc.path=/usr/local/arduino/hardware/tools/avr \
	-verbose /home/jesse/git/autoprogrammer/autoprogrammer.ino