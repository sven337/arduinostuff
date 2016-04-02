#!/bin/bash

PREF=preferences_ota.txt

if [ "$1" == "--serial" ]; then
	PREF=preferences_serial.txt
fi

arduino --verbose --upload --preferences-file "$PREF" thermometer_esp8266.ino
