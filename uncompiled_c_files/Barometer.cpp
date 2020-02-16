/*
Provided to you by Emlid Ltd (c) 2014.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Get pressure from MS5611 barometer onboard of Navio shield for Raspberry Pi

To run this example navigate to the directory containing it and run following commands:
make
./Barometer
*/

#include "/home/pi/ctypes_tests/navio/MS5611.h"
#include "/home/pi/ctypes_tests/navio/Util.h"
#include <unistd.h>
#include <stdio.h>

extern "C" {

	void* initialize() {

		MS5611* baro = new MS5611();
	    baro->initialize();
		return baro;
	}

	void refreshPressure(void* ptr) {

		MS5611* baro = reinterpret_cast<MS5611*>(ptr);
		baro->refreshPressure();
		return;
	}

	void readPressure(void* ptr) {

		MS5611* baro = reinterpret_cast<MS5611*>(ptr);
		baro->readPressure();
		baro->calculatePressureAndTemperature();
		return;
	}

	float returnPressure(void* ptr) {
		MS5611* baro = reinterpret_cast<MS5611*>(ptr);
		return baro->getPressure();
	}

}
