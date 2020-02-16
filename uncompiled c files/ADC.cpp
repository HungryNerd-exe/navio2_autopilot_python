#include <unistd.h>
#include <cstdio>
#include "/home/pi/ctypes_tests/navio/Util.h"
#include "/home/pi/ctypes_tests/navio/ADC_Navio2.h"
#include <memory>

extern "C" {

	void* initialize() {
	    ADC_Navio2* adc = new ADC_Navio2();
	    adc->initialize();
		return adc;
	}

	void measure(void* ptr, float* y) {
		ADC_Navio2* adc = reinterpret_cast<ADC_Navio2*>(ptr);
		for (int i=0; i<6; i++) {
			y[17+i] = adc->read(i);
		}
		return;
	}

}
