#include "/home/pi/ctypes_tests/navio/PWM.h"
#include "/home/pi/ctypes_tests/navio/RCOutput_Navio2.h"
#include "/home/pi/ctypes_tests/navio/Util.h"
#include <unistd.h>
#include <memory>

extern "C" {
	void* initialize() {
		RCOutput_Navio2 pwm = new RCOutput_Navio2();
		for (int i=0; i<6; i++) {
			if( !(pwm->initialize(i)) ) {
				return nullptr;
			}
			pwm->set_frequency(i, 50);
			if ( !(pwm->enable(i)) ) {
				return nullptr;
			}
		}
		return pwm;
	}

	int set_servo(void* ptr, int i, int p) {
		RCOutput_Navio2* pwm = reinterpret_cast<RCOutput_Navio2*>(ptr);
		pwm->set_duty_cycle(i,p);
		return 0;
	}
}
