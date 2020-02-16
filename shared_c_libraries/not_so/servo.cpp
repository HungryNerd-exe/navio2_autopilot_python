/*
Provided to you by Emlid Ltd (c) 2015.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Control servos connected to PWM driver onboard of Navio2 shield for Raspberry Pi.

Connect servo to Navio2's rc output and watch it work.
PWM_OUTPUT = 0 complies to channel number 1, 1 to channel number 2 and so on.
To use full range of your servo correct SERVO_MIN and SERVO_MAX according to it's specification.

To run this example navigate to the directory containing it and run following commands:
make
sudo ./Servo
*/

#include <unistd.h>
#include "/home/pi/shared_c_libraries/navio/PWM.h"
#include "/home/pi/shared_c_libraries/navio/RCOutput_Navio2.h"
#include "/home/pi/shared_c_libraries/navio/Util.h"
#include <unistd.h>
#include <memory>

// using namespace Navio;

// std::unique_ptr <RCOutput> get_rcout() {
//     auto ptr = std::unique_ptr <RCOutput>{ new RCOutput_Navio2() };
//     return ptr;
// }

extern "C" {
	void* initialize() {
		// auto pwm = get_rcout();
		auto pwm = new RCOutput_Navio2();
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
