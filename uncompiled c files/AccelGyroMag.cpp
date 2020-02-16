#include "/home/pi/ctypes_tests/navio/LSM9DS1.h"
#include "/home/pi/ctypes_tests/navio/Util.h"
#include <unistd.h>
#include <string>
#include <memory>

extern "C" {

	void* initialize() {
	    LSM9DS1* sensor = new LSM9DS1();
	    if (!sensor->probe()) {
	        printf("Sensor not enabled\n");
	        return nullptr;
	    }
	    sensor->initialize();
		return sensor;
	}

    int read_imu(void* ptr, float* y) {
		LSM9DS1* sensor = reinterpret_cast<LSM9DS1*>(ptr);
        sensor->update();
        sensor->read_accelerometer(&y[0], &y[1], &y[2]);
        sensor->read_gyroscope(&y[3], &y[4], &y[5]);
        sensor->read_magnetometer(&y[6], &y[7], &y[8]);
        // usleep(500000);
		return 0;
    }
}
