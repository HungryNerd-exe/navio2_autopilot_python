#include <new> //For std::nothrow
#include </home/pi/shared_c_libraries/navio/Ublox.h>
#include </home/pi/shared_c_libraries/navio/Util.h>

extern "C"  //Tells the compile to use C-linkage for the next scope.
{
    void * initialize() {
		Ublox* ubl = new Ublox;
		ubl->configureSolutionRate(1000);
        return ubl;
    }

    int decode_message(void* ptr, float* y) {
        try {
            Ublox* ubl = reinterpret_cast<Ublox *>(ptr);
			return ubl->decodeSingleMessage(y);
        }
        catch(...) {
           return -1; //assuming -1 is an error condition.
        }
    }

}
