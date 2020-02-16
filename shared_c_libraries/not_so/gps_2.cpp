#include <new> //For std::nothrow
#include </home/pi/Navio2/C++/Navio/Common/Ublox.h>
#include </home/pi/Navio2/C++/Navio/Common/Util.h>

extern "C"  //Tells the compile to use C-linkage for the next scope.
{
    void * ConstructGPS( void )
    {
        return new(std::nothrow) Ublox;
    }

    void DeleteGPS (void *ptr)
    {
         // delete(std::nothrow) ptr;
	 delete ptr;
    }

    int DecodePOSLLH(void *ptr)
    {
	std::vector<double> pos_data;
        try
        {
            Ublox * ref = reinterpret_cast<Ublox *>(ptr);
            // return ref->decodeSingleMessage(Ublox::NAV_POSLLH, pos_data);
	    ref->decodeMessage(pos_data);
	    printf("%.0lf",pos_data[0]);
	    return 0;
        }
        catch(...)
        {
           return -1; //assuming -1 is an error condition.
        }
    }

}
