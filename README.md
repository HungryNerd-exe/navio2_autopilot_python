# navio2_autopilot_python

air.py
    contains implementations of sensor reads and telemetry communication to be called in processes from test.py.

test.py
    initializes separate processes and arrays to share data with functions in air.py.
    contains placeholders for estimation and control functions to update values for servo and telemetry operations controlled by air.py.

both of these files also have a variant that takes advantage of shared C libraries derived from Emlid C++ example code. you can use test_c.py and air_c.py for a slight increase in performance, but this will require the presence of the folder named 'shared_c_libraries.'

the folder named 'Navio2' is a submodule from Emlid and is required for both variants.

execution of test.py is all that is required to run.
    example: sudo nice -n -10 python test_c.py
    
all files in the repository should be kept within the same directory, but your service file should be placed in /etc/systemd/system and should contain the full filepath of either variant of test.py
