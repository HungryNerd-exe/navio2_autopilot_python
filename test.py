#!/usr/bin/env python

from __future__ import print_function
from pymavlink import mavutil
import multiprocessing
import ctypes
import time
import numpy as np
import air_c as air

print_time_servo = False
print_time_estimator_nogps = False
print_time_estimator_gps = True
print_time_controller = False

# define indices of xh for easier access.
x, y, z, vt, alpha, beta, phi, theta, psi, p, q, r = range(12)

# define indices of y for easier access.
ax, ay, az, gyro_p, gyro_q, gyro_r, mag_x, mag_y, mag_z = range(9)
pres_baro = 9
gps_posn_n, gps_posn_e, gps_posn_d, gps_vel_n, gps_vel_e, gps_vel_d = range(10,16)

# define indices of servo for easier access.
mode_flag = 0
rcin_0, rcin_1, rcin_2, rcin_3, rcin_4, rcin_5 = range(1,7)
servo_0, servo_1, servo_2, servo_3, servo_4, servo_5 = range(7,13)
throttle, aileron, elevator, rudder, none, flaps = range(7,13)

# define indices of cmd for easier access.
psi_c, h_c = range(2)

def estimator_loop(y,xh,servo):
    # get sensors for read_sensor function call.
    sensors = air.initialize_sensors()
    time.sleep(3)
    while True:
        initial_time = time.time()
        new_gps = air.read_sensor(y,sensors) # updates values in y
        # print(y[0])
    #=====ESTIMATOR CODE STARTS HERE==================================
        # Predict step here

    # Measurement correction step here (depends on which sensors available)
        if (new_gps):
            pass # do estimation with gps here.
        else:
            pass # do estimation without gps here.
        # write estimated values to the xh array.
    #======ESTIMATOR CODE STOPS HERE===================================
        if (new_gps and print_time_estimator_gps):
            loop_time = (time.time()-initial_time)
            print('estimator_gps loop time: '+str(round(loop_time,5))+' sec\t['+str(int(round(1/loop_time)))+' hz]')
        elif (not new_gps and print_time_estimator_nogps):
            loop_time = (time.time()-initial_time)
            print('estimator_nogps loop time: '+str(round(loop_time,5))+' sec\t['+str(int(round(1/loop_time)))+' hz]')
        time.sleep(max(0.0125-(time.time()-initial_time),0) )

def controller_loop(xh,servo,cmd):

    while True:
        initial_time=time.time()
        if (servo[mode_flag] == 1):
        #======CONTROLLER CODE STARTS HERE===============================================
            # rewrite servo_out values to servo array based on their previous values and xh, cmd
            # if (servo[servo_1]<1.5): servo[servo_1] = 1.55
            # else: servo[servo_1] = 1.45
            # time.sleep(1)
            #Controller should assign values in range 1.25 to 1.75 to outputs;
            #WARNING, servo damage likely if values outside this range are assigned
            #Example: This is a manual passthrough function
            servo[throttle]=servo[rcin_0]
            servo[aileron]=servo[rcin_1]
            servo[elevator]=servo[rcin_2]
            servo[rudder]=servo[rcin_3]
            servo[servo_4]=servo[servo_4] #no servo; channel used for manual/auto switch
            servo[flaps]=servo[rcin_5]
        #=======CONTROLLER CODE STOPS HERE ======================================
        if print_time_controller:
            loop_time = (time.time()-initial_time)
            print('controller loop time: '+str(round(loop_time,5))+' sec\t['+str(int(round(1/loop_time)))+' hz]')
        time.sleep(max(0.0125-(time.time()-initial_time),0) )


if __name__ == "__main__":

    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600, source_system=255)

    # initialize arrays for sharing sensor data.
    y = multiprocessing.RawArray(ctypes.c_float, np.zeros(26)) # imu, baro, gps, adc
    xh = multiprocessing.Array('d', np.zeros(12)) # position, orientation, rates
    servo = multiprocessing.Array('d', np.zeros(13)) # mode_flag, rcin, servo_out
    cmd = multiprocessing.Array('d', np.zeros(2)) # psi_c, h_c

    # start processes for interpreting sensor data and setting servo pwm.
    estimator_process = multiprocessing.Process(target=estimator_loop, args=(y,xh,servo))
    estimator_process.daemon = True
    estimator_process.start()
    controller_process = multiprocessing.Process(target=controller_loop, args=(xh,servo,cmd))
    controller_process.daemon = True
    controller_process.start()
    servo_process = multiprocessing.Process(target=air.servo_loop, args=(servo,print_time_servo))
    servo_process.daemon = True
    servo_process.start()
    time.sleep(5)
    # start process for telemetry after other processes have initialized.
    telemetry_process = multiprocessing.Process(target=air.telemetry_loop, args=(y,xh,servo,master))
    telemetry_process.daemon = True
    telemetry_process.start()
    print('\nestimator_loop process id: {}'.format(estimator_process.pid))
    print('controller_loop process id: {}'.format(controller_process.pid))
    print('servo_loop process id: {}'.format(servo_process.pid))
    print('telemetry_loop process id: {}'.format(telemetry_process.pid))

    print("\nsending heartbeats to {} at 1hz.".format('/dev/ttyAMA0'))
    # loop for sending heartbeats and receiving messages from gcs.
    while True:
        # send heartbeat message periodically
        master.mav.heartbeat_send(1, 0, 0, 0, 4, 0)
        # still haven't figured out how to get mode to show up in mission planner.
        # print('heartbeat sent.')
        time.sleep(0.5)
    #=====WAYPOINT TRACKER STARTS HERE======================
    #Simple waypoint tracker
        #
    #=====WAYPOINT TRACKER STOPS HERE=======================

    # handle incoming commands over telemetry
        # try:
        #     msg = master.recv_match().to_dict()
        #     if (not (msg['mavpackettype'] == 'RADIO' or msg['mavpackettype'] == 'RADIO_STATUS' or msg['mavpackettype'] == 'HEARTBEAT')):
        #         print(msg)
        #         if (msg['mavpackettype'] == 'COMMAND_LONG'):
        #             master.mav.command_ack_send(msg['command'],4)
        #             print("acknowledge sent.")
        # except:
        #     pass
