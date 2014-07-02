#!/usr/bin/env python

###############################################################################################
###############################################################################################
##                                                                                           ##
## Hove's Raspberry Pi Python Quadcopter Flight Controller.  Open Source @ GitHub            ##
## PiStuffing/Quadcopterunder GPL for non-commercial application.  Any code derived from     ##
## this should retain this copyright comment.                                                ##
##                                                                                           ##
## Copyright 2014 Andy Baker (Hove) - andy@pistuffing.co.uk                                  ##
##                                                                                           ##
###############################################################################################
###############################################################################################

from __future__ import division
import signal
import socket
import time
import string
import sys
import getopt
import math
import threading
from array import *

import select
import os
import struct
import logging

#-------------------------------------------------------------------------------------------
# Only required when using GPIO.wait_for_event() to block for hardware interrupts
#-------------------------------------------------------------------------------------------
# import RPi.GPIO as RPIO

import RPIO
import subprocess
from datetime import datetime
import shutil
import ctypes
from ctypes.util import find_library
import ESC
import I2C
import MPU6050


		
############################################################################################
#
# GPIO pins initialization for MPU6050 interrupt and the sounder
#
############################################################################################
def RpioSetup():
	RPIO.setmode(RPIO.BCM)

	#-----------------------------------------------------------------------------------
	# Set the beeper output LOW
	#-----------------------------------------------------------------------------------
	logger.info('Set status sounder pin %s as out', RPIO_STATUS_SOUNDER)
	RPIO.setup(RPIO_STATUS_SOUNDER, RPIO.OUT, RPIO.LOW)

	#-----------------------------------------------------------------------------------
	# Set the MPU6050 interrupt input
	#-----------------------------------------------------------------------------------
	logger.info('Setup MPU6050 interrupt input %s', RPIO_DATA_READY_INTERRUPT)
	RPIO.setup(RPIO_DATA_READY_INTERRUPT, RPIO.IN) # , RPIO.PUD_DOWN)

############################################################################################
#
# Check CLI validity, set calibrate_sensors / fly or sys.exit(1)
#
############################################################################################
def CheckCLI(argv):
	cli_fly = False
	cli_calibrate_sensors = False
	cli_video = False

	cli_hover_target = 680

	#-----------------------------------------------------------------------------------
	# Defaults for vertical velocity PIDs
	#-----------------------------------------------------------------------------------
	cli_vvp_gain = 300.0
	cli_vvi_gain = 150.0
	cli_vvd_gain = 0.0

	#-----------------------------------------------------------------------------------
	# Defaults for horizontal velocity PIDs
	#-----------------------------------------------------------------------------------
	cli_hvp_gain = 0.6
	cli_hvi_gain = 0.1
	cli_hvd_gain = 0.0

	#-----------------------------------------------------------------------------------
	# Defaults for absolute angle PIDs
	#-----------------------------------------------------------------------------------
	cli_aap_gain = 2.5
	cli_aai_gain = 0.0
	cli_aad_gain = 0.0

	#-----------------------------------------------------------------------------------
	# Defaults for rotation rate PIDs
	#-----------------------------------------------------------------------------------
	cli_rrp_gain = 150
	cli_rri_gain = 0.0
	cli_rrd_gain = 0.0

	#-----------------------------------------------------------------------------------
	# Other configuration defaults
	#-----------------------------------------------------------------------------------
	cli_test_case = 0
	cli_tau = 2.0        # 0.25 * 100 = 25 samples averaged for -3dB merge
	cli_dlpf = 5
	cli_loop_frequency = 500 # 100
	cli_matrix = 2
	cli_statistics = False
	cli_yaw_control = False
	cli_motion_frequency = 40
	cli_attitude_frequency = 40

	hover_target_defaulted = True
	no_drift_control = False
	rrp_set = False
	rri_set = False
	rrd_set = False
	aap_set = False
	aai_set = False
	aad_set = False

	#-----------------------------------------------------------------------------------
	# Right, let's get on with reading the command line and checking consistency
	#-----------------------------------------------------------------------------------
	try:
		opts, args = getopt.getopt(argv,'a:fcvh:l:m:nsy', ['tc=', 'vvp=', 'vvi=', 'vvd=', 'hvp=', 'hvi=', 'hvd=', 'aap=', 'aai=', 'aad=', 'arp=', 'ari=', 'ard=', 'tau=', 'dlpf='])
	except getopt.GetoptError:
		logger.critical('qcpi.py [-f][-h hover_target][-v][')
		sys.exit(2)

	for opt, arg in opts:
		if opt == '-f':
			cli_fly = True

		elif opt in '-h':
			cli_hover_target = int(arg)
			hover_target_defaulted = False

		elif opt in '-v':
			cli_video = True

		elif opt in '-a':
			cli_attitude_frequency = int(arg)

		elif opt in '-c':
			cli_calibrate_sensors = True

		elif opt in '-l':
			cli_loop_frequency = int(arg)

		elif opt in '-m':
			cli_motion_frequency = int(arg)

		elif opt in '-n':
			no_drift_control = True

		elif opt in '-s':
			cli_statistics = True

		elif opt in '-y':
			cli_yaw_control = True

		elif opt in '--vvp':
			cli_vvp_gain = float(arg)

		elif opt in '--vvi':
			cli_vvi_gain = float(arg)

		elif opt in '--vvd':
			cli_vvd_gain = float(arg)

		elif opt in '--hvp':
			cli_hvp_gain = float(arg)

		elif opt in '--hvi':
			cli_hvi_gain = float(arg)

		elif opt in '--hvd':
			cli_hvd_gain = float(arg)

		elif opt in '--aap':
			cli_aap_gain = float(arg)
			aap_set = True

		elif opt in '--aai':
			cli_aai_gain = float(arg)
			aai_set = True

		elif opt in '--aad':
			cli_aad_gain = float(arg)
			aad_set = True

		elif opt in '--arp':
			cli_rrp_gain = float(arg)
			rrp_set = True

		elif opt in '--ari':
			cli_rri_gain = float(arg)
			rri_set = True

		elif opt in '--ard':
			cli_rrd_gain = float(arg)
			rrd_set = True

		elif opt in '--tc':
			cli_test_case = int(arg)

		elif opt in '--tau':
			cli_tau = float(arg)

		elif opt in '--dlpf':
			cli_dlpf = int(arg)

	if not cli_calibrate_sensors and not cli_fly and cli_test_case == 0:
		logger.critical('Must specify one of -f or -c or --tc')
		logger.critical('  qcpi.py [-f] [-t speed] [-c] [-v]')
		logger.critical('  -f set whether to fly')
		logger.critical('  -h set the hover speed for manual testing')
		logger.critical('  -c calibrate sensors against temperature and save')
		logger.critical('  -s enable diagnostic statistics')
		logger.critical('  -n disable drift control')
		logger.critical('  -y enable yaw control (unsupported')
		logger.critical('  -v video the flight')
		logger.critical('  -l ??  set the processing loop frequency')
		logger.critical('  -a ??  set attitude PID update frequency')
		logger.critical('  -m ??  set motion PID update frequency')
		logger.critical('  --vvp  set vertical speed PID P gain')
		logger.critical('  --vvi  set vertical speed PID P gain')
		logger.critical('  --vvd  set vertical speed PID P gain')
		logger.critical('  --hvp  set horizontal speed PID P gain')
		logger.critical('  --hvi  set horizontal speed PID I gain')
		logger.critical('  --hvd  set horizontal speed PID D gain')
		logger.critical('  --aap  set absolute angle PID P gain')
		logger.critical('  --aai  set absolute angle PID I gain')
		logger.critical('  --aad  set absolute angle PID D gain')
		logger.critical('  --arp  set angular PID P gain')
		logger.critical('  --ari  set angular PID I gain')
		logger.critical('  --ari  set angular PID D gain')
		logger.critical('  --tc   select which testcase to run')
		logger.critical('  --tau  set the complementary filter period')
		logger.critical('  --dlpf set the digital low pass filter')
		sys.exit(2)

	elif not cli_calibrate_sensors and (cli_hover_target < 0 or cli_hover_target > 1000):
		logger.critical('Hover speed must lie in the following range')
		logger.critical('0 <= test speed <= 1000')
		sys.exit(2)

	elif cli_yaw_control:
		logger.critical('YAW control is not supported yet')
		sys.exit(2)

	elif cli_test_case == 0 and cli_fly:
		logger.critical('Pre-flight checks passed, enjoy your flight, sir!')
		if no_drift_control:
			cli_hvp_gain = 0.0
			cli_hvi_gain = 0.0
			cli_hvd_gain = 0.0
			cli_aap_gain = 1.5
			cli_aai_gain = 0.5
			cli_aad_gain = 0.01
			cli_rrp_gain = 110
			cli_rri_gain = 100
			cli_rrd_gain = 2.5

	elif cli_test_case == 0 and cli_calibrate_sensors:
		logger.critical('Calibrate sensors is it, sir!')

	elif cli_test_case == 0:
		logger.critical('You must specify flight (-f) or gravity calibration (-c)')
		sys.exit(2)

	elif cli_fly or cli_calibrate_sensors:
		logger.critical('Choose a specific test case (--tc) or fly (-f) or calibrate gravity (-g)')
		sys.exit(2)

	#---------------------------------------------------------------------------------------
	# Test case 1: Check all the blades work and spin in the right direction
	# Test case 2: Tune the rotational rate PIDs
	# Test case 3: Tune the absolute angle PIDs
	# Test case 4: Tune the hover speed
	#---------------------------------------------------------------------------------------

	elif cli_test_case < 1 or cli_test_case > 4:
		logger.critical('Select test case 1, 2, 3 or 4')
		sys.exit(2)

	elif hover_target_defaulted:
		logger.critical('You must choose a specific hover speed (-h) for all test cases.')
		sys.exit(2)

	elif cli_test_case == 2 and (not rrp_set or not rri_set or not rrd_set):
		logger.critical('You must choose a starting point for the angular rate PID P, I and D gains')
		logger.critical('Try sudo python ./qc.py --tc 2 -h 450 --arp 50 --ari 0.0 --ard 0.0 and work up from there')
		sys.exit(2)

	elif cli_test_case == 3 and (not aap_set or not aai_set or not aad_set):
		logger.critical('You must choose a starting point for the absolute angle PID P, I and D gains')
		logger.critical('Try sudo python ./qc.py --tc 3 -h 450 --aap 1.5 --aai 0.5 --aad 0.001 and work up from there')
		sys.exit(2)

	elif cli_test_case == 2:
		cli_vvp_gain = 0.0
		cli_vvi_gain = 0.0
		cli_vvd_gain = 0.0
		cli_hvp_gain = 0.0
		cli_hvi_gain = 0.0
		cli_hvd_gain = 0.0
		cli_aap_gain = 0.0
		cli_aai_gain = 0.0
		cli_aad_gain = 0.0

	elif cli_test_case == 3 or cli_test_case == 4:
		cli_vvp_gain = 0.0
		cli_vvi_gain = 0.0
		cli_vvd_gain = 0.0
		cli_hvp_gain = 0.0
		cli_hvi_gain = 0.0
		cli_hvd_gain = 0.0

	return cli_calibrate_sensors, cli_fly, cli_hover_target, cli_video, cli_vvp_gain, cli_vvi_gain, cli_vvd_gain, cli_hvp_gain, cli_hvi_gain, cli_hvd_gain, cli_aap_gain, cli_aai_gain, cli_aad_gain, cli_rrp_gain, cli_rri_gain, cli_rrd_gain, cli_test_case, cli_tau, cli_dlpf, cli_loop_frequency, cli_motion_frequency, cli_attitude_frequency, cli_statistics, cli_yaw_control

############################################################################################
#
# Count down beeper
#
############################################################################################
def CountdownBeep(num_beeps):
	for beep in range(0, num_beeps):
		RPIO.output(RPIO_STATUS_SOUNDER, RPIO.HIGH)
		time.sleep(0.25)
		RPIO.output(RPIO_STATUS_SOUNDER, RPIO.LOW)
		time.sleep(0.25)
	time.sleep(2.0)

############################################################################################
#
# Shutdown triggered by early Ctrl-C or end of script
#
############################################################################################
def CleanShutdown():
	global esc_list
	global shoot_video
	global video

	#-----------------------------------------------------------------------------------
	# Stop the signal handler
	#-----------------------------------------------------------------------------------
	signal.signal(signal.SIGINT, signal.SIG_IGN)

	#-----------------------------------------------------------------------------------
	# Time for teddy bye byes
	#-----------------------------------------------------------------------------------
	for esc in esc_list:
		logger.info('Stop blade %d spinning', esc_index)
		esc.update(0)

	#-----------------------------------------------------------------------------------
	# Stop the video if it's running
	#-----------------------------------------------------------------------------------
	if shoot_video:
		video.send_signal(signal.SIGINT)

	#-----------------------------------------------------------------------------------
	# Copy logs from /dev/shm (shared / virtual memory) to the Logs directory.
	#-----------------------------------------------------------------------------------
	now = datetime.now()
	now_string = now.strftime("%y%m%d-%H:%M:%S")
	log_file_name = "qcstats" + now_string + ".csv"
	shutil.move("/dev/shm/qclogs", log_file_name)

	#-----------------------------------------------------------------------------------
	# Clean up PWM / GPIO
	#-----------------------------------------------------------------------------------
	PWM.cleanup()
	RPIO.output(RPIO_STATUS_SOUNDER, RPIO.LOW)
	RPIO.cleanup()

	#-----------------------------------------------------------------------------------
	# Unlock memory we've used from RAM
	#-----------------------------------------------------------------------------------
	munlockall()

	#-----------------------------------------------------------------------------------
	# Reset the signal handler to default
	#-----------------------------------------------------------------------------------
	signal.signal(signal.SIGINT, signal.SIG_DFL)

	sys.exit(0)

############################################################################################
#
# Signal handler for Ctrl-C => next FSM update if running else stop
#
############################################################################################
def SignalHandler(signal, frame):
	global loop_count
	global fsm_input
	global FSM_INPUT_STOP

	if loop_count > 0:
		fsm_input = FSM_INPUT_STOP
	else:
		CleanShutdown()

############################################################################################
#
# Conversion from quadcopter axes accelerometer values to earth axes equivalent
#
############################################################################################
def ConvertAxes(qax, qay, qaz, pa, ra, yaw_control):

	#===================================================================================
	# Axes: Convert the acceleration in g's to earth coordinates, then integrate to
	# convert to speeds in earth's X and Y axes meters per second.
	#
	# Matrix 1: Uses X, Y, and Y accelerometers but omit yaw
	# ---------
	# |eax|   | cos(pitch),         0,          -sin(pitch)| |qax|
	# |eay| = | 0,          cos(roll),           -sin(roll)| |qay|
	# |eaz|   | sin(pitch), sin(roll), cos(pitch).cos(roll)| |qaz|
	#
	# Matrix 2: Uses X, Y, and Y accelerometers and include yaw (unsupported)
	# ---------
	# |eax|   | cos(pitch), sin(roll),          -sin(pitch)| |qax|
	# |eay| = | sin(pitch), cos(roll),           -sin(roll)| |qay|
	# |eaz|   | sin(pitch), sin(roll), cos(pitch).cos(roll)| |qaz|
	#
	#===================================================================================
	if not yaw_control:
		eax = qax * math.cos(pa) - qaz * math.sin(pa)
		eay = qay * math.cos(ra) - qaz * math.sin(ra)
		eaz = qaz * math.cos(pa) * math.cos(ra) + qax * math.sin(pa) + qay * math.sin(ra) - 1.0

	else:
		eax = qax * math.cos(pa) + qay * math.sin(ra) - qaz * math.sin(pa)
		eay = qay * math.cos(ra) * qax * math.sin(pa) - qaz * math.sin(ra)
		eaz = qaz * math.cos(pa) * math.cos(ra) + qax * math.sin(pa) + qay * math.sin(ra) - 1.0

	return eax, eay, eaz


############################################################################################
#
# Main
#
############################################################################################

#-------------------------------------------------------------------------------------------
# Lock code permanently in memory - no swapping to disk
#-------------------------------------------------------------------------------------------
MCL_CURRENT = 1
MCL_FUTURE  = 2
def mlockall(flags = MCL_CURRENT| MCL_FUTURE):
	result = libc.mlockall(flags)
	if result != 0:
		raise Exception("cannot lock memmory, errno=%s" % ctypes.get_errno())

def munlockall():
	result = libc.munlockall()
	if result != 0:
		raise Exception("cannot lock memmory, errno=%s" % ctypes.get_errno())


libc_name = ctypes.util.find_library("c")
libc = ctypes.CDLL(libc_name, use_errno=True)
mlockall()

#-------------------------------------------------------------------------------------------
# Set up the global constants
#-------------------------------------------------------------------------------------------
G_FORCE = 9.80665

RPIO_DMA_CHANNEL = 1

ESC_BCM_BL = 22
ESC_BCM_FL = 17
ESC_BCM_FR = 18
ESC_BCM_BR = 23

MOTOR_LOCATION_FRONT = 0b00000001
MOTOR_LOCATION_BACK =  0b00000010
MOTOR_LOCATION_LEFT =  0b00000100
MOTOR_LOCATION_RIGHT = 0b00001000

MOTOR_ROTATION_CW = 1
MOTOR_ROTATION_ACW = 2

NUM_SOCK = 5
RC_SILENCE_LIMIT = 10

#-------------------------------------------------------------------------------------------
# Set the BCM outputs assigned to LED and sensor interrupt
#-------------------------------------------------------------------------------------------
RPIO_STATUS_SOUNDER = 27
RPIO_DATA_READY_INTERRUPT = 25

silent_scan_count = 0

#-------------------------------------------------------------------------------------------
# Set up the base logging
#-------------------------------------------------------------------------------------------
logger = logging.getLogger('QC logger')
logger.setLevel(logging.INFO)

#-------------------------------------------------------------------------------------------
# Create file and console logger handlers
#-------------------------------------------------------------------------------------------
file_handler = logging.FileHandler("/dev/shm/qclogs", 'w')
file_handler.setLevel(logging.WARNING)

console_handler = logging.StreamHandler()
console_handler.setLevel(logging.CRITICAL)

#-------------------------------------------------------------------------------------------
# Create a formatter and add it to both handlers
#-------------------------------------------------------------------------------------------
console_formatter = logging.Formatter('%(message)s')
console_handler.setFormatter(console_formatter)

file_formatter = logging.Formatter('[%(levelname)s] (%(threadName)-10s) %(funcName)s %(lineno)d, %(message)s')
file_handler.setFormatter(file_formatter)

#-------------------------------------------------------------------------------------------
# Add both handlers to the logger
#-------------------------------------------------------------------------------------------
logger.addHandler(console_handler)
logger.addHandler(file_handler)

#-------------------------------------------------------------------------------------------
# Check the command line to see if we are calibrating or flying - if neither are set, CheckCLI sys.exit(0)s
#-------------------------------------------------------------------------------------------
calibrate_sensors, flying, hover_target, shoot_video, vvp_gain, vvi_gain, vvd_gain, hvp_gain, hvi_gain, hvd_gain, aap_gain, aai_gain, aad_gain, rrp_gain, rri_gain, rrd_gain, test_case, tau, dlpf, loop_frequency, motion_frequency, attitude_frequency, statistics, yaw_control = CheckCLI(sys.argv[1:])
logger.critical("calibrate_sensors = %s, fly = %s, hover_target = %d, shoot_video = %s, vvp_gain = %f, vvi_gain = %f, vvd_gain= %f, hvp_gain = %f, hvi_gain = %f, hvd_gain = %f, aap_gain = %f, aai_gain = %f, aad_gain = %f, rrp_gain = %f, rri_gain = %f, rrd_gain = %f, test_case = %d, tau = %f, dlpf = %d, loop_frequency = %d, motion_frequency = %d, attitude_frequency = %d, statistics = %s, yaw_control = %d", calibrate_sensors, flying, hover_target, shoot_video, vvp_gain, vvi_gain, vvd_gain, hvp_gain, hvi_gain, hvd_gain, aap_gain, aai_gain, aad_gain, rrp_gain, rri_gain, rrd_gain, test_case, tau, dlpf, loop_frequency, motion_frequency, attitude_frequency, statistics, yaw_control)

#-------------------------------------------------------------------------------------------
# Initialize the gyroscope / accelerometer I2C object
#-------------------------------------------------------------------------------------------
mpu6050 = MPU6050(0x68, dlpf)

#-------------------------------------------------------------------------------------------
# From hereonin we're in flight mode.  First task is to shut the ESCs up.
#-------------------------------------------------------------------------------------------
#-------------------------------------------------------------------------------------------
# Assign motor properties to each ESC
#-------------------------------------------------------------------------------------------
pin_list = [ESC_BCM_FL, ESC_BCM_FR, ESC_BCM_BL, ESC_BCM_BR]
location_list = [MOTOR_LOCATION_FRONT | MOTOR_LOCATION_LEFT, MOTOR_LOCATION_FRONT | MOTOR_LOCATION_RIGHT, MOTOR_LOCATION_BACK | MOTOR_LOCATION_LEFT, MOTOR_LOCATION_BACK | MOTOR_LOCATION_RIGHT]
rotation_list = [MOTOR_ROTATION_ACW, MOTOR_ROTATION_CW, MOTOR_ROTATION_CW, MOTOR_ROTATION_ACW]
name_list = ['front left', 'front right', 'back left', 'back right']

#-------------------------------------------------------------------------------------------
# Prime the ESCs with the default 0 spin rotors
#-------------------------------------------------------------------------------------------
esc_list = []
for esc_index in range(0, 4):
	esc = ESC(pin_list[esc_index], location_list[esc_index], rotation_list[esc_index], name_list[esc_index])
	esc_list.append(esc)

#-------------------------------------------------------------------------------------------
# Now with some peace and quiet, enable RPIO for beeper and MPU 6050 interrupts
#-------------------------------------------------------------------------------------------
RpioSetup()

#-------------------------------------------------------------------------------------------
# Countdown: 6 beeps prior to gyro calibration
#-------------------------------------------------------------------------------------------
CountdownBeep(6)

#-------------------------------------------------------------------------------------------
# Calibrate the sensors to build a trend line for sensor offsets against temperature
#-------------------------------------------------------------------------------------------
if calibrate_sensors:
	mpu6050.calibrateSensors("./qcoffsets.csv")
	sys.exit(0)

#-------------------------------------------------------------------------------------------
# Calibrate the gyros
#-------------------------------------------------------------------------------------------
mpu6050.calibrateGyros()

#-------------------------------------------------------------------------------------------
# Countdown: 5 beeps prior calculating take-off platform tilt
#-------------------------------------------------------------------------------------------
CountdownBeep(5)

#-------------------------------------------------------------------------------------------
# Prime the complementary angle filter with the take-off platform tilt
#-------------------------------------------------------------------------------------------
qax_average = 0.0
qay_average = 0.0
qaz_average = 0.0
for loop_count in range(0, 50, 1):
	qax, qay, qaz, qgx, qgy, qgz = mpu6050.readSensors()
	qax_average += qax
	qay_average += qay
	qaz_average += qaz
	time.sleep(0.05)
qax = qax_average / 50.0
qay = qay_average / 50.0
qaz = qaz_average / 50.0

prev_c_pitch, prev_c_roll, prev_c_tilt  = mpu6050.getEulerAngles(qax, qay, qaz, yaw_control)
logger.critical("Platform tilt: pitch %f, roll %f", prev_c_pitch * 180 / math.pi, prev_c_roll * 180 / math.pi)

#-------------------------------------------------------------------------------------------
# Prime the earth axis accelerometer values for accurate earth axis speed integration
#-------------------------------------------------------------------------------------------
eax, eay, eaz = ConvertAxes(qax, qay, qaz, prev_c_pitch, prev_c_roll, yaw_control)
eax_offset = eax
eay_offset = eay
eaz_offset = eaz

logger.critical("Platform motion: qax %f, qay %f, qaz %f, g %f", qax, qay, qaz, math.pow(math.pow(qax, 2) + math.pow(qay, 2) + math.pow(qaz, 2), 0.5))
logger.critical("Platform motion: eax %f, eay %f, eaz %f, g %f", eax, eay, eaz, math.pow(math.pow(eax, 2) + math.pow(eay, 2) + math.pow(1.0 + eaz, 2), 0.5))

#-------------------------------------------------------------------------------------------
# Preset the integrated gyro to match the take-off angle
#-------------------------------------------------------------------------------------------
i_pitch = prev_c_pitch
i_roll = prev_c_roll
i_yaw = 0.0

#-------------------------------------------------------------------------------------------
# Countdown: 4 beeps prior to waiting for RC connection
#-------------------------------------------------------------------------------------------
CountdownBeep(4)

#-------------------------------------------------------------------------------------------
# Countdown: 3 beeps for successful RC connection
#-------------------------------------------------------------------------------------------
CountdownBeep(3)

#---------------------------------------------------------------------------
# Set the signal handler here so the spin can be cancelled when loop_count = 0
# or moved on when > 0. Prior to this point Ctrl-C does what you'd expect
#---------------------------------------------------------------------------
loop_count = 0
signal.signal(signal.SIGINT, SignalHandler)

#-------------------------------------------------------------------------------------------
# Countdown: Start the video
#-------------------------------------------------------------------------------------------
CountdownBeep(2)

#-------------------------------------------------------------------------------------------
# Start up the video camera if required - this runs from take-off through to shutdown automatically.
# Run it in its own process group so that Ctrl-C for QC doesn't get through and stop the video
#-------------------------------------------------------------------------------------------
def Daemonize():
	os.setpgrp()

if shoot_video:
	now = datetime.now()
	now_string = now.strftime("%y%m%d-%H:%M:%S")
	video = subprocess.Popen(["raspivid", "-rot", "180", "-w", "1280", "-h", "720", "-o", "/home/pi/Videos/qcvid_" + now_string + ".h264", "-n", "-t", "0", "-fps", "30", "-b", "5000000"], preexec_fn =  Daemonize)

#-------------------------------------------------------------------------------------------
# Countdown: Get those blades spinning
#-------------------------------------------------------------------------------------------
CountdownBeep(1)

#------------------------------------------------------------------------------------------
# Set up the bits of state setup before takeoff
#-------------------------------------------------------------------------------------------
keep_looping = True
delta_time = 0.0

eax_average = 0.0
eay_average = 0.0
eaz_average = 0.0

evx = 0.0
evy = 0.0
evz = 0.0

evx_target = 0.0
evy_target = 0.0
evz_target = 0.0

evz_out = 0.0

pa_target = 0.0
ra_target = 0.0
ya_target = 0.0

pr_out = 0.0
rr_out = 0.0
yr_out = 0.0

evx_diags = "0.0, 0.0, 0.0"
evy_diags = "0.0, 0.0, 0.0"
evz_diags = "0.0, 0.0, 0.0"
pa_diags = "0.0, 0.0, 0.0"
ra_diags = "0.0, 0.0, 0.0"
ya_diags = "0.0, 0.0, 0.0"
pr_diags = "0.0, 0.0, 0.0"
rr_diags = "0.0, 0.0, 0.0"
yr_diags = "0.0, 0.0, 0.0"

#-------------------------------------------------------------------------------------------
# Set up the flight plan FSM.
#-------------------------------------------------------------------------------------------
FSM_INPUT_NONE = 0
FSM_INPUT_START = 1
FSM_INPUT_STOP = 2
FSM_INPUT_UPDATE = 3
fsm_input = FSM_INPUT_START

FSM_STATE_OFF = 0
FSM_STATE_UPDATING = 1
FSM_STATE_STABLE = 2
fsm_state = FSM_STATE_OFF

FSM_UPDATE_PERIOD = 0.5
update_start = 0.0

ready_to_fly = False
hover_speed = 0
vert_out = 0

#-------------------------------------------------------------------------------------------
# Set up the steps of the flight plan.
#-------------------------------------------------------------------------------------------
fp_evx_target  = [       0.0,       0.0,       0.0,       0.0,       0.0]
fp_evy_target  = [       0.0,       0.0,       0.0,       0.0,       0.0]
fp_evz_target  = [       0.0,       0.3,       0.0,      -0.3,       0.0]
fp_time        = [       1.0,       3.0,       3.0,       3.0,       3.0]
fp_name        = [   "HOVER",  "ASCENT",   "HOVER", "DESCENT",     "OFF"]
FP_STEPS = 5
fp_index = 0
fp_total_time = 0.0

#-------------------------------------------------------------------------------------------
# START TESTCASE 1 CODE: spin up each blade individually for 10s each and check they all turn the right way
#-------------------------------------------------------------------------------------------
if test_case == 1:
	for esc in esc_list:
		for count in range(0, hover_target, 10):
			#-------------------------------------------------------------------
			# Spin up to user determined (-h) hover speeds ~200
			#-------------------------------------------------------------------
			esc.update(count)
			time.sleep(0.01)
		time.sleep(10.0)
		esc.update(0)
	CleanShutdown()
#-------------------------------------------------------------------------------------------
# END TESTCASE 1 CODE: spin up each blade individually for 10s each and check they all turn the right way
#-------------------------------------------------------------------------------------------

#===========================================================================================
# Tuning: Set up the PID gains - some are hard coded mathematical approximations, some come
# from the CLI parameters to allow for tuning
#===========================================================================================
# A PID is a simple algorithm which takes a desired state of a system (the "target", perhaps from a remote control)
# and the current state (the "input" or "feedback", perhaps from a sensor), subtracts them to determine the "error" in
# the system, and applies some simple math(s) to come up with a corrective "output". It does this repeatedly with the aim
# that the "error" reduces and is maintained at near zero due to the "feedback".
#
# This mechanism allows complex systems to be broken down and corrected even when a complete mathematical model of the
# system is too complicated or external factors mean the system cannot be modelled directly.
#
# In a quadcopter, the external targets (those a user control for flight) are horizontal and vertical speed plus tilt.
#
# - Horizontal speed error is corrected proportionally by providing the corrective output as the horizontal acceleration target
# - Horizontal acceleration can be converted directly to an angle of tilt using trigonometry, so the horizontal acceleration target is
#   converted to the pitch / roll angle target.
# - Tilt angle error is corrected proportionally by providing the corrective output to the angular speed PID target
# - Angular speed error is corrected proportionally by providing the corrective output to the motors' ESCs
#
# - Horizontal speed feedback is provided by integrated accelerometer (or GPS in future)
# - Angular feedback comes from accelerometer Euler and integrated gyro sensors passed through a complementary filter to track theta
# - Angular speed feedback comes directly from the gyros
#
# That's 3 PIDs each for horizontal X & Y axes movement - 6 in total so far.
#
# - Vertical speed error is corrected proportionally by providing the corrective output to the motors' ESCs (cf. angular speed above)
#
# - Vertical speed feedback comes from the Z-axis accelerometer integrated over time, and compensated for any tilt (cos(theta)cos(phi))
#
# That's 1 PID for vertical Z axis speed control.
#
# - Yaw speed error is corrected proportionally by providing the corrective output to the motors' ESCs
#
# - Yaw speed feedback comes directly from the Z-axis gyro
#
# That's 1 PID for Z axis yaw control.
#
# So 8 PIDs in total
#===========================================================================================

#-------------------------------------------------------------------------------------------
# The earth X axis speed controls forward / backward speed
#-------------------------------------------------------------------------------------------
PID_EVX_P_GAIN = hvp_gain
PID_EVX_I_GAIN = hvi_gain
PID_EVX_D_GAIN = hvd_gain	

#-------------------------------------------------------------------------------------------
# The earth Y axis speed controls left / right speed
#-------------------------------------------------------------------------------------------
PID_EVY_P_GAIN = hvp_gain
PID_EVY_I_GAIN = hvi_gain
PID_EVY_D_GAIN = hvd_gain	

#-------------------------------------------------------------------------------------------
# The earth Z axis speed controls rise / fall speed
#-------------------------------------------------------------------------------------------
PID_EVZ_P_GAIN = vvp_gain
PID_EVZ_I_GAIN = vvi_gain
PID_EVZ_D_GAIN = vvd_gain

#-------------------------------------------------------------------------------------------
# The PITCH ANGLE PID maintains a stable tilt angle about the Y-axis
#-------------------------------------------------------------------------------------------
PID_PA_P_GAIN = aap_gain
PID_PA_I_GAIN = aai_gain
PID_PA_D_GAIN = aad_gain

#-------------------------------------------------------------------------------------------
# The ROLL ANGLE PID maintains a stable tilt angle about the X-axis
#-------------------------------------------------------------------------------------------
PID_RA_P_GAIN = aap_gain
PID_RA_I_GAIN = aai_gain
PID_RA_D_GAIN = aad_gain

#-------------------------------------------------------------------------------------------
# The YAW ANGLE PID maintains a stable tilt angle about the Z-axis
#-------------------------------------------------------------------------------------------
PID_YA_P_GAIN = 0.0 # 2.5
PID_YA_I_GAIN = 0.0 # 5.0
PID_YA_D_GAIN = 0.0

#-------------------------------------------------------------------------------------------
# The PITCH RATE PID controls stable rotation rate around the Y-axis
#-------------------------------------------------------------------------------------------
PID_PR_P_GAIN = rrp_gain
PID_PR_I_GAIN = rri_gain
PID_PR_D_GAIN = rrd_gain

#-------------------------------------------------------------------------------------------
# The ROLL RATE PID controls stable rotation rate around the X-axis
#-------------------------------------------------------------------------------------------
PID_RR_P_GAIN = rrp_gain
PID_RR_I_GAIN = rri_gain
PID_RR_D_GAIN = rrd_gain

#-------------------------------------------------------------------------------------------
# The YAW RATE PID controls stable rotation speed around the Z-axis
#-------------------------------------------------------------------------------------------
PID_YR_P_GAIN = 0 # rrp_gain / 2.5
PID_YR_I_GAIN = 0 # rri_gain / 2.5
PID_YR_D_GAIN = 0 # rrd_gain / 2.5

#-------------------------------------------------------------------------------------------
# Diagnostic statistics log header
#-------------------------------------------------------------------------------------------
if statistics:
	logger.warning('Time, DT, Loop, evz_target, qgx, qgy, qgz, qax, qay, qaz, eax, eay, eaz, evx, evy, evz, i pitch, i roll, e pitch, e roll, c pitch, c roll, i yaw, e tilt, exp, exi, exd, pa_target, pap, pai, pad, prp, pri, prd, pr_out, eyp, eyi, eyd, ra_target, rap, rai, rad, rrp, rri, rrd, rr_out, ezp, ezi, ezd, evz_out, yap, yai, yap, yrp, yri, yrd, yr_out, FL spin, FR spin, BL spin, BR spin')

time_handling_fsm = 0.0
time_handling_sensors = 0.0
time_handling_eangles = 0.0
time_handling_iangles = 0.0
time_handling_angles_filter = 0.0
time_handling_axes_shift = 0.0
time_handling_motion_pids = 0.0
time_handling_attitude_pids = 0.0
time_handling_pid_outputs = 0.0
time_handling_diagnostics = 0.0
time_handling_sleep = 0.0

############################################################################################
# Enable time dependent factors PIDs - everything beyond here and "while keep_looping:" is time
# critical and should be kept to an absolute minimum.
############################################################################################


#-------------------------------------------------------------------------------------------
# Start the pitch, roll and yaw angle PIDs
#-------------------------------------------------------------------------------------------
pa_pid = PID(PID_PA_P_GAIN, PID_PA_I_GAIN, PID_PA_D_GAIN)
ra_pid = PID(PID_RA_P_GAIN, PID_RA_I_GAIN, PID_RA_D_GAIN)
ya_pid = PID(PID_YA_P_GAIN, PID_YA_I_GAIN, PID_YA_D_GAIN)

#-------------------------------------------------------------------------------------------
# Start the pitch, roll and yaw rate PIDs
#-------------------------------------------------------------------------------------------
pr_pid = PID(PID_PR_P_GAIN, PID_PR_I_GAIN, PID_PR_D_GAIN)
rr_pid = PID(PID_RR_P_GAIN, PID_RR_I_GAIN, PID_RR_D_GAIN)
yr_pid = PID(PID_YR_P_GAIN, PID_YR_I_GAIN, PID_YR_D_GAIN)

#-------------------------------------------------------------------------------------------
# Start the X, Y (horizontal) and Z (vertical) velocity PIDs
#-------------------------------------------------------------------------------------------
evx_pid = PID(PID_EVX_P_GAIN, PID_EVX_I_GAIN, PID_EVX_D_GAIN)
evy_pid = PID(PID_EVY_P_GAIN, PID_EVY_I_GAIN, PID_EVY_D_GAIN)
evz_pid = PID(PID_EVZ_P_GAIN, PID_EVZ_I_GAIN, PID_EVZ_D_GAIN)

rtf_time = 0.0
elapsed_time = 0.0
start_time = time.time()
last_log_time = start_time
current_time = start_time
prev_sample_time = current_time
last_motion_update = current_time
last_attitude_update = current_time
update_PWM = False
ea_averaging_start = current_time

while keep_looping:
	#-----------------------------------------------------------------------------------
	# Update the elapsed time since start, the time for the last iteration, and
	# set the next sleep time to compensate for any overrun in scheduling.
	#-----------------------------------------------------------------------------------
	current_time = time.time()
	delta_time = current_time - start_time - elapsed_time
	elapsed_time = current_time - start_time
	loop_count += 1

	#===================================================================================
	# Interpreter: FSM inputs are mostly generated on a timer for testing; the exceptions are
	# - SIGNAL generated by a Ctrl-C.  These produce the targets for the PIDs.  In this
	# case, only the vertical speed target is modified - the horizontal X and Y speed targets
	# are configured higher up to be 0.0 to create a stable hover regardless of take-off conditions
	# of any external factors such as wind or weight balance.
	#===================================================================================
	if not ready_to_fly:
#		logger.critical('-> RTF')
		fsm_input = FSM_INPUT_START

	elif fsm_state != FSM_STATE_UPDATING and (elapsed_time - rtf_time >= fp_total_time + fp_time[fp_index]):
			logger.critical('-> %s', fp_name[fp_index])
			fsm_input = FSM_INPUT_UPDATE
			next_evx_target = fp_evx_target[fp_index]
			next_evy_target = fp_evy_target[fp_index]
			next_evz_target = fp_evz_target[fp_index]
			fp_total_time += fp_time[fp_index]
			fp_index += 1
			if fp_index == FP_STEPS:
				fsm_input = FSM_INPUT_STOP

	#-----------------------------------------------------------------------------------
	# Now we've been told what to do, do it
	#-----------------------------------------------------------------------------------
	if fsm_state == FSM_STATE_OFF and fsm_input == FSM_INPUT_START:
		if hover_speed >= hover_target:
			rtf_time = elapsed_time
			ready_to_fly = True
			hover_speed = hover_target
			fsm_state = FSM_STATE_STABLE

		elif elapsed_time - rtf_time > 0.05:
			rtf_time += 0.05
			hover_speed += 25

	if fsm_state == FSM_STATE_STABLE and fsm_input == FSM_INPUT_UPDATE:
		fsm_state = FSM_STATE_UPDATING
		next_fsm_state = FSM_STATE_STABLE
		fsm_input = FSM_INPUT_NONE

		#---------------------AUTONOMOUS VERTICAL TAKE-OFF SPEED--------------------
		prev_evx_target = evx_target
		prev_evy_target = evy_target
		prev_evz_target = evz_target
		#---------------------AUTONOMOUS VERTICAL TAKE-OFF SPEED--------------------

	if fsm_state == FSM_STATE_UPDATING and fsm_input == FSM_INPUT_NONE:
		if update_start == 0.0:
			update_start = current_time

		update_fraction = (time.time() - update_start) / FSM_UPDATE_PERIOD

#		#-----------------------AUTONOMOUS BINARY TRANSITION------------------------
#		evz_target =  next_evz_target
#		#-----------------------AUTONOMOUS BINARY TRANSITION------------------------


#		#-----------------------AUTONOMOUS LINEAR TRANSITION------------------------
#		evz_target = prev_evx_target + update_fraction * (next_evz_target - prev_evz_target)
#		#-----------------------AUTONOMOUS LINEAR TRANSITION------------------------

		#-----------------------AUTONOMOUS SINUSOIDAL TRANSITION--------------------
		signed_one = math.copysign(1.0, next_evz_target - prev_evz_target)
		evz_target =  prev_evz_target + signed_one * 0.5 * (next_evz_target - prev_evz_target) * (signed_one + math.sin(signed_one * (2 * update_fraction - 1) * math.pi / 2))
		#-----------------------AUTONOMOUS SINUSOIDAL TRANSITION--------------------

		if update_fraction >= 1.00:
			evz_target = next_evz_target
			fsm_state = FSM_STATE_STABLE
			fsm_input = FSM_INPUT_NONE
			update_start = 0.0

	if fsm_state == FSM_STATE_UPDATING and fsm_input == FSM_INPUT_UPDATE:

		#---------------------AUTONOMOUS VERTICAL TAKE-OFF SPEED--------------------
		evz_target = next_evz_target
		#---------------------AUTONOMOUS VERTICAL TAKE-OFF SPEED--------------------
		fsm_state = FSM_STATE_STABLE
		fsm_input = FSM_INPUT_NONE
		update_start = 0.0

	if fsm_input == FSM_INPUT_STOP:
		keep_looping = False
		hover_speed = 0

	#-----------------------------------------------------------------------------------
	# Track proportion of time handling FSM
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_fsm += sample_time - prev_sample_time
	prev_sample_time = sample_time

	#===================================================================================
	# Inputs: Read the data from the accelerometer and gyro
	#===================================================================================
	qax, qay, qaz, qgx, qgy, qgz = mpu6050.readSensors()

	#-----------------------------------------------------------------------------------
	# Track proportion of time handling sensors
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_sensors += sample_time - prev_sample_time
	prev_sample_time = sample_time

	#===================================================================================
	# Angles: Get the Euler angles in radians
	#===================================================================================
	e_pitch, e_roll, e_tilt  = mpu6050.getEulerAngles(qax, qay, qaz, yaw_control)

	#-----------------------------------------------------------------------------------
	# Track proportion of time handling euler angles
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_eangles += sample_time - prev_sample_time
	prev_sample_time = sample_time

	#-----------------------------------------------------------------------------------
	# Integrate the gyros angular velocity to determine absolute angle of tilt in radians
	# Note that this is for diagnostic purposes only.
	#-----------------------------------------------------------------------------------
	i_pitch += qgy * delta_time
	i_roll += qgx * delta_time
	i_yaw += qgz * delta_time

	#-----------------------------------------------------------------------------------
	# Track proportion of time handling integrated angles
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_iangles += sample_time - prev_sample_time
	prev_sample_time = sample_time

	#===================================================================================
	# Filter: Apply complementary filter to ensure long-term accuracy of pitch / roll angles
	# 1/tau is the handover frequency that the integrated gyro high pass filter is taken over
	# by the accelerometer Euler low-pass filter providing fast reaction to change from the
	# gyro yet with low noise accurate Euler angles from the acclerometer.
	#
	# The combination of tau plus the time increment (delta_time) provides a fraction to mix
	# the two angles sources.
	#===================================================================================
	tau_fraction = tau / (tau + delta_time)

	c_pitch = tau_fraction * (prev_c_pitch + qgy * delta_time) + (1 - tau_fraction) * e_pitch
	prev_c_pitch = c_pitch

	c_roll = tau_fraction * (prev_c_roll + qgx * delta_time) + (1 - tau_fraction) * e_roll
	prev_c_roll = c_roll

	#-----------------------------------------------------------------------------------
	# Choose the best measure of the angles
	#-----------------------------------------------------------------------------------
	pa = c_pitch
	ra = c_roll
	ya = i_yaw

	#-----------------------------------------------------------------------------------
	# Track proportion of time handling angle filter
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_angles_filter += sample_time - prev_sample_time
	prev_sample_time = sample_time

	#-----------------------------------------------------------------------------------
	# Convert quad orientated axes accelerometer reading to earth orientated axes
	#-----------------------------------------------------------------------------------
	eax, eay, eaz = ConvertAxes(qax, qay, qaz, c_pitch, c_roll, yaw_control)

	#-----------------------------------------------------------------------------------
	# Integrate to earth axes' velocities
	#-----------------------------------------------------------------------------------
	evx += (eax - eax_offset) * delta_time * G_FORCE
	evy += (eay - eay_offset) * delta_time * G_FORCE
	evz += (eaz - eaz_offset) * delta_time * G_FORCE

	#-----------------------------------------------------------------------------------
	# Integrate out the accelerometer noise across the time between motion PID updates
	#-----------------------------------------------------------------------------------
	eax_average += (eax - eax_offset) * delta_time
	eay_average += (eay - eay_offset) * delta_time
	eaz_average += (eaz - eaz_offset) * delta_time

	#-----------------------------------------------------------------------------------
	# Track proportion of time handling sensor angles
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_axes_shift += sample_time - prev_sample_time
	prev_sample_time = sample_time

	#-----------------------------------------------------------------------------------
	# The attitude PID targets are updated with new motion PID outputs at 31Hz.
	# The attitude PID outputs are updated every 100Hz.
	#-----------------------------------------------------------------------------------
	if current_time - last_motion_update >= 1/motion_frequency:
		last_motion_update -= current_time

		#===========================================================================
		# Motion PIDs: Run the horizontal speed PIDs each rotation axis to determine
		# targets for absolute angle PIDs and the verical speed PID to control height.
		#===========================================================================
		[p_out, i_out, d_out] = evx_pid.Compute(evx, evx_target)
		evx_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		evx_out = p_out + i_out + d_out

		[p_out, i_out, d_out] = evy_pid.Compute(evy, evy_target)
		evy_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		evy_out =  p_out + i_out + d_out

		[p_out, i_out, d_out] = evz_pid.Compute(evz, evz_target)
		evz_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		evz_out = p_out + i_out + d_out

		#---------------------------------------------------------------------------
		# Work out the earth axis acceleration averages
		#---------------------------------------------------------------------------
		eax_average /= (current_time - ea_averaging_start)
		eay_average /= (current_time - ea_averaging_start)
		eaz_average /= (current_time - ea_averaging_start)

		#---------------------------------------------------------------------------
		# Convert the horizontal velocity PID output i.e. the horizontal acceleration
		# target in q's into the pitch and roll angle PID targets in radians
		#---------------------------------------------------------------------------
		if not yaw_control:
			pa_target = -math.atan2(evx_out, 1.0 + eaz_average)
			ra_target = -math.atan2(evy_out, 1.0 + eaz_average)
		else:
			pa_target = -math.atan2(evx_out, math.pow(math.pow(eay_average, 2) + math.pow(1.0 + eaz_average, 2), 0.5))
			ra_target = -math.atan2(evy_out, math.pow(math.pow(eax_average, 2) + math.pow(1.0 + eaz_average, 2), 0.5))

		#---------------------------------------------------------------------------
		# Restart integrating out accelerometer noise
		#---------------------------------------------------------------------------
		ea_averaging_start = current_time
		eax_average = 0.0
		eay_average = 0.0
		eaz_average = 0.0

		#---------------------------------------------------------------------------
		# Convert the vertical velocity PID output direct to PWM pulse width.
		#---------------------------------------------------------------------------
		vert_out = hover_speed + int(round(evz_out))

		#---------------------------------------------------------------------------
		# Track proportion of time handling speed PIDs
		#---------------------------------------------------------------------------
		sample_time = time.time()
		time_handling_motion_pids += sample_time - prev_sample_time
		prev_sample_time = sample_time


	if current_time - last_attitude_update >= 1/attitude_frequency:
		last_attitude_update -= current_time

		#===========================================================================
		# Attitude PIDs: Run the absolute and and rotoation rate PIDs each rotation
		# axis to determine overall PWM output.
		#===========================================================================
		[p_out, i_out, d_out] = pa_pid.Compute(pa, pa_target)
		pa_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		pr_target = p_out + i_out + d_out
		[p_out, i_out, d_out] = ra_pid.Compute(ra, ra_target)
		ra_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		rr_target = p_out + i_out + d_out
		[p_out, i_out, d_out] = ya_pid.Compute(ya, ya_target)
		ya_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		yr_target = p_out + i_out + d_out

		[p_out, i_out, d_out] = pr_pid.Compute(qgy, pr_target)
		pr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		pr_out = p_out + i_out + d_out
		[p_out, i_out, d_out] = rr_pid.Compute(qgx, rr_target)
		rr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		rr_out = p_out + i_out + d_out
		[p_out, i_out, d_out] = yr_pid.Compute(qgz, yr_target)
		yr_diags = "%f, %f, %f" % (p_out, i_out, d_out)
		yr_out = p_out + i_out + d_out

		#---------------------------------------------------------------------------
		# Convert the rotation rate PID outputs direct to PWM pulse width
		#---------------------------------------------------------------------------
		pr_out = int(round(pr_out / 2))
		rr_out = int(round(rr_out / 2))
		yr_out = int(round(yr_out / 2))

		#---------------------------------------------------------------------------
		# START TESTCASE 3 CODE: Disable yaw
		#---------------------------------------------------------------------------
		if test_case == 3:
			yr_out = 0
		#---------------------------------------------------------------------------
		# END TESTCASE 3 CODE: Disable front-left and back right blades
		#---------------------------------------------------------------------------

		#---------------------------------------------------------------------------
		# Only update the PWM if there's something worth updating.
		#---------------------------------------------------------------------------
		update_PWM = True

		#---------------------------------------------------------------------------
		# Track proportion of time handling angle PIDs
		#---------------------------------------------------------------------------
		sample_time = time.time()
		time_handling_attitude_pids += sample_time - prev_sample_time
		prev_sample_time = sample_time


	#-----------------------------------------------------------------------------------
	# Only update the PWM if there's something worth updating.
	#-----------------------------------------------------------------------------------
	if update_PWM:
		update_PWM = False


		#===========================================================================
		# Mixer: Walk through the ESCs, and depending on their location, apply the output accordingly
		#===========================================================================
		for esc in esc_list:
			#-------------------------------------------------------------------
			# Update all blades' power in accordance with the z error
			#-------------------------------------------------------------------
			delta_spin = vert_out

			#-------------------------------------------------------------------
			# For a left downwards roll, the x gyro goes negative, so the PID error is positive,
			# meaning PID output is positive, meaning this needs to be added to the left blades
			# and subtracted from the right.
			#-------------------------------------------------------------------
			if esc.motor_location & MOTOR_LOCATION_RIGHT:
				delta_spin -= rr_out
			else:
				delta_spin += rr_out

			#-------------------------------------------------------------------
			# For a forward downwards pitch, the y gyro goes negative, so the PID error is
			# postive, meaning PID output is positive, meaning this needs to be added to the
			# front blades and subreacted from the back.
			#-------------------------------------------------------------------
			if esc.motor_location & MOTOR_LOCATION_BACK:
				delta_spin -= pr_out
			else:
				delta_spin += pr_out

			#-------------------------------------------------------------------
			# An excess CW rotating of the front-right and back-left (FR & BL) blades
			# results in an CW rotation of the quadcopter body. The z gyro produces
			# a negative output as a result. This then leads to the PID error
			# being postive, meaning PID  output is positive. Since the PID output needs to reduce the
			# over-enthusiastic CW rotation of the FR & BL blades, the positive PID
			# output needs to be subtracted from those blades (thus slowing their rotation)
			# and added to the ACW FL & BR blades (thus speeding them up) to
			# compensate for the yaw.
			#-------------------------------------------------------------------
			if esc.motor_rotation == MOTOR_ROTATION_ACW:
				delta_spin -= yr_out
			else:
				delta_spin += yr_out

			#-------------------------------------------------------------------
			# Apply the blended outputs to the esc PWM signal
			#-------------------------------------------------------------------
			esc.update(delta_spin)

			#-------------------------------------------------------------------
			# Track proportion of time applying PWM outputs
			#-------------------------------------------------------------------
			sample_time = time.time()
			time_handling_pid_outputs += sample_time - prev_sample_time
			prev_sample_time = sample_time

	#-----------------------------------------------------------------------------------
	# Diagnostic statistics log - every 0.1s
	#-----------------------------------------------------------------------------------
	if statistics:
		logger.warning('%f, %f, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %s, %f, %s, %s, %f, %s, %f, %s, %s, %f, %s, %f, %s, %s, %f, %d, %d, %d, %d', elapsed_time, delta_time, loop_count, evz_target,qgx, qgy, qgz, qax, qay, qaz, eax, eay, eaz, evx, evy, evz, math.degrees(i_pitch), math.degrees(i_roll), math.degrees(e_pitch), math.degrees(e_roll), math.degrees(c_pitch), math.degrees(c_roll), math.degrees(i_yaw), math.degrees(e_tilt), evx_diags, pa_target, pa_diags, pr_diags, pr_out, evy_diags, ra_target, ra_diags, rr_diags, rr_out, evz_diags, evz_out, ya_diags, yr_diags, yr_out, esc_list[0].current_pulse_width, esc_list[1].current_pulse_width, esc_list[2].current_pulse_width, esc_list[3].current_pulse_width)

	#-----------------------------------------------------------------------------------
	# Track proportion of time logging diagnostics
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_diagnostics += sample_time - prev_sample_time
	prev_sample_time = sample_time

	#-----------------------------------------------------------------------------------
	# Slow down the scheduling loop to avoid making accelerometer noise.  This sleep critically
	# takes place between the update of the PWM and reading the sensors, so that any
	# PWM changes can stabilize (i.e. spikes reacted to) prior to reading the sensors.
	#-----------------------------------------------------------------------------------
	loop_time = time.time() - current_time
	sleep_time = 1 / loop_frequency - loop_time
	if sleep_time < 0.0:
		sleep_time = 0.0
	time.sleep(sleep_time)

	#-----------------------------------------------------------------------------------
	# Track proportion of time sleeping
	#-----------------------------------------------------------------------------------
	sample_time = time.time()
	time_handling_sleep += sample_time - prev_sample_time
	prev_sample_time = sample_time

#-------------------------------------------------------------------------------------------
# Dump the loops per second
#-------------------------------------------------------------------------------------------
logger.critical("loop speed %f loops per second", loop_count / elapsed_time)

#-------------------------------------------------------------------------------------------
# Dump the percentage time handling each step
#-------------------------------------------------------------------------------------------
logger.critical("%% fsm:              %f", time_handling_fsm / elapsed_time * 100.0)
logger.critical("%% sensors:          %f", time_handling_sensors / elapsed_time * 100.0)
logger.critical("%% eangles:          %f", time_handling_eangles / elapsed_time * 100.0)
logger.critical("%% iangles:          %f", time_handling_iangles / elapsed_time * 100.0)
logger.critical("%% angles_filter:    %f", time_handling_angles_filter / elapsed_time * 100.0)
logger.critical("%% axes_shift:       %f", time_handling_axes_shift / elapsed_time * 100.0)
logger.critical("%% motion_pids:      %f", time_handling_motion_pids / elapsed_time * 100.0)
logger.critical("%% attitude_pids:    %f", time_handling_attitude_pids / elapsed_time * 100.0)
logger.critical("%% pid_outputs:      %f", time_handling_pid_outputs / elapsed_time * 100.0)
logger.critical("%% pid_diagnosticss: %f", time_handling_diagnostics / elapsed_time * 100.0)
logger.critical("%% sleep:            %f", time_handling_sleep / elapsed_time * 100.0)

mpu6050_misses, i2c_misses = mpu6050.getMisses()
logger.critical("mpu6050 %d misses, i2c %d misses", mpu6050_misses, i2c_misses)

#-------------------------------------------------------------------------------------------
# Time for telly bye byes
#-------------------------------------------------------------------------------------------
CleanShutdown()
