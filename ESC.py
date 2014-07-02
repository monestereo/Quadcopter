import RPIO
from RPIO import PWM

############################################################################################
#
#  Class for managing each blade + motor configuration via its ESC
#
############################################################################################
class ESC:
	pwm = None

	def __init__(self, pin, location, rotation, name):

		#---------------------------------------------------------------------------
		# The GPIO BCM numbered pin providing PWM signal for this ESC
		#---------------------------------------------------------------------------
		self.bcm_pin = pin

		#---------------------------------------------------------------------------
		# The location on the quad, and the direction of the motor controlled by this ESC
		#---------------------------------------------------------------------------
		self.motor_location = location
		self.motor_rotation = rotation

		#---------------------------------------------------------------------------
		# The PWM pulse width range required by this ESC in microseconds
		#---------------------------------------------------------------------------
		self.min_pulse_width = 1000
		self.max_pulse_width = 2000

		#---------------------------------------------------------------------------
		# The PWM pulse range required by this ESC
		#---------------------------------------------------------------------------
		self.current_pulse_width = self.min_pulse_width
		self.name = name

		#---------------------------------------------------------------------------
		# Initialize the RPIO DMA PWM
		#---------------------------------------------------------------------------
		if not PWM.is_setup():
			PWM.set_loglevel(PWM.LOG_LEVEL_ERRORS)
			PWM.setup(1)    # 1us increment
			PWM.init_channel(RPIO_DMA_CHANNEL, 3000) # 3ms carrier period
		PWM.add_channel_pulse(RPIO_DMA_CHANNEL, self.bcm_pin, 0, self.current_pulse_width)


	def update(self, spin_rate):
		self.current_pulse_width = int(self.min_pulse_width + spin_rate)

		if self.current_pulse_width < self.min_pulse_width:
			self.current_pulse_width = self.min_pulse_width
		if self.current_pulse_width > self.max_pulse_width:
			self.current_pulse_width = self.max_pulse_width

		PWM.add_channel_pulse(RPIO_DMA_CHANNEL, self.bcm_pin, 0, self.current_pulse_width)
