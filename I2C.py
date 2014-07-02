import logging
import smbus

###########################################################################################
#
#  Adafruit i2c interface enhanced with performance / error handling enhancements
#
############################################################################################
class I2C:

	def __init__(self, address, bus=smbus.SMBus(1)):
		self.address = address
		self.bus = bus
		self.misses = 0

	def reverseByteOrder(self, data):
		"Reverses the byte order of an int (16-bit) or long (32-bit) value"
		# Courtesy Vishal Sapre
		dstr = hex(data)[2:].replace('L','')
		byteCount = len(dstr[::2])
		val = 0
		for i, n in enumerate(range(byteCount)):
			d = data & 0xFF
			val |= (d << (8 * (byteCount - i - 1)))
			data >>= 8
		return val

	def write8(self, reg, value):
		"Writes an 8-bit value to the specified register/address"
		while True:
			try:
				self.bus.write_byte_data(self.address, reg, value)
				logger.debug('I2C: Wrote 0x%02X to register 0x%02X', value, reg)
				break
			except IOError, err:
				self.misses += 1
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.0001)

	def writeList(self, reg, list):
		"Writes an array of bytes using I2C format"
		while True:
			try:
				self.bus.write_i2c_block_data(self.address, reg, list)
				break
			except IOError, err:
				self.misses += 1
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.0001)

	def readU8(self, reg):
		"Read an unsigned byte from the I2C device"
		while True:
			try:
				result = self.bus.read_byte_data(self.address, reg)
				logger.debug('I2C: Device 0x%02X returned 0x%02X from reg 0x%02X', self.address, result & 0xFF, reg)
				return result
			except IOError, err:
				self.misses += 1
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.0001)

	def readS8(self, reg):
		"Reads a signed byte from the I2C device"
		while True:
			try:
				result = self.bus.read_byte_data(self.address, reg)
				logger.debug('I2C: Device 0x%02X returned 0x%02X from reg 0x%02X', self.address, result & 0xFF, reg)
				if (result > 127):
					return result - 256
				else:
					return result
			except IOError, err:
				self.misses += 1
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.0001)

	def readU16(self, reg):
		"Reads an unsigned 16-bit value from the I2C device"
		while True:
			try:
				hibyte = self.bus.read_byte_data(self.address, reg)
				result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg+1)
				logger.debug('I2C: Device 0x%02X returned 0x%04X from reg 0x%02X', self.address, result & 0xFFFF, reg)
				if result == 0x7FFF or result == 0x8000:
					logger.critical('I2C read max value')
					time.sleep(0.0005)
				else:
					return result
			except IOError, err:
				self.misses += 1
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.0001)

	def readS16(self, reg):
		"Reads a signed 16-bit value from the I2C device"
		while True:
			try:
				hibyte = self.bus.read_byte_data(self.address, reg)
				if (hibyte > 127):
					hibyte -= 256
				result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg+1)
				logger.debug('I2C: Device 0x%02X returned 0x%04X from reg 0x%02X', self.address, result & 0xFFFF, reg)
				if result == 0x7FFF or result == 0x8000:
					logger.critical('I2C read max value')
					time.sleep(0.0005)
				else:
					return result
			except IOError, err:
				self.misses += 1
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.0001)
				
	def readList(self, reg, length):
		"Reads a a byte array value from the I2C device"
		while True:
			try:
				result = self.bus.read_i2c_block_data(self.address, reg, length)
				logger.debug('I2C: Device 0x%02X from reg 0x%02X', self.address, reg)
				return result
			except IOError, err:
				self.misses += 1
				logger.exception('Error %d, %s accessing 0x%02X: Check your I2C address', err.errno, err.strerror, self.address)
				time.sleep(0.0001)

	def getMisses(self):
		return self.misses

