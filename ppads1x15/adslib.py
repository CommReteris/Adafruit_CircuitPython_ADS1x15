# The MIT License (MIT)
#
# Copyright (c) 2018 Carter Nelson for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`adslib`
====================================================

Pigpio base class driver for ADS1015/1115 ADCs.

* Author(s): Lorenzo Seirup; based on the work of Carter Nelson
"""

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/CommReteris/pigpio_ads1x15.git"

#from micropython import const
#from adafruit_bus_device.i2c_device import I2CDevice
from micropython import const
import pigpio as pp
import struct
from time import sleep

_ADS1X15_DEFAULT_ADDRESS = const(0x48)
_ADS1X15_POINTER_CONVERSION = const(0x00)
_ADS1X15_POINTER_CONFIG = const(0x01)
_ADS1X15_CONFIG_OS_SINGLE = const(0x8000)
_ADS1X15_CONFIG_MUX_OFFSET = const(12)
_ADS1X15_CONFIG_COMP_QUE_DISABLE = const(0x0003)
_ADS1X15_CONFIG_GAIN = {
    2 / 3: 0x0000,
    1: 0x0200,
    2: 0x0400,
    4: 0x0600,
    8: 0x0800,
    16: 0x0A00,
}

# Data sample rates
_ADS1015_CONFIG_DR = {
    128: 0x0000,
    250: 0x0020,
    490: 0x0040,
    920: 0x0060,
    1600: 0x0080,
    2400: 0x00A0,
    3300: 0x00C0,
}
_ADS1115_CONFIG_DR = {
    8: 0x0000,
    16: 0x0020,
    32: 0x0040,
    64: 0x0060,
    128: 0x0080,
    250: 0x00A0,
    475: 0x00C0,
    860: 0x00E0,
}

# Channels
_ADS1X15_DIFF_CHANNELS = {(0, 1): 0, (0, 3): 1, (1, 3): 2, (2, 3): 3}
_ADS1X15_PGA_RANGE = {2 / 3: 6.144, 1: 4.096, 2: 2.048, 4: 1.024, 8: 0.512, 16: 0.256}

""" Channel definitions (dev use only):
in0: Voltage over AIN0 and AIN1.
in1: Voltage over AIN0 and AIN3.
in2: Voltage over AIN1 and AIN3.
in3: Voltage over AIN2 and AIN3.
in4: Voltage over AIN0 and GND.
in5: Voltage over AIN1 and GND.
in6: Voltage over AIN2 and GND.
in7: Voltage over AIN3 and GND.
"""

class Mode:
    """An enum-like class representing possible ADC operating modes."""

    # See datasheet "Operating Modes" section
    # values here are masks for setting MODE bit in Config Register
    # pylint: disable=too-few-public-methods
    CONTINUOUS = 0x0000
    SINGLE = 0x0100


class ADS1x15:
    """Base functionality for ADS1x15 analog to digital converters."""

    def __init__(
        self,
        pig,
        gain=1,
        data_rate=None,
        mode=Mode.SINGLE,
        address=_ADS1X15_DEFAULT_ADDRESS,
    ):
        # pylint: disable=too-many-arguments
        self._last_pin_read = None
        self.buf            = bytearray(3)
        self.buf_count      = 0
        self.word           = 0
        self.gain           = gain
        self.data_rate      = self._data_rate_default() if data_rate is None else data_rate
        self.mode           = mode
        self.pig            = pig
        self.i2c_device     = self.pig.i2c_open(1, address) # (i2c bus, ads1115 i2c address)

    def __del__(self):
        self.pig.i2c_close(self.i2c_device)
    @property
    def data_rate(self):
        """The data rate for ADC conversion in samples per second."""
        return self._data_rate

    @data_rate.setter
    def data_rate(self, rate):
        possible_rates = self.rates
        if rate not in possible_rates:
            raise ValueError("Data rate must be one of: {}".format(possible_rates))
        self._data_rate = rate

    @property
    def rates(self):
        """Possible data rate settings."""
        raise NotImplementedError("Subclass must implement rates property.")

    @property
    def rate_config(self):
        """Rate configuration masks."""
        raise NotImplementedError("Subclass must implement rate_config property.")

    @property
    def gain(self):
        """The ADC gain."""
        return self._gain

    @gain.setter
    def gain(self, gain):
        possible_gains = self.gains
        if gain not in possible_gains:
            raise ValueError("Gain must be one of: {}".format(possible_gains))
        self._gain = gain

    @property
    def gains(self):
        """Possible gain settings."""
        g = list(_ADS1X15_CONFIG_GAIN.keys())
        g.sort()
        return g

    @property
    def mode(self):
        """The ADC conversion mode."""
        return self._mode

    @mode.setter
    def mode(self, mode):
        if mode not in (Mode.CONTINUOUS, Mode.SINGLE):
            raise ValueError("Unsupported mode.")
        self._mode = mode

    def read(self, pin, is_differential=False):
        """I2C Interface for ADS1x15-based ADCs reads.

        params:
            :param pin: individual or differential pin.
            :param bool is_differential: single-ended or differential read.
        """
        if pin > 4: printf('I don\'t have %g pins',pin) 
        pin = pin if is_differential else pin + 0x04
        return self._read(pin)

    def _data_rate_default(self):
        """Retrieve the default data rate for this ADC (in samples per second).
        Should be implemented by subclasses.
        """
        raise NotImplementedError("Subclasses must implement _data_rate_default!")

    def _conversion_value(self, raw_adc):
        """Subclasses should override this function that takes the 16 raw ADC
        values of a conversion result and returns a signed integer value.
        """
        raise NotImplementedError("Subclass must implement _conversion_value function!")

    def _read(self, pin):
        """Perform an ADC read. Returns the signed integer result of the read."""
        if self.mode == Mode.CONTINUOUS and self._last_pin_read == pin:
            return self._conversion_value(self.get_last_result(True))
        self._last_pin_read = pin
        config = _ADS1X15_CONFIG_OS_SINGLE
        config |= (pin & 0x07) << _ADS1X15_CONFIG_MUX_OFFSET
        config |= _ADS1X15_CONFIG_GAIN[self.gain]
        config |= self.mode
        config |= self.rate_config[self.data_rate]
        config |= _ADS1X15_CONFIG_COMP_QUE_DISABLE
        self._write_register(_ADS1X15_POINTER_CONFIG, config)

# If driver is still slow, this might be the place to make further modifications 
        if self.mode == Mode.SINGLE:
            while not self._conversion_complete():
                pass
                #sleep(1/self.data_rate/5)

        return self._conversion_value(self.get_last_result(False))

    def _conversion_complete(self):
# Another place where things could be spead up - instead of performing a read over 
# the i2c interface, configure the ADC's comparater for "conversion pin ready"
# Then hook up ALERT/RDY on the ads1115 to a GPIO pin and open/start a pigpiod
# notify handle. Also would need to implement event handling.
        """Return status of ADC conversion."""
        # OS is bit 15
        # OS = 0: Device is currently performing a conversion
        # OS = 1: Device is not currently performing a conversion
        return self._read_register(_ADS1X15_POINTER_CONFIG) & 0x8000

    def get_last_result(self, fast=False):
        """Read the last conversion result when in continuous conversion mode.
        Will return a signed integer value. If fast is True, the register
        pointer is not updated as part of the read. This reduces I2C traffic
        and increases possible read rate.
        """
        return self._read_register(_ADS1X15_POINTER_CONVERSION, fast)

    def _write_register(self, reg, word):
        """Write 16 bit value to register."""
        self.buf[0] = reg
        # self.buf[1] = (value >> 8) & 0xFF
        # self.buf[2] = value & 0xFF
        self.word = self.b2l(word)
        self.pig.i2c_write_word_data(self.i2c_device,reg,self.word)

    def _read_register(self, reg, fast=False):
        """Read 16 bit register value. If fast is True, the pointer register
        is not updated.
        """
        self.buf[0] = reg
        if fast:
            # pigpio's i2c_read_device returns a bytearray - > needs additional conversion
            (self.buf_count,self.buf[1:]) = self.pig.i2c_read_device(self.i2c_device,2)
            self.word = self.buf[1] << 8 | self.buf[2]
        else:
            self.word = self.pig.i2c_read_word_data(self.i2c_device,reg)
        return self.b2l(self.word)
    
    def byteSwap(self,word):
        '''Revert Byte order for Words (2 Bytes, 16 Bit).'''
        word = (word>>8 |word<<8)&0xFFFF
        return word
     
    def l2b(self,word):
        '''Little Endian to BigEndian conversion for signed 2Byte integers (2 complement).'''
        word = self.byteSwap(word)
        if(word >= 2**15):
            word = self.word-2**16
        return word
     
    def b2l(self,word):
        '''BigEndian to LittleEndian conversion for signed 2 Byte integers (2 complement).'''
        if(word < 0):
            word = 2**16 + word
        return self.byteSwap(word)

class ads1015(ADS1x15):
    """Class for the ADS1015 12 bit ADC."""

    @property
    def bits(self):
        """The ADC bit resolution."""
        return 12

    @property
    def rates(self):
        """Possible data rate settings."""
        r = list(_ADS1015_CONFIG_DR.keys())
        r.sort()
        return r

    @property
    def rate_config(self):
        """Rate configuration masks."""
        return _ADS1015_CONFIG_DR

    def _data_rate_default(self):
        return 1600

    def _conversion_value(self, raw_adc):
        raw_adc = raw_adc.to_bytes(2, "big")
        value = struct.unpack(">h", raw_adc)[0]
        return value >> 4

class ads1115(ADS1x15):
    """Class for the ADS1115 16 bit ADC."""

    @property
    def bits(self):
        """The ADC bit resolution."""
        return 16

    @property
    def rates(self):
        """Possible data rate settings."""
        r = list(_ADS1115_CONFIG_DR.keys())
        r.sort()
        return r

    @property
    def rate_config(self):
        """Rate configuration masks."""
        return _ADS1115_CONFIG_DR

    def _data_rate_default(self):
        return 128

    def _conversion_value(self, raw_adc):
        raw_adc = raw_adc.to_bytes(2, "big")
        value = struct.unpack(">h", raw_adc)[0]
        return value

class AnalogIn:
    """AnalogIn Mock Implementation for ADC Reads."""

    def __init__(self, ads, positive_pin, negative_pin=None):
        """AnalogIn

        :param ads: The ads object.
        :param ~digitalio.DigitalInOut positive_pin: Required pin for single-ended.
        :param ~digitalio.DigitalInOut negative_pin: Optional pin for differential reads.
        """
        self._ads = ads
        self._pin_setting = positive_pin
        self._negative_pin = negative_pin
        self.is_differential = False
        if negative_pin is not None:
            pins = (self._pin_setting, self._negative_pin)
            if pins not in _ADS1X15_DIFF_CHANNELS:
                raise ValueError(
                    "Differential channels must be one of: {}".format(
                        list(_ADS1X15_DIFF_CHANNELS.keys())
                    )
                )
            self._pin_setting = _ADS1X15_DIFF_CHANNELS[pins]
            self.is_differential = True

    @property
    def value(self):
        """Returns the value of an ADC pin as an integer."""
        return self._ads.read(
            self._pin_setting, is_differential=self.is_differential
        ) << (16 - self._ads.bits)

    @property
    def voltage(self):
        """Returns the voltage from the ADC pin as a floating point value."""
        volts = self.value * _ADS1X15_PGA_RANGE[self._ads.gain] / 32767
        return volts
