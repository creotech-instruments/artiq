from artiq.language.core import kernel, delay_mu, portable, now_mu, rpc
from artiq.language.units import ns

from artiq.coredevice import spi2 as spi
from artiq.coredevice.eeprom import EEPROM_24AA02XEXX
from artiq.coredevice.pcf8574a import PCF8574A
from artiq.coredevice.sampler import adc_mu_to_volt

from artiq.coredevice.rtio import *

__all__ = ['Sampler']

MIN_PERIOD = 824 * ns
CONV_TIMEOUT_MULT = 8 * ns
MAX_SAMPLES = 1024

TRIGGER_ADDRESS = 0x0
CONV_TIMEOUT_ADDRESS = 0x1
SAMPLES_ADDRESS = 0x2

SPI_CONFIG = (0*spi.SPI_OFFLINE | 0*spi.SPI_END |
              0*spi.SPI_INPUT | 0*spi.SPI_CS_POLARITY |
              0*spi.SPI_CLK_POLARITY | 0*spi.SPI_CLK_PHASE |
              0*spi.SPI_LSB_FIRST | 0*spi.SPI_HALF_DUPLEX)

SPI_CS_PGIA = 1  # separate SPI bus, CS used as RCLK


class Sampler:
    """Sampler ADC.

    Controls the LTC2320-16 8 channel 16 bit ADC with SPI interface and
    the switchable gain instrumentation amplifiers.

    :param spi_adc_device: ADC SPI bus device name
    :param spi_pgia_device: PGIA SPI bus device name
    :param cnv_device: CNV RTIO TTLOut channel name
    :param div: SPI clock divider (default: 8)
    :param gains: Initial value for PGIA gains shift register
        (default: 0x0000). Knowledge of this state is not transferred
        between experiments.
    :param core_device: Core device name
    """
    kernel_invariants = {"bus_pgia", "core", "burst_channel", "div"}

    def __init__(self, dmgr, channel, eem0, eem1, eem_carrier, div=8, gains=0x0000, core_device="core"):

        self.core = dmgr.get(core_device)
        self.ref_period_mu = self.core.seconds_to_mu(
            self.core.coarse_ref_period)

        self.eem_carrier = dmgr.get(eem_carrier)  # type: EEMCarrier
        self.eem = [eem0, eem1]

        self.eeprom = EEPROM_24AA02XEXX(dmgr)
        self.term_ind_gpio = PCF8574A(dmgr)

        self.burst_channel = channel

        self.bus_pgia = spi.SPIMaster(dmgr, channel + 1)
        self.bus_pgia.update_xfer_duration_mu(div, 16)

        self.div = div
        self.gains = gains

        self.samples = [0] * MAX_SAMPLES * 4
        self.samples_num = 0

    @kernel
    def read_term_state(self):
        self.eem_carrier.set_active_eem(self.eem[0])
        return self.term_ind_gpio.get()

    @kernel
    def read_eui(self, eem):
        self.eem_carrier.set_active_eems([self.eem[eem]])
        return self.eeprom.read_eui()

    @kernel
    def write_eeprom(self, eem, address, data):
        self.eem_carrier.set_active_eem(self.eem[eem])
        return self.eeprom.write(address, data)

    @kernel
    def read_eeprom(self, eem, address, no_bytes):
        self.eem_carrier.set_active_eem(self.eem[eem])
        return self.eeprom.read(address, no_bytes)

    @kernel
    def init(self):
        """Initialize the device.

        Sets up SPI channels.
        """
        self.bus_pgia.set_config_mu(SPI_CONFIG | spi.SPI_END,
                                    16, self.div, SPI_CS_PGIA)

    @kernel
    def set_gain_mu(self, channel, gain):
        """Set instrumentation amplifier gain of a channel.

        The four gain settings (0, 1, 2, 3) corresponds to gains of
        (1, 10, 100, 1000) respectively.

        :param channel: Channel index
        :param gain: Gain setting
        """
        gains = self.gains
        gains &= ~(0b11 << (channel*2))
        gains |= gain << (channel*2)
        self.bus_pgia.write(gains << 16)
        self.gains = gains

    @kernel
    def get_gains_mu(self):
        """Read the PGIA gain settings of all channels.

        :return: The PGIA gain settings in machine units.
        """
        self.bus_pgia.set_config_mu(SPI_CONFIG | spi.SPI_END | spi.SPI_INPUT,
                                    16, self.div, SPI_CS_PGIA)
        self.bus_pgia.write(self.gains << 16)
        self.bus_pgia.set_config_mu(SPI_CONFIG | spi.SPI_END,
                                    16, self.div, SPI_CS_PGIA)
        self.gains = self.bus_pgia.read() & 0xffff
        return self.gains

    @kernel
    def sample(self, samples=MAX_SAMPLES, period=MIN_PERIOD):
        """Acquire a burst of samples.

        This function advances timeline by 3 * reference period.
        """

        self.prepare_sampling(samples, period)
        self.trigger()

    @kernel
    def prepare_sampling(self, samples=MAX_SAMPLES, period=MIN_PERIOD):
        if samples < 1:
            raise ValueError("Number of samples must be at least 1.")
        if samples > MAX_SAMPLES:
            raise ValueError("Too many samples to acquire.")
        if period < MIN_PERIOD:
            raise ValueError("Too small period.")

        conv_timeout = int(10 ** 9 * period) // 8 - int(MIN_PERIOD / CONV_TIMEOUT_MULT)

        self.core.break_realtime()
        rtio_output((self.burst_channel << 8) | CONV_TIMEOUT_ADDRESS, conv_timeout)
        delay_mu(self.ref_period_mu)
        rtio_output((self.burst_channel << 8) | SAMPLES_ADDRESS, samples)
        delay_mu(self.ref_period_mu)

        self.samples_num = samples

    @kernel
    def trigger(self):
        rtio_output((self.burst_channel << 8) | TRIGGER_ADDRESS, 1)
        delay_mu(self.ref_period_mu)

    @kernel
    def _sample_mu(self, samples, conv_timeout):
        rtio_output((self.burst_channel << 8) | CONV_TIMEOUT_ADDRESS, conv_timeout)
        delay_mu(self.ref_period_mu)
        rtio_output((self.burst_channel << 8) | SAMPLES_ADDRESS, samples)
        delay_mu(self.ref_period_mu)
        rtio_output((self.burst_channel << 8) | TRIGGER_ADDRESS, 1)
        delay_mu(self.ref_period_mu)

    @rpc(flags={"async"})
    def _store(self, idx, val):
        self.samples[idx] = val

    @kernel
    def read_single_sample(self):
        return adc_mu_to_volt(rtio_input_data(self.burst_channel))

    @kernel
    def _transfer_samples(self):
        for idx in range(self.samples_num*4):
            self._store(idx, rtio_input_data(self.burst_channel))
        self.samples_num = 0

    def get_samples(self):

        """Acquire a burst of samples.

        .. seealso:: :meth:`sample`

        sample function must be called first.

        :return: Dict where number of channel is a key and list of samples are values.
        """

        self._transfer_samples()

        channel_data = {}
        raw_channel_data = {}
        for x in range(8):
            channel_data[x] = []
            raw_channel_data[x] = []

        channel = 0
        mask = 1 << 15

        for val in self.samples:
            a = val & 0xFFFF
            a = -(a & mask) + (a & ~mask)
            b = (val >> 16)

            gain = (self.gains >> (channel * 2)) & 0b11
            channel_data[channel].append(adc_mu_to_volt(a, gain))
            raw_channel_data[channel].append(a)
            channel += 1

            gain = (self.gains >> (channel * 2)) & 0b11
            channel_data[channel].append(adc_mu_to_volt(b, gain))
            raw_channel_data[channel].append(b)
            channel += 1

            if channel == 8:
                channel = 0

        return channel_data, raw_channel_data
