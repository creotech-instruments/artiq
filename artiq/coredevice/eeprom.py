from artiq.coredevice.i2c import *
from artiq.language.core import kernel, delay
from artiq.language.units import *


class EEPROM_24AA02XEXX:
    """Driver for 24AA02XEXX Microchip EEPROMs.

    I2C transactions not real-time, and are performed by the CPU without
    involving RTIO.
    """

    def __init__(self, dmgr, busno=0, address=0xA0, model="24AA02E48", core_device="core"):

        self.core = dmgr.get(core_device)
        self.busno = busno
        self.address = address
        self.model = model

        if model == "24AA02E48":
            self.page_size = 8
            self.eui_size = 6
            self.eui = [0] * 6
            self.eui_start = 0xFA
        elif model == "24AA025E48":
            self.page_size = 16
            self.eui_size = 6
            self.eui = [0] * 6
            self.eui_start = 0xFA
        elif model == "24AA02E64":
            self.page_size = 8
            self.eui_size = 8
            self.eui = [0] * 8
            self.eui_start = 0xF8
        elif model == "24AA025E64":
            self.page_size = 16
            self.eui_size = 8
            self.eui = [0] * 8
            self.eui_start = 0xF8
        else:
            raise Exception("Unsupported EEPROM model: %s" % model)

    @kernel
    def _wait_for_ack(self):

        timeout = 7  # Datasheet states that TWC is at most 5 ms

        while timeout > 0:
            i2c_start(self.busno)
            if i2c_write(self.busno, self.address):
                i2c_stop(self.busno)
                break
            i2c_stop(self.busno)
            timeout -= 1
            delay(1*ms)

        if timeout == 0:
            raise I2CError("%s did not ack writing cycle in the specified time." % self.model)

    @kernel
    def _write_page(self, address, data):

        assert (address % self.page_size == 0), "Page address must be multiple of page size."
        assert (len(data) <= self.page_size), "Data to be written on page must fit page size"

        i2c_start(self.busno)
        try:
            if not i2c_write(self.busno, self.address):
                raise I2CError("EEPROM failed to ack for writing")

            if not i2c_write(self.busno, address):
                raise I2CError("EEPROM failed to ack address write")

            for d in data:
                if not i2c_write(self.busno, d):
                    raise I2CError("EEPROM failed to ack data write")
        finally:
            i2c_stop(self.busno)

        self._wait_for_ack()

    @kernel
    def _write_single(self, address, data):

        i2c_start(self.busno)
        try:
            if not i2c_write(self.busno, self.address):
                raise I2CError("EEPROM failed to ack for writing")

            if not i2c_write(self.busno, address):
                raise I2CError("EEPROM failed to ack address write")

            if not i2c_write(self.busno, data):
                raise I2CError("EEPROM failed to ack data write")
        finally:
            i2c_stop(self.busno)

        self._wait_for_ack()

    @kernel
    def write(self, address, data):

        # Page write
        if not (address % self.page_size) and len(data) > 1:
            pages = (len(data) + (self.page_size - 1)) // self.page_size
            last_page_bytes = len(data)-(pages-1)*self.page_size

            for p in range(pages-1):
                self._write_page(address + p*self.page_size, data[p*self.page_size:(p+1)*self.page_size])

            self._write_page(address + (pages-1)*self.page_size, data[-last_page_bytes:])

        # Single write
        else:
            for i, d in enumerate(data):
                self._write_single(address+i, d)

    @kernel
    def read(self, address, no_bytes):

        data = [0] * no_bytes
        i2c_start(self.busno)

        try:
            if not i2c_write(self.busno, self.address | 1):
                raise I2CError("EEPROM failed to ack for writing")

            if not i2c_write(self.busno, address):
                raise I2CError("EEPROM failed to ack address write")

            for i in range(no_bytes - 1):
                data[i] = i2c_read(self.busno, True)

            data[no_bytes - 1] = i2c_read(self.busno, False)
        finally:
            i2c_stop(self.busno)

        return data

    @kernel
    def read_eui(self):

        i2c_start(self.busno)
        try:
            if not i2c_write(self.busno, self.address):
                raise I2CError("EEPROM failed to ack for writing")
            if not i2c_write(self.busno, self.eui_start):
                raise I2CError("EEPROM failed to ack address write")
            i2c_restart(self.busno)
            if not i2c_write(self.busno, self.address | 1):
                raise I2CError("EEPROM failed to ack for reading")
            for i in range(self.eui_size-1):
                self.eui[i] = i2c_read(self.busno, True)
            self.eui[self.eui_size-1] = i2c_read(self.busno, False)

        finally:
            i2c_stop(self.busno)

        return self.eui
