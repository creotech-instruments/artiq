from artiq.coredevice.i2c import PCA9548
from artiq.language import kernel


class Kasli:

    def __init__(self, dmgr, core_device="core"):
        self.dmgr = dmgr
        self.core = dmgr.get(core_device)
        self.i2c_switch = [
            PCA9548(dmgr, address=(0x70 << 1), core_device=core_device),
            PCA9548(dmgr, address=(0x71 << 1), core_device=core_device)
        ]

        self.mux_mapping = [
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            1,
            1,
            1,
            1,
        ]
        self.channel_mapping = [7, 5, 4, 3, 2, 1, 0, 6, 4, 5, 7, 6]

    @kernel
    def set_active_eem(self, eem):
        self.set_active_eems([eem])

    @kernel
    def set_active_eems(self, eems):
        mask = [0, 0]
        for eem in eems:
            mask[self.mux_mapping[eem]] |= (1 << self.channel_mapping[eem])
        self.i2c_switch[0].select(mask[0])
        self.i2c_switch[1].select(mask[1])
