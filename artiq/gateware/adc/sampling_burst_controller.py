from migen import *

from artiq.gateware.rtio import rtlink, Channel
from artiq.gateware.suservo.adc_ser import ADC, ADCParams
from math import ceil, log2


class SamplingBurstController(Module):

    def __init__(self, adc_pads, max_samples=1024):

        self.rtio_channels = []

        # # #

        channels = 8
        channel_width = 16

        writes_per_sample = channels * channel_width // 32

        fifo_depth = writes_per_sample * max_samples
        # Round-up to the nearest power of two
        fifo_depth = 2 ** ceil(log2(fifo_depth) / log2(2))

        # Correct max_samples to use all memory
        max_samples = fifo_depth // writes_per_sample

        self.rtlink = rtlink.Interface(
            rtlink.OInterface(
                data_width=16,
                address_width=3
            ),
            rtlink.IInterface(
                data_width=32,
            )
        )

        self.rtio_channels.append(Channel.from_phy(self, ififo_depth=fifo_depth))

        trigger = Signal()
        samples = Signal(min=0, max=max_samples - 1)
        conv_timeout = Signal(16)

    # Configuration interface

        self.we = we = ~self.rtlink.o.address[-1]
        self.sel = sel = self.rtlink.o.address[:-1]

        self.sync.rio_phy += [
            If(self.rtlink.o.stb & we & (sel == 0),
                trigger.eq(self.rtlink.o.data)
            ).Elif(self.rtlink.o.stb & we & (sel == 1),
              conv_timeout.eq(self.rtlink.o.data)
            ).Elif(self.rtlink.o.stb & we & (sel == 2),
              samples.eq(self.rtlink.o.data)
            ).Else(
              trigger.eq(0)
            )
        ]

    # Create ADC interface

        # timings in units of RTIO coarse period
        adc_p = ADCParams(width=16, channels=8, lanes=4, t_cnvh=4,
                          # account for SCK DDR to CONV latency
                          # difference (4 cycles measured)
                          t_conv=57 - 4, t_rtt=4 + 4)

        adc_interface = ClockDomainsRenamer("rio_phy")(ADC(adc_pads, adc_p))
        self.submodules += adc_interface

    # Control logic

        data = Array([
            Cat(adc_interface.data[0], adc_interface.data[1]),
            Cat(adc_interface.data[2], adc_interface.data[3]),
            Cat(adc_interface.data[4], adc_interface.data[5]),
            Cat(adc_interface.data[6], adc_interface.data[7])
        ])

        mux_cnt = Signal(max=writes_per_sample+1)

        self.comb += self.rtlink.i.data.eq(data[mux_cnt])

        sample_cnt = Signal.like(samples)
        conv_timeout_cnt = Signal.like(conv_timeout)

        fsm = ClockDomainsRenamer("rio")(FSM(reset_state="idle"))
        self.submodules += fsm

        fsm.act("idle",
                adc_interface.start.eq(0),
                self.rtlink.i.stb.eq(0),
                If(trigger,
                   NextValue(sample_cnt, 0),
                   NextState("start_acq"))
                )
        fsm.act("start_acq",
                adc_interface.start.eq(1),
                self.rtlink.i.stb.eq(0),
                NextState("acq"),
                )
        fsm.act("acq",
                adc_interface.start.eq(0),
                self.rtlink.i.stb.eq(0),
                If(adc_interface.done == 1,
                   NextState("read"),
                   NextValue(mux_cnt, 0))
                )
        fsm.act("read",
                adc_interface.start.eq(0),
                self.rtlink.i.stb.eq(1),
                If(mux_cnt == writes_per_sample-1,
                   If(sample_cnt == samples-1,
                      NextState("idle")
                   ).Elif(conv_timeout == 0,
                          NextState("start_acq"),
                          NextValue(sample_cnt, sample_cnt + 1),
                  ).Else(
                       NextState("conv_timeout"),
                       NextValue(sample_cnt, sample_cnt + 1),
                       NextValue(conv_timeout_cnt, conv_timeout)
                   )
                ),
                NextValue(mux_cnt, mux_cnt + 1)
        )
        fsm.act("conv_timeout",
                adc_interface.start.eq(0),
                self.rtlink.i.stb.eq(0),
                If(conv_timeout_cnt == 0,
                   NextState("start_acq"),
                ).Else(
                   NextValue(conv_timeout_cnt, conv_timeout_cnt-1)
                )
        )