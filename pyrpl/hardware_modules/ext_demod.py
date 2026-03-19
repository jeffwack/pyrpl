###############################################################################
#    pyrpl-ext-demod - External reference demodulator for PDH locking
#    Based on pyrpl by Leonhard Neuhaus
#
#    Copyright (C) 2026  Jeffrey Wack
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.
###############################################################################
"""
External reference demodulator module.

Demodulates an input signal using an external reference signal (from the DSP
bus) rather than the internal NCO. The reference passes through an adjustable
delay line that provides phase rotation for PDH error signal optimization.

Architecture::

    ext_ref_i --> [delay line] --> delayed_ref
    dat_i     --> [mixer] * delayed_ref --> [LPF] --> signal_o

Usage example::

    # Feed photodiode signal into ext_demod, use in2 as external reference
    rp.ext_demod.input = 'in1'
    rp.ext_demod.ref_input = 'in2'

    # Set delay for phase rotation (at 29.5 MHz, 1 sample ~ 85 degrees)
    rp.ext_demod.delay = 2

    # Set LPF bandwidth to 1 kHz
    rp.ext_demod.bandwidth = [1000]

    # Route demodulated signal to PID for locking
    rp.pid0.input = 'ext_demod'

    # Helper: compute optimal delay for a given frequency and desired phase
    delay = rp.ext_demod.phase_to_delay(29.5e6, 90.0)
    rp.ext_demod.delay = delay
"""

from .dsp import DspModule, InputSelectRegister, all_inputs
from ..attributes import IntRegister, FilterRegister
from ..pyrpl_utils import sorted_dict


# FPGA clock frequency in Hz
_FCLK = 125e6


class ExtDemod(DspModule):
    """
    External reference demodulator for PDH locking.

    This module replaces the IQ1 slot (module index 6) in the DSP bus.
    It uses an external reference signal from the DSP bus instead of an
    internal NCO for demodulation.

    The ``ref_input`` attribute selects which DSP bus signal provides the
    demodulation reference. The ``delay`` attribute controls the phase
    shift applied to the reference before mixing.
    """

    _setup_attributes = ["input",
                         "ref_input",
                         "delay",
                         "bandwidth",
                         "output_direct"]

    _gui_attributes = _setup_attributes

    _delay = 4  # bare pipeline delay (cycles): 1 BRAM read + 1 pipeline + 1 multiply + 1 LPF input

    # --- Register definitions ---

    # Delay in samples (0 to 1023). Controls phase of demodulation reference.
    # Phase shift = 360 * f_ref * delay / 125e6 degrees.
    delay = IntRegister(0x100, bits=10, doc="Delay in samples (0-1023). "
                        "Phase shift = 360 * f_ref * delay / 125e6 degrees.")

    # Low-pass filter configuration. Same format as red_pitaya_filter_block.
    # Use positive values for lowpass, negative for highpass, 0 for off.
    bandwidth = FilterRegister(0x104,
                               filterstages=0x20C,
                               shiftbits=0x210,
                               minbw=0x214,
                               doc="Demodulation lowpass filter bandwidths "
                                   "[Hz]. 0 = off, positive = lowpass, "
                                   "negative = highpass.")

    @property
    def bandwidths(self):
        """List of valid filter bandwidth values."""
        return self.__class__.bandwidth.valid_frequencies(self)

    # Reference input select — selects which DSP bus signal feeds ext_ref_i.
    # This register lives at address 0x14 in the DSP reserved space (not in
    # the module's own register space), handled by red_pitaya_dsp.v.
    ref_input = InputSelectRegister(0x14,
                                    options=all_inputs,
                                    doc="Selects the reference signal for "
                                        "demodulation from the DSP bus.")

    # --- Read-only FPGA parameter registers (for introspection) ---

    _SIGNALBITS = IntRegister(0x200, doc="FPGA signal bit width")
    _LPFBITS = IntRegister(0x204, doc="FPGA LPF internal bit width")
    _MAXDELAY = IntRegister(0x208, doc="FPGA maximum delay in samples")

    # --- Helper methods ---

    def phase_to_delay(self, freq_hz, phase_deg):
        """
        Compute the delay (in samples) needed to achieve a given phase shift
        at a given reference frequency.

        Parameters
        ----------
        freq_hz : float
            Reference frequency in Hz.
        phase_deg : float
            Desired phase shift in degrees.

        Returns
        -------
        int
            Delay in samples (clamped to 0..1023).

        Notes
        -----
        The relationship is: phase_deg = 360 * freq_hz * delay / f_clk
        So: delay = phase_deg * f_clk / (360 * freq_hz)

        At high frequencies (e.g. 29.5 MHz), 1 sample = 85 degrees,
        so fine phase control requires sweeping delay by 1 sample at a time.
        At lower frequencies (e.g. 10 MHz), 1 sample = 28.8 degrees.
        """
        if freq_hz <= 0:
            return 0
        delay = phase_deg * _FCLK / (360.0 * freq_hz)
        return int(max(0, min(1023, round(delay))))

    def delay_to_phase(self, freq_hz, delay=None):
        """
        Compute the phase shift in degrees for the current (or given) delay
        at a given reference frequency.

        Parameters
        ----------
        freq_hz : float
            Reference frequency in Hz.
        delay : int or None
            Delay in samples. If None, reads current delay from FPGA.

        Returns
        -------
        float
            Phase shift in degrees.
        """
        if delay is None:
            delay = self.delay
        return 360.0 * freq_hz * delay / _FCLK
