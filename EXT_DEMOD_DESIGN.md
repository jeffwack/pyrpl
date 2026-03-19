# External Reference Demodulator (ext_demod) — Design Document

This document describes the design of the `ext_demod` module added to the
pyrpl FPGA image. It is written for a future agent or developer who will
build the bitfile with Vivado and debug on hardware.

## Purpose

PDH (Pound-Drever-Hall) locking requires demodulating a photodiode signal
at the modulation frequency. The standard pyrpl IQ block uses an internal
NCO — but for experiments where the modulation reference is an external
signal (e.g. from a function generator driving an EOM), we need to
demodulate using that external signal directly.

The `ext_demod` block:
- Takes two inputs from the DSP bus: `dat_i` (photodiode) and `ext_ref_i` (reference)
- Passes the reference through an adjustable delay line (phase rotation)
- Multiplies signal × delayed_reference
- Low-pass filters the product
- Outputs the demodulated (DC) error signal

## Architecture

```
ext_ref_i ──► [BRAM delay line, 0-1023 samples] ──► delayed_ref
                                                          │
dat_i ──► [pipeline reg] ──► [14×14 mixer] ◄─────────────┘
                                  │
                            28-bit product
                                  │
                          [round + truncate to 24-bit]
                                  │
                          [red_pitaya_filter_block LPF]
                                  │
                          [truncate 24→14 bit]
                                  │
                          signal_o / dat_o
```

## DSP Bus Integration

The ext_demod block replaces **IQ1 at slot 6**. IQ0 (slot 5) and IQ2 (slot 7)
remain as standard IQ modules, renamed to `iq0` and `iq1` in Python.

### Module Slot Mapping (after change)

| Slot | Module | Python name | DSP_INPUTS key |
|------|--------|-------------|----------------|
| 0 | PID0 | `pid0` | `pid0` |
| 1 | PID1 | `pid1` | `pid1` |
| 2 | PID2 | `pid2` | `pid2` |
| 3 | TRIG | `trig` | `trig` |
| 4 | IIR | `iir` | `iir` |
| 5 | IQ0 | `iq0` | `iq0` |
| **6** | **ExtDemod** | **`extdemod`** | **`extdemod`** |
| 7 | IQ2 | `iq1` | `iq1` |

Note: What was formerly `iq2` (slot 7) is now `iq1` since there are only
two IQ instances. The dual-output feature (`iq1_2` → DSP signal index 14)
is preserved.

### Reference Input Routing

The ext_demod module needs a **second input** for the reference signal.
Standard DSP modules only have one `input_select`. We added a new register:

- **Address**: `0x14` within the DSP reserved address space (same as
  `input_select` at `0x00`, `output_select` at `0x04`, etc.)
- **Scope**: Only writable when the module-index bits of the address
  select the EXTDEMOD slot (module 6). The address check is:
  `sys_addr[16-1:0]==16'h14 && sys_addr[16+LOG_MODULES-1:16]==EXTDEMOD`
- **Read-back**: Address `0x14` from any module slot returns the
  `ref_input_select` value. This is a minor quirk — ideally it would
  only respond for the ext_demod slot, but since it's a single register
  and reads are harmless, this simplification is acceptable.
- **Reset value**: `ADC2` (analog input 2), so by default the reference
  comes from the second ADC channel.

The Python `ExtDemod.ref_input` attribute is an `InputSelectRegister` at
address `0x14`, following the same pattern as `DspModule.input` at `0x00`.

## Verilog Design Details

### File: `pyrpl/fpga/rtl/red_pitaya_ext_demod_block.v`

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| SIGNALBITS | 14 | Input/output signal width |
| LPFBITS | 24 | Internal LPF precision |
| MAXDELAY | 1024 | Maximum delay line depth |
| DELAYBITS | 10 | log2(MAXDELAY) |
| SHIFTBITS | 1 | Product right-shift (see below) |
| FILTERSTAGES | 2 | Cascaded LPF stages |
| FILTERSHIFTBITS | 5 | LPF shift range |
| FILTERMINBW | 10 | Minimum LPF bandwidth (Hz) |

#### SHIFTBITS Rationale

This follows the same logic as `red_pitaya_iq_demodulator_block.v`:
- 14-bit signed signals exclude the most-negative value (−8192 is never
  produced by the ADC or DSP bus)
- Therefore `dat_i × delayed_ref` occupies at most 14+14−2 = 26 bits
  plus sign = 27 bits
- With SHIFTBITS=1, we discard 1 MSB that is always a sign extension
- The remaining bits are `product[26:4]` = 23 bits → fits in LPFBITS=24

#### Bit-Width Arithmetic (Critical for Debugging)

```
product = dat_i_reg[13:0] × delayed_ref[13:0]
        = 28 bits signed (SIGNALBITS + SIGNALBITS)

Round offset = 1 << (28 - 24 - 1 - 1) = 1 << 2 = 4
  (asymmetric rounding, same as IQ demodulator)

Output bit-select:
  MSB = product[28-1-1] = product[26]     (= PRODUCTBITS-1-SHIFTBITS)
  LSB = product[28-24-1] = product[3]     (= PRODUCTBITS-LPFBITS-SHIFTBITS)
  → mixer_out[23:0] = product[26:3]

Wait — let me recheck. The assign is:
  mixer_out = product[PRODUCTBITS-1-SHIFTBITS : PRODUCTBITS-LPFBITS-SHIFTBITS]
            = product[26 : 3]
  That's 26-3+1 = 24 bits. ✓
```

#### Delay Line

- Implemented as `reg [13:0] delay_mem [0:1023]` — a 1024×14-bit array
- Vivado should infer this as a single BRAM18 tile (14 Kbit < 18 Kbit)
- **If Vivado uses LUT RAM instead**: Add `(* ram_style = "block" *)`
  attribute before the `delay_mem` declaration
- Write pointer increments every clock cycle (free-running)
- Read pointer = write_ptr - delay (wrapping unsigned subtraction)
- Read has 1 cycle latency (registered output)

#### Pipeline Latency

| Stage | Cycles | Description |
|-------|--------|-------------|
| BRAM read | 1 | `delayed_ref <= delay_mem[read_ptr]` |
| Input pipeline | 1 | `dat_i_reg <= dat_i` |
| Multiply + round | 1 | `product <= dat_i_reg * delayed_ref + ROUND_OFFSET` |
| LPF | N | Depends on filter_block (typically 1-2 cycles per stage) |
| Output truncate | 0 | Combinational |

Total: 3 + LPF_latency cycles from inputs to output.

#### Register Map

| Address | Name | Width | R/W | Description |
|---------|------|-------|-----|-------------|
| 0x000 | input_select | 4 | R/W | Standard DSP input (handled by dsp.v) |
| 0x004 | output_select | 2 | R/W | DAC routing (handled by dsp.v) |
| 0x008 | saturation | 2 | R | DAC saturation flags (handled by dsp.v) |
| 0x00C | sync | 8 | R/W | Module sync (handled by dsp.v) |
| 0x010 | current_output | 14 | R | Current output value (handled by dsp.v) |
| 0x014 | ref_input_select | 4 | R/W | Reference input select (handled by dsp.v) |
| 0x100 | delay | 10 | R/W | Delay in samples (0-1023) |
| 0x104 | lpf_config | 32 | R/W | LPF config (filter_block format) |
| 0x200 | SIGNALBITS | 32 | R | Parameter readback: 14 |
| 0x204 | LPFBITS | 32 | R | Parameter readback: 24 |
| 0x208 | MAXDELAY | 32 | R | Parameter readback: 1024 |
| 0x20C | FILTERSTAGES | 32 | R | Parameter readback: 2 |
| 0x210 | FILTERSHIFTBITS | 32 | R | Parameter readback: 5 |
| 0x214 | FILTERMINBW | 32 | R | Parameter readback: 10 |

The full AXI address for register `offset` is:
`0x40300000 + 6*0x10000 + offset = 0x40360000 + offset`

## Python Module Details

### File: `pyrpl/hardware_modules/ext_demod.py`

Class `ExtDemod(DspModule)` — does NOT extend `FilterModule` because
there is no input pre-filter (the ext_demod block has no `inputfilter`
register at 0x120). The LPF is the demodulation lowpass, not an input
filter.

#### Attributes

| Attribute | Register | Address | Type | Description |
|-----------|----------|---------|------|-------------|
| `input` | input_select | 0x00 | InputSelectRegister | Signal to demodulate |
| `output_direct` | output_select | 0x04 | SelectRegister | DAC routing |
| `ref_input` | ref_input_select | 0x14 | InputSelectRegister | Reference signal |
| `delay` | delay | 0x100 | IntRegister(bits=10) | Delay in samples |
| `bandwidth` | lpf_config | 0x104 | FilterRegister | LPF bandwidth |

#### Helper Methods

- `phase_to_delay(freq_hz, phase_deg)` → int: Compute delay for desired
  phase at given frequency
- `delay_to_phase(freq_hz, delay=None)` → float: Compute phase for
  current/given delay at given frequency

### Module Registration

- `DSP_INPUTS['extdemod'] = 6` — maps the Python name to DSP slot 6
- `DSP_INPUTS['iq1'] = 7` — formerly `iq2`, renamed since only 2 IQ instances remain
- `DSP_INPUTS['iq1_2'] = 14` — formerly `iq2_2`, the second output of the dual-output IQ
- `RedPitaya.cls_modules` has `[rp.Iq]*2 + [rp.ExtDemod]` producing names
  `iq0`, `iq1`, `extdemod`

## Files Changed (Summary)

### New Files
| File | Description |
|------|-------------|
| `pyrpl/fpga/rtl/red_pitaya_ext_demod_block.v` | Verilog ext_demod module |
| `pyrpl/hardware_modules/ext_demod.py` | Python register bindings |
| `EXT_DEMOD_DESIGN.md` | This document |

### Modified Files
| File | Change |
|------|--------|
| `pyrpl/fpga/rtl/red_pitaya_dsp.v` | Replace IQ1 with ext_demod instantiation; add ref_input_select register |
| `pyrpl/fpga/red_pitaya_vivado.tcl` | Add `read_verilog` for ext_demod_block |
| `pyrpl/hardware_modules/__init__.py` | Import ExtDemod |
| `pyrpl/hardware_modules/dsp.py` | Rename `iq1`→`extdemod` (slot 6), `iq2`→`iq1` (slot 7), `iq2_2`→`iq1_2` |
| `pyrpl/hardware_modules/iq.py` | Update `synchronize_iqs` to sync `iq0, iq1`; update docstring examples |
| `pyrpl/redpitaya.py` | `cls_modules`: 2 Iq + 1 ExtDemod instead of 3 Iq |
| `pyrpl/software_modules/spectrum_analyzer.py` | References `iq1`/`iq1_2` instead of `iq2`/`iq2_2` |

## FPGA Resource Impact

Replacing one IQ block with ext_demod **saves** resources:

| Resource | IQ block (removed) | ext_demod (added) | Net |
|----------|-------------------|-------------------|-----|
| DSP48 slices | ~4 (demod+mod) | 1 (mixer) | −3 |
| BRAM18 tiles | 1 (sin LUT) | 1 (delay line) | 0 |
| LUTs | ~800 | ~200 | −600 |

This is significant on the xc7z010 which has only 17,600 LUTs.

## Known Risks and Debugging Checklist

### 1. BRAM Inference
**Risk**: Vivado might infer the delay line as distributed (LUT) RAM
instead of block RAM.

**Check**: In the synthesis report, look for "Block RAM" utilization.
The `delay_mem` should appear as 1 BRAM18 tile.

**Fix**: If LUT RAM is inferred, add before the `delay_mem` declaration:
```verilog
(* ram_style = "block" *)
reg signed [SIGNALBITS-1:0] delay_mem [0:MAXDELAY-1];
```

### 2. DSP48 Inference
**Risk**: The 14×14 multiply might not map to a DSP48 slice.

**Check**: Synthesis report should show 1 DSP48 used by ext_demod.

**Fix**: Should work automatically since 14×14 fits within DSP48's 25×18
multiplier. If not, the multiply can be wrapped in a DSP48 instantiation.

### 3. Timing Closure
**Risk**: The multiply + LPF chain might not meet 125 MHz timing.

**Check**: Look for negative slack in the timing report involving
`ext_demod` paths.

**Fix**: The pipeline register `dat_i_reg` ensures the multiply has
registered inputs. The BRAM read of `delayed_ref` is also registered.
This should give plenty of slack. If not, add another pipeline stage
after the multiply.

### 4. ref_input_select Read-Back Scope
**Quirk**: Address `0x14` returns `ref_input_select` regardless of which
module's address space is being read. This is because the DSP casez
block matches `20'h14` globally. The `ref_input` register in Python only
reads/writes from the ext_demod's address space (slot 6), so this is
fine in practice.

**If it causes issues**: Make the read-back conditional:
```verilog
20'h14 : begin
    sys_ack <= sys_en;
    if (sys_addr[16+LOG_MODULES-1:16] == EXTDEMOD)
        sys_rdata <= {{32-LOG_MODULES{1'b0}}, ref_input_select};
    else
        sys_rdata <= 32'b0;
end
```

### 5. Output Scaling
**Note**: The output is the top 14 bits of the 24-bit LPF output. This
means the demodulated signal is effectively divided by 2^10 = 1024
relative to the raw mixer product. For a full-scale input and reference
(both at ±8191), the mixer product peak is ~67M, which in 24 bits after
SHIFTBITS=1 is about ±33M. The top 14 bits represent ±8191, so the
output range is correct — a full-scale input × full-scale reference
produces a full-scale output.

Verify this on hardware: with both inputs at full scale and the LPF
off, the output should reach near ±8191 at the optimal phase delay.

### 6. Output Saturation
The output uses simple truncation (MSB select), not saturation. Since
the LPF output magnitude is bounded by the input magnitudes, saturation
should not occur. But if it does:
- The MSB (bit 23) is the sign bit; if it disagrees with bits 22:10,
  the signal has wrapped
- Fix: replace the truncation with `red_pitaya_saturate`

## Build Instructions

```bash
cd pyrpl/fpga
vivado -mode tcl -source red_pitaya_vivado.tcl
# Output: out/red_pitaya.bin
# Copy to Red Pitaya: scp out/red_pitaya.bin root@<rp_ip>:/opt/pyrpl/fpga.bit.bin
```

## Hardware Test Procedure

1. Connect a function generator to both ADC inputs (using a splitter):
   - in1: signal (e.g. from photodiode or test signal)
   - in2: reference (same signal, or the RF modulation signal)

2. Configure in Python:
   ```python
   rp.extdemod.input = 'in1'
   rp.extdemod.ref_input = 'in2'
   rp.extdemod.bandwidth = [1000]  # 1 kHz LPF
   rp.extdemod.delay = 0
   ```

3. Monitor output:
   ```python
   rp.scope.input1 = 'extdemod'
   ```

4. Sweep delay and observe output amplitude change:
   ```python
   for d in range(20):
       rp.extdemod.delay = d
       time.sleep(0.1)
       print(f"delay={d}, phase={rp.extdemod.delay_to_phase(29.5e6, d):.1f}°, "
             f"output={rp.extdemod.current_output_signal:.4f}")
   ```

5. The output should show a sinusoidal dependence on delay, with period
   equal to one cycle of the reference frequency in samples.
