/*
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
*/

/*
 * DESIGN NOTES FOR FUTURE DEBUGGING (no Vivado available at dev time)
 * ===================================================================
 *
 * PURPOSE:
 *   Demodulate an input signal (e.g. photodiode) using an EXTERNAL reference
 *   signal (e.g. from a function generator driving an EOM), rather than the
 *   internal NCO used by the standard IQ block. This enables PDH locking
 *   with phase-coherent demodulation without hardware modifications.
 *
 * ARCHITECTURE:
 *   ext_ref_i --> [delay line (BRAM)] --> delayed_ref
 *   dat_i     --> [mixer] * delayed_ref --> [LPF] --> [truncate] --> signal_o
 *
 *   The delay line provides adjustable phase shift: for reference frequency f,
 *   delay of N samples gives phase = 2*pi*f*N/125e6.
 *
 * BIT-WIDTH ARITHMETIC (critical for Vivado debugging):
 *   - Input signals: 14-bit signed (dat_i, ext_ref_i)
 *   - Mixer product: 14+14 = 28 bits signed
 *   - Rounding offset: 1 << (28-LPFBITS-SHIFTBITS-1) = 1 << 2
 *     where SHIFTBITS=1 (same rationale as iq_demodulator_block:
 *     14-bit signals exclude the most-negative value, so the product
 *     occupies at most 14+14-2 = 26 bits + sign = 27 bits, making
 *     SHIFTBITS=1 safe)
 *   - After rounding + bit-select: LPFBITS (24) bits
 *   - LPF operates at 24-bit internal width
 *   - Output truncation: 24 -> 14 bits (take MSBs)
 *
 * RESOURCE USAGE (estimated):
 *   - 1 DSP48 slice (14x14 multiplier, inferred)
 *   - 1 BRAM18 tile (1024 x 14-bit = 14 Kbit, fits in 18 Kbit BRAM)
 *   - ~200 LUTs (filter + control)
 *   - Much less than one IQ block (which uses 4 DSP48 slices)
 *
 * DELAY LINE IMPLEMENTATION:
 *   Uses a simple dual-port BRAM inferred by Vivado from reg array.
 *   Write pointer advances every clock; read pointer = write_ptr - delay.
 *   IMPORTANT: The delay register is 10 bits (0-1023). Setting delay=0
 *   means the read pointer equals the write pointer, giving 1 clock cycle
 *   of latency (because we read the just-written value next cycle).
 *
 * TIMING:
 *   Total pipeline latency from dat_i to signal_o:
 *   - 1 cycle: delay line read (BRAM output registered)
 *   - 1 cycle: firstproduct_reg registration (pipeline stage)
 *   - 1 cycle: product multiplication + rounding
 *   - N cycles: LPF latency (depends on filter_block)
 *   - 0 cycles: output truncation (combinational)
 *   Total: 3 + LPF_latency cycles
 *
 * REGISTER MAP:
 *   The ext_demod block uses addresses 0x100-0x13F within its module
 *   address space (0x40300000 + slot*0x10000 + offset). Addresses
 *   0x00-0x0F are reserved by red_pitaya_dsp.v for input_select,
 *   output_select, saturation, and sync registers.
 *
 *   0x100: delay [9:0]       - Delay in samples (0 to 1023)
 *   0x104: lpf_config [31:0] - Low-pass filter config (same format as
 *                               red_pitaya_filter_block set_filter)
 *   0x108: ref_input_select [3:0] - Which DSP bus signal feeds ext_ref_i
 *                                    (handled in red_pitaya_dsp.v, not here)
 *
 *   Read-back registers (active-low reset values shown):
 *   0x200: SIGNALBITS parameter (14)
 *   0x204: LPFBITS parameter (24)
 *   0x208: MAXDELAY parameter (1024)
 *   0x20C: FILTERSTAGES parameter (2)
 *   0x210: FILTERSHIFTBITS parameter (5)
 *   0x214: FILTERMINBW parameter (10)
 *
 * KNOWN RISKS / THINGS TO CHECK IN VIVADO:
 *   1. BRAM inference: The delay_mem array should be inferred as BRAM.
 *      If Vivado uses LUT RAM instead, add (* ram_style = "block" *)
 *      attribute before the reg declaration.
 *   2. DSP48 inference: The 14x14 multiply should map to one DSP48.
 *      Check synthesis report for DSP48 usage.
 *   3. Timing: The mixer multiply is 14x14 -> 28 bits which is well
 *      within DSP48 capability (25x18). No timing issues expected.
 *   4. The filter_block's SIGNALBITS is set to LPFBITS (24), not 14.
 *      This is intentional - we want full precision through the LPF
 *      before truncating at the output.
 *   5. Output saturation: We use simple truncation (MSB select) rather
 *      than saturation. Since the LPF output is bounded by the mixer
 *      product magnitude (which is bounded by input magnitudes), and
 *      the LPF only reduces magnitude, saturation should not occur in
 *      practice. If it does, add red_pitaya_saturate.
 */

module red_pitaya_ext_demod_block #(
    parameter SIGNALBITS       = 14,
    parameter LPFBITS          = 24,
    parameter MAXDELAY         = 1024,  // max delay in samples
    parameter DELAYBITS        = 10,    // log2(MAXDELAY)
    parameter SHIFTBITS        = 1,     // right-shift for product (see iq_demodulator_block)
    parameter FILTERSTAGES     = 2,
    parameter FILTERSHIFTBITS  = 5,
    parameter FILTERMINBW      = 10
)
(
    input                          clk_i,
    input                          rstn_i,
    input  signed [SIGNALBITS-1:0] dat_i,      // signal to demodulate
    input  signed [SIGNALBITS-1:0] ext_ref_i,  // external reference
    output signed [SIGNALBITS-1:0] dat_o,       // output_direct (for DAC summing)
    output signed [SIGNALBITS-1:0] signal_o,    // output_signal (for DSP bus routing)

    // AXI register interface (active addresses 0x100+)
    input  [16-1:0] addr,
    input           wen,
    input           ren,
    output reg      ack,
    output reg [32-1:0] rdata,
    input      [32-1:0] wdata
);

// =========================================================================
// Registers
// =========================================================================
reg [DELAYBITS-1:0] delay;          // 0x100: delay in samples
reg [32-1:0]        lpf_config;     // 0x104: LPF configuration

// Register write
always @(posedge clk_i) begin
    if (rstn_i == 1'b0) begin
        delay      <= {DELAYBITS{1'b0}};
        lpf_config <= 32'b0;
    end
    else if (wen) begin
        if (addr == 16'h100) delay      <= wdata[DELAYBITS-1:0];
        if (addr == 16'h104) lpf_config <= wdata;
    end
end

// Register read
wire sys_en;
assign sys_en = wen | ren;
always @(posedge clk_i) begin
    if (rstn_i == 1'b0) begin
        ack   <= 1'b0;
        rdata <= 32'b0;
    end
    else begin
        ack <= sys_en;
        casez (addr)
            16'h100 : rdata <= {{32-DELAYBITS{1'b0}}, delay};
            16'h104 : rdata <= lpf_config;
            // Read-only parameter registers (for Python driver introspection)
            16'h200 : rdata <= SIGNALBITS;
            16'h204 : rdata <= LPFBITS;
            16'h208 : rdata <= MAXDELAY;
            16'h20C : rdata <= FILTERSTAGES;
            16'h210 : rdata <= FILTERSHIFTBITS;
            16'h214 : rdata <= FILTERMINBW;
            default : rdata <= 32'b0;
        endcase
    end
end

// =========================================================================
// 1. Delay line (circular buffer in BRAM)
// =========================================================================
// Vivado should infer this as a BRAM. If it doesn't, add:
//   (* ram_style = "block" *)
// before this declaration.
reg signed [SIGNALBITS-1:0] delay_mem [0:MAXDELAY-1];

reg [DELAYBITS-1:0] write_ptr;
wire [DELAYBITS-1:0] read_ptr;
reg signed [SIGNALBITS-1:0] delayed_ref;

assign read_ptr = write_ptr - delay;

always @(posedge clk_i) begin
    if (rstn_i == 1'b0) begin
        write_ptr <= {DELAYBITS{1'b0}};
        delayed_ref <= {SIGNALBITS{1'b0}};
    end
    else begin
        // Write the current reference into BRAM
        delay_mem[write_ptr] <= ext_ref_i;
        // Read the delayed reference from BRAM (1-cycle read latency)
        delayed_ref <= delay_mem[read_ptr];
        // Advance write pointer
        write_ptr <= write_ptr + 1'b1;
    end
end

// =========================================================================
// 2. Mixer (14-bit x 14-bit -> 24-bit with rounding)
// =========================================================================
// Following the pattern from red_pitaya_iq_demodulator_block.v:
//   - Pipeline the input data for timing
//   - Multiply + add rounding offset
//   - Bit-select the output
//
// Bit widths:
//   product = dat_i[13:0] * delayed_ref[13:0] = 28 bits signed
//   We want LPFBITS (24) output bits.
//   SHIFTBITS = 1 (safe because 14-bit signals exclude most-negative)
//   Round offset = 1 << (SIGNALBITS+SIGNALBITS - LPFBITS - SHIFTBITS - 1)
//                = 1 << (14+14-24-1-1) = 1 << 2
//   Output bit range: [27-1 : 27-24-1+1] = [26:4] with SHIFTBITS=1
//     -> [SIGNALBITS+SIGNALBITS-1-SHIFTBITS : SIGNALBITS+SIGNALBITS-LPFBITS-SHIFTBITS]
//     -> [26 : 4]

localparam PRODUCTBITS = SIGNALBITS + SIGNALBITS;  // 28
localparam ROUND_OFFSET = 1 << (PRODUCTBITS - LPFBITS - SHIFTBITS - 1);  // 1 << 2 = 4

reg signed [SIGNALBITS-1:0] dat_i_reg;  // pipeline register for dat_i
reg signed [PRODUCTBITS-1:0] product;

always @(posedge clk_i) begin
    dat_i_reg <= dat_i;
end

always @(posedge clk_i) begin
    product <= dat_i_reg * delayed_ref + $signed(ROUND_OFFSET);
end

wire signed [LPFBITS-1:0] mixer_out;
assign mixer_out = product[PRODUCTBITS-1-SHIFTBITS : PRODUCTBITS-LPFBITS-SHIFTBITS];

// =========================================================================
// 3. Low-pass filter
// =========================================================================
wire signed [LPFBITS-1:0] lpf_out;

red_pitaya_filter_block #(
    .STAGES(FILTERSTAGES),
    .SHIFTBITS(FILTERSHIFTBITS),
    .SIGNALBITS(LPFBITS),
    .MINBW(FILTERMINBW)
) lpf (
    .clk_i(clk_i),
    .rstn_i(rstn_i),
    .set_filter(lpf_config),
    .dat_i(mixer_out),
    .dat_o(lpf_out)
);

// =========================================================================
// 4. Output truncation (24-bit -> 14-bit, take MSBs)
// =========================================================================
// The LPF output is LPFBITS wide. We take the top SIGNALBITS bits.
// This is equivalent to dividing by 2^(LPFBITS-SIGNALBITS) = 2^10 = 1024.
//
// DESIGN NOTE: The mixer output magnitude is at most
//   (2^13 - 1) * (2^13 - 1) ≈ 2^26, represented in 24 bits after
//   the SHIFTBITS=1 right-shift. The LPF preserves or reduces this.
//   Taking the top 14 bits means the output represents the demodulated
//   signal scaled by 1/1024 relative to the raw mixer output.
//   This matches the IQ block's behavior where quadrature outputs
//   are LPFBITS wide internally and truncated for the DSP bus.
wire signed [SIGNALBITS-1:0] output_truncated;
assign output_truncated = lpf_out[LPFBITS-1 : LPFBITS-SIGNALBITS];

// Both outputs carry the same signal
assign signal_o = output_truncated;
assign dat_o    = output_truncated;

endmodule
