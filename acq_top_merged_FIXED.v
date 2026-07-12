// ============================================================
// Module 9: Full Acquisition Top Level
// ============================================================
// Connects the complete acquisition chain:
//
//   M1 Carrier NCO
//   M2 Carrier Mixer
//   M3 Code Generator + Code Shift Register
//   M4 Matched Filter Bank
//   M5 Power Calculation Bank
//   M6 Acquisition RAM
//   M7 Peak Detector
//   M8 Acquisition Controller
//
// The controller sweeps Doppler/frequency bins.  For each bin, the
// datapath integrates one code epoch, stores all N code-phase powers
// into RAM, then moves to the next frequency bin.  After all bins are
// stored, the peak detector scans RAM and reports the best cell.
// ============================================================

module acq_top #(
    parameter N                = 1023,
    parameter NUM_FREQ         = 21,
    parameter ADC_WIDTH        = 3,
    parameter NCO_WIDTH        = 5,
    parameter MIX_WIDTH        = ADC_WIDTH + NCO_WIDTH,
    parameter CORR_WIDTH       = 18,
    parameter POWER_WIDTH      = (2 * CORR_WIDTH) + 1,
    parameter ACCUM_WIDTH      = 16,
    parameter LUT_DEPTH        = 256,
    parameter FCW_WIDTH        = 16,
    parameter FCW_START        = 16'd0,
    parameter FCW_STEP         = 16'd4096,
    parameter THRESHOLD_SHIFT  = 1,
    parameter RAM_DEPTH        = N * NUM_FREQ,
    parameter RAM_ADDR_WIDTH   = $clog2(RAM_DEPTH)
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         start,

    // PRN satellite selection.  PRN-1 uses tap1=2, tap2=6.
    input  wire [3:0]                   tap1,
    input  wire [3:0]                   tap2,

    // Digitized RF/IF input from ADC.
    input  wire signed [ADC_WIDTH-1:0]  adc_in,

    // Final acquisition result.
    output wire                         acq_valid,
    output wire [$clog2(N)-1:0]         best_code_phase,
    output wire [$clog2(NUM_FREQ)-1:0]  best_freq_bin,
    output wire                         busy,
    output wire                         done,

    // Debug/visibility outputs used by the final testbench.
    output wire                         datapath_enable,
    output wire [FCW_WIDTH-1:0]         current_fcw,
    output wire [$clog2(NUM_FREQ)-1:0]  current_freq_bin,
    output wire                         power_valid,
    output wire                         write_done,
    output wire                         scan_start
);

// ------------------------------------------------------------
// Controller wires
// ------------------------------------------------------------
wire scan_done;

// ------------------------------------------------------------
// Datapath wires
// ------------------------------------------------------------
wire signed [NCO_WIDTH-1:0] sine_out;
wire signed [NCO_WIDTH-1:0] cos_out;

wire signed [MIX_WIDTH-1:0] I_mix;
wire signed [MIX_WIDTH-1:0] Q_mix;

wire code_chip;
wire epoch_pulse;
wire [N-1:0] parallel_code;

wire [(CORR_WIDTH*N)-1:0] I_corr_bus;
wire [(CORR_WIDTH*N)-1:0] Q_corr_bus;
wire [(POWER_WIDTH*N)-1:0] power_bus;

// ------------------------------------------------------------
// RAM / peak-detector wires
// ------------------------------------------------------------
wire [RAM_ADDR_WIDTH-1:0] ram_addr_b;
wire [POWER_WIDTH-1:0]    ram_dout_b;

// ------------------------------------------------------------
// M8: Acquisition Controller
// ------------------------------------------------------------
acq_control #(
    .NUM_FREQ  (NUM_FREQ),
    .FCW_WIDTH (FCW_WIDTH),
    .FCW_START (FCW_START),
    .FCW_STEP  (FCW_STEP)
) u_control (
    .clk        (clk),
    .reset_n    (rst_n),
    .start      (start),
    .power_valid(power_valid),
    .write_done (write_done),
    .scan_done  (scan_done),
    .enable     (datapath_enable),
    .fcw        (current_fcw),
    .freq_bin   (current_freq_bin),
    .scan_start (scan_start),
    .busy       (busy),
    .done       (done)
);

// ------------------------------------------------------------
// M1: Carrier NCO
// ------------------------------------------------------------
carrier_nco #(
    .ACCUM_WIDTH    (ACCUM_WIDTH),
    .LUT_ADDR_WIDTH (8),
    .LUT_DEPTH      (LUT_DEPTH),
    .AMP_WIDTH      (NCO_WIDTH)
) u_nco (
    .clk      (clk),
    .rst_n    (rst_n),
    .fcw      (current_fcw),
    .sine_out (sine_out),
    .cos_out  (cos_out)
);

// ------------------------------------------------------------
// M2: Carrier Mixer
// ------------------------------------------------------------
carrier_mixer #(
    .ADC_WIDTH (ADC_WIDTH),
    .NCO_WIDTH (NCO_WIDTH),
    .OUT_WIDTH (MIX_WIDTH)
) u_mixer (
    .clk    (clk),
    .rst_n  (rst_n),
    .adc_in (adc_in),
    .cos_in (cos_out),
    .sin_in (sine_out),
    .I_out  (I_mix),
    .Q_out  (Q_mix)
);

// ------------------------------------------------------------
// M3a: Code Generator
// ------------------------------------------------------------
code_generator u_codegen (
    .clk         (clk),
    .reset_n     (rst_n),
    .enable      (datapath_enable),
    .tap1        (tap1),
    .tap2        (tap2),
    .epoch_pulse (epoch_pulse),
    .out         (code_chip)
);

// ------------------------------------------------------------
// M3b: Code Shift Register
// ------------------------------------------------------------
code_shift_register #(
    .N (N)
) u_shift_reg (
    .clk        (clk),
    .reset_n    (rst_n),
    .enable     (datapath_enable),
    .codegenout (code_chip),
    .out        (parallel_code)
);

// ------------------------------------------------------------
// M4: Matched Filter Bank
// ------------------------------------------------------------
matched_filter_bank #(
    .N (N)
) u_mf_bank (
    .clk           (clk),
    .reset_n       (rst_n),
    .enable        (datapath_enable),
    .parallel_code (parallel_code),
    .I_in          (I_mix),
    .Q_in          (Q_mix),
    .epoch_pulse   (epoch_pulse),
    .I_out         (I_corr_bus),
    .Q_out         (Q_corr_bus)
);

// ------------------------------------------------------------
// M5: Power Calculation Bank
// ------------------------------------------------------------
power_calc_bank #(
    .N           (N),
    .CORR_WIDTH  (CORR_WIDTH),
    .POWER_WIDTH (POWER_WIDTH)
) u_power_bank (
    .clk         (clk),
    .reset_n     (rst_n),
    .enable      (datapath_enable),
    .epoch_pulse (epoch_pulse),
    .I_in        (I_corr_bus),
    .Q_in        (Q_corr_bus),
    .power_out   (power_bus),
    .power_valid (power_valid)
);

// ------------------------------------------------------------
// M6: Acquisition RAM
// ------------------------------------------------------------
acq_ram #(
    .N          (N),
    .NUM_FREQ   (NUM_FREQ),
    .DATA_WIDTH (POWER_WIDTH),
    .DEPTH      (RAM_DEPTH),
    .ADDR_WIDTH (RAM_ADDR_WIDTH)
) u_ram (
    .clk         (clk),
    .reset_n     (rst_n),
    .power_valid (power_valid),
    .power_bus   (power_bus),
    .freq_bin    (current_freq_bin),
    .addr_b      (ram_addr_b),
    .dout_b      (ram_dout_b),
    .write_done  (write_done)
);

// ------------------------------------------------------------
// M7: Peak Detector
// ------------------------------------------------------------
peak_detector #(
    .N               (N),
    .NUM_FREQ        (NUM_FREQ),
    .DATA_WIDTH      (POWER_WIDTH),
    .DEPTH           (RAM_DEPTH),
    .ADDR_WIDTH      (RAM_ADDR_WIDTH),
    .THRESHOLD_SHIFT (THRESHOLD_SHIFT)
) u_peak_detector (
    .clk             (clk),
    .reset_n         (rst_n),
    .scan_start      (scan_start),
    .ram_addr        (ram_addr_b),
    .ram_dout        (ram_dout_b),
    .acq_valid       (acq_valid),
    .best_code_phase (best_code_phase),
    .best_freq_bin   (best_freq_bin),
    .scan_done       (scan_done)
);

endmodule
module acq_ram #(
    parameter N          = 1023,
    parameter NUM_FREQ   = 21,
    parameter DATA_WIDTH = 37,
    parameter DEPTH      = N * NUM_FREQ,
    parameter ADDR_WIDTH = $clog2(DEPTH)
)(
    input  wire                    clk,
    input  wire                    reset_n,

    // ---- Port A (write side) ----
    input  wire                    power_valid,     // pulse from Module 5
    input  wire [(DATA_WIDTH*N)-1:0] power_bus,    // all N power values, flat
    input  wire [$clog2(NUM_FREQ)-1:0] freq_bin,   // current Doppler bin index

    // ---- Port B (read side) ----
    input  wire [ADDR_WIDTH-1:0]   addr_b,         // read address from Module 7
    output reg  [DATA_WIDTH-1:0]   dout_b,         // read data (1-cycle latency)

    // ---- Handshake ----
    output reg                     write_done       // pulses high when write loop ends
);


// 1. Memory array

reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];


// 2. Write controller registers

reg                    writing;                     // high while write loop active
reg [$clog2(N)-1:0]   wr_idx;                      // counts 0 to N-1
reg [ADDR_WIDTH-1:0]  wr_base;                     // freq_bin * N, latched once


// 3. Port A -- write controller state machine

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        writing    <= 1'b0;
        wr_idx     <= {$clog2(N){1'b0}};
        wr_base    <= {ADDR_WIDTH{1'b0}};
        write_done <= 1'b0;
    end else begin
        // default: deassert write_done every cycle
        write_done <= 1'b0;

        if (!writing) begin
            // ----- IDLE state -----
            if (power_valid) begin
                writing <= 1'b1;
                wr_idx  <= {$clog2(N){1'b0}};
                wr_base <= freq_bin * N;           // latch base address once
            end
        end else begin
            // ----- WRITING state -----
            // Perform the actual memory write
            mem[wr_base + wr_idx] <=
                power_bus[(wr_idx * DATA_WIDTH) +: DATA_WIDTH];

            if (wr_idx == N - 1) begin
                // last tap written, go back to IDLE
                writing    <= 1'b0;
                write_done <= 1'b1;
            end else begin
                wr_idx <= wr_idx + 1'b1;
            end
        end
    end
end


// 4. Port B -- synchronous read (BRAM inference)

always @(posedge clk) begin
    dout_b <= mem[addr_b];
end

endmodule
module acq_control #(
    parameter NUM_FREQ  = 21,
    parameter FCW_WIDTH = 16,
    parameter FCW_START = 16'd0,
    parameter FCW_STEP  = 16'd1
)(
    input  wire                         clk,
    input  wire                         reset_n,

    input  wire                         start,

    input  wire                         power_valid,
    input  wire                         write_done,
    input  wire                         scan_done,

    output reg                          enable,
    output reg  [FCW_WIDTH-1:0]         fcw,
    output reg  [$clog2(NUM_FREQ)-1:0]  freq_bin,
    output reg                          scan_start,
    output reg                          busy,
    output reg                          done
);

    // State definitions
    localparam [2:0] IDLE       = 3'd0,
                     SET_BIN    = 3'd1,
                     WAIT_EPOCH = 3'd2,
                     WAIT_WRITE = 3'd3,
                     NEXT_BIN   = 3'd4,
                     START_SCAN = 3'd5,
                     WAIT_SCAN  = 3'd6,
                     DONE       = 3'd7;

    reg [2:0] state;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state      <= IDLE;
            enable     <= 1'b0;
            fcw        <= {FCW_WIDTH{1'b0}};
            freq_bin   <= 0;
            scan_start <= 1'b0;
            busy       <= 1'b0;
            done       <= 1'b0;
        end else begin
            // Default pulse values
            scan_start <= 1'b0;
            done       <= 1'b0;

            case (state)
                IDLE: begin
                    enable <= 1'b0;
                    if (start) begin
                        busy     <= 1'b1;
                        freq_bin <= 0;
                        state    <= SET_BIN;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                SET_BIN: begin
                    // Compute the frequency control word for the current bin
                    fcw    <= FCW_START + (freq_bin * FCW_STEP);
                    state  <= WAIT_EPOCH;
                end

                WAIT_EPOCH: begin
                    enable <= 1'b1; // Start/keep datapath running
                    if (power_valid) begin
                        enable <= 1'b0; // Pause datapath during RAM write
                        state  <= WAIT_WRITE;
                    end
                end

                WAIT_WRITE: begin
                    if (write_done) begin
                        state <= NEXT_BIN;
                    end
                end

                NEXT_BIN: begin
                    if (freq_bin < NUM_FREQ - 1) begin
                        freq_bin <= freq_bin + 1'b1;
                        state    <= SET_BIN;
                    end else begin
                        // All bins have been written to RAM
                        state <= START_SCAN;
                    end
                end

                START_SCAN: begin
                    scan_start <= 1'b1; // Pulse scan_start
                    state      <= WAIT_SCAN;
                end

                WAIT_SCAN: begin
                    if (scan_done) begin
                        state <= DONE;
                    end
                end

                DONE: begin
                    done  <= 1'b1; // Pulse done
                    busy  <= 1'b0;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
module power_calc_unit #(
    parameter CORR_WIDTH = 18,
    parameter POWER_WIDTH = (2*CORR_WIDTH) + 1
)(
    input wire clk,
    input wire reset_n,
    input wire enable,
    input wire epoch_pulse,
    input wire signed [CORR_WIDTH-1:0] I_in,
    input wire signed [CORR_WIDTH-1:0] Q_in,
    output reg [POWER_WIDTH-1:0] power_out,
    output reg power_valid
);

localparam SQ_WIDTH = 2*CORR_WIDTH;

wire [SQ_WIDTH-1:0] I_sq = I_in*I_in;
wire [SQ_WIDTH-1:0] Q_sq = Q_in*Q_in;
wire [POWER_WIDTH-1:0] I_sq_ext = {{(POWER_WIDTH-SQ_WIDTH){1'b0}}, I_sq};
wire [POWER_WIDTH-1:0] Q_sq_ext = {{(POWER_WIDTH-SQ_WIDTH){1'b0}}, Q_sq};
reg calc_valid;

always @(posedge clk or negedge reset_n) begin
    if(!reset_n) begin
       power_out <= {POWER_WIDTH{1'b0}};
       power_valid <=1'b0;
       calc_valid <=1'b0;
    end
    else if (enable) begin
        calc_valid <= epoch_pulse;

        if (calc_valid) begin
            power_out <= I_sq_ext + Q_sq_ext;
            power_valid <=1'b1;
        end
        else
            power_valid <=1'b0;
    end
    else begin
        power_out <= {POWER_WIDTH{1'b0}};
        power_valid <=1'b0;
        calc_valid <=1'b0;
    end
    
end
endmodule
module power_calc_bank #(
    parameter N = 1023,
    parameter CORR_WIDTH = 18,
    parameter POWER_WIDTH = (2*CORR_WIDTH) + 1
)(
    input wire clk,
    input wire reset_n,
    input wire enable,
    input wire epoch_pulse,
    input wire [(CORR_WIDTH*N)-1:0] I_in,
    input wire [(CORR_WIDTH*N)-1:0] Q_in,
    output wire [(POWER_WIDTH*N)-1:0] power_out,
    output wire power_valid
);

wire [N-1:0] power_valid_bus;

genvar i;
generate
    for (i = 0; i < N; i = i + 1) begin : power_loop
        wire signed [CORR_WIDTH-1:0] I_slice;
        wire signed [CORR_WIDTH-1:0] Q_slice;

        assign I_slice = I_in[(i*CORR_WIDTH)+CORR_WIDTH-1 : i*CORR_WIDTH];
        assign Q_slice = Q_in[(i*CORR_WIDTH)+CORR_WIDTH-1 : i*CORR_WIDTH];

        power_calc_unit #(
            .CORR_WIDTH(CORR_WIDTH),
            .POWER_WIDTH(POWER_WIDTH)
        ) unit_inst (
            .clk(clk),
            .reset_n(reset_n),
            .enable(enable),
            .epoch_pulse(epoch_pulse),
            .I_in(I_slice),
            .Q_in(Q_slice),
            .power_out(power_out[(i*POWER_WIDTH)+POWER_WIDTH-1 : i*POWER_WIDTH]),
            .power_valid(power_valid_bus[i])
        );
    end
endgenerate

assign power_valid = power_valid_bus[0];

endmodule
module peak_detector #(
    parameter N               = 1023,
    parameter NUM_FREQ        = 21,
    parameter DATA_WIDTH      = 37,
    parameter DEPTH           = N * NUM_FREQ,
    parameter ADDR_WIDTH      = $clog2(DEPTH),
    parameter THRESHOLD_SHIFT = 1
)(
    input  wire                          clk,
    input  wire                          reset_n,

    // ---- Control ----
    input  wire                          scan_start,    // pulse to begin scanning

    // ---- RAM Port B interface ----
    output reg  [ADDR_WIDTH-1:0]         ram_addr,      // read address to acq_ram
    input  wire [DATA_WIDTH-1:0]         ram_dout,      // read data from acq_ram (1-cycle latency)

    // ---- Results ----
    output reg                           acq_valid,     // high if peak passed threshold
    output reg  [$clog2(N)-1:0]          best_code_phase,
    output reg  [$clog2(NUM_FREQ)-1:0]   best_freq_bin,
    output reg                           scan_done      // pulses high when scan + decision complete
);


    // State encoding

    localparam [1:0] S_IDLE     = 2'd0,
                     S_PRIME    = 2'd1,   // 1-cycle pipeline priming
                     S_SCAN     = 2'd2,
                     S_DECISION = 2'd3;

    reg [1:0] state;


    // Scan counters (dual-counter approach — avoids division)

    reg [$clog2(N)-1:0]        scan_tap;
    reg [$clog2(NUM_FREQ)-1:0] scan_freq;

    // 1-cycle delayed tap/freq tracks which cell's data is arriving.
    reg [$clog2(N)-1:0]        prev_tap;
    reg [$clog2(NUM_FREQ)-1:0] prev_freq;


    // Max tracking registers

    reg [DATA_WIDTH-1:0] max1, max2;
    reg [$clog2(N)-1:0]        max1_tap;
    reg [$clog2(NUM_FREQ)-1:0] max1_freq;


    // Address computation: ram_addr = (scan_freq × N) + scan_tap

    wire [ADDR_WIDTH-1:0] scan_address = (scan_freq * N) + scan_tap;


    // Flags

    wire last_tap  = (scan_tap  == N - 1);
    wire last_freq = (scan_freq == NUM_FREQ - 1);
    wire last_cell = last_tap && last_freq;


    // Main FSM

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state           <= S_IDLE;
            scan_tap        <= 0;
            scan_freq       <= 0;
            prev_tap        <= 0;
            prev_freq       <= 0;
            max1            <= 0;
            max2            <= 0;
            max1_tap        <= 0;
            max1_freq       <= 0;
            ram_addr        <= 0;
            acq_valid       <= 1'b0;
            best_code_phase <= 0;
            best_freq_bin   <= 0;
            scan_done       <= 1'b0;
        end else begin
            prev_tap  <= scan_tap;
            prev_freq <= scan_freq;

            // Default: deassert one-cycle pulses
            scan_done <= 1'b0;

            case (state)
                
                // IDLE — wait for scan_start
                
                S_IDLE: begin
                    if (scan_start) begin
                        // Zero everything for a fresh scan
                        scan_tap   <= 0;
                        scan_freq  <= 0;
                        max1       <= 0;
                        max2       <= 0;
                        max1_tap   <= 0;
                        max1_freq  <= 0;
                        acq_valid  <= 1'b0;

                        // Present first address to RAM
                        ram_addr   <= 0;       // address 0 = (freq=0 × N) + tap=0

                        state      <= S_PRIME;
                    end
                end

                
                // PRIME — wait 1 cycle for RAM read pipeline to fill
                //   addr=0 was presented last cycle; ram_dout will have
                //   mem[0] at the END of this cycle.  Meanwhile, present
                //   addr=1 so that data is ready next cycle.
                
                S_PRIME: begin
                    // Advance to second address
                    scan_tap <= scan_tap + 1'b1;
                    ram_addr <= scan_address + 1;  // address 1

                    state    <= S_SCAN;
                end

                
                // SCAN — compare ram_dout (data from PREVIOUS address)
                //        and present the next address
                
                S_SCAN: begin
                    // --- Max1 / Max2 comparison ---
                    // ram_dout now holds data for the address presented
                    // TWO cycles ago (i.e. the cell we want to compare).
                    //
                    // The "current comparison" address is one behind
                    // the address we're about to present.
                    // We track which tap/freq we're comparing using
                    // delayed versions, but since we increment AFTER
                    // the comparison, scan_tap-1/scan_freq is the cell
                    // whose data just arrived.  However, it's simpler
                    // to latch the tap/freq at the time we present the
                    // address and keep a 1-stage pipeline.  Here we
                    // use a simpler approach: we latch on new-max.

                    if (ram_dout > max1) begin
                        max2      <= max1;
                        max1      <= ram_dout;
                        // The data arriving now was for the address
                        // presented LAST cycle.  At that time, the
                        // counters had their previous values. We need
                        // to track those.  Because we incremented
                        // scan_tap at the end of the previous cycle,
                        // the "previous" tap is scan_tap - 1 (with
                        // wrap).  Instead of subtracting, we keep a
                        // 1-cycle delayed copy (see below).
                        max1_tap  <= prev_tap;
                        max1_freq <= prev_freq;
                    end else if (ram_dout > max2) begin
                        max2      <= ram_dout;
                    end

                    // --- Check if we just processed the LAST cell ---
                    // prev_tap/prev_freq represent the cell whose data
                    // arrived this cycle.  If that was the last cell,
                    // we're done scanning.
                    if (prev_tap == N - 1 && prev_freq == NUM_FREQ - 1) begin
                        state <= S_DECISION;
                    end else begin
                        // Advance to next address
                        if (last_tap) begin
                            scan_tap  <= 0;
                            scan_freq <= scan_freq + 1'b1;
                        end else begin
                            scan_tap  <= scan_tap + 1'b1;
                        end
                        ram_addr <= (last_tap)
                                    ? ((scan_freq + 1'b1) * N)
                                    : scan_address + 1;
                    end
                end

                
                // DECISION — threshold test and output results
                
                S_DECISION: begin
                    // Threshold test: max1 > (max2 << THRESHOLD_SHIFT)
                    // For THRESHOLD_SHIFT=1 this is max1 > 2×max2
                    acq_valid       <= (max1 > (max2 << THRESHOLD_SHIFT));
                    best_code_phase <= max1_tap;
                    best_freq_bin   <= max1_freq;
                    scan_done       <= 1'b1;

                    state           <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
module matched_filter_bank #(
    parameter N=1023
)(
    input wire clk,
    input wire reset_n,
    input wire enable,
    input wire [N-1:0] parallel_code,
    input wire signed [7:0] I_in,
    input wire signed [7:0] Q_in,
    input wire epoch_pulse,
    output wire [(18*N) -1:0] I_out,
    output wire [(18*N) -1:0] Q_out
);
genvar i;
generate
for (i=0;i<N;i=i+1) begin : tap_loop
correlator_tap tap_inst(
    .clk(clk),
    .reset_n(reset_n),
    .enable(enable),
    .epoch_pulse(epoch_pulse),
    .code_chip(parallel_code[i]),
    .I_in(I_in),
    .Q_in(Q_in),
    .I_out(I_out[(i*18)+17:i*18]),
    .Q_out(Q_out[(i*18)+17:i*18])
);
end
endgenerate
endmodule


module correlator_tap (
    input wire clk,
    input wire reset_n,
    input wire enable,
    input wire epoch_pulse,
    input wire code_chip,
    input wire signed [7:0] I_in,
    input wire signed [7:0] Q_in,
    output reg signed [17:0] I_out,
    output reg signed [17:0] Q_out
);
// first lets conver 8 bit into 18 bit signed
wire signed [17:0] I_ext = I_in;
wire signed [17:0] Q_ext = Q_in;
// multiplication
wire signed [17:0] I_corr = (code_chip == 0) ? I_ext : -I_ext;
wire signed [17:0] Q_corr = (code_chip == 0) ? Q_ext : -Q_ext;
// Accumulator 
reg signed [17:0] I_acc;
reg signed [17:0] Q_acc;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        I_out <= 18'd0;
        Q_out <= 18'd0;
        I_acc <= 18'd0;
        Q_acc <= 18'd0;
    end
    
    else if(enable) begin
        
        if(!epoch_pulse) begin
            I_acc <= I_acc + I_corr ;
            Q_acc <= Q_acc + Q_corr ;
        end
        else begin
            I_out <= I_acc;
            Q_out <= Q_acc;
            
            // Start the next 1023-chip epoch with the current chip
            I_acc <= I_corr;
            Q_acc <= Q_corr;
        end
    end

    else begin
        I_out <= 18'd0;
        Q_out <= 18'd0;
        I_acc <= 18'd0;
        Q_acc <= 18'd0;

        end
    end
endmodule






module code_shift_register #(
    parameter N=1023
)(
    input wire clk,
    input wire codegenout,
    input wire reset_n,
    input wire enable,
    output reg [N-1:0] out
);


always@(posedge clk) begin
    if(!reset_n)
    out<=0;
    else if (enable) begin
        out<= {out[N-2:0],codegenout};
    end
end

endmodule
module code_generator (
    input  wire clk,
    input wire reset_n,
    input wire enable,
    input wire [3:0] tap1,
    input wire [3:0] tap2,
    output reg epoch_pulse,
    output reg out
);

reg [10:1] G1;
reg [10:1] G2;
reg [9:0] epoch;
integer i;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        G1 <= 10'b1111111111;
        G2 <= 10'b1111111111;
        epoch<= 10'd0;
        epoch_pulse<=1'b0;
        out <= 1'b0;
    end
    
    else if (enable) begin
        if (epoch == 10'd1022) begin
        epoch_pulse <= 1'b1 ;
        epoch <= 10'd0;
        end
        else begin
            epoch <= epoch + 1'b1;
            epoch_pulse <= 0;
        end

        for (i=1; i<10; i=i+1) begin
            G1[i+1]<= G1[i];
            G2[i+1]<= G2[i];
        end
        G1[1]<=G1[10]^G1[3];
        G2[1]<=G2[2]^G2[3]^G2[6]^G2[8]^G2[9]^G2[10];
        out <= G1[10]^G2[tap1]^G2[tap2];
    end 
    else 
        epoch_pulse <= 1'b0 ;
    
end
endmodule



module carrier_nco #(
    parameter ACCUM_WIDTH    = 16,
    parameter LUT_ADDR_WIDTH = 8,
    parameter LUT_DEPTH      = 256,
    parameter AMP_WIDTH      = 5
)(
    input  wire                  clk,
    input  wire                  rst_n,        
    input  wire [15:0]           fcw,          
    output reg  signed [4:0]     sine_out,   
    output reg  signed [4:0]     cos_out     
);

// Internal signals

reg  [ACCUM_WIDTH-1:0]    phase_acc;
wire [LUT_ADDR_WIDTH-1:0] lut_addr_sin;
wire [LUT_ADDR_WIDTH-1:0] lut_addr_cos;

// Sine ROM  (256 x 5-bit signed entries)
// The initial block pre-fills it using $sin().
// Vivado infers this automatically as a LUT ROM.
// -------------------------------------------------------
reg signed [AMP_WIDTH-1:0] sine_rom [0:LUT_DEPTH-1];

integer i;
initial begin
    for (i = 0; i < LUT_DEPTH; i = i + 1)
        sine_rom[i] = $rtoi($sin(2.0 * 3.14159265358979 * i / LUT_DEPTH) * 15.0 + 0.5);
end

// -------------------------------------------------------
// Step 1: Phase Accumulator
// Adds FCW to itself every clock cycle. Wraps automatically.
// -------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        phase_acc <= {ACCUM_WIDTH{1'b0}};
    else
        phase_acc <= phase_acc + fcw;
end

// -------------------------------------------------------
// Step 2: LUT Address Extraction
// Use top 8 bits only. Cosine = sine + 90 degrees = +64 steps.
// -------------------------------------------------------
assign lut_addr_sin = phase_acc[15:8];
assign lut_addr_cos = lut_addr_sin + 8'd64;   // Wraps naturally at 256

// -------------------------------------------------------
// Step 3: Output Registers
// Register the ROM output for clean FPGA timing.
// -------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sine_out <= {AMP_WIDTH{1'b0}};
        cos_out  <= {AMP_WIDTH{1'b0}};
    end else begin
        sine_out <= sine_rom[lut_addr_sin];
        cos_out  <= sine_rom[lut_addr_cos];
    end
end

endmodule
module carrier_mixer #(
    parameter ADC_WIDTH = 3,
    parameter NCO_WIDTH = 5,
    parameter OUT_WIDTH = ADC_WIDTH + NCO_WIDTH
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire signed [ADC_WIDTH-1:0]  adc_in,
    input  wire signed [NCO_WIDTH-1:0]  cos_in,   // from NCO cos_out  → I branch
    input  wire signed [NCO_WIDTH-1:0]  sin_in,   // from NCO sine_out → Q branch
    output reg  signed [OUT_WIDTH-1:0]  I_out,    // I = adc × cos
    output reg  signed [OUT_WIDTH-1:0]  Q_out     // Q = adc × sin
);

wire signed [OUT_WIDTH-1:0] I_mix = adc_in * cos_in;
wire signed [OUT_WIDTH-1:0] Q_mix = adc_in * sin_in;

// Registered outputs – one pipeline stage for FPGA timing

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        I_out <= {OUT_WIDTH{1'b0}};
        Q_out <= {OUT_WIDTH{1'b0}};
    end else begin
        I_out <= I_mix;
        Q_out <= Q_mix;
    end
end
endmodule