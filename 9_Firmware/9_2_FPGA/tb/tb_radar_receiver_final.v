`timescale 1ns / 1ps
// ============================================================================
// tb_radar_receiver_final.v -- P0 Integration Test for radar_receiver_final
//
// Tests the full RX pipeline from ADC input to Doppler output:
//   ad9484_interface (stub) -> CDC -> DDC -> ddc_input_interface
//     -> matched_filter_multi_segment -> range_bin_decimator
//     -> doppler_processor_optimized -> doppler_output
//
// Strategy:
//   - Uses behavioral stub for ad9484_interface_400m (no Xilinx primitives)
//   - Overrides radar_mode_controller timing params for fast simulation
//   - Feeds 120 MHz tone at ADC input (IF frequency -> DDC passband)
//   - Verifies structural correctness of Doppler outputs:
//     * Outputs appear (doppler_valid asserted)
//     * Correct output count per frame (64 range bins x 32 Doppler bins = 2048)
//     * Range bin index covers 0..63
//     * Doppler bin index covers 0..31
//     * Output values are non-trivial (not all zeros)
//   - Does NOT require bit-perfect match (too many cascaded stages)
//
// Convention: check task, VCD dump, CSV output, pass/fail summary
// ============================================================================

module tb_radar_receiver_final;

// ============================================================================
// CLOCK AND RESET
// ============================================================================
reg clk_100m;       // 100 MHz system clock
reg clk_400m;       // 400 MHz ADC clock
reg reset_n;

// 100 MHz: period = 10 ns
initial clk_100m = 0;
always #5 clk_100m = ~clk_100m;

// 400 MHz: period = 2.5 ns
initial clk_400m = 0;
always #1.25 clk_400m = ~clk_400m;

// ============================================================================
// ADC STIMULUS
// ============================================================================
// Feed a 120 MHz tone (IF frequency) sampled at 400 MHz
// Phase increment per sample: 120/400 * 65536 = 19660.8
// This produces a strong DC component after DDC downconversion
reg [7:0] adc_data;
reg [15:0] phase_acc;  // 16-bit phase accumulator for precision
localparam [15:0] PHASE_INC = 16'd19661;  // 120/400 * 65536

always @(posedge clk_400m or negedge reset_n) begin
    if (!reset_n) begin
        phase_acc <= 16'd0;
        adc_data <= 8'd128;  // Mid-scale
    end else begin
        phase_acc <= phase_acc + PHASE_INC;
        // Use phase_acc[15:8] directly as pseudo-sinusoidal data
        // A sawtooth/triangle wave has energy at IF -- good enough for integration test
        adc_data <= phase_acc[15:8];
    end
end

// ============================================================================
// CHIRP COUNTER (external input to DUT)
// ============================================================================
// In the real system, this comes from the transmitter. For the test,
// we increment it on each mc_new_chirp toggle from the mode controller.
// Access the internal signal via hierarchical reference.
reg [5:0] chirp_counter;
reg mc_new_chirp_prev;

always @(posedge clk_100m or negedge reset_n) begin
    if (!reset_n) begin
        chirp_counter <= 6'd0;
        mc_new_chirp_prev <= 1'b0;
    end else begin
        mc_new_chirp_prev <= dut.mc_new_chirp;
        if (dut.mc_new_chirp != mc_new_chirp_prev) begin
            chirp_counter <= chirp_counter + 1;
        end
    end
end

// ============================================================================
// DUT INSTANTIATION
// ============================================================================
wire [31:0] doppler_output;
wire doppler_valid;
wire [4:0] doppler_bin;
wire [5:0] range_bin_out;

radar_receiver_final dut (
    .clk(clk_100m),
    .reset_n(reset_n),

    // ADC "LVDS" -- stub treats adc_d_p as single-ended data
    .adc_d_p(adc_data),
    .adc_d_n(~adc_data),       // Complement (ignored by stub)
    .adc_dco_p(clk_400m),      // 400 MHz clock
    .adc_dco_n(~clk_400m),     // Complement (ignored by stub)
    .adc_pwdn(),

    .chirp_counter(chirp_counter),

    .doppler_output(doppler_output),
    .doppler_valid(doppler_valid),
    .doppler_bin(doppler_bin),
    .range_bin(range_bin_out)
);

// ============================================================================
// OVERRIDE TIMING PARAMETERS via defparam
// ============================================================================
// Reduce radar_mode_controller timing to keep simulation tractable.
// Real values: LONG_CHIRP=3000, LONG_LISTEN=13700, GUARD=17540,
//              SHORT_CHIRP=50, SHORT_LISTEN=17450  (total ~51740 per chirp)
// Need enough DDC samples to fill MF buffer (896) plus latency buffer (3187).
// At ~1 DDC sample per sys_clk, we need at least ~5000 sys_clk per chirp.
// Use moderately reduced values: ~5000 cycles per chirp pair
defparam dut.rmc.LONG_CHIRP_CYCLES   = 500;
defparam dut.rmc.LONG_LISTEN_CYCLES  = 2000;
defparam dut.rmc.GUARD_CYCLES        = 500;
defparam dut.rmc.SHORT_CHIRP_CYCLES  = 50;
defparam dut.rmc.SHORT_LISTEN_CYCLES = 1000;

// ============================================================================
// TEST INFRASTRUCTURE
// ============================================================================
integer pass_count;
integer fail_count;
integer total_tests;

task check;
    input cond;
    input [512*8-1:0] label;
    begin
        total_tests = total_tests + 1;
        if (cond) begin
            pass_count = pass_count + 1;
            $display("[PASS %0d] %0s", total_tests, label);
        end else begin
            fail_count = fail_count + 1;
            $display("[FAIL %0d] %0s", total_tests, label);
        end
    end
endtask

// ============================================================================
// OUTPUT MONITORING
// ============================================================================
integer doppler_output_count;
integer doppler_frame_count;
reg [63:0] range_bin_seen;     // Bitmap: which range bins appeared
reg [31:0] doppler_bin_seen;   // Bitmap: which Doppler bins appeared
integer nonzero_output_count;
reg [31:0] first_doppler_time; // Cycle when first doppler_valid appears
reg first_doppler_seen;

// Per-frame tracking
integer frame_output_count;
reg frame_done_prev;

// CSV output
integer csv_fd;

initial begin
    doppler_output_count = 0;
    doppler_frame_count = 0;
    range_bin_seen = 64'd0;
    doppler_bin_seen = 32'd0;
    nonzero_output_count = 0;
    first_doppler_seen = 0;
    first_doppler_time = 0;
    frame_output_count = 0;
    frame_done_prev = 0;

    csv_fd = $fopen("tb/cosim/rx_final_doppler_out.csv", "w");
    if (csv_fd) $fdisplay(csv_fd, "cycle,range_bin,doppler_bin,output_hex");
end

// Monitor doppler outputs -- only after reset released
always @(posedge clk_100m) begin
    if (reset_n && doppler_valid) begin
        doppler_output_count = doppler_output_count + 1;
        frame_output_count = frame_output_count + 1;

        // Track which bins we've seen
        if (range_bin_out < 64)
            range_bin_seen = range_bin_seen | (64'd1 << range_bin_out);
        if (doppler_bin < 32)
            doppler_bin_seen = doppler_bin_seen | (32'd1 << doppler_bin);

        // Track non-zero outputs
        if (doppler_output != 32'd0)
            nonzero_output_count = nonzero_output_count + 1;

        // Record first output time
        if (!first_doppler_seen) begin
            first_doppler_seen = 1;
            first_doppler_time = $time;
            $display("[INFO] First doppler_valid at time %0t", $time);
        end

        // CSV logging
        if (csv_fd)
            $fdisplay(csv_fd, "%0t,%0d,%0d,%08h", $time, range_bin_out, doppler_bin, doppler_output);

        // Progress reporting (every 256 outputs)
        if ((doppler_output_count % 256) == 0)
            $display("[INFO] %0d doppler outputs so far (t=%0t)", doppler_output_count, $time);
    end

    // Track frame completions via doppler_proc -- only after reset
    if (reset_n && dut.doppler_frame_done && !frame_done_prev) begin
        doppler_frame_count = doppler_frame_count + 1;
        $display("[INFO] Doppler frame %0d complete: %0d outputs (t=%0t)",
                 doppler_frame_count, frame_output_count, $time);
        frame_output_count = 0;
    end
    frame_done_prev = dut.doppler_frame_done;
end

// ============================================================================
// PROGRESS MONITOR -- pipeline stage activity
// ============================================================================
reg [31:0] ddc_valid_count;
reg [31:0] mf_valid_count;
reg [31:0] range_decim_count;
reg [31:0] range_data_valid_count;

initial begin
    ddc_valid_count = 0;
    mf_valid_count = 0;
    range_decim_count = 0;
    range_data_valid_count = 0;
end

always @(posedge clk_100m) begin
    if (dut.adc_valid_sync) ddc_valid_count = ddc_valid_count + 1;
    if (dut.range_valid) mf_valid_count = mf_valid_count + 1;
    if (dut.decimated_range_valid) range_decim_count = range_decim_count + 1;
    if (dut.range_data_valid) range_data_valid_count = range_data_valid_count + 1;
end

// Periodic progress dump
reg [31:0] progress_timer;
initial progress_timer = 0;
always @(posedge clk_100m) begin
    progress_timer = progress_timer + 1;
    if (progress_timer % 50000 == 0) begin
        $display("[PROGRESS t=%0t] ddc_valid=%0d mf_out=%0d range_decim=%0d doppler_out=%0d chirp=%0d",
                 $time, ddc_valid_count, mf_valid_count, range_decim_count,
                 doppler_output_count, chirp_counter);
    end
end

// ============================================================================
// MF PIPELINE DEBUG MONITOR — track state transitions
// ============================================================================
reg [3:0] mf_state_prev;
reg [3:0] chain_state_prev;
initial begin
    mf_state_prev = 0;
    chain_state_prev = 0;
end

always @(posedge clk_100m) begin
    // Multi-segment FSM state changes
    if (dut.mf_dual.state != mf_state_prev) begin
        $display("[MF_DBG t=%0t] multi_seg state: %0d -> %0d (seg=%0d, wr_ptr=%0d, rd_ptr=%0d, samples=%0d)",
                 $time, mf_state_prev, dut.mf_dual.state,
                 dut.mf_dual.current_segment, dut.mf_dual.buffer_write_ptr,
                 dut.mf_dual.buffer_read_ptr, dut.mf_dual.chirp_samples_collected);
        mf_state_prev = dut.mf_dual.state;
    end
    // Processing chain state changes
    if (dut.mf_dual.m_f_p_c.state != chain_state_prev) begin
        $display("[CHAIN_DBG t=%0t] chain state: %0d -> %0d (fwd_count=%0d, out_count=%0d)",
                 $time, chain_state_prev, dut.mf_dual.m_f_p_c.state,
                 dut.mf_dual.m_f_p_c.fwd_in_count, dut.mf_dual.m_f_p_c.out_count);
        chain_state_prev = dut.mf_dual.m_f_p_c.state;
    end
    // Watch for fft_pc_valid while multi-seg is in ST_WAIT_FFT
    if (dut.mf_dual.state == 5 && dut.mf_dual.fft_pc_valid) begin
        $display("[MF_DBG t=%0t] *** fft_pc_valid=1 while in ST_WAIT_FFT! Should transition!", $time);
    end
    // Watch for fft_pc_valid while multi-seg is NOT in ST_WAIT_FFT
    if (dut.mf_dual.state != 5 && dut.mf_dual.fft_pc_valid) begin
        $display("[MF_DBG t=%0t] WARNING: fft_pc_valid=1 but multi_seg state=%0d (NOT ST_WAIT_FFT)",
                 $time, dut.mf_dual.state);
    end
end

// ============================================================================
// MAIN TEST SEQUENCE
// ============================================================================
// Simulation timeout calculation:
// 1. DDC pipeline fill: ~4 sys_clk cycles
// 2. MF overlap-save buffer fill: 896 valid DDC samples
// 3. Latency buffer priming: 3187 valid_in assertions
// 4. 1024 MF outputs -> range_bin_decimator -> 64 decimated outputs
// 5. 32 chirps of decimated data -> Doppler FFT
//
// With shortened mode controller timing (~600 cycles per chirp pair),
// DDC output rate depends on how many 400MHz samples per chirp period
// produce valid 100MHz outputs (CIC 4x decimation = ~1 per 4 clk_400m).
//
// Conservative estimate: ~500K 100MHz cycles for the full pipeline.
// ~4050 cycles/chirp x 32 chirps = ~130K, plus latency buffer priming,
// plus Doppler processing time. Set generous timeout.

localparam SIM_TIMEOUT = 2_000_000;  // 2M cycles — full pipeline with multi-segment drain

initial begin
    // VCD dump disabled for long integration test -- uncomment for debug
    // $dumpfile("tb/tb_radar_receiver_final.vcd");
    // $dumpvars(0, tb_radar_receiver_final);

    pass_count = 0;
    fail_count = 0;
    total_tests = 0;

    // ---- RESET ----
    reset_n = 0;
    #100;
    reset_n = 1;
    $display("[INFO] Reset released at t=%0t", $time);

    // ---- WAIT FOR PIPELINE ----
    // Poll until first Doppler frame completes or timeout
    begin : wait_loop
        integer wait_cycles;
        wait_cycles = 0;
        while (doppler_frame_count < 1 && wait_cycles < SIM_TIMEOUT) begin
            @(posedge clk_100m);
            wait_cycles = wait_cycles + 1;
        end
        if (doppler_frame_count >= 1) begin
            $display("[INFO] First Doppler frame completed at t=%0t", $time);
            #1000;
        end else begin
            $display("[WARN] Simulation timeout reached at t=%0t (%0d cycles)", $time, wait_cycles);
            $display("[WARN] Pipeline progress: ddc_valid=%0d mf_out=%0d range_decim=%0d doppler=%0d",
                     ddc_valid_count, mf_valid_count, range_decim_count, doppler_output_count);
        end
    end

    // ---- RUN CHECKS ----
    $display("");
    $display("============================================================");
    $display("RADAR RECEIVER FINAL -- INTEGRATION TEST RESULTS");
    $display("============================================================");
    $display("Total doppler outputs:   %0d", doppler_output_count);
    $display("Doppler frames complete: %0d", doppler_frame_count);
    $display("Non-zero outputs:        %0d", nonzero_output_count);
    $display("DDC valid count:         %0d", ddc_valid_count);
    $display("MF output count:         %0d", mf_valid_count);
    $display("Range decim count:       %0d", range_decim_count);
    $display("============================================================");
    $display("");

    // ---- CHECK 1: Pipeline activity ----
    check(ddc_valid_count > 0,
          "DDC produces valid outputs (adc_valid_sync asserted)");

    // ---- CHECK 2: MF outputs appear ----
    check(mf_valid_count > 0,
          "Matched filter produces outputs (range_valid asserted)");

    // ---- CHECK 3: Range decimator outputs appear ----
    check(range_decim_count > 0,
          "Range bin decimator produces outputs");

    // ---- CHECK 4: Doppler outputs appear ----
    check(doppler_output_count > 0,
          "Doppler processor produces outputs (doppler_valid asserted)");

    // ---- CHECK 5: Correct output count per frame ----
    // A complete Doppler frame should produce 64 x 32 = 2048 outputs
    if (doppler_frame_count > 0) begin
        check(doppler_output_count >= 2048,
              "At least 2048 doppler outputs (one full frame: 64 rbins x 32 dbins)");
    end else begin
        check(0, "At least 2048 doppler outputs (NO FRAME COMPLETED)");
    end

    // ---- CHECK 6: Range bin coverage ----
    // Count how many unique range bins appeared
    begin : count_range_bins
        integer rb_count, rb_i;
        rb_count = 0;
        for (rb_i = 0; rb_i < 64; rb_i = rb_i + 1) begin
            if (range_bin_seen[rb_i]) rb_count = rb_count + 1;
        end
        $display("[INFO] Unique range bins seen: %0d / 64", rb_count);
        check(rb_count == 64,
              "All 64 range bins present in Doppler output");
    end

    // ---- CHECK 7: Doppler bin coverage ----
    begin : count_doppler_bins
        integer db_count, db_i;
        db_count = 0;
        for (db_i = 0; db_i < 32; db_i = db_i + 1) begin
            if (doppler_bin_seen[db_i]) db_count = db_count + 1;
        end
        $display("[INFO] Unique Doppler bins seen: %0d / 32", db_count);
        check(db_count == 32,
              "All 32 Doppler bins present in Doppler output");
    end

    // ---- CHECK 8: Non-trivial outputs ----
    check(nonzero_output_count > 0,
          "At least some Doppler outputs are non-zero");

    // ---- CHECK 9: Non-zero fraction ----
    // With a tone input, we expect most outputs to have some energy
    if (doppler_output_count > 0) begin
        check(nonzero_output_count > doppler_output_count / 4,
              "More than 25pct of Doppler outputs are non-zero");
    end else begin
        check(0, "More than 25pct of Doppler outputs are non-zero (NO OUTPUTS)");
    end

    // ---- CHECK 10: Pipeline didn't stall ----
    check(ddc_valid_count > 100,
          "DDC produced substantial output (>100 valid samples)");

    // ---- SUMMARY ----
    $display("");
    $display("============================================================");
    $display("SUMMARY: %0d / %0d tests passed", pass_count, total_tests);
    if (fail_count == 0)
        $display("ALL TESTS PASSED");
    else
        $display("SOME TESTS FAILED (%0d failures)", fail_count);
    $display("============================================================");

    if (csv_fd) $fclose(csv_fd);
    $finish;
end

endmodule
