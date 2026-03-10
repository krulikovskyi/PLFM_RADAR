`timescale 1ns / 1ps

/**
 * radar_system_tb.v
 * 
 * Comprehensive Testbench for Radar System Top Module
 * Tests:
 * - Transmitter chirp generation
 * - Receiver signal processing
 * - USB data transfer via FT601
 * - STM32 control interface
 */

module radar_system_tb;

// ============================================================================
// PARAMETERS
// ============================================================================

// Clock periods
parameter CLK_100M_PERIOD = 10.0;        // 100MHz = 10ns
parameter CLK_120M_PERIOD = 8.333;       // 120MHz = 8.333ns
parameter FT601_CLK_PERIOD = 10.0;       // 100MHz = 10ns
parameter ADC_DCO_PERIOD = 2.5;          // 400MHz = 2.5ns

// Simulation time
parameter SIM_TIME = 500_000;            // 500us simulation

// Test parameters
parameter NUM_CHIRPS = 64;                // Number of chirps to simulate
parameter ENABLE_DOPPLER = 1;             // Enable Doppler processing
parameter ENABLE_CFAR = 1;                 // Enable CFAR detection
parameter ENABLE_USB = 1;                  // Enable USB interface

// ============================================================================
// CLOCK AND RESET SIGNALS
// ============================================================================

reg clk_100m;
reg clk_120m_dac;
reg ft601_clk_in;
reg reset_n;

// ADC clocks
reg adc_dco_p;
reg adc_dco_n;

// ADC data
reg [7:0] adc_data_pattern;
reg [7:0] adc_d_p;
reg [7:0] adc_d_n;

// FT601 interface
wire [31:0] ft601_data;
wire [1:0] ft601_be;
wire ft601_txe_n;
wire ft601_rxf_n;
reg ft601_txe;
reg ft601_rxf;
wire ft601_wr_n;
wire ft601_rd_n;
wire ft601_oe_n;
wire ft601_siwu_n;
reg [1:0] ft601_srb;
reg [1:0] ft601_swb;
wire ft601_clk_out;

// STM32 control signals
reg stm32_new_chirp;
reg stm32_new_elevation;
reg stm32_new_azimuth;
reg stm32_mixers_enable;

// ADAR1000 SPI signals
reg stm32_sclk_3v3;
reg stm32_mosi_3v3;
wire stm32_miso_3v3;
reg stm32_cs_adar1_3v3;
reg stm32_cs_adar2_3v3;
reg stm32_cs_adar3_3v3;
reg stm32_cs_adar4_3v3;

wire stm32_sclk_1v8;
wire stm32_mosi_1v8;
reg stm32_miso_1v8;
wire stm32_cs_adar1_1v8;
wire stm32_cs_adar2_1v8;
wire stm32_cs_adar3_1v8;
wire stm32_cs_adar4_1v8;

// DAC outputs
wire [7:0] dac_data;
wire dac_clk;
wire dac_sleep;

// RF control
wire fpga_rf_switch;
wire rx_mixer_en;
wire tx_mixer_en;

// ADAR1000 control
wire adar_tx_load_1, adar_rx_load_1;
wire adar_tx_load_2, adar_rx_load_2;
wire adar_tx_load_3, adar_rx_load_3;
wire adar_tx_load_4, adar_rx_load_4;
wire adar_tr_1, adar_tr_2, adar_tr_3, adar_tr_4;

// Status outputs
wire [5:0] current_elevation;
wire [5:0] current_azimuth;
wire [5:0] current_chirp;
wire new_chirp_frame;
wire [31:0] dbg_doppler_data;
wire dbg_doppler_valid;
wire [4:0] dbg_doppler_bin;
wire [5:0] dbg_range_bin;
wire [3:0] system_status;

// ============================================================================
// CLOCK GENERATION
// ============================================================================

// 100MHz system clock
initial begin
    clk_100m = 0;
    forever #(CLK_100M_PERIOD/2) clk_100m = ~clk_100m;
end

// 120MHz DAC clock
initial begin
    clk_120m_dac = 0;
    forever #(CLK_120M_PERIOD/2) clk_120m_dac = ~clk_120m_dac;
end

// FT601 clock (100MHz)
initial begin
    ft601_clk_in = 0;
    forever #(FT601_CLK_PERIOD/2) ft601_clk_in = ~ft601_clk_in;
end

// ADC DCO clock (400MHz)
initial begin
    adc_dco_p = 0;
    adc_dco_n = 1;
    forever begin
        #(ADC_DCO_PERIOD/2) begin
            adc_dco_p = ~adc_dco_p;
            adc_dco_n = ~adc_dco_n;
        end
    end
end

// ============================================================================
// RESET GENERATION
// ============================================================================

initial begin
    reset_n = 0;
    #100;
    reset_n = 1;
    #10;
end

// ============================================================================
// FT601 INTERFACE SIMULATION
// ============================================================================

// FT601 FIFO status
initial begin
    ft601_txe = 1'b0;      // TX FIFO not empty (ready to write)
    ft601_rxf = 1'b1;      // RX FIFO full (not ready to read)
    ft601_srb = 2'b00;
    ft601_swb = 2'b00;
    
    // Simulate occasional FIFO full conditions
    forever begin
        #1000;
        ft601_txe = $random % 2;
        ft601_rxf = $random % 2;
    end
end

// FT601 data bus monitoring
reg [31:0] ft601_captured_data;
reg [31:0] usb_packet_buffer [0:1023];
integer usb_packet_count = 0;
integer usb_byte_count = 0;

always @(negedge ft601_wr_n) begin
    if (!ft601_wr_n) begin
        ft601_captured_data = ft601_data;
        usb_packet_buffer[usb_packet_count] = ft601_captured_data;
        usb_byte_count = usb_byte_count + 4;
        
        if (usb_packet_count < 100) begin
            $display("[USB @%0t] WRITE: data=0x%08h, be=%b, count=%0d", 
                     $time, ft601_captured_data, ft601_be, usb_packet_count);
        end
        
        usb_packet_count = usb_packet_count + 1;
    end
end

// ============================================================================
// STM32 CONTROL SIGNAL GENERATION
// ============================================================================

integer chirp_num = 0;
integer elevation_num = 0;
integer azimuth_num = 0;

initial begin
    // Initialize
    stm32_new_chirp = 0;
    stm32_new_elevation = 0;
    stm32_new_azimuth = 0;
    stm32_mixers_enable = 1;
    
    stm32_sclk_3v3 = 0;
    stm32_mosi_3v3 = 0;
    stm32_cs_adar1_3v3 = 1;
    stm32_cs_adar2_3v3 = 1;
    stm32_cs_adar3_3v3 = 1;
    stm32_cs_adar4_3v3 = 1;
    stm32_miso_1v8 = 0;
    
    #200;
    
    // Generate chirp sequence
    for (chirp_num = 0; chirp_num < NUM_CHIRPS; chirp_num = chirp_num + 1) begin
        // New chirp toggle
        stm32_new_chirp = 1;
        #20;
        stm32_new_chirp = 0;
        
        // Every 8 chirps, change elevation
        if ((chirp_num % 8) == 0) begin
            stm32_new_elevation = 1;
            #20;
            stm32_new_elevation = 0;
            elevation_num = elevation_num + 1;
        end
        
        // Every 16 chirps, change azimuth
        if ((chirp_num % 16) == 0) begin
            stm32_new_azimuth = 1;
            #20;
            stm32_new_azimuth = 0;
            azimuth_num = azimuth_num + 1;
        end
        
        // Wait for chirp duration
        #3000;  // ~30us between chirps
    end
    
    // Disable mixers at the end
    #5000;
    stm32_mixers_enable = 0;
end

// ============================================================================
// ADC DATA GENERATION (Simulated Radar Echo)
// ============================================================================

integer sample_count = 0;
integer target_count = 0;
reg [31:0] echo_delay [0:9];
reg [7:0] echo_amplitude [0:9];
reg [7:0] echo_phase [0:9];
integer target_idx;

initial begin
    // Initialize targets (simulated objects)
    // Format: {delay_samples, amplitude, phase_index}
    echo_delay[0] = 500;   echo_amplitude[0] = 100; echo_phase[0] = 0;
    echo_delay[1] = 1200;  echo_amplitude[1] = 80;  echo_phase[1] = 45;
    echo_delay[2] = 2500;  echo_amplitude[2] = 60;  echo_phase[2] = 90;
    echo_delay[3] = 4000;  echo_amplitude[3] = 40;  echo_phase[3] = 135;
    echo_delay[4] = 6000;  echo_amplitude[4] = 20;  echo_phase[4] = 180;
    
    for (target_idx = 5; target_idx < 10; target_idx = target_idx + 1) begin
        echo_delay[target_idx] = 0;
        echo_amplitude[target_idx] = 0;
        echo_phase[target_idx] = 0;
    end
    
    adc_d_p = 8'h00;
    adc_d_n = ~8'h00;
    
    // Wait for reset and chirp start
    #500;
    
    // Generate ADC data synchronized with chirps
    forever begin
        @(posedge adc_dco_p);
        sample_count = sample_count + 1;
        
        // Generate echo signal when transmitter is active
        if (tx_mixer_en && fpga_rf_switch) begin
            adc_data_pattern = generate_radar_echo(sample_count);
        end else begin
            adc_data_pattern = 8'h80;  // Mid-scale noise floor
        end
        
        // Add noise
        adc_data_pattern = adc_data_pattern + ($random % 16) - 8;
        
        // LVDS output
        adc_d_p = adc_data_pattern;
        adc_d_n = ~adc_data_pattern;
    end
end

// Function to generate radar echo based on multiple targets
function [7:0] generate_radar_echo;
    input integer sample;
    integer t;
    integer echo_sum;
    integer chirp_phase;
    reg [7:0] result;
begin
    echo_sum = 128;  // DC offset
    
    for (t = 0; t < 5; t = t + 1) begin
        if (echo_delay[t] > 0 && sample > echo_delay[t]) begin
            // Simple Doppler modulation
            chirp_phase = ((sample - echo_delay[t]) * 10) % 256;
            echo_sum = echo_sum + $signed({1'b0, echo_amplitude[t]}) * 
                       $signed({1'b0, sin_lut[chirp_phase + echo_phase[t]]}) / 128;
        end
    end
    
    // Clamp to 8-bit range
    if (echo_sum > 255) echo_sum = 255;
    if (echo_sum < 0) echo_sum = 0;
    
    result = echo_sum[7:0];
    generate_radar_echo = result;
end
endfunction

// Sine LUT for echo modulation
reg [7:0] sin_lut [0:255];
integer lut_i;
initial begin
    for (lut_i = 0; lut_i < 256; lut_i = lut_i + 1) begin
        sin_lut[lut_i] = 128 + 127 * $sin(2 * 3.14159 * lut_i / 256);
    end
end

// ============================================================================
// SPI COMMUNICATION MONITORING
// ============================================================================

always @(posedge stm32_sclk_1v8) begin
    if (!stm32_cs_adar1_1v8) begin
        $display("[SPI @%0t] ADAR1: MOSI=%b, MISO=%b", 
                 $time, stm32_mosi_1v8, stm32_miso_1v8);
    end
    if (!stm32_cs_adar2_1v8) begin
        $display("[SPI @%0t] ADAR2: MOSI=%b, MISO=%b", 
                 $time, stm32_mosi_1v8, stm32_miso_1v8);
    end
end

// ============================================================================
// DUT INSTANTIATION
// ============================================================================

radar_system_top dut (
    // System Clocks
    .clk_100m(clk_100m),
    .clk_120m_dac(clk_120m_dac),
    .ft601_clk_in(ft601_clk_in),
    .reset_n(reset_n),
    
    // Transmitter Interfaces
    .dac_data(dac_data),
    .dac_clk(dac_clk),
    .dac_sleep(dac_sleep),
    .fpga_rf_switch(fpga_rf_switch),
    .rx_mixer_en(rx_mixer_en),
    .tx_mixer_en(tx_mixer_en),
    
    // ADAR1000 Control
    .adar_tx_load_1(adar_tx_load_1),
    .adar_rx_load_1(adar_rx_load_1),
    .adar_tx_load_2(adar_tx_load_2),
    .adar_rx_load_2(adar_rx_load_2),
    .adar_tx_load_3(adar_tx_load_3),
    .adar_rx_load_3(adar_rx_load_3),
    .adar_tx_load_4(adar_tx_load_4),
    .adar_rx_load_4(adar_rx_load_4),
    .adar_tr_1(adar_tr_1),
    .adar_tr_2(adar_tr_2),
    .adar_tr_3(adar_tr_3),
    .adar_tr_4(adar_tr_4),
    
    // Level Shifter SPI
    .stm32_sclk_3v3(stm32_sclk_3v3),
    .stm32_mosi_3v3(stm32_mosi_3v3),
    .stm32_miso_3v3(stm32_miso_3v3),
    .stm32_cs_adar1_3v3(stm32_cs_adar1_3v3),
    .stm32_cs_adar2_3v3(stm32_cs_adar2_3v3),
    .stm32_cs_adar3_3v3(stm32_cs_adar3_3v3),
    .stm32_cs_adar4_3v3(stm32_cs_adar4_3v3),
    
    .stm32_sclk_1v8(stm32_sclk_1v8),
    .stm32_mosi_1v8(stm32_mosi_1v8),
    .stm32_miso_1v8(stm32_miso_1v8),
    .stm32_cs_adar1_1v8(stm32_cs_adar1_1v8),
    .stm32_cs_adar2_1v8(stm32_cs_adar2_1v8),
    .stm32_cs_adar3_1v8(stm32_cs_adar3_1v8),
    .stm32_cs_adar4_1v8(stm32_cs_adar4_1v8),
    
    // Receiver Interfaces
    .adc_d_p(adc_d_p),
    .adc_d_n(adc_d_n),
    .adc_dco_p(adc_dco_p),
    .adc_dco_n(adc_dco_n),
    .adc_pwdn(adc_pwdn),
    
    // STM32 Control
    .stm32_new_chirp(stm32_new_chirp),
    .stm32_new_elevation(stm32_new_elevation),
    .stm32_new_azimuth(stm32_new_azimuth),
    .stm32_mixers_enable(stm32_mixers_enable),
    
    // FT601 Interface
    .ft601_data(ft601_data),
    .ft601_be(ft601_be),
    .ft601_txe_n(ft601_txe_n),
    .ft601_rxf_n(ft601_rxf_n),
    .ft601_txe(ft601_txe),
    .ft601_rxf(ft601_rxf),
    .ft601_wr_n(ft601_wr_n),
    .ft601_rd_n(ft601_rd_n),
    .ft601_oe_n(ft601_oe_n),
    .ft601_siwu_n(ft601_siwu_n),
    .ft601_srb(ft601_srb),
    .ft601_swb(ft601_swb),
    .ft601_clk_out(ft601_clk_out),
    
    // Status Outputs
    .current_elevation(current_elevation),
    .current_azimuth(current_azimuth),
    .current_chirp(current_chirp),
    .new_chirp_frame(new_chirp_frame),
    .dbg_doppler_data(dbg_doppler_data),
    .dbg_doppler_valid(dbg_doppler_valid),
    .dbg_doppler_bin(dbg_doppler_bin),
    .dbg_range_bin(dbg_range_bin),
    .system_status(system_status)
);

// ============================================================================
// MONITORING AND CHECKING
// ============================================================================

// Transmitter monitoring
always @(posedge clk_100m) begin
    if (new_chirp_frame) begin
        $display("[MON @%0t] New chirp frame: chirp=%0d, elev=%0d, az=%0d",
                 $time, current_chirp, current_elevation, current_azimuth);
    end
end

// DAC output monitoring
integer dac_sample_count = 0;
always @(posedge dac_clk) begin
    if (dac_data != 8'h80) begin
        dac_sample_count = dac_sample_count + 1;
        if (dac_sample_count < 50) begin
            $display("[DAC @%0t] data=0x%02h", $time, dac_data);
        end
    end
end

// Doppler output monitoring
integer doppler_count = 0;
always @(posedge clk_100m) begin
    if (dbg_doppler_valid) begin
        doppler_count = doppler_count + 1;
        if (doppler_count < 100) begin
            $display("[DOPPLER @%0t] bin=%0d, range=%0d, data=0x%08h", 
                     $time, dbg_doppler_bin, dbg_range_bin, dbg_doppler_data);
        end
    end
end

// ============================================================================
// TEST COMPLETION
// ============================================================================

initial begin
    #SIM_TIME;
    
    $display("");
    $display("========================================");
    $display("SIMULATION COMPLETE");
    $display("========================================");
    $display("Total simulation time: %0t ns", $time);
    $display("Chirps generated: %0d", chirp_num);
    $display("USB packets sent: %0d", usb_packet_count);
    $display("USB bytes sent: %0d", usb_byte_count);
    $display("Doppler outputs: %0d", doppler_count);
    $display("DAC samples: %0d", dac_sample_count);
    $display("========================================");
    
    // Verify USB data format
    if (usb_packet_count > 0) begin
        $display("");
        $display("USB Packet Analysis:");
        $display("First 10 packets:");
        for (integer p = 0; p < 10 && p < usb_packet_count; p = p + 1) begin
            $display("  Packet[%0d]: 0x%08h", p, usb_packet_buffer[p]);
        end
    end
    
    $finish;
end

// ============================================================================
// ASSERTIONS AND CHECKS
// ============================================================================

// Check that chirp counter increments properly
property chirp_counter_check;
    @(posedge clk_100m) $rose(new_chirp_frame) |-> ##[1:10] (current_chirp != $past(current_chirp));
endproperty
assert property (chirp_counter_check) else $error("Chirp counter not incrementing");

// Check that USB writes occur when data is valid
property usb_write_check;
    @(posedge ft601_clk_in) (dbg_doppler_valid) |-> ##[1:100] (!$stable(ft601_wr_n));
endproperty
assert property (usb_write_check) else $warning("USB not writing when data valid");

// Check that system reset works
property reset_check;
    @(negedge reset_n) (1'b1) |-> ##1 (system_status == 4'b0000);
endproperty
assert property (reset_check) else $error("Reset failed to clear status");

// ============================================================================
// WAVEFORM DUMPING
// ============================================================================

initial begin
    $dumpfile("radar_system_tb.vcd");
    $dumpvars(0, radar_system_tb);
    
    // Optional: dump specific signals for debugging
    $dumpvars(1, dut.tx_inst);
    $dumpvars(1, dut.rx_inst);
    $dumpvars(1, dut.usb_inst);
end

endmodule
