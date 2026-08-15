module sensor_scheduler_tb (
  input  logic clk,
  input  logic rst_n,
  input  logic init_ready,
  input  logic [6:0] mpu_address,
  input  logic [6:0] bmp_address,
  input  logic enable_1khz,
  input  logic enable_100hz,
  input  logic enable_50hz,
  input  logic enable_10hz,
  input  logic [63:0] timestamp_cycles,
  input  logic snapshot_capture,
  input  logic fifo_pop,
  input  logic scl_i,
  input  logic sda_i,
  output logic scl_drive_low,
  output logic sda_drive_low,
  output logic mpu_sample,
  output logic ak_sample,
  output logic bmp_sample,
  output logic mpu_sample_pulse,
  output logic ak_sample_pulse,
  output logic bmp_sample_pulse,
  output logic mpu_valid,
  output logic ak_valid,
  output logic bmp_valid,
  output logic [255:0] mpu_latest_record,
  output logic [255:0] ak_latest_record,
  output logic [255:0] bmp_latest_record,
  output logic mpu_fresh,
  output logic ak_fresh,
  output logic bmp_fresh,
  output logic mpu_hard_stale,
  output logic [767:0] snapshot_records,
  output logic [31:0] snapshot_status,
  output logic [31:0] snapshot_sequence,
  output logic [255:0] fifo_head_data,
  output logic [2:0] fifo_level,
  output logic fifo_empty,
  output logic fifo_full,
  output logic fifo_overflow_sticky,
  output logic [31:0] fifo_overflow_count,
  output logic [31:0] fifo_underflow_count,
  output logic [31:0] mpu_release_count,
  output logic [31:0] ak_release_count,
  output logic [31:0] bmp_release_count,
  output logic [31:0] mpu_accepted_sample_count,
  output logic [31:0] ak_accepted_sample_count,
  output logic [31:0] bmp_accepted_sample_count,
  output logic [31:0] mpu_missed_release_count,
  output logic [31:0] ak_missed_release_count,
  output logic [31:0] bmp_missed_release_count,
  output logic [31:0] mpu_missed_sample_count,
  output logic [31:0] ak_missed_sample_count,
  output logic [31:0] bmp_missed_sample_count,
  output logic [31:0] mpu_i2c_error_count,
  output logic [31:0] ak_i2c_error_count,
  output logic [31:0] bmp_i2c_error_count,
  output logic [31:0] mpu_invalid_sample_count,
  output logic [31:0] ak_invalid_sample_count,
  output logic [31:0] bmp_invalid_sample_count,
  output logic [31:0] mpu_duplicate_poll_count,
  output logic [31:0] ak_duplicate_poll_count,
  output logic [31:0] ak_overrun_count,
  output logic [31:0] mpu_retry_count,
  output logic [31:0] mpu_max_latency_cycles,
  output logic [31:0] ak_max_latency_cycles,
  output logic [31:0] bmp_max_latency_cycles,
  output logic [31:0] deadline_miss_count,
  output logic [31:0] bus_transaction_count,
  output logic [31:0] bus_error_count,
  output logic [63:0] bus_busy_cycles,
  output logic [31:0] bus_window_busy_cycles,
  output logic [3:0] last_i2c_error,
  output logic runtime_fault,
  output logic monitor_cmd_accepted,
  output logic [6:0] monitor_cmd_address,
  output logic [7:0] monitor_cmd_write_count,
  output logic [7:0] monitor_cmd_read_count,
  output logic monitor_cmd_fast_mode,
  output logic i2c_busy
);
  logic runtime_rst_n;
  logic req_valid, req_ready, req_write, req_fast_mode;
  logic [6:0] req_address;
  logic [7:0] req_register, req_write_data, req_read_count;
  logic rx_valid, rx_ready, rsp_done, rsp_error;
  logic [7:0] rx_data, rx_index;
  logic [3:0] rsp_error_code;
  logic [7:0] rsp_error_index;
  logic cmd_valid, cmd_ready, cmd_fast_mode;
  logic [6:0] cmd_address;
  logic [7:0] cmd_write_count, cmd_read_count;
  logic tx_valid, tx_ready;
  logic [7:0] tx_data;
  logic i2c_rx_valid, i2c_rx_ready;
  logic [7:0] i2c_rx_data;
  logic i2c_done, i2c_error;
  logic [3:0] i2c_error_code;
  logic [7:0] i2c_error_byte_index;

  assign runtime_rst_n = rst_n && init_ready;
  assign monitor_cmd_accepted = cmd_valid && cmd_ready;
  assign monitor_cmd_address = cmd_address;
  assign monitor_cmd_write_count = cmd_write_count;
  assign monitor_cmd_read_count = cmd_read_count;
  assign monitor_cmd_fast_mode = cmd_fast_mode;

  sensor_scheduler #(
    .FIFO_DEPTH(4),
    .MPU_RETRY_DELAY_CYCLES(500),
    .MPU_FRESH_CYCLES(20_000),
    .MPU_HARD_STALE_CYCLES(50_000),
    .AK_FRESH_CYCLES(300_000),
    .BMP_FRESH_CYCLES(600_000),
    .MPU_DEADLINE_CYCLES(10_000),
    .AK_DEADLINE_CYCLES(100_000),
    .BMP_DEADLINE_CYCLES(200_000)
  ) scheduler (
    .clk, .rst_n, .init_ready, .mpu_address, .bmp_address,
    .enable_1khz, .enable_100hz, .enable_50hz, .enable_10hz,
    .timestamp_cycles, .req_valid, .req_ready, .req_address,
    .req_register, .req_write, .req_write_data, .req_read_count,
    .req_fast_mode, .rx_valid, .rx_ready, .rx_data, .rx_index,
    .rsp_done, .rsp_error, .rsp_error_code, .i2c_busy,
    .snapshot_capture, .fifo_pop, .mpu_sample, .ak_sample, .bmp_sample,
    .mpu_sample_pulse, .ak_sample_pulse, .bmp_sample_pulse,
    .mpu_valid, .ak_valid, .bmp_valid, .mpu_latest_record,
    .ak_latest_record, .bmp_latest_record, .mpu_fresh, .ak_fresh,
    .bmp_fresh, .mpu_hard_stale, .snapshot_records, .snapshot_status,
    .snapshot_sequence, .fifo_head_data, .fifo_level, .fifo_empty,
    .fifo_full, .fifo_overflow_sticky, .fifo_overflow_count,
    .fifo_underflow_count, .mpu_release_count, .ak_release_count,
    .bmp_release_count, .mpu_accepted_sample_count,
    .ak_accepted_sample_count, .bmp_accepted_sample_count,
    .mpu_missed_release_count, .ak_missed_release_count,
    .bmp_missed_release_count, .mpu_missed_sample_count,
    .ak_missed_sample_count, .bmp_missed_sample_count,
    .mpu_i2c_error_count, .ak_i2c_error_count, .bmp_i2c_error_count,
    .mpu_invalid_sample_count, .ak_invalid_sample_count,
    .bmp_invalid_sample_count, .mpu_duplicate_poll_count,
    .ak_duplicate_poll_count, .ak_overrun_count, .mpu_retry_count,
    .mpu_max_latency_cycles, .ak_max_latency_cycles,
    .bmp_max_latency_cycles, .deadline_miss_count,
    .bus_transaction_count, .bus_error_count, .bus_busy_cycles,
    .bus_window_busy_cycles, .last_i2c_error, .runtime_fault
  );

  i2c_register_master adapter (
    .clk, .rst_n(runtime_rst_n), .req_valid, .req_ready, .req_address,
    .req_register, .req_write, .req_write_data, .req_read_count,
    .req_fast_mode, .rx_valid, .rx_ready, .rx_data, .rx_index,
    .rsp_done, .rsp_error, .rsp_error_code, .rsp_error_index,
    .i2c_cmd_valid(cmd_valid), .i2c_cmd_ready(cmd_ready),
    .i2c_cmd_address(cmd_address), .i2c_cmd_write_count(cmd_write_count),
    .i2c_cmd_read_count(cmd_read_count), .i2c_cmd_fast_mode(cmd_fast_mode),
    .i2c_tx_valid(tx_valid), .i2c_tx_ready(tx_ready), .i2c_tx_data(tx_data),
    .i2c_rx_valid, .i2c_rx_ready, .i2c_rx_data, .i2c_done,
    .i2c_error, .i2c_error_code, .i2c_error_byte_index
  );

  i2c_master #(
    .FAST_LOW_CYCLES(13), .FAST_HIGH_CYCLES(12),
    .FAST_BUS_FREE_CYCLES(13), .STANDARD_LOW_CYCLES(50),
    .STANDARD_HIGH_CYCLES(50), .STANDARD_BUS_FREE_CYCLES(47),
    .STRETCH_TIMEOUT_CYCLES(2_000),
    .TRANSACTION_TIMEOUT_CYCLES(20_000)
  ) master (
    .clk, .rst_n(runtime_rst_n), .cmd_valid, .cmd_ready, .cmd_address,
    .cmd_write_count, .cmd_read_count, .cmd_fast_mode,
    .tx_valid, .tx_ready, .tx_data, .rx_valid(i2c_rx_valid),
    .rx_ready(i2c_rx_ready), .rx_data(i2c_rx_data), .busy(i2c_busy),
    .done(i2c_done), .error(i2c_error), .error_code(i2c_error_code),
    .error_byte_index(i2c_error_byte_index), .scl_i, .sda_i,
    .scl_drive_low, .sda_drive_low
  );
endmodule
