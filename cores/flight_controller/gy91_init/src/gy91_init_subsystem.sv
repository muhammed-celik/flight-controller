module gy91_init_subsystem #(
  parameter int unsigned POWER_UP_CYCLES          = 10_000_000,
  parameter int unsigned MPU_RESET_CYCLES         = 10_000_000,
  parameter int unsigned MPU_WAKE_CYCLES          = 3_500_000,
  parameter int unsigned AK_MODE_CYCLES           = 10_000,
  parameter int unsigned BMP_POLL_INTERVAL_CYCLES = 50_000,
  parameter int unsigned BMP_POLL_LIMIT           = 20,
  parameter int unsigned RETRY_DELAY_CYCLES       = 100_000,
  parameter int unsigned MAX_TRANSACTION_RETRIES  = 2,
  parameter int unsigned I2C_LOW_CYCLES           = 500,
  parameter int unsigned I2C_HIGH_CYCLES          = 500,
  parameter int unsigned I2C_BUS_FREE_CYCLES      = 470,
  parameter int unsigned I2C_STRETCH_TIMEOUT       = 100_000,
  parameter int unsigned I2C_TRANSACTION_TIMEOUT  = 5_000_000
) (
  input  logic clk,
  input  logic rst_n,
  input  logic reinitialize,
  output logic reinitialize_ready,
  input  logic scl_i,
  input  logic sda_i,
  output logic scl_drive_low,
  output logic sda_drive_low,
  output logic initializing,
  output logic ready,
  output logic failed,
  output logic mpu_present,
  output logic mpu_ready,
  output logic mpu_failed,
  output logic ak_present,
  output logic ak_ready,
  output logic ak_failed,
  output logic bmp_present,
  output logic bmp_ready,
  output logic bmp_failed,
  output logic [6:0] mpu_address,
  output logic [6:0] bmp_address,
  output logic [23:0] ak_asa,
  output logic [191:0] bmp_calibration,
  output logic [15:0] mpu_error_count,
  output logic [15:0] ak_error_count,
  output logic [15:0] bmp_error_count,
  output logic [3:0] last_i2c_error,
  output logic [1:0] last_failed_device,
  output logic [7:0] last_failed_step,
  output logic [15:0] init_sequence,
  output logic monitor_cmd_accepted,
  output logic [6:0] monitor_cmd_address,
  output logic [7:0] monitor_cmd_write_count,
  output logic [7:0] monitor_cmd_read_count,
  output logic monitor_cmd_fast_mode
);

  logic req_valid, req_ready, req_write;
  logic [6:0] req_address;
  logic [7:0] req_register, req_write_data, req_read_count;
  logic rx_valid, rx_ready;
  logic [7:0] rx_data, rx_index;
  logic rsp_done, rsp_error;
  logic [3:0] rsp_error_code;
  logic [7:0] rsp_error_index;
  logic cmd_valid, cmd_ready, tx_valid, tx_ready, i2c_rx_valid, i2c_rx_ready;
  logic [6:0] cmd_address;
  logic [7:0] cmd_write_count, cmd_read_count, tx_data, i2c_rx_data;
  logic cmd_fast_mode, i2c_done, i2c_error;
  logic [3:0] i2c_error_code;
  logic [7:0] i2c_error_byte_index;

  assign monitor_cmd_accepted    = cmd_valid && cmd_ready;
  assign monitor_cmd_address     = cmd_address;
  assign monitor_cmd_write_count = cmd_write_count;
  assign monitor_cmd_read_count  = cmd_read_count;
  assign monitor_cmd_fast_mode   = cmd_fast_mode;

  gy91_init #(
    .POWER_UP_CYCLES(POWER_UP_CYCLES),
    .MPU_RESET_CYCLES(MPU_RESET_CYCLES),
    .MPU_WAKE_CYCLES(MPU_WAKE_CYCLES),
    .AK_MODE_CYCLES(AK_MODE_CYCLES),
    .BMP_POLL_INTERVAL_CYCLES(BMP_POLL_INTERVAL_CYCLES),
    .BMP_POLL_LIMIT(BMP_POLL_LIMIT),
    .RETRY_DELAY_CYCLES(RETRY_DELAY_CYCLES),
    .MAX_TRANSACTION_RETRIES(MAX_TRANSACTION_RETRIES)
  ) initializer (
    .clk, .rst_n, .reinitialize, .reinitialize_ready,
    .req_valid, .req_ready, .req_address, .req_register, .req_write,
    .req_write_data, .req_read_count, .rx_valid, .rx_ready, .rx_data,
    .rx_index, .rsp_done, .rsp_error, .rsp_error_code,
    .initializing, .ready, .failed, .mpu_present, .mpu_ready, .mpu_failed,
    .ak_present, .ak_ready, .ak_failed, .bmp_present, .bmp_ready, .bmp_failed,
    .mpu_address, .bmp_address, .ak_asa, .bmp_calibration, .mpu_error_count,
    .ak_error_count, .bmp_error_count, .last_i2c_error, .last_failed_device,
    .last_failed_step, .init_sequence
  );

  i2c_register_master adapter (
    .clk, .rst_n, .req_valid, .req_ready, .req_address, .req_register,
    .req_write, .req_write_data, .req_read_count, .rx_valid, .rx_ready,
    .rx_data, .rx_index, .rsp_done, .rsp_error, .rsp_error_code,
    .rsp_error_index, .i2c_cmd_valid(cmd_valid), .i2c_cmd_ready(cmd_ready),
    .i2c_cmd_address(cmd_address), .i2c_cmd_write_count(cmd_write_count),
    .i2c_cmd_read_count(cmd_read_count), .i2c_cmd_fast_mode(cmd_fast_mode),
    .i2c_tx_valid(tx_valid), .i2c_tx_ready(tx_ready), .i2c_tx_data(tx_data),
    .i2c_rx_valid, .i2c_rx_ready, .i2c_rx_data, .i2c_done,
    .i2c_error, .i2c_error_code, .i2c_error_byte_index
  );

  i2c_master #(
    .FAST_LOW_CYCLES(I2C_LOW_CYCLES),
    .FAST_HIGH_CYCLES(I2C_HIGH_CYCLES),
    .FAST_BUS_FREE_CYCLES(I2C_BUS_FREE_CYCLES),
    .STANDARD_LOW_CYCLES(I2C_LOW_CYCLES),
    .STANDARD_HIGH_CYCLES(I2C_HIGH_CYCLES),
    .STANDARD_BUS_FREE_CYCLES(I2C_BUS_FREE_CYCLES),
    .STRETCH_TIMEOUT_CYCLES(I2C_STRETCH_TIMEOUT),
    .TRANSACTION_TIMEOUT_CYCLES(I2C_TRANSACTION_TIMEOUT)
  ) bus_master (
    .clk, .rst_n, .cmd_valid, .cmd_ready, .cmd_address,
    .cmd_write_count, .cmd_read_count, .cmd_fast_mode, .tx_valid,
    .tx_ready, .tx_data, .rx_valid(i2c_rx_valid), .rx_ready(i2c_rx_ready),
    .rx_data(i2c_rx_data), .busy(), .done(i2c_done), .error(i2c_error),
    .error_code(i2c_error_code), .error_byte_index(i2c_error_byte_index),
    .scl_i, .sda_i, .scl_drive_low, .sda_drive_low
  );

endmodule
