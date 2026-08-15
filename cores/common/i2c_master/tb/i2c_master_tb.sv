module i2c_master_tb (
  input  logic       clk,
  input  logic       rst_n,
  input  logic       cmd_valid,
  output logic       cmd_ready,
  input  logic [6:0] cmd_address,
  input  logic [7:0] cmd_write_count,
  input  logic [7:0] cmd_read_count,
  input  logic       cmd_fast_mode,
  input  logic       tx_valid,
  output logic       tx_ready,
  input  logic [7:0] tx_data,
  output logic       rx_valid,
  input  logic       rx_ready,
  output logic [7:0] rx_data,
  output logic       busy,
  output logic       done,
  output logic       error,
  output logic [3:0] error_code,
  output logic [7:0] error_byte_index,
  input  logic       scl_i,
  input  logic       sda_i,
  output logic       scl_drive_low,
  output logic       sda_drive_low
);

  i2c_master #(
    .FAST_LOW_CYCLES(4),
    .FAST_HIGH_CYCLES(4),
    .FAST_BUS_FREE_CYCLES(4),
    .STANDARD_LOW_CYCLES(10),
    .STANDARD_HIGH_CYCLES(10),
    .STANDARD_BUS_FREE_CYCLES(10),
    .STRETCH_TIMEOUT_CYCLES(30),
    .TRANSACTION_TIMEOUT_CYCLES(2000)
  ) dut (
    .*
  );

endmodule
