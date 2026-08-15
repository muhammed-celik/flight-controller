module timebase_tb (
  input  logic       clk,
  input  logic       rst_n,
  output logic [7:0] timestamp_cycles,
  output logic       enable_1khz,
  output logic       enable_250hz,
  output logic       enable_100hz,
  output logic       enable_50hz,
  output logic       enable_10hz
);

  fc_timebase #(
    .CLOCK_HZ       (1_000),
    .TIMESTAMP_WIDTH(8)
  ) u_dut (
    .clk,
    .rst_n,
    .timestamp_cycles,
    .enable_1khz,
    .enable_250hz,
    .enable_100hz,
    .enable_50hz,
    .enable_10hz
  );

endmodule
