module fc_timebase #(
  parameter int unsigned CLOCK_HZ       = 100_000_000,
  parameter int unsigned TIMESTAMP_WIDTH = 64
) (
  input  logic                       clk,
  input  logic                       rst_n,
  output logic [TIMESTAMP_WIDTH-1:0] timestamp_cycles,
  output logic                       enable_1khz,
  output logic                       enable_250hz,
  output logic                       enable_100hz,
  output logic                       enable_50hz,
  output logic                       enable_10hz
);

  initial begin
    if (TIMESTAMP_WIDTH == 0) begin
      $fatal(1, "fc_timebase TIMESTAMP_WIDTH must be nonzero");
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      timestamp_cycles <= '0;
    end else begin
      timestamp_cycles <= timestamp_cycles + 1'b1;
    end
  end

  rate_enable #(.CLOCK_HZ(CLOCK_HZ), .RATE_HZ(1_000)) u_enable_1khz (
    .clk, .rst_n, .enable(enable_1khz)
  );

  rate_enable #(.CLOCK_HZ(CLOCK_HZ), .RATE_HZ(250)) u_enable_250hz (
    .clk, .rst_n, .enable(enable_250hz)
  );

  rate_enable #(.CLOCK_HZ(CLOCK_HZ), .RATE_HZ(100)) u_enable_100hz (
    .clk, .rst_n, .enable(enable_100hz)
  );

  rate_enable #(.CLOCK_HZ(CLOCK_HZ), .RATE_HZ(50)) u_enable_50hz (
    .clk, .rst_n, .enable(enable_50hz)
  );

  rate_enable #(.CLOCK_HZ(CLOCK_HZ), .RATE_HZ(10)) u_enable_10hz (
    .clk, .rst_n, .enable(enable_10hz)
  );

endmodule
