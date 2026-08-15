module rate_enable #(
  parameter int unsigned CLOCK_HZ = 100_000_000,
  parameter int unsigned RATE_HZ  = 1_000
) (
  input  logic clk,
  input  logic rst_n,
  output logic enable
);

  localparam int unsigned DIVISOR =
      (RATE_HZ == 0) ? 1 : CLOCK_HZ / RATE_HZ;
  localparam int unsigned COUNTER_WIDTH =
      (DIVISOR <= 1) ? 1 : $clog2(DIVISOR);

  logic [COUNTER_WIDTH-1:0] counter;

  initial begin
    if (CLOCK_HZ == 0 || RATE_HZ == 0) begin
      $fatal(1, "rate_enable frequencies must be nonzero");
    end
    if (RATE_HZ > CLOCK_HZ) begin
      $fatal(1, "rate_enable RATE_HZ cannot exceed CLOCK_HZ");
    end
    if ((CLOCK_HZ % RATE_HZ) != 0) begin
      $fatal(1, "rate_enable CLOCK_HZ must be divisible by RATE_HZ");
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      counter <= '0;
      enable  <= 1'b0;
    end else if (counter == COUNTER_WIDTH'(DIVISOR - 1)) begin
      counter <= '0;
      enable  <= 1'b1;
    end else begin
      counter <= counter + 1'b1;
      enable  <= 1'b0;
    end
  end

endmodule
