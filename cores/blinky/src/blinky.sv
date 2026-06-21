module blinky #(
  parameter int CLK_FREQ = 12_000_000,
  parameter int BLINK_HZ = 1
)(
  input  logic clk,
  input  logic rst_n,
  output logic led
);

  localparam int COUNT_MAX = CLK_FREQ / (2 * BLINK_HZ) - 1;
  localparam int COUNT_W   = $clog2(COUNT_MAX + 1);

  logic [COUNT_W-1:0] count;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      count <= '0;
      led   <= 1'b0;
    end else if (count == COUNT_MAX[COUNT_W-1:0]) begin
      count <= '0;
      led   <= ~led;
    end else begin
      count <= count + 1'b1;
    end
  end

endmodule
