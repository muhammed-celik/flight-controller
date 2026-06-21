module top (
  input  logic clk,
  input  logic btn,
  output logic led,
  output logic locked
);

  logic sys_clk;
  logic rst_n;

  assign rst_n = locked & ~btn;

  clk_gen #(
    .CLKIN_FREQ  (12.0),
    .VCO_MULT    (50.0),
    .OUT_DIV     (6.0),
    .DIV_CLK     (1)
  ) u_clk_gen (
    .clk_in  (clk),
    .rst     (btn),
    .clk_out (sys_clk),
    .locked  (locked)
  );

  blinky #(
    .CLK_FREQ (100_000_000),
    .BLINK_HZ (1)
  ) u_blinky (
    .clk   (sys_clk),
    .rst_n (rst_n),
    .led   (led)
  );

endmodule
