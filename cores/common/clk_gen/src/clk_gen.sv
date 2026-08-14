module clk_gen #(
  parameter real CLKIN_FREQ  = 12.0,
  parameter real CLKOUT_FREQ = 100.0,
  parameter real VCO_MULT    = 50.0,
  parameter real OUT_DIV     = 6.0,
  parameter int  DIV_CLK     = 1
)(
  input  logic clk_in,
  input  logic rst,
  output logic clk_out,
  output logic locked
);

  logic clk_fb;
  logic clk_out_unbuf;

  MMCME2_BASE #(
    .CLKIN1_PERIOD    (1000.0 / CLKIN_FREQ),
    .CLKFBOUT_MULT_F (VCO_MULT),
    .CLKOUT0_DIVIDE_F(OUT_DIV),
    .DIVCLK_DIVIDE   (DIV_CLK),
    .CLKOUT0_PHASE   (0.0),
    .STARTUP_WAIT    ("FALSE")
  ) u_mmcm (
    .CLKIN1  (clk_in),
    .RST     (rst),
    .CLKFBIN (clk_fb),
    .CLKFBOUT(clk_fb),
    .CLKOUT0 (clk_out_unbuf),
    .LOCKED  (locked),
    .PWRDWN  (1'b0),
    .CLKOUT0B(),
    .CLKOUT1 (),
    .CLKOUT1B(),
    .CLKOUT2 (),
    .CLKOUT2B(),
    .CLKOUT3 (),
    .CLKOUT3B(),
    .CLKOUT4 (),
    .CLKOUT5 (),
    .CLKOUT6 (),
    .CLKFBOUTB()
  );

  BUFG u_bufg (
    .I(clk_out_unbuf),
    .O(clk_out)
  );

endmodule
