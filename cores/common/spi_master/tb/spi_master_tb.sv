module spi_master_tb (
  input  logic       i_clk,
  input  logic       i_rstn,

  input  logic       m0_i_en,
  input  logic [7:0] m0_i_data,
  input  logic       m0_i_miso,
  output logic       m0_o_done,
  output logic [7:0] m0_o_data,
  output logic       m0_o_cs,
  output logic       m0_o_sclk,
  output logic       m0_o_mosi,

  input  logic       m1_i_en,
  input  logic [7:0] m1_i_data,
  input  logic       m1_i_miso,
  output logic       m1_o_done,
  output logic [7:0] m1_o_data,
  output logic       m1_o_cs,
  output logic       m1_o_sclk,
  output logic       m1_o_mosi,

  input  logic       m2_i_en,
  input  logic [7:0] m2_i_data,
  input  logic       m2_i_miso,
  output logic       m2_o_done,
  output logic [7:0] m2_o_data,
  output logic       m2_o_cs,
  output logic       m2_o_sclk,
  output logic       m2_o_mosi,

  input  logic       m3_i_en,
  input  logic [7:0] m3_i_data,
  input  logic       m3_i_miso,
  output logic       m3_o_done,
  output logic [7:0] m3_o_data,
  output logic       m3_o_cs,
  output logic       m3_o_sclk,
  output logic       m3_o_mosi
);

  spi_master #(.CPOL(1'b0), .CPHA(1'b0)) mode0 (
    .i_clk, .i_rstn, .i_en(m0_i_en), .i_data(m0_i_data), .i_miso(m0_i_miso),
    .o_done(m0_o_done), .o_data(m0_o_data), .o_cs(m0_o_cs),
    .o_sclk(m0_o_sclk), .o_mosi(m0_o_mosi)
  );

  spi_master #(.CPOL(1'b0), .CPHA(1'b1)) mode1 (
    .i_clk, .i_rstn, .i_en(m1_i_en), .i_data(m1_i_data), .i_miso(m1_i_miso),
    .o_done(m1_o_done), .o_data(m1_o_data), .o_cs(m1_o_cs),
    .o_sclk(m1_o_sclk), .o_mosi(m1_o_mosi)
  );

  spi_master #(.CPOL(1'b1), .CPHA(1'b0)) mode2 (
    .i_clk, .i_rstn, .i_en(m2_i_en), .i_data(m2_i_data), .i_miso(m2_i_miso),
    .o_done(m2_o_done), .o_data(m2_o_data), .o_cs(m2_o_cs),
    .o_sclk(m2_o_sclk), .o_mosi(m2_o_mosi)
  );

  spi_master #(.CPOL(1'b1), .CPHA(1'b1)) mode3 (
    .i_clk, .i_rstn, .i_en(m3_i_en), .i_data(m3_i_data), .i_miso(m3_i_miso),
    .o_done(m3_o_done), .o_data(m3_o_data), .o_cs(m3_o_cs),
    .o_sclk(m3_o_sclk), .o_mosi(m3_o_mosi)
  );

endmodule
