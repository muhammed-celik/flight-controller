module spi_master_controller 
  import spi_master_pkg::*;
(
  input  logic i_clk,
  input  logic i_rstn,

  input  logic i_req_valid,
  output logic o_req_ready,

  input  logic [$clog2(CS_CNT)-1:0] i_req_device,
  input  logic i_req_write,
  input  logic [7:0] i_req_addr,
  input  logic [7:0] i_req_len,
  input  logic i_req_auto_inc,

  input  logic i_wdata_valid,
  input  logic [7:0] i_wdata,

  output logic o_rdata_valid,
  input  logic i_rdata_ready,
  output logic [7:0] o_rdata,

  output logic o_busy,
  output logic o_done,
  output logic o_error,

  output logic [CS_CNT-1:0] o_cs,
  output logic o_sclk,
  output logic o_mosi,
  input  logic i_miso
);

spi_master_driver  spi_master_driver_inst (
  .i_clk(i_clk),
  .i_rstn(i_rstn),
  .i_start(start_spi),
  .o_busy(spi_busy),
  .o_done(spi_done),
  .i_cs_sel(cs_sel),
  .i_clk_div(clk_div),
  .i_mode(spi_mode),
  .i_nbytes(nbytes),
  .i_tx_valid(tx_valid),
  .i_tx_data(tx_data),
  .o_rx_valid(rx_valid),
  .o_rx_data(rx_data),
  .o_cs(o_cs),
  .o_sclk(o_sclk),
  .o_mosi(o_mosi),
  .i_miso(i_miso)
);


endmodule
