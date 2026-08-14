module spi_master 
  import spi_master_pkg::*;
(
  input  logic i_clk,
  input  logic i_rstn,

  input  logic i_req_valid,
  input  logic [$clog2(CS_CNT)-1:0] i_req_device,
  input  logic i_req_write,
  input  logic [7:0] i_req_addr,
  input  logic [7:0] i_req_len,
  input  logic i_req_auto_inc,

  input  logic i_wdata_valid,
  input  logic [7:0] i_wdata,

  output logic o_rdata_valid,
  output logic [7:0] o_rdata,

  output logic o_busy,
  output logic o_done,

  output logic [CS_CNT-1:0] o_cs,
  output logic o_sclk,
  output logic o_mosi,
  input  logic i_miso
);

logic start_spi, spi_busy, spi_done;
logic [$clog2(CS_CNT)-1:0] cs_sel;
logic [15:0] clk_div;
logic [1:0] spi_mode;
logic [7:0] nbytes;
logic tx_valid, rx_valid;
logic [7:0] tx_data, rx_data;

assign start_spi = i_req_valid;
assign o_busy = spi_busy;
assign o_done = spi_done;
assign cs_sel = i_req_device;
assign clk_div = 16'd1; // we can make this configurable if needed
assign spi_mode = 2'd3; // we can make this configurable if needed
assign nbytes = i_req_len + 1; // input request length excludes the address byte, so we add 1 to account for it
assign tx_valid = i_wdata_valid;
assign tx_data = i_req_valid ? i_req_addr : i_wdata; // if request is valid, we send the address first, then the data
assign o_rdata_valid = rx_valid;
assign o_rdata = rx_data;

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
