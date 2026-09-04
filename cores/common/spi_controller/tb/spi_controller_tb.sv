module spi_controller_tb (
  input  logic       i_clk,
  input  logic       i_rstn,
  input  logic       i_cmd_valid,
  input  logic       i_cmd_type,
  input  logic [4:0] i_cmd_nbytes,
  input  logic [7:0] i_cmd_addr,
  output logic       o_cmd_ready,
  input  logic       i_data_valid,
  input  logic [7:0] i_data,
  output logic       o_data_valid,
  output logic [7:0] o_data,
  output logic       o_cs,
  output logic       o_sclk,
  output logic       o_mosi,
  input  logic       i_miso,
  output logic       o_spi_done
);

  spi_controller dut (
    .i_clk,
    .i_rstn,
    .i_cmd_valid,
    .i_cmd_type,
    .i_cmd_nbytes,
    .i_cmd_addr,
    .o_cmd_ready,
    .i_data_valid,
    .i_data,
    .o_data_valid,
    .o_data,
    .o_cs,
    .o_sclk,
    .o_mosi,
    .i_miso
  );

  assign o_spi_done = dut.spi_done;

endmodule
