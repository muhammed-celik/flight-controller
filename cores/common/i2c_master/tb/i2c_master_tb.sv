module i2c_master_tb (
  input  logic       i_clk,
  input  logic       i_rstn,
  input  logic       i_en,
  input  logic [6:0] i_dev_addr,
  input  logic       i_rw,
  input  logic [7:0] i_data,
  input  logic       slave_scl_low,
  input  logic       slave_sda_low,
  output logic       o_done,
  output logic       o_error,
  output logic [7:0] o_data,
  output logic       scl,
  output logic       sda
);

  tri1 scl_bus;
  tri1 sda_bus;

  assign scl_bus = slave_scl_low ? 1'b0 : 1'bz;
  assign sda_bus = slave_sda_low ? 1'b0 : 1'bz;
  assign scl = scl_bus;
  assign sda = sda_bus;

  i2c_master dut (
    .i_clk,
    .i_rstn,
    .i_en,
    .i_dev_addr,
    .i_rw,
    .i_data,
    .o_done,
    .o_error,
    .o_data,
    .io_scl(scl_bus),
    .io_sda(sda_bus)
  );

endmodule
