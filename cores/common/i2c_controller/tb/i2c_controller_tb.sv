module i2c_controller_tb (
  input  logic       i_clk,
  input  logic       i_rstn,
  input  logic       i_cmd_valid,
  input  logic       i_cmd_type,
  input  logic [4:0] i_cmd_nbytes,
  input  logic [6:0] i_cmd_dev_addr,
  input  logic [7:0] i_cmd_reg_addr,
  output logic       o_cmd_ready,
  output logic       o_cmd_error,
  input  logic       i_data_valid,
  input  logic [7:0] i_data,
  output logic       o_data_valid,
  output logic [7:0] o_data,
  input  logic       slave_scl_low,
  input  logic       slave_sda_low,
  output logic       scl,
  output logic       sda,
  output logic       o_i2c_done
);

  tri1 scl_bus;
  tri1 sda_bus;

  assign scl_bus = slave_scl_low ? 1'b0 : 1'bz;
  assign sda_bus = slave_sda_low ? 1'b0 : 1'bz;
  assign scl = scl_bus;
  assign sda = sda_bus;

  i2c_controller dut (
    .i_clk,
    .i_rstn,
    .i_cmd_valid,
    .i_cmd_type,
    .i_cmd_nbytes,
    .i_cmd_dev_addr,
    .i_cmd_reg_addr,
    .o_cmd_ready,
    .o_cmd_error,
    .i_data_valid,
    .i_data,
    .o_data_valid,
    .o_data,
    .io_scl(scl_bus),
    .io_sda(sda_bus)
  );

  assign o_i2c_done = dut.i2c_done;

endmodule
