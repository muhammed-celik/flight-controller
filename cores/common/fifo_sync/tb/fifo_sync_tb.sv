module fifo_sync_tb (
  input  logic        i_clk,
  input  logic        i_rstn,

  // Instance a: 32-bit data, 4-bit user, depth 16
  input  logic        a_i_wr_en,
  input  logic [31:0] a_i_wr_data,
  input  logic [3:0]  a_i_wr_user,
  output logic        a_o_full,
  output logic        a_o_almost_full,
  output logic        a_o_empty,
  output logic        a_o_almost_empty,
  output logic [4:0]  a_o_level,
  input  logic        a_i_rd_en,
  output logic [31:0] a_o_rd_data,
  output logic [3:0]  a_o_rd_user,

  // Instance b: 8-bit data, 1-bit user, depth 8
  input  logic        b_i_wr_en,
  input  logic [7:0]  b_i_wr_data,
  input  logic        b_i_wr_user,
  output logic        b_o_full,
  output logic        b_o_almost_full,
  output logic        b_o_empty,
  output logic        b_o_almost_empty,
  output logic [3:0]  b_o_level,
  input  logic        b_i_rd_en,
  output logic [7:0]  b_o_rd_data,
  output logic        b_o_rd_user
);

  fifo_sync #(
    .DATA_WIDTH(32), .USER_WIDTH(4), .ADDR_WIDTH(4),
    .ALMOST_FULL_THRESH(14), .ALMOST_EMPTY_THRESH(1)
  ) u_a (
    .i_clk(i_clk), .i_rstn(i_rstn),
    .i_wr_en(a_i_wr_en), .i_wr_data(a_i_wr_data), .i_wr_user(a_i_wr_user),
    .o_full(a_o_full), .o_almost_full(a_o_almost_full),
    .i_rd_en(a_i_rd_en), .o_rd_data(a_o_rd_data), .o_rd_user(a_o_rd_user),
    .o_empty(a_o_empty), .o_almost_empty(a_o_almost_empty), .o_level(a_o_level)
  );

  fifo_sync #(
    .DATA_WIDTH(8), .USER_WIDTH(1), .ADDR_WIDTH(3),
    .ALMOST_FULL_THRESH(6), .ALMOST_EMPTY_THRESH(1)
  ) u_b (
    .i_clk(i_clk), .i_rstn(i_rstn),
    .i_wr_en(b_i_wr_en), .i_wr_data(b_i_wr_data), .i_wr_user(b_i_wr_user),
    .o_full(b_o_full), .o_almost_full(b_o_almost_full),
    .i_rd_en(b_i_rd_en), .o_rd_data(b_o_rd_data), .o_rd_user(b_o_rd_user),
    .o_empty(b_o_empty), .o_almost_empty(b_o_almost_empty), .o_level(b_o_level)
  );

endmodule
