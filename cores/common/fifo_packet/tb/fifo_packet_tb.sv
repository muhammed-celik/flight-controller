module fifo_packet_tb (
  input  logic        i_clk_w,
  input  logic        i_clk_r,
  input  logic        i_rstn_w,
  input  logic        i_rstn_r,

  // Sync instance (CDC = 0)
  input  logic        s_i_wr_en,
  input  logic [31:0] s_i_wr_data,
  input  logic [3:0]  s_i_wr_user,
  input  logic        s_i_wr_sop,
  input  logic        s_i_wr_eop,
  output logic        s_o_full,
  output logic        s_o_almost_full,
  input  logic        s_i_rd_en,
  output logic [31:0] s_o_rd_data,
  output logic [3:0]  s_o_rd_user,
  output logic        s_o_rd_sop,
  output logic        s_o_rd_eop,
  output logic        s_o_empty,
  output logic        s_o_almost_empty,
  output logic [4:0]  s_o_level,

  // Async instance (CDC = 1)
  input  logic        a_i_wr_en,
  input  logic [31:0] a_i_wr_data,
  input  logic [3:0]  a_i_wr_user,
  input  logic        a_i_wr_sop,
  input  logic        a_i_wr_eop,
  output logic        a_o_full,
  output logic        a_o_almost_full,
  input  logic        a_i_rd_en,
  output logic [31:0] a_o_rd_data,
  output logic [3:0]  a_o_rd_user,
  output logic        a_o_rd_sop,
  output logic        a_o_rd_eop,
  output logic        a_o_empty,
  output logic        a_o_almost_empty,
  output logic [4:0]  a_o_level
);

  fifo_packet #(
    .DATA_WIDTH(32), .USER_WIDTH(4), .ADDR_WIDTH(4), .CDC(1'b0)
  ) u_s (
    .i_clk_w(i_clk_w), .i_rstn_w(i_rstn_w), .i_clk_r(i_clk_r), .i_rstn_r(i_rstn_r),
    .i_wr_en(s_i_wr_en), .i_wr_data(s_i_wr_data), .i_wr_user(s_i_wr_user),
    .i_wr_sop(s_i_wr_sop), .i_wr_eop(s_i_wr_eop),
    .o_full(s_o_full), .o_almost_full(s_o_almost_full),
    .i_rd_en(s_i_rd_en), .o_rd_data(s_o_rd_data), .o_rd_user(s_o_rd_user),
    .o_rd_sop(s_o_rd_sop), .o_rd_eop(s_o_rd_eop),
    .o_empty(s_o_empty), .o_almost_empty(s_o_almost_empty), .o_level(s_o_level)
  );

  fifo_packet #(
    .DATA_WIDTH(32), .USER_WIDTH(4), .ADDR_WIDTH(4), .CDC(1'b1)
  ) u_a (
    .i_clk_w(i_clk_w), .i_rstn_w(i_rstn_w), .i_clk_r(i_clk_r), .i_rstn_r(i_rstn_r),
    .i_wr_en(a_i_wr_en), .i_wr_data(a_i_wr_data), .i_wr_user(a_i_wr_user),
    .i_wr_sop(a_i_wr_sop), .i_wr_eop(a_i_wr_eop),
    .o_full(a_o_full), .o_almost_full(a_o_almost_full),
    .i_rd_en(a_i_rd_en), .o_rd_data(a_o_rd_data), .o_rd_user(a_o_rd_user),
    .o_rd_sop(a_o_rd_sop), .o_rd_eop(a_o_rd_eop),
    .o_empty(a_o_empty), .o_almost_empty(a_o_almost_empty), .o_level(a_o_level)
  );

endmodule
