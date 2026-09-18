module axis_fifo_tb (
  input  logic        i_clk_w,
  input  logic        i_clk_r,
  input  logic        i_rstn_w,
  input  logic        i_rstn_r,

  // Sync instance
  input  logic        s_i_s_tvalid,
  output logic        s_o_s_tready,
  input  logic [31:0] s_i_s_tdata,
  input  logic [3:0]  s_i_s_tuser,
  input  logic        s_i_s_tlast,
  output logic        s_o_m_tvalid,
  input  logic        s_i_m_tready,
  output logic [31:0] s_o_m_tdata,
  output logic [3:0]  s_o_m_tuser,
  output logic        s_o_m_tlast,

  // Async instance
  input  logic        a_i_s_tvalid,
  output logic        a_o_s_tready,
  input  logic [31:0] a_i_s_tdata,
  input  logic [3:0]  a_i_s_tuser,
  input  logic        a_i_s_tlast,
  output logic        a_o_m_tvalid,
  input  logic        a_i_m_tready,
  output logic [31:0] a_o_m_tdata,
  output logic [3:0]  a_o_m_tuser,
  output logic        a_o_m_tlast
);

  axis_fifo #(
    .TDATA_WIDTH(32), .TUSER_WIDTH(4), .ADDR_WIDTH(4), .CDC(1'b0)
  ) u_s (
    .i_clk_w(i_clk_w), .i_rstn_w(i_rstn_w), .i_clk_r(i_clk_r), .i_rstn_r(i_rstn_r),
    .i_s_axis_tvalid(s_i_s_tvalid), .o_s_axis_tready(s_o_s_tready),
    .i_s_axis_tdata(s_i_s_tdata), .i_s_axis_tuser(s_i_s_tuser), .i_s_axis_tlast(s_i_s_tlast),
    .o_m_axis_tvalid(s_o_m_tvalid), .i_m_axis_tready(s_i_m_tready),
    .o_m_axis_tdata(s_o_m_tdata), .o_m_axis_tuser(s_o_m_tuser), .o_m_axis_tlast(s_o_m_tlast)
  );

  axis_fifo #(
    .TDATA_WIDTH(32), .TUSER_WIDTH(4), .ADDR_WIDTH(4), .CDC(1'b1)
  ) u_a (
    .i_clk_w(i_clk_w), .i_rstn_w(i_rstn_w), .i_clk_r(i_clk_r), .i_rstn_r(i_rstn_r),
    .i_s_axis_tvalid(a_i_s_tvalid), .o_s_axis_tready(a_o_s_tready),
    .i_s_axis_tdata(a_i_s_tdata), .i_s_axis_tuser(a_i_s_tuser), .i_s_axis_tlast(a_i_s_tlast),
    .o_m_axis_tvalid(a_o_m_tvalid), .i_m_axis_tready(a_i_m_tready),
    .o_m_axis_tdata(a_o_m_tdata), .o_m_axis_tuser(a_o_m_tuser), .o_m_axis_tlast(a_o_m_tlast)
  );

endmodule
