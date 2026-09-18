module sdp_ram_tb (
  // Instance a runs from a single clock (write and read clocks tied together).
  input  logic        i_clk,

  input  logic        a_i_en_w,
  input  logic        a_i_we_w,
  input  logic [3:0]  a_i_addr_w,
  input  logic [31:0] a_i_wdata,
  input  logic [3:0]  a_i_be,
  input  logic        a_i_en_r,
  input  logic [3:0]  a_i_addr_r,
  output logic [31:0] a_o_rdata,

  // Instance b has independent write and read clocks.
  input  logic        b_i_clk_w,
  input  logic        b_i_clk_r,
  input  logic        b_i_en_w,
  input  logic        b_i_we_w,
  input  logic [3:0]  b_i_addr_w,
  input  logic [15:0] b_i_wdata,
  input  logic [1:0]  b_i_be,
  input  logic        b_i_en_r,
  input  logic [3:0]  b_i_addr_r,
  output logic [15:0] b_o_rdata
);

  // a: 32-bit x 16, single clock, no byte enable, no output register
  sdp_ram #(
    .DATA_WIDTH(32), .ADDR_WIDTH(4), .BYTE_EN(1'b0), .REG_OUT(1'b0)
  ) u_a (
    .i_clk_w(i_clk), .i_en_w(a_i_en_w), .i_we_w(a_i_we_w), .i_addr_w(a_i_addr_w),
    .i_wdata(a_i_wdata), .i_be(a_i_be),
    .i_clk_r(i_clk), .i_en_r(a_i_en_r), .i_addr_r(a_i_addr_r), .o_rdata(a_o_rdata)
  );

  // b: 16-bit x 16, dual clock, byte enable, output register
  sdp_ram #(
    .DATA_WIDTH(16), .ADDR_WIDTH(4), .BYTE_EN(1'b1), .REG_OUT(1'b1)
  ) u_b (
    .i_clk_w(b_i_clk_w), .i_en_w(b_i_en_w), .i_we_w(b_i_we_w), .i_addr_w(b_i_addr_w),
    .i_wdata(b_i_wdata), .i_be(b_i_be),
    .i_clk_r(b_i_clk_r), .i_en_r(b_i_en_r), .i_addr_r(b_i_addr_r), .o_rdata(b_o_rdata)
  );

endmodule
