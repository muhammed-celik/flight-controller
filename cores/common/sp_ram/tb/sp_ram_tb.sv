module sp_ram_tb (
  input  logic        i_clk,

  // Instance a: 32-bit x 16, no byte enable, no output register, READ_FIRST
  input  logic        a_i_en,
  input  logic        a_i_we,
  input  logic [3:0]  a_i_addr,
  input  logic [31:0] a_i_wdata,
  input  logic [3:0]  a_i_be,
  output logic [31:0] a_o_rdata,

  // Instance b: 16-bit x 8, byte enable, output register, WRITE_FIRST
  input  logic        b_i_en,
  input  logic        b_i_we,
  input  logic [2:0]  b_i_addr,
  input  logic [15:0] b_i_wdata,
  input  logic [1:0]  b_i_be,
  output logic [15:0] b_o_rdata,

  // Instance c: 8-bit x 8, no byte enable, no output register, NO_CHANGE
  input  logic        c_i_en,
  input  logic        c_i_we,
  input  logic [2:0]  c_i_addr,
  input  logic [7:0]  c_i_wdata,
  input  logic        c_i_be,
  output logic [7:0]  c_o_rdata
);

  sp_ram #(
    .DATA_WIDTH(32), .ADDR_WIDTH(4), .BYTE_EN(1'b0), .REG_OUT(1'b0), .RDW_MODE(0)
  ) u_a (
    .i_clk, .i_en(a_i_en), .i_we(a_i_we), .i_addr(a_i_addr),
    .i_wdata(a_i_wdata), .i_be(a_i_be), .o_rdata(a_o_rdata)
  );

  sp_ram #(
    .DATA_WIDTH(16), .ADDR_WIDTH(3), .BYTE_EN(1'b1), .REG_OUT(1'b1), .RDW_MODE(1)
  ) u_b (
    .i_clk, .i_en(b_i_en), .i_we(b_i_we), .i_addr(b_i_addr),
    .i_wdata(b_i_wdata), .i_be(b_i_be), .o_rdata(b_o_rdata)
  );

  sp_ram #(
    .DATA_WIDTH(8), .ADDR_WIDTH(3), .BYTE_EN(1'b0), .REG_OUT(1'b0), .RDW_MODE(2)
  ) u_c (
    .i_clk, .i_en(c_i_en), .i_we(c_i_we), .i_addr(c_i_addr),
    .i_wdata(c_i_wdata), .i_be(c_i_be), .o_rdata(c_o_rdata)
  );

endmodule
