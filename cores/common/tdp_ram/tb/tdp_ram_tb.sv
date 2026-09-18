module tdp_ram_tb (
  // Instance 0 has both ports on a single clock.
  input  logic        i_clk,

  input  logic        p0_i_en_a,
  input  logic        p0_i_we_a,
  input  logic [3:0]  p0_i_addr_a,
  input  logic [31:0] p0_i_wdata_a,
  input  logic [3:0]  p0_i_be_a,
  output logic [31:0] p0_o_rdata_a,

  input  logic        p0_i_en_b,
  input  logic        p0_i_we_b,
  input  logic [3:0]  p0_i_addr_b,
  input  logic [31:0] p0_i_wdata_b,
  input  logic [3:0]  p0_i_be_b,
  output logic [31:0] p0_o_rdata_b,

  // Instance 1 has independent clocks per port.
  input  logic        p1_i_clk_a,
  input  logic        p1_i_clk_b,

  input  logic        p1_i_en_a,
  input  logic        p1_i_we_a,
  input  logic [3:0]  p1_i_addr_a,
  input  logic [15:0] p1_i_wdata_a,
  input  logic [1:0]  p1_i_be_a,
  output logic [15:0] p1_o_rdata_a,

  input  logic        p1_i_en_b,
  input  logic        p1_i_we_b,
  input  logic [3:0]  p1_i_addr_b,
  input  logic [15:0] p1_i_wdata_b,
  input  logic [1:0]  p1_i_be_b,
  output logic [15:0] p1_o_rdata_b
);

  // p0: 32-bit x 16, single clock, no byte enable, no output register, READ_FIRST
  tdp_ram #(
    .DATA_WIDTH(32), .ADDR_WIDTH(4), .BYTE_EN(1'b0), .REG_OUT(1'b0), .RDW_MODE(0)
  ) u_p0 (
    .i_clk_a(i_clk), .i_en_a(p0_i_en_a), .i_we_a(p0_i_we_a), .i_addr_a(p0_i_addr_a),
    .i_wdata_a(p0_i_wdata_a), .i_be_a(p0_i_be_a), .o_rdata_a(p0_o_rdata_a),
    .i_clk_b(i_clk), .i_en_b(p0_i_en_b), .i_we_b(p0_i_we_b), .i_addr_b(p0_i_addr_b),
    .i_wdata_b(p0_i_wdata_b), .i_be_b(p0_i_be_b), .o_rdata_b(p0_o_rdata_b)
  );

  // p1: 16-bit x 16, dual clock, byte enable, output register, WRITE_FIRST
  tdp_ram #(
    .DATA_WIDTH(16), .ADDR_WIDTH(4), .BYTE_EN(1'b1), .REG_OUT(1'b1), .RDW_MODE(1)
  ) u_p1 (
    .i_clk_a(p1_i_clk_a), .i_en_a(p1_i_en_a), .i_we_a(p1_i_we_a), .i_addr_a(p1_i_addr_a),
    .i_wdata_a(p1_i_wdata_a), .i_be_a(p1_i_be_a), .o_rdata_a(p1_o_rdata_a),
    .i_clk_b(p1_i_clk_b), .i_en_b(p1_i_en_b), .i_we_b(p1_i_we_b), .i_addr_b(p1_i_addr_b),
    .i_wdata_b(p1_i_wdata_b), .i_be_b(p1_i_be_b), .o_rdata_b(p1_o_rdata_b)
  );

endmodule
