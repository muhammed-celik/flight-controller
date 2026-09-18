//-----------------------------------------------------------------------------
// fifo_sync.sv
// Parametric single-clock, first-word fall-through (FWFT) FIFO.
//
// Storage is an sdp_ram instance. A small read-data register plus valid bit is
// maintained so that o_rd_data is valid whenever o_empty is low, i.e. the first
// word is available without asserting i_rd_en (first-word fall-through).
//
//   - i_wr_en is ignored while o_full is asserted.
//   - i_rd_en pops the head word when o_empty is low.
//   - USER_WIDTH sideband bits travel alongside each word.
//
// Parameters:
//   DATA_WIDTH  : payload width
//   USER_WIDTH  : sideband width travelling with each word (0 allowed)
//   ADDR_WIDTH  : number of address bits, DEPTH = 2**ADDR_WIDTH
//   ALMOST_FULL_THRESH  : o_almost_full asserts when level >= this value
//   ALMOST_EMPTY_THRESH : o_almost_empty asserts when level <= this value
//   INIT_FILE   : optional $readmemh file for the underlying RAM
//-----------------------------------------------------------------------------
module fifo_sync #(
  parameter int unsigned DATA_WIDTH  = 32,
  parameter int unsigned USER_WIDTH  = 0,
  parameter int unsigned ADDR_WIDTH  = 4,
  parameter int unsigned ALMOST_FULL_THRESH  = (1 << ADDR_WIDTH) - 1,
  parameter int unsigned ALMOST_EMPTY_THRESH = 0,
  parameter string       INIT_FILE   = ""
) (
  input  logic                    i_clk,
  input  logic                    i_rstn,

  input  logic                    i_wr_en,
  input  logic [DATA_WIDTH-1:0]   i_wr_data,
  input  logic [((USER_WIDTH < 1) ? 1 : USER_WIDTH)-1:0] i_wr_user,
  output logic                    o_full,
  output logic                    o_almost_full,

  input  logic                    i_rd_en,
  output logic [DATA_WIDTH-1:0]   o_rd_data,
  output logic [((USER_WIDTH < 1) ? 1 : USER_WIDTH)-1:0] o_rd_user,
  output logic                    o_empty,
  output logic                    o_almost_empty,
  output logic [ADDR_WIDTH:0]     o_level
);

  localparam int unsigned DEPTH = 1 << ADDR_WIDTH;
  localparam int unsigned PW    = ADDR_WIDTH + 1;
  // Floor the sideband at one bit so a zero-width port never becomes [-1:0].
  localparam int unsigned UW    = (USER_WIDTH < 1) ? 1 : USER_WIDTH;
  localparam int unsigned DW    = DATA_WIDTH + UW;

  logic [PW-1:0] wr_ptr, rd_ptr;
  logic          rd_valid;
  logic [DW-1:0] fifo_dout;

  wire pop      = i_rd_en && rd_valid;
  wire has_next = (rd_ptr != wr_ptr);
  wire refill   = has_next && (!rd_valid || pop);
  // Accept a write while full if a word is being popped this cycle.
  wire wr_fire  = i_wr_en && (!o_full || pop);

  logic [PW-1:0] count;
  assign count = (wr_ptr - rd_ptr) + rd_valid;

  assign o_full         = (count == DEPTH[PW-1:0]);
  assign o_almost_full  = (count >= ALMOST_FULL_THRESH);
  assign o_empty        = !rd_valid;
  assign o_almost_empty = (count <= ALMOST_EMPTY_THRESH);
  assign o_level        = count;
  assign o_rd_data      = fifo_dout[DATA_WIDTH-1:0];

  assign o_rd_user = fifo_dout[DW-1:DATA_WIDTH];

  always_ff @(posedge i_clk) begin
    if (!i_rstn) begin
      wr_ptr   <= '0;
      rd_ptr   <= '0;
      rd_valid <= 1'b0;
    end else begin
      if (wr_fire) begin
        wr_ptr <= wr_ptr + 1'b1;
      end

      if (refill) begin
        rd_ptr   <= rd_ptr + 1'b1;
        rd_valid <= 1'b1;
      end else if (pop) begin
        rd_valid <= 1'b0;
      end
    end
  end

  sdp_ram #(
    .DATA_WIDTH(DW),
    .ADDR_WIDTH(ADDR_WIDTH),
    .BYTE_EN(1'b0),
    .REG_OUT(1'b0),
    .INIT_FILE(INIT_FILE)
  ) u_mem (
    .i_clk_w(i_clk),
    .i_en_w(wr_fire),
    .i_we_w(1'b1),
    .i_addr_w(wr_ptr[ADDR_WIDTH-1:0]),
    .i_wdata({i_wr_user, i_wr_data}),
    .i_be('0),
    .i_clk_r(i_clk),
    .i_en_r(refill),
    .i_addr_r(rd_ptr[ADDR_WIDTH-1:0]),
    .o_rdata(fifo_dout)
  );

endmodule
