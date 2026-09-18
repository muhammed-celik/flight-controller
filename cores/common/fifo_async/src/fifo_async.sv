//-----------------------------------------------------------------------------
// fifo_async.sv
// Parametric dual-clock, first-word fall-through (FWFT) FIFO.
//
// Independent write (i_clk_w) and read (i_clk_r) domains with gray-coded
// pointers crossing through two-flop synchronisers. Storage is an sdp_ram
// instance. o_rd_data is valid whenever o_empty is low.
//
// The read side prefetches words from the RAM into a registered output stage to
// give true first-word fall-through. The read pointer used to synchronise the
// full/almost_full flags into the write domain advances only when a word is
// actually popped, so o_full is conservative and safe across clock domains.
// o_level and o_almost_empty are measured in the read clock domain.
//
// Parameters:
//   DATA_WIDTH  : payload width
//   USER_WIDTH  : sideband width travelling with each word (0 allowed)
//   ADDR_WIDTH  : number of address bits, RAM depth = 2**ADDR_WIDTH
//   ALMOST_FULL_THRESH  : o_almost_full asserts when write occupancy >= value
//   ALMOST_EMPTY_THRESH : o_almost_empty asserts when read occupancy <= value
//   INIT_FILE   : optional $readmemh file for the underlying RAM
//-----------------------------------------------------------------------------
module fifo_async #(
  parameter int unsigned DATA_WIDTH  = 32,
  parameter int unsigned USER_WIDTH  = 0,
  parameter int unsigned ADDR_WIDTH  = 4,
  parameter int unsigned ALMOST_FULL_THRESH  = (1 << ADDR_WIDTH) - 1,
  parameter int unsigned ALMOST_EMPTY_THRESH = 0,
  parameter string       INIT_FILE   = ""
) (
  // Write clock domain
  input  logic                    i_clk_w,
  input  logic                    i_rstn_w,
  input  logic                    i_wr_en,
  input  logic [DATA_WIDTH-1:0]   i_wr_data,
  input  logic [((USER_WIDTH < 1) ? 1 : USER_WIDTH)-1:0] i_wr_user,
  output logic                    o_full,
  output logic                    o_almost_full,

  // Read clock domain
  input  logic                    i_clk_r,
  input  logic                    i_rstn_r,
  input  logic                    i_rd_en,
  output logic [DATA_WIDTH-1:0]   o_rd_data,
  output logic [((USER_WIDTH < 1) ? 1 : USER_WIDTH)-1:0] o_rd_user,
  output logic                    o_empty,
  output logic                    o_almost_empty,
  output logic [ADDR_WIDTH:0]     o_level
);

  localparam int unsigned DEPTH = 1 << ADDR_WIDTH;
  localparam int unsigned PW    = ADDR_WIDTH + 1;
  localparam int unsigned UW    = (USER_WIDTH < 1) ? 1 : USER_WIDTH;
  localparam int unsigned DW    = DATA_WIDTH + UW;

  function automatic [PW-1:0] bin2gray(input [PW-1:0] bin);
    bin2gray = bin ^ (bin >> 1);
  endfunction

  function automatic [PW-1:0] gray2bin(input [PW-1:0] gray);
    gray2bin[PW-1] = gray[PW-1];
    for (int i = PW - 2; i >= 0; i--) begin
      gray2bin[i] = gray2bin[i+1] ^ gray[i];
    end
  endfunction

  // Write domain
  logic [PW-1:0] wr_ptr, rd_ptr_w;
  logic [PW-1:0] wr_gray;
  logic [PW-1:0] rd_gray_w_meta, rd_gray_w_sync;

  // Read domain
  logic [PW-1:0] rd_ptr, wr_ptr_r;
  logic [PW-1:0] wr_gray_r_meta, wr_gray_r_sync;
  // Pop pointer advances only when a word is actually consumed. It is the one
  // synchronised to the write domain so the prefetch pipeline never disturbs
  // the full/almost_full flags.
  logic [PW-1:0] rd_pop_ptr, rd_pop_gray;
  logic          rd_valid;
  logic [DW-1:0] fifo_dout;

  wire pop      = i_rd_en && rd_valid;
  wire has_next = (rd_ptr != wr_ptr_r);
  wire refill   = has_next && (!rd_valid || pop);

  wire [PW-1:0] wr_used = wr_ptr - rd_ptr_w;
  wire [PW-1:0] rd_used = wr_ptr_r - rd_pop_ptr;

  wire wr_fire = i_wr_en && !o_full;

  assign o_full         = (wr_used == DEPTH[PW-1:0]);
  assign o_almost_full  = (wr_used >= ALMOST_FULL_THRESH);
  assign o_empty        = !rd_valid;
  assign o_almost_empty = (rd_used <= ALMOST_EMPTY_THRESH);
  assign o_level        = rd_used;
  assign o_rd_data      = fifo_dout[DATA_WIDTH-1:0];

  assign o_rd_user = fifo_dout[DW-1:DATA_WIDTH];

  // Write pointers and gray synchronization (read pointer into write domain)
  always_ff @(posedge i_clk_w or negedge i_rstn_w) begin
    if (!i_rstn_w) begin
      wr_ptr       <= '0;
      wr_gray      <= '0;
      rd_gray_w_meta <= '0;
      rd_gray_w_sync <= '0;
      rd_ptr_w     <= '0;
    end else begin
      if (wr_fire) begin
        wr_ptr <= wr_ptr + 1'b1;
      end
      wr_gray        <= bin2gray(wr_ptr);
      rd_gray_w_meta <= rd_pop_gray;
      rd_gray_w_sync <= rd_gray_w_meta;
      rd_ptr_w       <= gray2bin(rd_gray_w_sync);
    end
  end

  // Read pointers and gray synchronization (write pointer into read domain)
  always_ff @(posedge i_clk_r or negedge i_rstn_r) begin
    if (!i_rstn_r) begin
      rd_ptr       <= '0;
      rd_pop_ptr   <= '0;
      rd_pop_gray  <= '0;
      rd_valid     <= 1'b0;
      wr_gray_r_meta <= '0;
      wr_gray_r_sync <= '0;
      wr_ptr_r     <= '0;
    end else begin
      if (refill) begin
        rd_ptr   <= rd_ptr + 1'b1;
        rd_valid <= 1'b1;
      end else if (pop) begin
        rd_valid <= 1'b0;
      end
      if (pop) begin
        rd_pop_ptr <= rd_pop_ptr + 1'b1;
      end
      rd_pop_gray    <= bin2gray(rd_pop_ptr);
      wr_gray_r_meta <= wr_gray;
      wr_gray_r_sync <= wr_gray_r_meta;
      wr_ptr_r       <= gray2bin(wr_gray_r_sync);
    end
  end

  sdp_ram #(
    .DATA_WIDTH(DW),
    .ADDR_WIDTH(ADDR_WIDTH),
    .BYTE_EN(1'b0),
    .REG_OUT(1'b0),
    .INIT_FILE(INIT_FILE)
  ) u_mem (
    .i_clk_w(i_clk_w),
    .i_en_w(wr_fire),
    .i_we_w(1'b1),
    .i_addr_w(wr_ptr[ADDR_WIDTH-1:0]),
    .i_wdata({i_wr_user, i_wr_data}),
    .i_be('0),
    .i_clk_r(i_clk_r),
    .i_en_r(refill),
    .i_addr_r(rd_ptr[ADDR_WIDTH-1:0]),
    .o_rdata(fifo_dout)
  );

endmodule
