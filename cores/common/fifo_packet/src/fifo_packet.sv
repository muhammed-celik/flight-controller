//-----------------------------------------------------------------------------
// fifo_packet.sv
// Parametric first-word fall-through FIFO that carries packet boundaries.
//
// Every stored word is tagged with a start-of-packet (SOP) and end-of-packet
// (EOP) flag alongside the payload and user sideband. A single base FIFO is
// instantiated, selected by CDC:
//   CDC = 0 -> fifo_sync  (single clock, i_clk_w)
//   CDC = 1 -> fifo_async (i_clk_w / i_clk_r)
//
// This is a streaming packet FIFO: the producer is expected to honour o_full
// (backpressure). SOP/EOP are preserved so the consumer can recover packet
// boundaries. Store-and-forward gating is provided by axis_packet_fifo.
//
// Parameters:
//   DATA_WIDTH  : packet payload width
//   USER_WIDTH  : additional per-word sideband (0 allowed)
//   ADDR_WIDTH  : base FIFO depth = 2**ADDR_WIDTH
//   CDC         : 0 = single clock, 1 = independent write/read clocks
//   ALMOST_FULL_THRESH / ALMOST_EMPTY_THRESH / INIT_FILE as for the base FIFO
//-----------------------------------------------------------------------------
module fifo_packet #(
  parameter int unsigned DATA_WIDTH  = 32,
  parameter int unsigned USER_WIDTH  = 0,
  parameter int unsigned ADDR_WIDTH  = 4,
  parameter bit          CDC         = 1'b0,
  parameter int unsigned ALMOST_FULL_THRESH  = (1 << ADDR_WIDTH) - 1,
  parameter int unsigned ALMOST_EMPTY_THRESH = 0,
  parameter string       INIT_FILE   = ""
) (
  // Write domain
  input  logic                    i_clk_w,
  input  logic                    i_rstn_w,
  input  logic                    i_wr_en,
  input  logic [DATA_WIDTH-1:0]   i_wr_data,
  input  logic [((USER_WIDTH < 1) ? 1 : USER_WIDTH)-1:0] i_wr_user,
  input  logic                    i_wr_sop,
  input  logic                    i_wr_eop,
  output logic                    o_full,
  output logic                    o_almost_full,

  // Read domain
  input  logic                    i_clk_r,
  input  logic                    i_rstn_r,
  input  logic                    i_rd_en,
  output logic [DATA_WIDTH-1:0]   o_rd_data,
  output logic [((USER_WIDTH < 1) ? 1 : USER_WIDTH)-1:0] o_rd_user,
  output logic                    o_rd_sop,
  output logic                    o_rd_eop,
  output logic                    o_empty,
  output logic                    o_almost_empty,
  output logic [ADDR_WIDTH:0]     o_level
);

  localparam int unsigned UW = (USER_WIDTH < 1) ? 1 : USER_WIDTH;

  logic [DATA_WIDTH-1:0] base_rd_data;
  logic [UW+1:0]         base_rd_user;
  logic [UW+1:0]         base_wr_user;

  assign base_wr_user = {i_wr_eop, i_wr_sop, i_wr_user};
  assign o_rd_data    = base_rd_data;
  assign {o_rd_eop, o_rd_sop, o_rd_user} = base_rd_user;

  generate
    if (CDC) begin : gen_async
      fifo_async #(
        .DATA_WIDTH(DATA_WIDTH), .USER_WIDTH(UW + 2), .ADDR_WIDTH(ADDR_WIDTH),
        .ALMOST_FULL_THRESH(ALMOST_FULL_THRESH),
        .ALMOST_EMPTY_THRESH(ALMOST_EMPTY_THRESH),
        .INIT_FILE(INIT_FILE)
      ) u_base (
        .i_clk_w(i_clk_w), .i_rstn_w(i_rstn_w),
        .i_wr_en(i_wr_en), .i_wr_data(i_wr_data), .i_wr_user(base_wr_user),
        .o_full(o_full), .o_almost_full(o_almost_full),
        .i_clk_r(i_clk_r), .i_rstn_r(i_rstn_r),
        .i_rd_en(i_rd_en), .o_rd_data(base_rd_data), .o_rd_user(base_rd_user),
        .o_empty(o_empty), .o_almost_empty(o_almost_empty), .o_level(o_level)
      );
    end else begin : gen_sync
      fifo_sync #(
        .DATA_WIDTH(DATA_WIDTH), .USER_WIDTH(UW + 2), .ADDR_WIDTH(ADDR_WIDTH),
        .ALMOST_FULL_THRESH(ALMOST_FULL_THRESH),
        .ALMOST_EMPTY_THRESH(ALMOST_EMPTY_THRESH),
        .INIT_FILE(INIT_FILE)
      ) u_base (
        .i_clk(i_clk_w), .i_rstn(i_rstn_w),
        .i_wr_en(i_wr_en), .i_wr_data(i_wr_data), .i_wr_user(base_wr_user),
        .o_full(o_full), .o_almost_full(o_almost_full),
        .i_rd_en(i_rd_en), .o_rd_data(base_rd_data), .o_rd_user(base_rd_user),
        .o_empty(o_empty), .o_almost_empty(o_almost_empty), .o_level(o_level)
      );
    end
  endgenerate

endmodule
