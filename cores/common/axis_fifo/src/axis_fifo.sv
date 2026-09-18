//-----------------------------------------------------------------------------
// axis_fifo.sv
// AXI4-Stream FIFO built on the FWFT base FIFOs.
//
// A single base FIFO is instantiated, selected by CDC:
//   CDC = 0 -> fifo_sync  (single clock, i_clk_w)
//   CDC = 1 -> fifo_async (i_clk_w / i_clk_r)
//
// tlast is stored as a sideband bit alongside every beat and regenerated on the
// master side, so packet boundaries are preserved. tuser is carried through
// unchanged. Backpressure is provided by tready; the slave side is always ready
// while the FIFO is not full, and the master side asserts tvalid while the FIFO
// is not empty (first-word fall-through).
//
// Parameters:
//   TDATA_WIDTH : AXI-Stream data width
//   TUSER_WIDTH : AXI-Stream user width (0 allowed)
//   ADDR_WIDTH  : base FIFO depth = 2**ADDR_WIDTH
//   CDC         : 0 = single clock, 1 = independent write/read clocks
//   INIT_FILE   : optional $readmemh file for the base RAM
//-----------------------------------------------------------------------------
module axis_fifo #(
  parameter int unsigned TDATA_WIDTH = 32,
  parameter int unsigned TUSER_WIDTH = 0,
  parameter int unsigned ADDR_WIDTH  = 4,
  parameter bit          CDC         = 1'b0,
  parameter string       INIT_FILE   = ""
) (
  // Write clock domain
  input  logic                    i_clk_w,
  input  logic                    i_rstn_w,
  // Read clock domain
  input  logic                    i_clk_r,
  input  logic                    i_rstn_r,

  // AXI-Stream slave (into the FIFO)
  input  logic                    i_s_axis_tvalid,
  output logic                    o_s_axis_tready,
  input  logic [TDATA_WIDTH-1:0]  i_s_axis_tdata,
  input  logic [((TUSER_WIDTH < 1) ? 1 : TUSER_WIDTH)-1:0] i_s_axis_tuser,
  input  logic                    i_s_axis_tlast,

  // AXI-Stream master (out of the FIFO)
  output logic                    o_m_axis_tvalid,
  input  logic                    i_m_axis_tready,
  output logic [TDATA_WIDTH-1:0]  o_m_axis_tdata,
  output logic [((TUSER_WIDTH < 1) ? 1 : TUSER_WIDTH)-1:0] o_m_axis_tuser,
  output logic                    o_m_axis_tlast
);

  localparam int unsigned TUW = (TUSER_WIDTH < 1) ? 1 : TUSER_WIDTH;

  logic [TDATA_WIDTH-1:0] base_rd_data;
  logic [TUW:0]           base_rd_user;
  logic [TUW:0]           base_wr_user;
  logic                   base_full;
  logic                   base_empty;
  logic                   wr_fire;
  logic                   rd_fire;

  assign base_wr_user = {i_s_axis_tlast, i_s_axis_tuser};
  assign {o_m_axis_tlast, o_m_axis_tuser} = base_rd_user;
  assign o_m_axis_tdata = base_rd_data;

  assign o_s_axis_tready = ~base_full;
  assign wr_fire         = i_s_axis_tvalid && o_s_axis_tready;
  assign o_m_axis_tvalid = ~base_empty;
  assign rd_fire         = o_m_axis_tvalid && i_m_axis_tready;

  generate
    if (CDC) begin : gen_async
      fifo_async #(
        .DATA_WIDTH(TDATA_WIDTH), .USER_WIDTH(TUW + 1), .ADDR_WIDTH(ADDR_WIDTH),
        .INIT_FILE(INIT_FILE)
      ) u_base (
        .i_clk_w(i_clk_w), .i_rstn_w(i_rstn_w),
        .i_wr_en(wr_fire), .i_wr_data(i_s_axis_tdata), .i_wr_user(base_wr_user),
        .o_full(base_full), .o_almost_full(),
        .i_clk_r(i_clk_r), .i_rstn_r(i_rstn_r),
        .i_rd_en(rd_fire), .o_rd_data(base_rd_data), .o_rd_user(base_rd_user),
        .o_empty(base_empty), .o_almost_empty(), .o_level()
      );
    end else begin : gen_sync
      fifo_sync #(
        .DATA_WIDTH(TDATA_WIDTH), .USER_WIDTH(TUW + 1), .ADDR_WIDTH(ADDR_WIDTH),
        .INIT_FILE(INIT_FILE)
      ) u_base (
        .i_clk(i_clk_w), .i_rstn(i_rstn_w),
        .i_wr_en(wr_fire), .i_wr_data(i_s_axis_tdata), .i_wr_user(base_wr_user),
        .o_full(base_full), .o_almost_full(),
        .i_rd_en(rd_fire), .o_rd_data(base_rd_data), .o_rd_user(base_rd_user),
        .o_empty(base_empty), .o_almost_empty(), .o_level()
      );
    end
  endgenerate

endmodule
