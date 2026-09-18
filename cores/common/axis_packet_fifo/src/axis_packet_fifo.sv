//-----------------------------------------------------------------------------
// axis_packet_fifo.sv
// Store-and-forward AXI4-Stream packet FIFO.
//
// A packet (a run of beats terminated by tlast) becomes visible on the master
// side only once its tlast has been stored. The slave side is backpressured
// while the FIFO is full, so a packet is never partially stored; a packet must
// therefore fit inside the FIFO.
//
// A single base FIFO is instantiated, selected by CDC:
//   CDC = 0 -> fifo_sync  (single clock, i_clk_w)
//   CDC = 1 -> fifo_async (i_clk_w / i_clk_r)
//
// Packet completion is tracked by gray-coded counters synchronised into the
// read domain, so the store-and-forward gate also works across clock domains.
//
// Parameters:
//   TDATA_WIDTH / TUSER_WIDTH : AXI-Stream widths
//   ADDR_WIDTH : base FIFO depth = 2**ADDR_WIDTH
//   CDC        : 0 = single clock, 1 = independent write/read clocks
//   INIT_FILE  : optional $readmemh file for the base RAM
//-----------------------------------------------------------------------------
module axis_packet_fifo #(
  parameter int unsigned TDATA_WIDTH = 32,
  parameter int unsigned TUSER_WIDTH = 0,
  parameter int unsigned ADDR_WIDTH  = 4,
  parameter bit          CDC         = 1'b0,
  parameter string       INIT_FILE   = ""
) (
  input  logic                    i_clk_w,
  input  logic                    i_rstn_w,
  input  logic                    i_clk_r,
  input  logic                    i_rstn_r,

  input  logic                    i_s_axis_tvalid,
  output logic                    o_s_axis_tready,
  input  logic [TDATA_WIDTH-1:0]  i_s_axis_tdata,
  input  logic [((TUSER_WIDTH < 1) ? 1 : TUSER_WIDTH)-1:0] i_s_axis_tuser,
  input  logic                    i_s_axis_tlast,

  output logic                    o_m_axis_tvalid,
  input  logic                    i_m_axis_tready,
  output logic [TDATA_WIDTH-1:0]  o_m_axis_tdata,
  output logic [((TUSER_WIDTH < 1) ? 1 : TUSER_WIDTH)-1:0] o_m_axis_tuser,
  output logic                    o_m_axis_tlast
);

  localparam int unsigned TUW = (TUSER_WIDTH < 1) ? 1 : TUSER_WIDTH;
  localparam int unsigned PKW = ADDR_WIDTH + 1;

  function automatic [PKW-1:0] bin2gray(input [PKW-1:0] bin);
    bin2gray = bin ^ (bin >> 1);
  endfunction

  function automatic [PKW-1:0] gray2bin(input [PKW-1:0] gray);
    gray2bin[PKW-1] = gray[PKW-1];
    for (int i = PKW - 2; i >= 0; i--) begin
      gray2bin[i] = gray2bin[i+1] ^ gray[i];
    end
  endfunction

  wire clk_r_int = CDC ? i_clk_r : i_clk_w;
  wire rst_r_int = CDC ? i_rstn_r : i_rstn_w;

  logic [TDATA_WIDTH-1:0] base_rd_data;
  logic [TUW:0]           base_rd_user;
  logic [TUW:0]           base_wr_user;
  logic                   base_full;
  logic                   base_empty;
  logic                   wr_fire;
  logic                   rd_fire;
  logic                   packets_avail;

  logic [PKW-1:0] wr_pkts, rd_pkts;
  logic [PKW-1:0] wr_pkts_gray, wr_pkts_meta, wr_pkts_sync, wr_pkts_r;

  assign base_wr_user = {i_s_axis_tlast, i_s_axis_tuser};
  assign {o_m_axis_tlast, o_m_axis_tuser} = base_rd_user;
  assign o_m_axis_tdata = base_rd_data;

  assign o_s_axis_tready = ~base_full;
  assign wr_fire         = i_s_axis_tvalid && o_s_axis_tready;
  assign packets_avail   = (wr_pkts_r != rd_pkts);
  assign o_m_axis_tvalid = ~base_empty && packets_avail;
  assign rd_fire         = o_m_axis_tvalid && i_m_axis_tready;

  // Write-domain packet counter (one per stored tlast)
  always_ff @(posedge i_clk_w or negedge i_rstn_w) begin
    if (!i_rstn_w) begin
      wr_pkts      <= '0;
      wr_pkts_gray <= '0;
    end else begin
      if (wr_fire && i_s_axis_tlast) begin
        wr_pkts <= wr_pkts + 1'b1;
      end
      wr_pkts_gray <= bin2gray(wr_pkts);
    end
  end

  // Read-domain packet counter plus write-counter synchroniser
  always_ff @(posedge clk_r_int or negedge rst_r_int) begin
    if (!rst_r_int) begin
      rd_pkts      <= '0;
      wr_pkts_meta <= '0;
      wr_pkts_sync <= '0;
    end else begin
      if (rd_fire && o_m_axis_tlast) begin
        rd_pkts <= rd_pkts + 1'b1;
      end
      wr_pkts_meta <= wr_pkts_gray;
      wr_pkts_sync <= wr_pkts_meta;
    end
  end

  assign wr_pkts_r = CDC ? gray2bin(wr_pkts_sync) : wr_pkts;

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
