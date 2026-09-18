//-----------------------------------------------------------------------------
// fifo_adapter.sv  (documentation example - not part of a fusesoc core)
//
// Shows the two canonical ways an upper module drives the native FIFO
// interface used by fifo_sync / fifo_async:
//
//   1. fifo_adapter      : bridge a simple valid/ready stream to the FIFO.
//   2. fifo_producer_fsm : an explicit FSM that respects o_full.
//   3. fifo_consumer_fsm : an explicit FSM that uses first-word fall-through.
//
// The golden rules are:
//   WRITE : only assert i_wr_en on a cycle where o_full is low.
//   READ  : o_rd_data is already valid whenever o_empty is low (FWFT).
//           Assert i_rd_en for one cycle to pop that word.
//-----------------------------------------------------------------------------
`default_nettype none

//------------------------------------------------------------------------------
// 1. valid/ready <-> FIFO bridge (same clock)
//------------------------------------------------------------------------------
module fifo_adapter #(
  parameter int unsigned DATA_WIDTH = 32,
  parameter int unsigned ADDR_WIDTH = 4
) (
  input  logic                  i_clk,
  input  logic                  i_rstn,

  // Upstream (fills the FIFO)
  input  logic                  i_s_valid,
  output logic                  o_s_ready,
  input  logic [DATA_WIDTH-1:0] i_s_data,

  // Downstream (drains the FIFO)
  output logic                  o_m_valid,
  input  logic                  i_m_ready,
  output logic [DATA_WIDTH-1:0] o_m_data
);

  logic                  full;
  logic                  empty;
  logic [DATA_WIDTH-1:0] rd_data;

  fifo_sync #(
    .DATA_WIDTH(DATA_WIDTH),
    .USER_WIDTH(1),
    .ADDR_WIDTH(ADDR_WIDTH)
  ) u_fifo (
    .i_clk        (i_clk),
    .i_rstn       (i_rstn),
    .i_wr_en      (i_s_valid & ~full),
    .i_wr_data    (i_s_data),
    .i_wr_user    (1'b0),
    .o_full       (full),
    .o_almost_full(),
    .i_rd_en      (o_m_valid & i_m_ready),
    .o_rd_data    (rd_data),
    .o_rd_user    (),
    .o_empty      (empty),
    .o_almost_empty(),
    .o_level      ()
  );

  assign o_s_ready = ~full;
  // FWFT: the head word is presented without asking, so o_m_valid is just
  // "not empty" and o_m_data is valid in the same cycle.
  assign o_m_valid = ~empty;
  assign o_m_data  = rd_data;

endmodule

//------------------------------------------------------------------------------
// 2. Explicit producer FSM. Never write while full.
//------------------------------------------------------------------------------
module fifo_producer_fsm #(
  parameter int unsigned DATA_WIDTH = 32,
  parameter int unsigned N_WORDS    = 8
) (
  input  logic                  i_clk,
  input  logic                  i_rstn,
  input  logic                  i_start,
  output logic                  o_busy,
  // FIFO write port
  output logic                  o_wr_en,
  output logic [DATA_WIDTH-1:0] o_wr_data,
  input  logic                  i_full
);

  typedef enum logic [1:0] { IDLE, SEND, DONE } state_t;
  state_t                 state;
  logic [DATA_WIDTH-1:0]  counter;

  always_ff @(posedge i_clk or negedge i_rstn) begin
    if (!i_rstn) begin
      state     <= IDLE;
      counter   <= '0;
      o_wr_en   <= 1'b0;
      o_wr_data <= '0;
    end else begin
      o_wr_en <= 1'b0;                     // one-cycle strobe by default

      case (state)
        IDLE: if (i_start) begin
          counter <= '0;
          state   <= SEND;
        end

        SEND: if (!i_full) begin           // handshake: space available
          o_wr_en   <= 1'b1;
          o_wr_data <= counter;
          if (counter == N_WORDS - 1) begin
            state <= DONE;
          end else begin
            counter <= counter + 1'b1;
          end
        end

        DONE: state <= IDLE;
        default: state <= IDLE;
      endcase
    end
  end

  assign o_busy = (state != IDLE);

endmodule

//------------------------------------------------------------------------------
// 3. Explicit consumer FSM using first-word fall-through.
//    o_rd_data is valid *now*; i_rd_en pops it for the next cycle.
//    i_ready is the downstream "I consumed it" flag.
//------------------------------------------------------------------------------
module fifo_consumer_fsm #(
  parameter int unsigned DATA_WIDTH = 32
) (
  input  logic                  i_clk,
  input  logic                  i_rstn,
  input  logic                  i_ready,
  // FIFO read port
  input  logic                  i_empty,
  input  logic [DATA_WIDTH-1:0] i_rd_data,
  output logic                  o_rd_en,
  output logic                  o_valid,
  output logic [DATA_WIDTH-1:0] o_data
);

  // FWFT means we can present data immediately when the FIFO is not empty.
  assign o_valid = ~i_empty;
  assign o_data  = i_rd_data;
  // Pop exactly when the downstream accepts the word.
  assign o_rd_en = o_valid & i_ready;

endmodule

`default_nettype wire
