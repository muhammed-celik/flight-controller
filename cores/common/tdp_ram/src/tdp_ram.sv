//-----------------------------------------------------------------------------
// tdp_ram.sv
// Parametric true dual-port RAM: two fully independent read/write ports.
//
// Each port has its own clock, enable, write-enable, address and data. A port
// performs a write when i_en_* and i_we_* are both high, otherwise it performs
// a read when i_en_* is high. Reads are registered, giving a read latency of
// one clock cycle (plus one more when REG_OUT is set).
//
// RDW_MODE selects the behaviour of a port's registered output while that same
// port is writing:
//   0 = READ_FIRST  -> old memory content
//   1 = WRITE_FIRST -> written data bypassed to o_rdata_*
//   2 = NO_CHANGE   -> output holds its previous value
//
// Simultaneous access to the same address from both ports (write/write or
// write/read) is undefined; use external arbitration if that can occur.
//
// Parameters:
//   DATA_WIDTH : word width in bits (must be a multiple of 8)
//   ADDR_WIDTH : number of address bits, depth = 2**ADDR_WIDTH
//   BYTE_EN    : enable byte-granular writes using i_be_a / i_be_b
//   REG_OUT    : add an extra output register stage on both ports
//   RDW_MODE   : read-during-write behaviour (see above)
//   INIT_FILE  : optional $readmemh file used to initialise the memory
//-----------------------------------------------------------------------------
module tdp_ram #(
  parameter int unsigned DATA_WIDTH = 32,
  parameter int unsigned ADDR_WIDTH = 10,
  parameter bit          BYTE_EN    = 1'b0,
  parameter bit          REG_OUT    = 1'b0,
  parameter int unsigned RDW_MODE   = 0,
  parameter string       INIT_FILE  = ""
) (
  // Port A
  input  logic                    i_clk_a,
  input  logic                    i_en_a,
  input  logic                    i_we_a,
  input  logic [ADDR_WIDTH-1:0]   i_addr_a,
  input  logic [DATA_WIDTH-1:0]   i_wdata_a,
  input  logic [DATA_WIDTH/8-1:0] i_be_a,
  output logic [DATA_WIDTH-1:0]   o_rdata_a,
  // Port B
  input  logic                    i_clk_b,
  input  logic                    i_en_b,
  input  logic                    i_we_b,
  input  logic [ADDR_WIDTH-1:0]   i_addr_b,
  input  logic [DATA_WIDTH-1:0]   i_wdata_b,
  input  logic [DATA_WIDTH/8-1:0] i_be_b,
  output logic [DATA_WIDTH-1:0]   o_rdata_b
);

  localparam int unsigned DEPTH = 2 ** ADDR_WIDTH;
  localparam int unsigned NB    = DATA_WIDTH / 8;

  localparam int unsigned RDW_READ_FIRST  = 0;
  localparam int unsigned RDW_WRITE_FIRST = 1;
  localparam int unsigned RDW_NO_CHANGE   = 2;

  // Both ports write the same memory from independent clocks; Verilator reports
  // this as MULTIDRIVEN even though it is the intended dual-port behaviour.
  /* verilator lint_off MULTIDRIVEN */
  (* ram_style = "block" *) logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];

  initial begin
    if (INIT_FILE != "") begin
      $readmemh(INIT_FILE, mem);
    end
  end

  logic [DATA_WIDTH-1:0] rdata_a_q;
  logic [DATA_WIDTH-1:0] rdata_b_q;

  // Port A
  always_ff @(posedge i_clk_a) begin
    if (i_en_a) begin
      if (i_we_a) begin
        if (BYTE_EN) begin
          for (int unsigned b = 0; b < NB; b++) begin
            if (i_be_a[b]) begin
              mem[i_addr_a][b*8 +: 8] <= i_wdata_a[b*8 +: 8];
            end
          end
        end else begin
          mem[i_addr_a] <= i_wdata_a;
        end

        case (RDW_MODE)
          RDW_WRITE_FIRST: rdata_a_q <= i_wdata_a;
          RDW_READ_FIRST:  rdata_a_q <= mem[i_addr_a];
          RDW_NO_CHANGE:   ; // hold o_rdata_a
          default:         ; // hold o_rdata_a
        endcase
      end else begin
        rdata_a_q <= mem[i_addr_a];
      end
    end
  end

  // Port B
  always_ff @(posedge i_clk_b) begin
    if (i_en_b) begin
      if (i_we_b) begin
        if (BYTE_EN) begin
          for (int unsigned b = 0; b < NB; b++) begin
            if (i_be_b[b]) begin
              mem[i_addr_b][b*8 +: 8] <= i_wdata_b[b*8 +: 8];
            end
          end
        end else begin
          mem[i_addr_b] <= i_wdata_b;
        end

        case (RDW_MODE)
          RDW_WRITE_FIRST: rdata_b_q <= i_wdata_b;
          RDW_READ_FIRST:  rdata_b_q <= mem[i_addr_b];
          RDW_NO_CHANGE:   ; // hold o_rdata_b
          default:         ; // hold o_rdata_b
        endcase
      end else begin
        rdata_b_q <= mem[i_addr_b];
      end
    end
  end
  /* verilator lint_on MULTIDRIVEN */

  generate
    if (REG_OUT) begin : gen_reg_out
      always_ff @(posedge i_clk_a) o_rdata_a <= rdata_a_q;
      always_ff @(posedge i_clk_b) o_rdata_b <= rdata_b_q;
    end else begin : gen_no_reg_out
      assign o_rdata_a = rdata_a_q;
      assign o_rdata_b = rdata_b_q;
    end
  endgenerate

endmodule
