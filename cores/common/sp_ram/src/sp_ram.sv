//-----------------------------------------------------------------------------
// sp_ram.sv
// Parametric single-port RAM with a synchronous read/write interface.
//
// One combined read/write port. The read data is registered, so the read
// latency is one clock cycle (plus one more when REG_OUT is set).
//
// Parameters:
//   DATA_WIDTH : word width in bits (must be a multiple of 8)
//   ADDR_WIDTH : number of address bits, depth = 2**ADDR_WIDTH
//   BYTE_EN    : enable byte-granular writes using i_be
//   REG_OUT    : add an extra output register stage
//   RDW_MODE   : read-during-write behaviour while i_we is asserted
//                  0 = READ_FIRST  -> old memory content
//                  1 = WRITE_FIRST -> written data bypassed to o_rdata
//                  2 = NO_CHANGE   -> output holds its previous value
//   INIT_FILE  : optional $readmemh file used to initialise the memory
//-----------------------------------------------------------------------------
module sp_ram #(
  parameter int unsigned DATA_WIDTH = 32,
  parameter int unsigned ADDR_WIDTH = 10,
  parameter bit          BYTE_EN    = 1'b0,
  parameter bit          REG_OUT    = 1'b0,
  parameter int unsigned RDW_MODE   = 0,
  parameter string       INIT_FILE  = ""
) (
  input  logic                    i_clk,
  input  logic                    i_en,
  input  logic                    i_we,
  input  logic [ADDR_WIDTH-1:0]   i_addr,
  input  logic [DATA_WIDTH-1:0]   i_wdata,
  input  logic [DATA_WIDTH/8-1:0] i_be,
  output logic [DATA_WIDTH-1:0]   o_rdata
);

  localparam int unsigned DEPTH = 2 ** ADDR_WIDTH;
  localparam int unsigned NB    = DATA_WIDTH / 8;

  localparam int unsigned RDW_READ_FIRST  = 0;
  localparam int unsigned RDW_WRITE_FIRST = 1;
  localparam int unsigned RDW_NO_CHANGE   = 2;

  (* ram_style = "block" *) logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];

  initial begin
    if (INIT_FILE != "") begin
      $readmemh(INIT_FILE, mem);
    end
  end

  logic [DATA_WIDTH-1:0] rdata_q;

  always_ff @(posedge i_clk) begin
    if (i_en) begin
      if (i_we) begin
        if (BYTE_EN) begin
          for (int unsigned b = 0; b < NB; b++) begin
            if (i_be[b]) begin
              mem[i_addr][b*8 +: 8] <= i_wdata[b*8 +: 8];
            end
          end
        end else begin
          mem[i_addr] <= i_wdata;
        end

        case (RDW_MODE)
          RDW_WRITE_FIRST: rdata_q <= i_wdata;
          RDW_READ_FIRST:  rdata_q <= mem[i_addr];
          RDW_NO_CHANGE:   ; // hold o_rdata
          default:         ; // hold o_rdata
        endcase
      end else begin
        rdata_q <= mem[i_addr];
      end
    end
  end

  generate
    if (REG_OUT) begin : gen_reg_out
      always_ff @(posedge i_clk) begin
        o_rdata <= rdata_q;
      end
    end else begin : gen_no_reg_out
      assign o_rdata = rdata_q;
    end
  endgenerate

endmodule
