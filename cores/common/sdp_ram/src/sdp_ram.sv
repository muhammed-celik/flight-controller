//-----------------------------------------------------------------------------
// sdp_ram.sv
// Parametric simple dual-port RAM: one write-only port and one read-only port.
//
// The ports have independent clocks so the RAM can be used as a synchronous
// clock-domain crossing buffer (tie i_clk_w and i_clk_r together for a
// single-clock design). The read data is registered, giving a read latency of
// one clock cycle (plus one more when REG_OUT is set).
//
// When the two clocks are identical and the write and read addresses match in
// the same cycle, the read port returns the old memory content (READ_FIRST).
// Reads and writes at the same location from unrelated clocks are undefined.
//
// Parameters:
//   DATA_WIDTH : word width in bits (must be a multiple of 8)
//   ADDR_WIDTH : number of address bits, depth = 2**ADDR_WIDTH
//   BYTE_EN    : enable byte-granular writes using i_be
//   REG_OUT    : add an extra output register stage
//   INIT_FILE  : optional $readmemh file used to initialise the memory
//-----------------------------------------------------------------------------
module sdp_ram #(
  parameter int unsigned DATA_WIDTH = 32,
  parameter int unsigned ADDR_WIDTH = 10,
  parameter bit          BYTE_EN    = 1'b0,
  parameter bit          REG_OUT    = 1'b0,
  parameter string       INIT_FILE  = ""
) (
  // Write port
  input  logic                    i_clk_w,
  input  logic                    i_en_w,
  input  logic                    i_we_w,
  input  logic [ADDR_WIDTH-1:0]   i_addr_w,
  input  logic [DATA_WIDTH-1:0]   i_wdata,
  input  logic [DATA_WIDTH/8-1:0] i_be,
  // Read port
  input  logic                    i_clk_r,
  input  logic                    i_en_r,
  input  logic [ADDR_WIDTH-1:0]   i_addr_r,
  output logic [DATA_WIDTH-1:0]   o_rdata
);

  localparam int unsigned DEPTH = 2 ** ADDR_WIDTH;
  localparam int unsigned NB    = DATA_WIDTH / 8;

  (* ram_style = "block" *) logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];

  initial begin
    if (INIT_FILE != "") begin
      $readmemh(INIT_FILE, mem);
    end
  end

  always_ff @(posedge i_clk_w) begin
    if (i_en_w && i_we_w) begin
      if (BYTE_EN) begin
        for (int unsigned b = 0; b < NB; b++) begin
          if (i_be[b]) begin
            mem[i_addr_w][b*8 +: 8] <= i_wdata[b*8 +: 8];
          end
        end
      end else begin
        mem[i_addr_w] <= i_wdata;
      end
    end
  end

  logic [DATA_WIDTH-1:0] rdata_q;

  always_ff @(posedge i_clk_r) begin
    if (i_en_r) begin
      rdata_q <= mem[i_addr_r];
    end
  end

  generate
    if (REG_OUT) begin : gen_reg_out
      always_ff @(posedge i_clk_r) begin
        o_rdata <= rdata_q;
      end
    end else begin : gen_no_reg_out
      assign o_rdata = rdata_q;
    end
  endgenerate

endmodule
