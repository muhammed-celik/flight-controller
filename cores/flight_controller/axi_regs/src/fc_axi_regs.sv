module fc_axi_regs #(
  parameter int unsigned ADDR_WIDTH = 16,
  parameter logic [31:0] IP_ID      = 32'h4643_0001,
  parameter logic [31:0] VERSION    = 32'h0001_0000
) (
  input  logic                  aclk,
  input  logic                  aresetn,

  input  logic [ADDR_WIDTH-1:0] s_axi_awaddr,
  input  logic [2:0]            s_axi_awprot,
  input  logic                  s_axi_awvalid,
  output logic                  s_axi_awready,
  input  logic [31:0]           s_axi_wdata,
  input  logic [3:0]            s_axi_wstrb,
  input  logic                  s_axi_wvalid,
  output logic                  s_axi_wready,
  output logic [1:0]            s_axi_bresp,
  output logic                  s_axi_bvalid,
  input  logic                  s_axi_bready,

  input  logic [ADDR_WIDTH-1:0] s_axi_araddr,
  input  logic [2:0]            s_axi_arprot,
  input  logic                  s_axi_arvalid,
  output logic                  s_axi_arready,
  output logic [31:0]           s_axi_rdata,
  output logic [1:0]            s_axi_rresp,
  output logic                  s_axi_rvalid,
  input  logic                  s_axi_rready,

  input  logic [63:0]           time_cycles,
  input  logic [31:0]           status_in,
  input  logic [31:0]           irq_sources,
  output logic                  irq,
  output logic [63:0]           active_config,
  output logic                  config_commit
);

  localparam logic [ADDR_WIDTH-1:0] ADDR_ID             = 'h0000;
  localparam logic [ADDR_WIDTH-1:0] ADDR_VERSION        = 'h0004;
  localparam logic [ADDR_WIDTH-1:0] ADDR_CAPABILITIES   = 'h0008;
  localparam logic [ADDR_WIDTH-1:0] ADDR_SCRATCH        = 'h000C;
  localparam logic [ADDR_WIDTH-1:0] ADDR_STATUS         = 'h0010;
  localparam logic [ADDR_WIDTH-1:0] ADDR_IRQ_STATUS     = 'h0014;
  localparam logic [ADDR_WIDTH-1:0] ADDR_IRQ_ENABLE     = 'h0018;
  localparam logic [ADDR_WIDTH-1:0] ADDR_IRQ_CLEAR      = 'h001C;
  localparam logic [ADDR_WIDTH-1:0] ADDR_TIME_CAPTURE   = 'h0020;
  localparam logic [ADDR_WIDTH-1:0] ADDR_TIME_LO        = 'h0024;
  localparam logic [ADDR_WIDTH-1:0] ADDR_TIME_HI        = 'h0028;
  localparam logic [ADDR_WIDTH-1:0] ADDR_TIME_SEQUENCE  = 'h002C;
  localparam logic [ADDR_WIDTH-1:0] ADDR_CFG_SHADOW_LO  = 'h0030;
  localparam logic [ADDR_WIDTH-1:0] ADDR_CFG_SHADOW_HI  = 'h0034;
  localparam logic [ADDR_WIDTH-1:0] ADDR_CFG_COMMIT     = 'h0038;
  localparam logic [ADDR_WIDTH-1:0] ADDR_CFG_ACTIVE_LO  = 'h003C;
  localparam logic [ADDR_WIDTH-1:0] ADDR_CFG_ACTIVE_HI  = 'h0040;
  localparam logic [ADDR_WIDTH-1:0] ADDR_CFG_SEQUENCE   = 'h0044;

  logic                  write_valid;
  logic [ADDR_WIDTH-1:0] write_addr;
  logic [31:0]           write_data;
  logic [3:0]            write_strb;
  logic [1:0]            write_resp;
  logic                  read_valid;
  logic [ADDR_WIDTH-1:0] read_addr;
  logic [31:0]           read_data;
  logic [1:0]            read_resp;

  logic [31:0] scratch;
  logic [31:0] irq_pending;
  logic [31:0] irq_enable;
  logic [31:0] irq_clear_mask;
  logic        time_capture;
  logic [63:0] time_snapshot;
  logic [31:0] time_sequence;
  logic        config_shadow_write;
  logic        config_shadow_word;
  logic        config_commit_request;
  logic [63:0] config_shadow;
  logic [31:0] config_sequence;

  function automatic logic [31:0] apply_wstrb(
    input logic [31:0] current,
    input logic [31:0] update,
    input logic [3:0]  strb
  );
    logic [31:0] result;
    result = current;
    for (int unsigned byte_index = 0; byte_index < 4; byte_index++) begin
      if (strb[byte_index]) begin
        result[byte_index*8 +: 8] = update[byte_index*8 +: 8];
      end
    end
    return result;
  endfunction

  axi_lite_slave #(
    .ADDR_WIDTH(ADDR_WIDTH)
  ) u_axi_slave (
    .aclk,
    .aresetn,
    .s_axi_awaddr,
    .s_axi_awprot,
    .s_axi_awvalid,
    .s_axi_awready,
    .s_axi_wdata,
    .s_axi_wstrb,
    .s_axi_wvalid,
    .s_axi_wready,
    .s_axi_bresp,
    .s_axi_bvalid,
    .s_axi_bready,
    .s_axi_araddr,
    .s_axi_arprot,
    .s_axi_arvalid,
    .s_axi_arready,
    .s_axi_rdata,
    .s_axi_rresp,
    .s_axi_rvalid,
    .s_axi_rready,
    .write_valid,
    .write_addr,
    .write_data,
    .write_strb,
    .write_resp,
    .read_valid,
    .read_addr,
    .read_data,
    .read_resp
  );

  always_comb begin
    write_resp = axi_pkg::AXI_RESP_SLVERR;
    if (write_addr[1:0] == 2'b00) begin
      case (write_addr)
        ADDR_SCRATCH,
        ADDR_IRQ_ENABLE,
        ADDR_IRQ_CLEAR,
        ADDR_TIME_CAPTURE,
        ADDR_CFG_SHADOW_LO,
        ADDR_CFG_SHADOW_HI,
        ADDR_CFG_COMMIT: write_resp = axi_pkg::AXI_RESP_OKAY;
        default: write_resp = axi_pkg::AXI_RESP_SLVERR;
      endcase
    end
  end

  always_comb begin
    read_data = '0;
    read_resp = axi_pkg::AXI_RESP_SLVERR;
    if (read_addr[1:0] == 2'b00) begin
      case (read_addr)
        ADDR_ID: begin
          read_data = IP_ID;
          read_resp = axi_pkg::AXI_RESP_OKAY;
        end
        ADDR_VERSION: begin
          read_data = VERSION;
          read_resp = axi_pkg::AXI_RESP_OKAY;
        end
        ADDR_CAPABILITIES: begin
          read_data = 32'h0000_0003;
          read_resp = axi_pkg::AXI_RESP_OKAY;
        end
        ADDR_SCRATCH: begin
          read_data = scratch;
          read_resp = axi_pkg::AXI_RESP_OKAY;
        end
        ADDR_STATUS: begin
          read_data = status_in;
          read_resp = axi_pkg::AXI_RESP_OKAY;
        end
        ADDR_IRQ_STATUS: begin
          read_data = irq_pending;
          read_resp = axi_pkg::AXI_RESP_OKAY;
        end
        ADDR_IRQ_ENABLE: begin
          read_data = irq_enable;
          read_resp = axi_pkg::AXI_RESP_OKAY;
        end
        ADDR_TIME_LO: begin
          read_data = time_snapshot[31:0];
          read_resp = axi_pkg::AXI_RESP_OKAY;
        end
        ADDR_TIME_HI: begin
          read_data = time_snapshot[63:32];
          read_resp = axi_pkg::AXI_RESP_OKAY;
        end
        ADDR_TIME_SEQUENCE: begin
          read_data = time_sequence;
          read_resp = axi_pkg::AXI_RESP_OKAY;
        end
        ADDR_CFG_SHADOW_LO: begin
          read_data = config_shadow[31:0];
          read_resp = axi_pkg::AXI_RESP_OKAY;
        end
        ADDR_CFG_SHADOW_HI: begin
          read_data = config_shadow[63:32];
          read_resp = axi_pkg::AXI_RESP_OKAY;
        end
        ADDR_CFG_ACTIVE_LO: begin
          read_data = active_config[31:0];
          read_resp = axi_pkg::AXI_RESP_OKAY;
        end
        ADDR_CFG_ACTIVE_HI: begin
          read_data = active_config[63:32];
          read_resp = axi_pkg::AXI_RESP_OKAY;
        end
        ADDR_CFG_SEQUENCE: begin
          read_data = config_sequence;
          read_resp = axi_pkg::AXI_RESP_OKAY;
        end
        default: begin
          read_data = '0;
          read_resp = axi_pkg::AXI_RESP_SLVERR;
        end
      endcase
    end
  end

  assign irq = |(irq_pending & irq_enable);

  always_comb begin
    irq_clear_mask = '0;
    if (write_valid && write_resp == axi_pkg::AXI_RESP_OKAY &&
        write_addr == ADDR_IRQ_CLEAR) begin
      irq_clear_mask = apply_wstrb('0, write_data, write_strb);
    end
  end

  always_ff @(posedge aclk) begin
    if (!aresetn) begin
      scratch     <= '0;
      irq_pending <= '0;
      irq_enable  <= '0;
    end else begin
      irq_pending <= (irq_pending & ~irq_clear_mask) | irq_sources;
      if (write_valid && write_resp == axi_pkg::AXI_RESP_OKAY) begin
        case (write_addr)
          ADDR_SCRATCH:
            scratch <= apply_wstrb(scratch, write_data, write_strb);
          ADDR_IRQ_ENABLE:
            irq_enable <= apply_wstrb(irq_enable, write_data, write_strb);
          default: begin
          end
        endcase
      end
    end
  end

  assign time_capture = write_valid &&
                        write_resp == axi_pkg::AXI_RESP_OKAY &&
                        write_addr == ADDR_TIME_CAPTURE &&
                        write_strb[0] && write_data[0];

  coherent_snapshot #(
    .WIDTH(64)
  ) u_time_snapshot (
    .clk          (aclk),
    .rst_n        (aresetn),
    .capture      (time_capture),
    .live_data    (time_cycles),
    .snapshot_data(time_snapshot),
    .snapshot_sequence(time_sequence)
  );

  assign config_shadow_write = write_valid &&
                               write_resp == axi_pkg::AXI_RESP_OKAY &&
                               (write_addr == ADDR_CFG_SHADOW_LO ||
                                write_addr == ADDR_CFG_SHADOW_HI);
  assign config_shadow_word = write_addr == ADDR_CFG_SHADOW_HI;
  assign config_commit_request = write_valid &&
                                 write_resp == axi_pkg::AXI_RESP_OKAY &&
                                 write_addr == ADDR_CFG_COMMIT &&
                                 write_strb[0] && write_data[0];

  shadow_commit #(
    .WIDTH(64)
  ) u_config_commit (
    .clk          (aclk),
    .rst_n        (aresetn),
    .shadow_write (config_shadow_write),
    .shadow_word  (config_shadow_word),
    .shadow_wdata (write_data),
    .shadow_wstrb (write_strb),
    .commit       (config_commit_request),
    .shadow_data  (config_shadow),
    .active_data  (active_config),
    .commit_sequence(config_sequence),
    .commit_pulse (config_commit)
  );

  logic unused_read_valid;
  assign unused_read_valid = read_valid;

endmodule
