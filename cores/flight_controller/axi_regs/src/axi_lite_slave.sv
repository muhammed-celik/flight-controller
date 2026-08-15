module axi_lite_slave #(
  parameter int unsigned ADDR_WIDTH = 16
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

  output logic                  write_valid,
  output logic [ADDR_WIDTH-1:0] write_addr,
  output logic [31:0]           write_data,
  output logic [3:0]            write_strb,
  input  logic [1:0]            write_resp,

  output logic                  read_valid,
  output logic [ADDR_WIDTH-1:0] read_addr,
  input  logic [31:0]           read_data,
  input  logic [1:0]            read_resp
);

  logic                  aw_stored;
  logic [ADDR_WIDTH-1:0] awaddr_stored;
  logic                  w_stored;
  logic [31:0]           wdata_stored;
  logic [3:0]            wstrb_stored;
  logic                  unused_prot;

  assign s_axi_awready = !aw_stored && !s_axi_bvalid;
  assign s_axi_wready  = !w_stored && !s_axi_bvalid;
  assign write_valid   = aw_stored && w_stored && !s_axi_bvalid;
  assign write_addr    = awaddr_stored;
  assign write_data    = wdata_stored;
  assign write_strb    = wstrb_stored;

  assign s_axi_arready = !s_axi_rvalid;
  assign read_valid    = s_axi_arvalid && s_axi_arready;
  assign read_addr     = s_axi_araddr;

  assign unused_prot = ^{s_axi_awprot, s_axi_arprot};

  always_ff @(posedge aclk) begin
    if (!aresetn) begin
      aw_stored    <= 1'b0;
      awaddr_stored <= '0;
      w_stored     <= 1'b0;
      wdata_stored <= '0;
      wstrb_stored <= '0;
      s_axi_bresp  <= axi_pkg::AXI_RESP_OKAY;
      s_axi_bvalid <= 1'b0;
    end else begin
      if (s_axi_awvalid && s_axi_awready) begin
        aw_stored     <= 1'b1;
        awaddr_stored <= s_axi_awaddr;
      end
      if (s_axi_wvalid && s_axi_wready) begin
        w_stored     <= 1'b1;
        wdata_stored <= s_axi_wdata;
        wstrb_stored <= s_axi_wstrb;
      end

      if (write_valid) begin
        aw_stored    <= 1'b0;
        w_stored     <= 1'b0;
        s_axi_bresp  <= write_resp;
        s_axi_bvalid <= 1'b1;
      end else if (s_axi_bvalid && s_axi_bready) begin
        s_axi_bvalid <= 1'b0;
      end
    end
  end

  always_ff @(posedge aclk) begin
    if (!aresetn) begin
      s_axi_rdata  <= '0;
      s_axi_rresp  <= axi_pkg::AXI_RESP_OKAY;
      s_axi_rvalid <= 1'b0;
    end else begin
      if (read_valid) begin
        s_axi_rdata  <= read_data;
        s_axi_rresp  <= read_resp;
        s_axi_rvalid <= 1'b1;
      end else if (s_axi_rvalid && s_axi_rready) begin
        s_axi_rvalid <= 1'b0;
      end
    end
  end

endmodule
