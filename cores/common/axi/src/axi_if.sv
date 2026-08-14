interface axi_if #(
  parameter int ADDR_W = 32,
  parameter int DATA_W = 32,
  parameter int ID_W   = 4,
  parameter int USER_W = 1
);

  localparam int STRB_W = DATA_W / 8;

  // Write address channel
  logic [ID_W-1:0]    awid;
  logic [ADDR_W-1:0]  awaddr;
  logic [7:0]         awlen;
  logic [2:0]         awsize;
  logic [1:0]         awburst;
  logic               awlock;
  logic [3:0]         awcache;
  logic [2:0]         awprot;
  logic [3:0]         awqos;
  logic [USER_W-1:0]  awuser;
  logic               awvalid;
  logic               awready;

  // Write data channel
  logic [DATA_W-1:0]  wdata;
  logic [STRB_W-1:0]  wstrb;
  logic               wlast;
  logic [USER_W-1:0]  wuser;
  logic               wvalid;
  logic               wready;

  // Write response channel
  logic [ID_W-1:0]    bid;
  logic [1:0]         bresp;
  logic [USER_W-1:0]  buser;
  logic               bvalid;
  logic               bready;

  // Read address channel
  logic [ID_W-1:0]    arid;
  logic [ADDR_W-1:0]  araddr;
  logic [7:0]         arlen;
  logic [2:0]         arsize;
  logic [1:0]         arburst;
  logic               arlock;
  logic [3:0]         arcache;
  logic [2:0]         arprot;
  logic [3:0]         arqos;
  logic [USER_W-1:0]  aruser;
  logic               arvalid;
  logic               arready;

  // Read data channel
  logic [ID_W-1:0]    rid;
  logic [DATA_W-1:0]  rdata;
  logic [1:0]         rresp;
  logic               rlast;
  logic [USER_W-1:0]  ruser;
  logic               rvalid;
  logic               rready;

  modport master (
    output awid, awaddr, awlen, awsize, awburst, awlock, awcache, awprot, awqos, awuser, awvalid,
    input  awready,
    output wdata, wstrb, wlast, wuser, wvalid,
    input  wready,
    input  bid, bresp, buser, bvalid,
    output bready,
    output arid, araddr, arlen, arsize, arburst, arlock, arcache, arprot, arqos, aruser, arvalid,
    input  arready,
    input  rid, rdata, rresp, rlast, ruser, rvalid,
    output rready
  );

  modport slave (
    input  awid, awaddr, awlen, awsize, awburst, awlock, awcache, awprot, awqos, awuser, awvalid,
    output awready,
    input  wdata, wstrb, wlast, wuser, wvalid,
    output wready,
    output bid, bresp, buser, bvalid,
    input  bready,
    input  arid, araddr, arlen, arsize, arburst, arlock, arcache, arprot, arqos, aruser, arvalid,
    output arready,
    output rid, rdata, rresp, rlast, ruser, rvalid,
    input  rready
  );

endinterface
