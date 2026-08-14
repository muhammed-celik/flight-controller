package axi_pkg;

  typedef enum logic [1:0] {
    AXI_RESP_OKAY   = 2'b00,
    AXI_RESP_EXOKAY = 2'b01,
    AXI_RESP_SLVERR = 2'b10,
    AXI_RESP_DECERR = 2'b11
  } axi_resp_t;

  typedef enum logic [1:0] {
    AXI_BURST_FIXED = 2'b00,
    AXI_BURST_INCR  = 2'b01,
    AXI_BURST_WRAP  = 2'b10
  } axi_burst_t;

  typedef enum logic [2:0] {
    AXI_SIZE_1B   = 3'b000,
    AXI_SIZE_2B   = 3'b001,
    AXI_SIZE_4B   = 3'b010,
    AXI_SIZE_8B   = 3'b011,
    AXI_SIZE_16B  = 3'b100,
    AXI_SIZE_32B  = 3'b101,
    AXI_SIZE_64B  = 3'b110,
    AXI_SIZE_128B = 3'b111
  } axi_size_t;

  typedef enum logic [1:0] {
    AXI_LOCK_NORMAL    = 2'b00,
    AXI_LOCK_EXCLUSIVE = 2'b01
  } axi_lock_t;

endpackage
