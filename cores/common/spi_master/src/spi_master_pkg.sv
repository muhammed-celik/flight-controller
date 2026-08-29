package spi_master_pkg;
  localparam bit          CPOL             = 1;
  localparam bit          CPHA             = 1;
  localparam int unsigned CLK_DIV          = 100;
  localparam int unsigned CS_SETUP_CYCLE   = 1;
  localparam int unsigned CS_HOLD_CYCLE    = 2;

endpackage : spi_master_pkg
