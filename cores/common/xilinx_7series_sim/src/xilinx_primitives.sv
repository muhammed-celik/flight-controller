module MMCME2_BASE #(
  parameter real CLKIN1_PERIOD     = 0.0,
  parameter real CLKFBOUT_MULT_F   = 5.0,
  parameter real CLKOUT0_DIVIDE_F  = 1.0,
  parameter int  DIVCLK_DIVIDE     = 1,
  parameter real CLKOUT0_PHASE     = 0.0,
  parameter      STARTUP_WAIT      = "FALSE"
) (
  input  logic CLKIN1,
  input  logic RST,
  input  logic CLKFBIN,
  input  logic PWRDWN,
  output logic CLKFBOUT,
  output logic CLKFBOUTB,
  output logic CLKOUT0,
  output logic CLKOUT0B,
  output logic CLKOUT1,
  output logic CLKOUT1B,
  output logic CLKOUT2,
  output logic CLKOUT2B,
  output logic CLKOUT3,
  output logic CLKOUT3B,
  output logic CLKOUT4,
  output logic CLKOUT5,
  output logic CLKOUT6,
  output logic LOCKED
);

  localparam realtime CLKOUT0_HALF_PERIOD =
      CLKIN1_PERIOD * DIVCLK_DIVIDE * CLKOUT0_DIVIDE_F /
      CLKFBOUT_MULT_F / 2.0;

  int unsigned lock_count;

  assign CLKFBOUT  = CLKIN1;
  assign CLKFBOUTB = ~CLKIN1;
  assign CLKOUT0B  = ~CLKOUT0;
  assign CLKOUT1   = 1'b0;
  assign CLKOUT1B  = 1'b0;
  assign CLKOUT2   = 1'b0;
  assign CLKOUT2B  = 1'b0;
  assign CLKOUT3   = 1'b0;
  assign CLKOUT3B  = 1'b0;
  assign CLKOUT4   = 1'b0;
  assign CLKOUT5   = 1'b0;
  assign CLKOUT6   = 1'b0;

  initial begin
    CLKOUT0 = 1'b0;
    forever begin
      #(CLKOUT0_HALF_PERIOD);
      if (RST || PWRDWN) begin
        CLKOUT0 = 1'b0;
      end else begin
        CLKOUT0 = ~CLKOUT0;
      end
    end
  end

  always_ff @(posedge CLKIN1 or posedge RST) begin
    if (RST) begin
      lock_count <= '0;
      LOCKED     <= 1'b0;
    end else if (PWRDWN) begin
      lock_count <= '0;
      LOCKED     <= 1'b0;
    end else if (!LOCKED) begin
      if (lock_count == 7) begin
        LOCKED <= 1'b1;
      end else begin
        lock_count <= lock_count + 1'b1;
      end
    end
  end

  logic unused;
  assign unused = &{1'b0, CLKFBIN, CLKOUT0_PHASE != 0.0,
                    STARTUP_WAIT == "TRUE"};

endmodule

module BUFG (
  input  logic I,
  output logic O
);

  assign O = I;

endmodule
