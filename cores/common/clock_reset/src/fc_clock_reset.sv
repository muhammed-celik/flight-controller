module fc_clock_reset #(
  parameter int unsigned LOCK_STABLE_CYCLES = 16,
  parameter int unsigned RESET_SYNC_STAGES  = 3
) (
  input  logic clk_12mhz,
  input  logic ext_reset,
  output logic clk_100mhz,
  output logic rst_n,
  output logic clock_locked
);

  localparam int unsigned LOCK_COUNT_WIDTH =
      (LOCK_STABLE_CYCLES <= 1) ? 1 : $clog2(LOCK_STABLE_CYCLES);

  logic mmcm_locked;
  logic raw_run_n;
  logic lock_qualified;
  logic [LOCK_COUNT_WIDTH-1:0] lock_count;

  initial begin
    if (LOCK_STABLE_CYCLES == 0) begin
      $fatal(1, "fc_clock_reset LOCK_STABLE_CYCLES must be nonzero");
    end
  end

  assign raw_run_n   = mmcm_locked & ~ext_reset;
  assign clock_locked = lock_qualified;

  clk_gen #(
    .CLKIN_FREQ(12.0),
    .VCO_MULT  (50.0),
    .OUT_DIV   (6.0),
    .DIV_CLK   (1)
  ) u_clk_gen (
    .clk_in (clk_12mhz),
    .rst    (ext_reset),
    .clk_out(clk_100mhz),
    .locked (mmcm_locked)
  );

  always_ff @(posedge clk_100mhz or negedge raw_run_n) begin
    if (!raw_run_n) begin
      lock_count     <= '0;
      lock_qualified <= 1'b0;
    end else if (!lock_qualified) begin
      if (lock_count == LOCK_COUNT_WIDTH'(LOCK_STABLE_CYCLES - 1)) begin
        lock_qualified <= 1'b1;
      end else begin
        lock_count <= lock_count + 1'b1;
      end
    end
  end

  reset_sync #(
    .STAGES(RESET_SYNC_STAGES)
  ) u_reset_sync (
    .clk   (clk_100mhz),
    .arst_n(lock_qualified),
    .srst_n(rst_n)
  );

endmodule
