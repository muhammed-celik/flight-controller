module reset_sync #(
  parameter int unsigned STAGES = 3
) (
  input  logic clk,
  input  logic arst_n,
  output logic srst_n
);

  (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *)
  logic [STAGES-1:0] sync_ff;

  initial begin
    if (STAGES < 2) begin
      $fatal(1, "reset_sync STAGES must be at least 2");
    end
  end

  always_ff @(posedge clk or negedge arst_n) begin
    if (!arst_n) begin
      sync_ff <= '0;
    end else begin
      sync_ff <= {sync_ff[STAGES-2:0], 1'b1};
    end
  end

  assign srst_n = sync_ff[STAGES-1];

endmodule
