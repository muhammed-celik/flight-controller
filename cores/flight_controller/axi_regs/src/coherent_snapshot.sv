module coherent_snapshot #(
  parameter int unsigned WIDTH = 64
) (
  input  logic             clk,
  input  logic             rst_n,
  input  logic             capture,
  input  logic [WIDTH-1:0] live_data,
  output logic [WIDTH-1:0] snapshot_data,
  output logic [31:0]      snapshot_sequence
);

  initial begin
    if (WIDTH == 0) begin
      $fatal(1, "coherent_snapshot WIDTH must be nonzero");
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      snapshot_data <= '0;
      snapshot_sequence <= '0;
    end else if (capture) begin
      snapshot_data <= live_data;
      snapshot_sequence <= snapshot_sequence + 1'b1;
    end
  end

endmodule
