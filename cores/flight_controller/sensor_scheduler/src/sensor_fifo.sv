module sensor_fifo #(
  parameter int unsigned DATA_WIDTH = 256,
  parameter int unsigned DEPTH      = 16
) (
  input  logic                    clk,
  input  logic                    rst_n,
  input  logic                    push_valid,
  input  logic [DATA_WIDTH-1:0]   push_data,
  input  logic                    pop,
  output logic [DATA_WIDTH-1:0]   head_data,
  output logic [$clog2(DEPTH+1)-1:0] level,
  output logic                    empty,
  output logic                    full,
  output logic                    overflow,
  output logic                    underflow
);

  localparam int unsigned POINTER_WIDTH = $clog2(DEPTH);
  localparam int unsigned LEVEL_WIDTH = $clog2(DEPTH + 1);

  logic [DATA_WIDTH-1:0] memory [0:DEPTH-1];
  logic [POINTER_WIDTH-1:0] read_pointer;
  logic [POINTER_WIDTH-1:0] write_pointer;
  logic pop_accepted;
  logic push_accepted;

  initial begin
    if ((DEPTH < 2) || ((DEPTH & (DEPTH - 1)) != 0)) begin
      $fatal(1, "sensor_fifo DEPTH must be a power of two and at least 2");
    end
    if (DATA_WIDTH == 0) begin
      $fatal(1, "sensor_fifo DATA_WIDTH must be nonzero");
    end
  end

  always_comb begin
    empty         = (level == 0);
    full          = (level == LEVEL_WIDTH'(DEPTH));
    pop_accepted  = pop && !empty;
    push_accepted = push_valid && (!full || pop_accepted);
    head_data     = empty ? '0 : memory[read_pointer];
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      read_pointer  <= '0;
      write_pointer <= '0;
      level         <= '0;
      overflow      <= 1'b0;
      underflow     <= 1'b0;
    end else begin
      overflow  <= push_valid && !push_accepted;
      underflow <= pop && !pop_accepted;

      if (push_accepted) begin
        memory[write_pointer] <= push_data;
        write_pointer         <= write_pointer + 1'b1;
      end
      if (pop_accepted) begin
        read_pointer <= read_pointer + 1'b1;
      end

      unique case ({push_accepted, pop_accepted})
        2'b10: level <= level + 1'b1;
        2'b01: level <= level - 1'b1;
        default: level <= level;
      endcase
    end
  end

endmodule
