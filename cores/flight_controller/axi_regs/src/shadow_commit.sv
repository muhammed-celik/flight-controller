module shadow_commit #(
  parameter int unsigned WIDTH = 64,
  parameter int unsigned WORD_INDEX_WIDTH = $clog2(WIDTH / 32)
) (
  input  logic                         clk,
  input  logic                         rst_n,
  input  logic                         shadow_write,
  input  logic [WORD_INDEX_WIDTH-1:0]  shadow_word,
  input  logic [31:0]                  shadow_wdata,
  input  logic [3:0]                   shadow_wstrb,
  input  logic                         commit,
  output logic [WIDTH-1:0]             shadow_data,
  output logic [WIDTH-1:0]             active_data,
  output logic [31:0]                  commit_sequence,
  output logic                         commit_pulse
);

  localparam int unsigned WORD_COUNT = WIDTH / 32;

  initial begin
    if (WIDTH < 64 || (WIDTH % 32) != 0) begin
      $fatal(1, "shadow_commit WIDTH must be a multiple of 32 and at least 64");
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      shadow_data <= '0;
      active_data  <= '0;
      commit_sequence <= '0;
      commit_pulse <= 1'b0;
    end else begin
      commit_pulse <= commit;
      if (shadow_write) begin
        for (int unsigned word_index = 0; word_index < WORD_COUNT; word_index++) begin
          if (shadow_word == WORD_INDEX_WIDTH'(word_index)) begin
            for (int unsigned byte_index = 0; byte_index < 4; byte_index++) begin
              if (shadow_wstrb[byte_index]) begin
                shadow_data[word_index*32 + byte_index*8 +: 8] <=
                    shadow_wdata[byte_index*8 +: 8];
              end
            end
          end
        end
      end
      if (commit) begin
        active_data <= shadow_data;
        commit_sequence <= commit_sequence + 1'b1;
      end
    end
  end

endmodule
