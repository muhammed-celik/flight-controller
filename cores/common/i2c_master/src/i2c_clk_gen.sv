module i2c_clk_gen
  import i2c_master_pkg::*;
(
  input  logic i_clk,
  input  logic i_rstn,
  input  logic i_i2c_en,
  inout  tri   io_scl,
  output logic o_scl_wr_tick,
  output logic o_scl_rd_tick
);

localparam int QUARTER_PERIOD = SYS_CLK_FREQ / (I2C_CLK_FREQ * 4);
logic [$clog2(QUARTER_PERIOD)-1:0] clk_cntr;
logic [1:0] phase;

logic scl_out, scl_in, scl_wait;
assign io_scl = (!scl_out) ? 1'b0 : 1'bz;
assign scl_in = io_scl;

//clock stretching logic
assign scl_wait = (scl_out == 1'b1 && scl_in == 1'b0);

always_ff @(posedge i_clk or negedge i_rstn) begin
  if(!i_rstn) begin
    clk_cntr <= '0;
  end else if(i_i2c_en && !scl_wait) begin
    if(clk_cntr == QUARTER_PERIOD-1) begin
      clk_cntr <= '0;
    end else begin
      clk_cntr <= clk_cntr + 1'b1;
    end
  end else if(!i_i2c_en) begin
    clk_cntr <= '0;
  end
end

always_ff @(posedge i_clk or negedge i_rstn) begin
  if(!i_rstn) begin
    phase <= 2'b00;
    scl_out <= 1'b1;
    o_scl_wr_tick <= 1'b0;
    o_scl_rd_tick <= 1'b0;
  end else if(i_i2c_en) begin
    if(clk_cntr == QUARTER_PERIOD-1 && !scl_wait) begin
      phase <= phase + 2'd1;
      case (phase)
        2'd0: begin
          scl_out <= 1'b1;
          o_scl_wr_tick <= 1'b0;
          o_scl_rd_tick <= 1'b0;
        end
        2'd1: begin
          scl_out <= 1'b1;
          o_scl_wr_tick <= 1'b0;
          o_scl_rd_tick <= 1'b1; // Read tick occurs at the middle of the high phase
        end
        2'd2: begin
          scl_out <= 1'b0;
          o_scl_wr_tick <= 1'b0;
          o_scl_rd_tick <= 1'b0;
        end
        2'd3: begin
          scl_out <= 1'b0;
          o_scl_wr_tick <= 1'b1; // Write tick occurs at the middle of the low phase
          o_scl_rd_tick <= 1'b0;
        end 
        default: begin
          scl_out <= 1'b1;
          o_scl_wr_tick <= 1'b0;
          o_scl_rd_tick <= 1'b0;
        end
      endcase
    end else begin
      o_scl_wr_tick <= 1'b0;
      o_scl_rd_tick <= 1'b0;
    end
  end else begin
    scl_out <= 1'b1;
    o_scl_wr_tick <= 1'b0;
    o_scl_rd_tick <= 1'b0;
    phase <= 2'b00;
  end
end

endmodule