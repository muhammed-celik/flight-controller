module spi_clk_gen
  import spi_master_pkg::*;
#(
  parameter bit CPOL = 1,
  parameter bit CPHA = 1
)
(
  input logic i_clk,
  input logic i_rstn,
  input logic i_sck_en,
  output logic o_sclk,
  output logic o_sclk_rise,
  output logic o_sclk_fall
);

logic sclk, sclk_prev;
logic [$clog2(CLK_DIV)-1:0] clk_div_cntr;

typedef enum logic {
  SCK_IDLE,
  SCK_ACTIVE
} sck_state_t;

sck_state_t sck_state;

always @(posedge i_clk or negedge i_rstn) begin
  if(!i_rstn) begin
    sck_state <= SCK_IDLE;
    sclk <= CPOL;
    clk_div_cntr <= '0;
  end else begin
    case(sck_state)
      SCK_IDLE: begin
        if(i_sck_en) begin
          sck_state <= SCK_ACTIVE;
          sclk <= CPHA ? ~CPOL : CPOL; // Set initial clock state based on CPHA
          clk_div_cntr <= '0;
        end else begin
          sclk <= CPOL; // Maintain idle state
        end
      end
      SCK_ACTIVE: begin
        if(!i_sck_en) begin
          sck_state <= SCK_IDLE;
          sclk <= CPOL; // Return to idle state
          clk_div_cntr <= '0;
        end else if(clk_div_cntr == (CLK_DIV - 1)) begin
          sclk <= ~sclk; // Toggle clock
          clk_div_cntr <= '0;
        end else begin
          clk_div_cntr <= clk_div_cntr + 1;
        end
      end
    endcase
  end
end

always @(posedge i_clk or negedge i_rstn) begin
  if(!i_rstn) begin
    sclk_prev <= CPOL;
  end else begin
    sclk_prev <= sclk;
  end
end

assign o_sclk_rise = (sclk_prev == 0 && sclk == 1);
assign o_sclk_fall = (sclk_prev == 1 && sclk == 0);
assign o_sclk = sclk;

endmodule
