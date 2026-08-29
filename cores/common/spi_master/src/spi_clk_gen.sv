module spi_clk_gen
  import spi_master_pkg::*;
(
  input logic i_clk,
  input logic i_rstn,
  input logic i_sck_en,
  output logic o_sclk,
  output logic o_sclk_rise,
  output logic o_sclk_fall
);

logic sclk, sclk_prev, sclk_rise, sclk_fall;
logic [$clog2(CLK_DIV)-1:0] clk_div_cntr;

always @(posedge i_clk or negedge i_rstn) begin
  if(!i_rstn) begin
    sclk <= CPOL;
  end else begin
    if(i_sck_en) begin
      if(clk_div_cntr == (CLK_DIV/2)-1) begin
        sclk <= ~sclk;
        clk_div_cntr <= '0;
      end else begin
        clk_div_cntr <= clk_div_cntr + 1;
      end
    end else begin
      o_sclk <= CPOL;
    end
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