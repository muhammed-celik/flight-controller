module spi_master 
  import spi_master_pkg::*;
(
  input logic i_clk,
  input logic i_rstn,
  //Top Level Interface
  input logic i_en,
  input logic [7:0] i_data,
  output logic o_done,
  output logic [7:0] o_data,
  output logic o_busy,
  //SPI Pinout
  output logic o_cs,
  output logic o_sclk,
  output logic o_mosi,
  input  logic i_miso
);

//SCLK generation
logic [$clog2(CLK_DIV)-1:0] clk_div_cntr;
logic sclk;

always_ff @(posedge i_clk or negedge i_rstn) begin
  if(!i_rstn) begin
    sclk <= CPOL;
    clk_div_cntr <= '0;
  end else begin
    case (state)
      ST_IDLE, ST_CS_SETUP, ST_CS_HOLD: begin
        sclk <= CPOL;
        clk_div_cntr <= '0;
      end
      ST_START, ST_TRANSFER, ST_FINISH: begin
        if(clk_div_cntr == CLK_DIV-1) begin
          sclk <= ~sclk;
          clk_div_cntr <= '0;
        end else begin
          clk_div_cntr <= clk_div_cntr + 1;
        end
      end
    endcase
  end
end

//Data sample-shift edge generation
logic sclk_prev, sclk_rise, sclk_fall;
logic sample_edge, shift_edge, first_edge, second_edge;

always_ff @(posedge i_clk or negedge i_rstn) begin
  if(!i_rstn) begin
    sclk_prev <= CPOL;
    sclk_rise <= 1'b0;
    sclk_fall <= 1'b0;
  end else begin
    sclk_prev <= sclk;
    sclk_rise <= (sclk_prev == 1'b0 && sclk == 1'b1);
    sclk_fall <= (sclk_prev == 1'b1 && sclk == 1'b0);
  end
end

assign sample_edge = CPHA ^ CPOL ? sclk_fall   : sclk_rise;
assign shift_edge  = CPHA ^ CPOL ? sclk_rise   : sclk_fall;
assign first_edge  = CPOL        ? sclk_fall   : sclk_rise;
assign last_edge   = CPOL        ? sclk_rise   : sclk_fall;


//State Machine
typedef enum logic [2:0] {ST_IDLE,ST_CS_SETUP,ST_START,ST_TRANSFER,ST_FINISH,ST_CS_HOLD} state_t;
state_t state;

logic [7:0] tx_shift_reg, rx_shift_reg;
logic [3:0] bit_cntr;
logic [$clog2(CS_SETUP_CYCLE)-1:0] cs_setup_cntr;
logic [$clog2(CS_HOLD_CYCLE)-1:0] cs_hold_cntr;

always_ff @(posedge i_clk or negedge i_rstn) begin
  if(!i_rstn) begin
    state <= ST_IDLE;
    o_cs <= 1'b1;
    rx_shift_reg <= '0;
    tx_shift_reg <= '0;
    bit_cntr <= '0;
    cs_setup_cntr <= '0;
    cs_hold_cntr <= '0;
  end else begin
    case(state)
      ST_IDLE: begin
        if(i_en) begin
          state <= ST_CS_SETUP;
          o_cs <= 1'b0;
          cs_setup_cntr <= '0;
          tx_shift_reg <= i_data;
          rx_shift_reg <= '0;
          bit_cntr <= '0;
        end
      end
      ST_CS_SETUP: begin
        if(cs_setup_cntr == CS_SETUP_CYCLE-1) begin
          state <= ST_START;
          cs_setup_cntr <= '0;
        end else begin
          cs_setup_cntr <= cs_setup_cntr + 1;
        end
      end
      ST_START: begin
        if(CPOL) begin
          if(first_edge) begin
            state <= ST_TRANSFER;
          end
        end else begin
          if(first_edge) begin
            if(CPHA) begin
              state <= ST_TRANSFER;
              bit_cntr <= bit_cntr + 1;
              tx_shift_reg <= {tx_shift_reg[6:0], 1'b0};
            end else begin
              state <= ST_TRANSFER;
              bit_cntr <= bit_cntr + 1;
              rx_shift_reg <= {rx_shift_reg[6:0], i_miso};
            end
          end
        end
      end
      ST_TRANSFER: begin
        if(CPOL ^ CPHA) begin
          if(shift_edge) begin
            tx_shift_reg <= {tx_shift_reg[6:0], 1'b0};
            bit_cntr <= bit_cntr + 1;
          end
          if(sample_edge) begin
            rx_shift_reg <= {rx_shift_reg[6:0], i_miso};
            if(bit_cntr == 4'd15) begin
              state <= ST_FINISH;
              bit_cntr <= '0;
              o_done <= 1'b1;
            end else begin
              bit_cntr <= bit_cntr + 1;
            end
          end
        end else begin
          if(shift_edge) begin
            tx_shift_reg <= {tx_shift_reg[6:0], 1'b0};
            if(bit_cntr == 4'd15) begin
              state <= ST_FINISH;
              bit_cntr <= '0;
              o_done <= 1'b1;
            end else begin
              bit_cntr <= bit_cntr + 1;
            end
          end
          if(sample_edge) begin
            rx_shift_reg <= {rx_shift_reg[6:0], i_miso};
          end
        end
      end
      ST_FINISH: begin
        o_done <= 1'b0;
        if(i_en) begin
          state <= ST_TRANSFER;
          tx_shift_reg <= i_data;
          rx_shift_reg <= '0;
        end else begin
          if(CPOL) begin
            if(last_edge) begin
              state <= ST_CS_HOLD;
              cs_hold_cntr <= '0;
            end
          end else begin
            state <= ST_CS_HOLD;
            cs_hold_cntr <= '0;
          end
        end
      end
      ST_CS_HOLD: begin
        if(cs_hold_cntr == CS_HOLD_CYCLE-1) begin
          state <= ST_IDLE;
          o_cs <= 1'b1;
          cs_hold_cntr <= '0;
        end else begin
          cs_hold_cntr <= cs_hold_cntr + 1;
        end
      end
    endcase
  end
end

endmodule
