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
  //SPI Pinout
  output logic o_cs,
  output logic o_sclk,
  output logic o_mosi,
  input  logic i_miso
);

//Data sample-shift edge generation
logic sck_en, sclk, sclk_rise, sclk_fall;
logic sample_edge, shift_edge;

assign sck_en = (state == ST_TRANSFER) || (state == ST_START) || (state == ST_FINISH);
assign o_sclk = sclk;

spi_clk_gen  spi_clk_gen_inst (
  .i_clk(i_clk),
  .i_rstn(i_rstn),
  .i_sck_en(sck_en),
  .o_sclk(sclk),
  .o_sclk_rise(sclk_rise),
  .o_sclk_fall(sclk_fall)
);


assign sample_edge = CPHA ^ CPOL ? sclk_fall : sclk_rise;
assign shift_edge  = CPHA ^ CPOL ? sclk_rise : sclk_fall;

//State Machine
typedef enum logic [1:0] {ST_IDLE, ST_CS_SETUP, ST_TRANSFER, ST_CS_HOLD} state_t;
state_t state;

logic [7:0] tx_shift_reg, rx_shift_reg;
logic [2:0] bit_cntr;
logic [$clog2(CS_SETUP_CYCLE)-1:0] cs_setup_cntr;
logic [$clog2(CS_HOLD_CYCLE)-1:0] cs_hold_cntr;

always_ff @(posedge i_clk or negedge i_rstn) begin
  if(!i_rstn) begin
    state <= ST_IDLE;
    o_cs <= 1'b1;
    o_mosi <= 1'b0;
    o_done <= 1'b0;
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
          if(CPOL) begin
            state <= ST_START;
          end else begin
            state <= ST_TRANSFER;
          end
          cs_setup_cntr <= '0;
        end else begin
          cs_setup_cntr <= cs_setup_cntr + 1;
        end
      end
      ST_START: begin
        if(sclk_fall) begin
          state <= ST_TRANSFER;
        end
      end
      ST_TRANSFER: begin
        if(bit_cntr == 3'd7 && sclk_fall) begin
          state <= ST_FINISH;
          bit_cntr <= '0;
          o_done <= 1'b1;
        end else if(sclk_fall) begin
          bit_cntr <= bit_cntr + 1;
        end

        if(shift_edge) begin
          o_mosi <= tx_shift_reg[7];
          tx_shift_reg <= {tx_shift_reg[6:0], 1'b0};
        end

        if(sample_edge) begin
          rx_shift_reg <= {rx_shift_reg[6:0], i_miso};
        end
      end
      ST_FINISH: begin
        o_done <= 1'b0;
        if(i_en) begin
          state <= ST_TRANSFER;
          tx_shift_reg <= i_data;
          rx_shift_reg <= '0;
          bit_cntr <= '0;
        end else begin
          if(CPOL) begin
            if(sclk_rise) begin
              state <= ST_CS_HOLD;
              cs_hold_cntr <= '0;
            end else begin
              state <= ST_FINISH;
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

assign o_data = rx_shift_reg;

endmodule
