module spi_master 
  import spi_master_pkg::*;
#(
  parameter bit CPOL = 1,
  parameter bit CPHA = 1
)
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

//State Machine
typedef enum logic [2:0] {
  ST_IDLE,
  ST_CS_SETUP,
  ST_START,
  ST_TRANSFER,
  ST_FINISH,
  ST_CS_HOLD
} state_t;
state_t state;

//Data sample-shift edge generation
logic sck_en, sclk, sclk_rise, sclk_fall;
logic sample_edge, shift_edge;

assign sck_en = (state == ST_START) || (state == ST_TRANSFER) || (state == ST_FINISH);
assign o_sclk = sclk;

spi_clk_gen #(
  .CPOL(CPOL),
  .CPHA(CPHA)
) spi_clk_gen_inst (
  .i_clk(i_clk),
  .i_rstn(i_rstn),
  .i_sck_en(sck_en),
  .o_sclk(sclk),
  .o_sclk_rise(sclk_rise),
  .o_sclk_fall(sclk_fall)
);


assign sample_edge = CPHA ^ CPOL ? sclk_fall : sclk_rise;
assign shift_edge  = CPHA ^ CPOL ? sclk_rise : sclk_fall;

logic [7:0] tx_shift_reg, rx_shift_reg;
logic [2:0] bit_cntr;
logic [$clog2(CS_SETUP_CYCLE)-1:0] cs_setup_cntr;
logic [$clog2(CS_HOLD_CYCLE)-1:0] cs_hold_cntr;
logic start_ready;

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
    start_ready <= 1'b0;
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
        if(cs_setup_cntr == (CS_SETUP_CYCLE - 1)) begin
          state <= ST_START;
          cs_setup_cntr <= '0;
          start_ready <= 1'b0;
        end else begin
          cs_setup_cntr <= cs_setup_cntr + 1;
        end
      end

      ST_START: begin
        if(start_ready) begin
          state <= ST_TRANSFER;
          o_mosi <= tx_shift_reg[7];
          tx_shift_reg <= {tx_shift_reg[6:0], 1'b0};
          bit_cntr <= '0;
          start_ready <= 1'b0;
        end else begin
          start_ready <= 1'b1; // Wait for one clock cycle before starting transfer
        end
      end

      ST_TRANSFER: begin
        if(shift_edge) begin
          if(bit_cntr == 3'd7) begin
            o_done <= 1'b0; // Clear done signal
            bit_cntr <= '0;
            if(i_en) begin
              state <= ST_TRANSFER; // Continue transferring if i_en is still high
              o_mosi <= i_data[7];
              tx_shift_reg <= {i_data[6:0], 1'b0};
            end else begin
              state <= ST_FINISH; // Move to finish state if i_en is low
            end
          end else begin
            o_mosi <= tx_shift_reg[7];
            tx_shift_reg <= {tx_shift_reg[6:0], 1'b0};
            bit_cntr <= bit_cntr + 1;
          end
        end else if(sample_edge) begin
          rx_shift_reg <= {rx_shift_reg[6:0], i_miso};
          if(bit_cntr == 3'd7) begin
            o_done <= 1'b1;
          end
        end
      end
      ST_FINISH: begin
        if(CPHA) begin
          state <= ST_CS_HOLD;
          cs_hold_cntr <= '0;
        end else begin
          if(sample_edge) begin
            state <= ST_CS_HOLD;
            cs_hold_cntr <= '0;
          end
        end
      end
      ST_CS_HOLD: begin
        if(cs_hold_cntr == (CS_HOLD_CYCLE - 1)) begin
          state <= ST_IDLE;
          o_cs <= 1'b1;
          cs_hold_cntr <= '0;
        end else begin
          cs_hold_cntr <= cs_hold_cntr + 1;
        end
      end
      default: begin
        state <= ST_IDLE;
      end
    endcase
  end
end

assign o_data = rx_shift_reg;

endmodule
