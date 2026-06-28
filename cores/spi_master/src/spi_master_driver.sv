module spi_master_driver
  import spi_master_pkg::*;
(
  input  logic i_clk,
  input  logic i_rstn,

  input  logic i_start,
  output logic o_busy,
  output logic o_done,

  input  logic [$clog2(CS_CNT)-1:0] i_cs_sel, 
  input  logic [15:0] i_clk_div,
  input  logic [1:0] i_mode, // SPI mode (0, 1, 2, 3) [00,01,11,10] [CPOL, CPHA]
  input  logic [7:0] i_nbytes,

  input  logic i_tx_valid,
  input  logic [7:0] i_tx_data,

  output logic o_rx_valid,
  output logic [7:0] o_rx_data,

  output logic [CS_CNT-1:0] o_cs,
  output logic o_sclk,
  output logic o_mosi,
  input  logic i_miso

);

//Utility signals
logic busy_int, done_int, rx_valid_int;
logic [1:0] mode_int;
logic [$clog2(CS_CNT)-1:0] cs_sel_int;
logic [15:0] clk_div_int;
logic [7:0] tx_shift_reg, rx_shift_reg, nbytes_int, nbytes_counter;
logic [3:0] bit_cntr;

//FSM for SPI Master Driver
typedef enum logic {
  ST_IDLE,
  ST_TRANSFER_BYTE
} spi_state_t;

spi_state_t state;

// SPI serial clock generation
logic [15:0] sclk_cntr;
logic sclk, sclk_prev, sclk_rise, sclk_fall;

always_ff @(posedge i_clk or negedge i_rstn) begin
  if (!i_rstn) begin
    sclk_cntr <= '0;
    sclk <= mode_int[1]==0 ? 1'b0 : 1'b1; // CPOL = 0 for mode 0 and 1, CPOL = 1 for mode 2 and 3
    sclk_prev <= mode_int[1]==0 ? 1'b0 : 1'b1;
  end else begin
    sclk_prev <= sclk;
    if(state == ST_TRANSFER_BYTE) begin
      if(sclk_cntr == clk_div_int - 1) begin
        sclk_cntr <= '0;
        sclk <= ~sclk; // Toggle SCLK   
      end else begin
        sclk_cntr <= sclk_cntr + 1;
      end
    end else begin
      sclk_cntr <= '0;
      sclk <= mode_int[1]==0 ? 1'b0 : 1'b1; // Reset SCLK to idle state
    end
  end
end

// output/direct assignments
assign busy_int = (state == ST_TRANSFER_BYTE);
assign o_busy = busy_int;
assign o_done = done_int;

assign o_rx_valid = rx_valid_int;
assign o_rx_data = rx_shift_reg;

assign sclk_rise = (sclk_prev == 1'b0 && sclk == 1'b1);
assign sclk_fall = (sclk_prev == 1'b1 && sclk == 1'b0);
assign o_sclk = sclk;
assign o_mosi = tx_shift_reg[7]; // MSB first
assign o_cs = (state == ST_TRANSFER_BYTE) ? ~(1'b1 << cs_sel_int) : {CS_CNT{1'b1}}; // Active low CS


// FSM for SPI Master Driver
always_ff @(posedge i_clk or negedge i_rstn) begin
  if (!i_rstn) begin
    state <= ST_IDLE;
    nbytes_int <= 0;
    clk_div_int <= 0;
    cs_sel_int <= 0;
    mode_int <= 0;
  end else begin
    case (state)
      ST_IDLE: begin
        if(i_start && !busy_int) begin
          state <= ST_TRANSFER_BYTE;
          nbytes_int <= i_nbytes;
          clk_div_int <= i_clk_div;
          cs_sel_int <= i_cs_sel;
          mode_int <= i_mode;
        end
      end
      ST_TRANSFER_BYTE: begin
        if(done_int) begin
          state <= ST_IDLE;
        end
      end
      default: begin
        state <= ST_IDLE;
      end 
    endcase
  end
end

// Shift bits logic
always_ff @(posedge i_clk or negedge i_rstn) begin
  if (!i_rstn) begin
    tx_shift_reg <= 8'h00;
    rx_shift_reg <= 8'h00;
  end else begin
    if(i_tx_valid) begin
      tx_shift_reg <= i_tx_data;
    end
    if(state == ST_TRANSFER_BYTE) begin
      if(sclk_rise) begin
        if(^mode_int == 0) begin
          tx_shift_reg <= {tx_shift_reg[6:0], 1'b0}; // Shift out data on rising edge
        end else begin
          rx_shift_reg <= {rx_shift_reg[6:0], i_miso}; // Shift in data on rising edge
        end
      end
      if(sclk_fall) begin
        if(^mode_int == 1) begin
          tx_shift_reg <= {tx_shift_reg[6:0], 1'b0}; // Shift out data on falling edge
        end else begin
          rx_shift_reg <= {rx_shift_reg[6:0], i_miso}; // Shift in data on falling edge
        end
      end
    end else begin
      tx_shift_reg <= 8'h00;
      rx_shift_reg <= 8'h00;
    end
  end
end

// bit-byte counter and rx-tx handshake logic
always_ff @(posedge i_clk or negedge i_rstn) begin
  if (!i_rstn) begin
    bit_cntr <= 0;
    nbytes_counter <= 0;
    rx_valid_int <= 1'b0;
    done_int <= 1'b0;
  end else begin
    if(state == ST_TRANSFER_BYTE) begin
      if(sclk_rise || sclk_fall) begin
        if(bit_cntr == 15) begin
          bit_cntr <= 0;
          rx_valid_int <= 1'b1; // Indicate that a byte can be received
          if(nbytes_counter == nbytes_int - 1) begin
            done_int <= 1'b1; // Indicate that the transfer is done
            nbytes_counter <= 0;
          end else begin
            done_int <= 1'b0;
            nbytes_counter <= nbytes_counter + 1;
          end
        end else begin
          bit_cntr <= bit_cntr + 1;
          rx_valid_int <= 1'b0;
        end
      end
    end else begin
      bit_cntr <= 0;
      nbytes_counter <= 0;
      done_int <= 1'b0;
      rx_valid_int <= 1'b0;
    end
  end
end 


endmodule