module spi_master 
  import spi_master_pkg::*;
(
  input  logic i_clk,
  input  logic i_rstn,
  // Sensor Acquisition Unit interface
  input  logic i_valid,
  input  logic [1:0] i_sclk_sel, // 2-bit selection for SCLK frequency (1Mhz, 2Mhz, 5Mhz, 10Mhz)
  input  logic [$clog2(CS_CNT)-1:0] i_cs_sel,
  input  logic [7:0] i_data,
  output logic o_valid,
  output logic o_busy,
  output logic [7:0] o_data,
  // SPI interface
  output logic [CS_CNT-1:0] o_cs,
  output logic o_mosi,
  input  logic i_miso,
  output logic o_sclk
);

typedef enum logic {
  ST_IDLE,
  ST_TRANSFER
} spi_state_t;

spi_state_t state;

// SPI serial clock generation
logic [$clog2(SCLK_FREQ_0_LIM)-1:0] sclk_cntr, sclk_cntr_lim;
logic sclk, sclk_prev, sclk_rise, sclk_fall;

always_ff @(posedge i_clk or negedge i_rstn) begin
  if (!i_rstn) begin
    sclk_cntr <= '0;
    sclk <= CPOL==0 ? 1'b0 : 1'b1; // CPOL = 0 for mode 0 and 1, CPOL = 1 for mode 2 and 3
  end else begin
    sclk_prev <= sclk;
    if(state == ST_TRANSFER) begin
      if(sclk_cntr == sclk_cntr_lim - 1) begin
        sclk_cntr <= '0;
        sclk <= ~sclk; // Toggle SCLK   
      end else begin
        sclk_cntr <= sclk_cntr + 1;
      end
    end else begin
      sclk_cntr <= '0;
      sclk <= CPOL==0 ? 1'b0 : 1'b1; // Reset SCLK to idle state
    end
  end
end

assign sclk_rise = (sclk_prev == 1'b0 && sclk == 1'b1);
assign sclk_fall = (sclk_prev == 1'b1 && sclk == 1'b0);
assign o_sclk = sclk;
assign o_busy = (state == ST_TRANSFER);
assign o_data = shift_reg_in; 

// SPI State Machine
logic [7:0] shift_reg_in, shift_reg_out;
logic [3:0] bit_cntr;

always_ff @(posedge i_clk or negedge i_rstn) begin
  if (!i_rstn) begin
    state <= ST_IDLE;
    o_valid <= 1'b0;
    o_cs <= {CS_CNT{1'b1}};
    o_mosi <= 1'b0;
  end else begin
    case (state)
      ST_IDLE: begin
        o_valid <= 1'b0; // Data is not valid yet
        if (i_valid) begin
          state <= ST_TRANSFER;
          shift_reg_out <= i_data;
          shift_reg_in <= '0;
          o_valid <= 1'b0;
          o_cs[i_cs_sel] <= 1'b0; // Assert CS
          case (i_sclk_sel)
            2'b00: sclk_cntr_lim <= SCLK_FREQ_0_LIM;
            2'b01: sclk_cntr_lim <= SCLK_FREQ_1_LIM;
            2'b10: sclk_cntr_lim <= SCLK_FREQ_2_LIM;
            2'b11: sclk_cntr_lim <= SCLK_FREQ_3_LIM;
            default: sclk_cntr_lim <= SCLK_FREQ_0_LIM;
          endcase
        end
      end
      ST_TRANSFER: begin
        if(CPHA == 0) begin
          if(bit_cntr == 15) begin
            if(sclk_rise) begin
              shift_reg_in <= {shift_reg_in[6:0], i_miso}; // Read MISO on falling edge
              o_valid <= 1'b1; // Indicate that data is valid
              bit_cntr <= '0;
              if(i_valid) begin
                state <= ST_TRANSFER; // Continue transferring if new data is valid
                shift_reg_out <= i_data; // Load new data for next transfer
              end else begin
                state <= ST_IDLE; // No new data, go to idle state
                shift_reg_out <= '0; // No new data, send zeros
              end
            end else if(sclk_fall) begin
              o_mosi <= shift_reg_out[7]; // Send MSB first
              o_valid <= 1'b0; // Data is not valid yet
              bit_cntr <= '0;
              if(i_valid) begin
                state <= ST_TRANSFER; // Continue transferring if new data is valid
                shift_reg_out <= i_data; // Load new data for next transfer
              end else begin
                state <= ST_IDLE; // No new data, go to idle state
                shift_reg_out <= {shift_reg_out[6:0], 1'b0}; // Shift left
              end
            end else begin
              o_valid <= 1'b0; // Data is not valid yet
              bit_cntr <= bit_cntr;
            end
          end else begin
            o_valid <= 1'b0; // Data is not valid yet
            if(sclk_rise) begin
              shift_reg_in <= {shift_reg_in[6:0], i_miso}; // Read MISO on falling edge
              bit_cntr <= bit_cntr + 1;
            end else if(sclk_fall) begin
              o_mosi <= shift_reg_out[7]; // Send MSB first
              bit_cntr <= bit_cntr + 1;
              shift_reg_out <= {shift_reg_out[6:0], 1'b0}; // Shift left
            end else begin
              bit_cntr <= bit_cntr;
            end
          end
        end else begin //CPHA=1
          if(bit_cntr == 15) begin
            if(sclk_fall) begin
              shift_reg_in <= {shift_reg_in[6:0], i_miso}; // Read MISO on falling edge
              o_valid <= 1'b1; // Indicate that data is valid
              bit_cntr <= '0;
              if(i_valid) begin
                state <= ST_TRANSFER; // Continue transferring if new data is valid
                shift_reg_out <= i_data; // Load new data for next transfer
              end else begin
                state <= ST_IDLE; // No new data, go to idle state
                shift_reg_out <= '0; // No new data, send zeros
              end
            end else if(sclk_rise) begin
              o_mosi <= shift_reg_out[7]; // Send MSB first
              o_valid <= 1'b0; // Data is not valid yet
              bit_cntr <= '0;
              if(i_valid) begin
                state <= ST_TRANSFER; // Continue transferring if new data is valid
                shift_reg_out <= i_data; // Load new data for next transfer
              end else begin
                state <= ST_IDLE; // No new data, go to idle state
                shift_reg_out <= {shift_reg_out[6:0], 1'b0}; // Shift left
              end
            end else begin
              o_valid <= 1'b0; // Data is not valid yet
              bit_cntr <= bit_cntr;
            end
          end else begin
            o_valid <= 1'b0; // Data is not valid yet
            if(sclk_fall) begin
              shift_reg_in <= {shift_reg_in[6:0], i_miso}; // Read MISO on falling edge
              bit_cntr <= bit_cntr + 1;
            end else if(sclk_rise) begin
              o_mosi <= shift_reg_out[7]; // Send MSB first
              bit_cntr <= bit_cntr + 1;
              shift_reg_out <= {shift_reg_out[6:0], 1'b0}; // Shift left
            end else begin
              bit_cntr <= bit_cntr;
            end
          end
        end
      end
    endcase
  end
end
endmodule
