module i2c_controller (
  input logic i_clk,
  input logic i_rstn,
  //Top Level Interface
  input logic i_cmd_valid,
  input logic i_cmd_type, // 1: Read, 0: Write
  input logic [4:0] i_cmd_nbytes, // Number of bytes to transfer (minimum 1 (5'b00000), maximum 32 (5'b11111))
  input logic [7:0] i_cmd_dev_addr, // Device Address
  input logic [7:0] i_cmd_reg_addr, // Register Address
  output logic o_cmd_ready, // Ready to accept new command

  input logic i_data_valid, // Register Data for Write is valid
  input logic [7:0] i_data, // Register Data for Write

  output logic o_data_valid, // Register Data for Read is valid
  output logic [7:0] o_data, // Register Data for Read
  //I2C Pinout
  output logic o_sda,
  output logic o_scl
);

localparam bit CPOL = 1;
localparam bit CPHA = 1;

logic spi_en;
logic [7:0] spi_data_in;
logic [7:0] spi_data_out;
logic spi_done;

spi_master # (
  .CPOL(CPOL),
  .CPHA(CPHA)
)
spi_master_inst (
  .i_clk(i_clk),
  .i_rstn(i_rstn),
  .i_en(spi_en),
  .i_data(spi_data_in),
  .o_done(spi_done),
  .o_data(spi_data_out),
  .o_cs(o_cs),
  .o_sclk(o_sclk),
  .o_mosi(o_mosi),
  .i_miso(i_miso)
);

typedef enum logic [2:0] {
  ST_IDLE,
  ST_CMD_WRITE,
  ST_DATA_READ,
  ST_DATA_WRITE,
  ST_WAIT_HANDSHAKE
} state_t;

state_t state;
logic cmd_ready_int;
logic cmd_type_reg;
logic [4:0] cmd_nbytes_reg;
logic spi_done_prev;

assign o_cmd_ready = cmd_ready_int;


always_ff @(posedge i_clk or negedge i_rstn) begin
  if(!i_rstn) begin
    state <= ST_IDLE;
    spi_en <= 1'b0;
    spi_data_in <= 8'h00;
    cmd_ready_int <= 1'b1;
    o_data_valid <= 1'b0;
    o_data <= 8'h00;
    cmd_type_reg <= 1'b0;
    cmd_nbytes_reg <= '0;
    spi_done_prev <= 1'b0;
  end else begin
    spi_done_prev <= spi_done; // Store the previous value of spi_done for edge detection
    case (state)
      ST_IDLE: begin
        if(i_cmd_valid && cmd_ready_int) begin
          state <= ST_CMD_WRITE; // Move to command write state
          cmd_ready_int <= 1'b0;
          spi_data_in <= {i_cmd_type, i_cmd_addr[6:0]}; // Send the register address first and RW bit
          spi_en <= 1'b1;
          cmd_type_reg <= i_cmd_type;
          cmd_nbytes_reg <= i_cmd_nbytes;
        end else begin
          cmd_ready_int <= 1'b1;
          spi_en <= 1'b0;
          o_data_valid <= 1'b0;
        end
      end

      ST_CMD_WRITE: begin
        if(spi_done && !spi_done_prev) begin
          if(cmd_type_reg) begin
            state <= ST_DATA_READ; // Move to data read state for read operation
            spi_en <= 1'b1; // Enable SPI transfer for read operation
            spi_data_in <= 8'h00; // Assign dummy write data for read operation
          end else begin
            state <= ST_WAIT_HANDSHAKE; // Move to wait handshake state for write operation
          end
        end
      end

      ST_WAIT_HANDSHAKE: begin
        if(i_data_valid) begin
          state <= ST_DATA_WRITE; // Move to data write state
          spi_en <= 1'b1; // Enable SPI transfer for write operation
          spi_data_in <= i_data; // Send the data to write
        end
      end

      ST_DATA_READ: begin
        if(spi_done && !spi_done_prev) begin
          o_data_valid <= 1'b1; // Indicate that read data is valid
          o_data <= spi_data_out; // Capture the read data from SPI
          if(cmd_nbytes_reg == 0) begin
            state <= ST_IDLE; // Return to idle state after data transfer is complete
            spi_en <= 1'b0; // Disable SPI transfer
            cmd_ready_int <= 1'b1;
          end else begin
            state <= ST_DATA_READ; // Continue reading data
            cmd_nbytes_reg <= cmd_nbytes_reg - 1; // Decrement the number of bytes to transfer
            spi_en <= 1'b1; // Enable SPI transfer for next byte
            spi_data_in <= 8'h00; // Assign dummy write data for read operation
          end
        end else begin
          o_data_valid <= 1'b0; // Clear the data valid signal if not done
        end
      end

      ST_DATA_WRITE: begin
        if(spi_done && !spi_done_prev) begin
          if(cmd_nbytes_reg == 0) begin
            state <= ST_IDLE; // Return to idle state after data transfer is complete
            spi_en <= 1'b0; // Disable SPI transfer
            cmd_ready_int <= 1'b1;
          end else begin
            state <= ST_WAIT_HANDSHAKE; // Continue writing data
            cmd_nbytes_reg <= cmd_nbytes_reg - 1; // Decrement the number of bytes to transfer
          end
        end
      end

      default: begin
        state <= ST_IDLE; // Default to idle state
        spi_en <= 1'b0; // Disable SPI transfer
        cmd_ready_int <= 1'b1;
      end

    endcase
  end
end

endmodule