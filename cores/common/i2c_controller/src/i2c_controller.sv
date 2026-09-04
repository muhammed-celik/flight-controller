module i2c_controller (
  input logic i_clk,
  input logic i_rstn,
  //Top Level Interface
  input logic i_cmd_valid,
  input logic i_cmd_type, // 1: Read, 0: Write
  input logic [4:0] i_cmd_nbytes, // Number of bytes to transfer (minimum 1 (5'b00000), maximum 32 (5'b11111))
  input logic [6:0] i_cmd_dev_addr, // Device Address
  input logic [7:0] i_cmd_reg_addr, // Register Address
  output logic o_cmd_ready, // Ready to accept new command
  output logic o_cmd_error, // Command Error

  input logic i_data_valid, // Register Data for Write is valid
  input logic [7:0] i_data, // Register Data for Write

  output logic o_data_valid, // Register Data for Read is valid
  output logic [7:0] o_data, // Register Data for Read
  //I2C Pinout
  inout tri io_sda,
  inout tri io_scl
);


logic i2c_en;
logic i2c_rw_reg;
logic [7:0] i2c_data_in;
logic [7:0] i2c_data_out;
logic i2c_done, i2c_error, i2c_busy;

assign o_cmd_error = i2c_error;

i2c_master  I2C_MASTER (
.i_clk(i_clk),
.i_rstn(i_rstn),
.i_en(i2c_en),
.i_dev_addr(i_cmd_dev_addr),
.i_rw(i2c_rw_reg),
.i_data(i2c_data_in),
.o_done(i2c_done),
.o_busy(i2c_busy),
.o_error(i2c_error),
.o_data(i2c_data_out),
.io_scl(io_scl),
.io_sda(io_sda)
);

typedef enum logic [2:0] {
  ST_IDLE,
  ST_ADDR_WRITE,
  ST_DATA_READ,
  ST_DATA_WRITE,
  ST_WAIT_HANDSHAKE
} state_t;

state_t state;
logic cmd_ready_int;
logic cmd_type_reg;
logic [4:0] cmd_nbytes_reg;
logic i2c_done_prev;

assign o_cmd_ready = cmd_ready_int && !i2c_busy; // Ready when internal ready and I2C master is not busy


always_ff @(posedge i_clk or negedge i_rstn) begin
  if(!i_rstn) begin
    state <= ST_IDLE;
    i2c_en <= 1'b0;
    i2c_data_in <= 8'h00;
    cmd_ready_int <= 1'b1;
    o_data_valid <= 1'b0;
    o_data <= 8'h00;
    cmd_type_reg <= 1'b0;
    i2c_rw_reg <= 1'b0;
    cmd_nbytes_reg <= '0;
    i2c_done_prev <= 1'b0;
  end else begin
    i2c_done_prev <= i2c_done; // Store the previous value of i2c_done for edge detection
    case (state)
      ST_IDLE: begin
        if(i_cmd_valid && cmd_ready_int && !i2c_busy) begin
          state <= ST_ADDR_WRITE; // Move to device address write state
          cmd_ready_int <= 1'b0;
          i2c_en <= 1'b1;
          i2c_rw_reg <= 1'b0; // First write the device address with write operation
          cmd_type_reg <= i_cmd_type;
          i2c_data_in <= i_cmd_reg_addr;
          cmd_nbytes_reg <= i_cmd_nbytes;
        end else begin
          cmd_ready_int <= 1'b1;
          i2c_en <= 1'b0;
          o_data_valid <= 1'b0;
        end
      end

      ST_ADDR_WRITE: begin
        if(i2c_done && !i2c_done_prev) begin
          if(i2c_error) begin
            state <= ST_IDLE; // Return to idle state on error
            cmd_ready_int <= 1'b1;
            i2c_en <= 1'b0;
          end else begin
            if(cmd_type_reg) begin
              state <= ST_DATA_READ; // Move to data read state for read operation
              i2c_en <= 1'b1;
              i2c_rw_reg <= 1'b1; // Set to read operation
            end else begin
              state <= ST_WAIT_HANDSHAKE; // Move to data write state for write operation
            end
          end
        end
      end

      ST_WAIT_HANDSHAKE: begin
        if(i_data_valid) begin
          state <= ST_DATA_WRITE; // Move to data write state
          i2c_en <= 1'b1; // Enable I2C transfer for write operation
          i2c_data_in <= i_data; // Send the data to write
        end
      end

      ST_DATA_READ: begin
        if(i2c_done && !i2c_done_prev) begin
          if(i2c_error) begin
            state <= ST_IDLE; // Return to idle state on error
            cmd_ready_int <= 1'b1;
            i2c_en <= 1'b0;
          end else begin
            o_data_valid <= 1'b1; // Indicate that read data is valid
            o_data <= i2c_data_out; // Output the read data
            if(cmd_nbytes_reg == 0) begin
              state <= ST_IDLE; // Return to idle state after data transfer is complete
              i2c_en <= 1'b0; // Disable I2C transfer
              cmd_ready_int <= 1'b1;
            end else begin
              state <= ST_DATA_READ; // Wait for handshake for next byte
              cmd_nbytes_reg <= cmd_nbytes_reg - 1; // Decrement the number of bytes to transfer
            end
          end
        end else begin
          o_data_valid <= 1'b0; // Clear data valid signal if not done
        end
      end

      ST_DATA_WRITE: begin
        if(i2c_done && !i2c_done_prev) begin
          if(i2c_error) begin
            state <= ST_IDLE; // Return to idle state on error
            cmd_ready_int <= 1'b1;
            i2c_en <= 1'b0;
          end else begin
            if(cmd_nbytes_reg == 0) begin
              state <= ST_IDLE; // Return to idle state after data transfer is complete
              i2c_en <= 1'b0; // Disable I2C transfer
              cmd_ready_int <= 1'b1;
            end else begin
              state <= ST_WAIT_HANDSHAKE; // Wait for handshake for next byte
              cmd_nbytes_reg <= cmd_nbytes_reg - 1; // Decrement the number of bytes to transfer
            end
          end
        end
      end

      default: begin
        state <= ST_IDLE; // Default to idle state
        i2c_en <= 1'b0; // Disable I2C transfer
        cmd_ready_int <= 1'b1;
      end

    endcase
  end
end

endmodule