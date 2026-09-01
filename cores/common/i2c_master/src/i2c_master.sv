module i2c_master 
  import i2c_master_pkg::*;
(
  input logic i_clk,
  input logic i_rstn,
  // Top Level Interface
  input logic i_en,
  input logic [6:0] i_dev_addr,
  input logic i_rw,
  input logic [7:0] i_data,

  output logic o_done,
  output logic o_error,
  output logic [7:0] o_data,
  // I2C Pinout
  inout  tri   io_scl,
  inout  tri   io_sda
);

// Internal pin interface signals
logic sda_out, sda_in, sda_oe;

// Assign i2c sda pin
assign io_sda = (sda_oe & !sda_out) ? 1'b0 : 1'bz;
assign sda_in = io_sda;

// SCL Clock generation
logic i2c_clk_en, scl_wr_tick, scl_rd_tick;

i2c_clk_gen  i2c_clk_gen_inst (
  .i_clk(i_clk),
  .i_rstn(i_rstn),
  .i_i2c_en(i2c_clk_en),
  .io_scl(io_scl),
  .o_scl_wr_tick(scl_wr_tick),
  .o_scl_rd_tick(scl_rd_tick)
);

//State Machine
typedef enum logic [3:0] {
  ST_IDLE,
  ST_START,
  ST_DEV_ADDR_RW,
  ST_SLAVE_ACK_1,
  ST_REG_ADDR,
  ST_SLAVE_ACK_2,
  ST_WRITE_DATA,
  ST_MASTER_ACK,
  ST_RSTART,
  ST_READ_DATA,
  ST_STOP
} state_t;

state_t state;
logic [7:0] addr_rw_reg;
logic rw_reg;
logic [7:0] data_reg;
logic [2:0] bit_cntr;
logic done_int;
logic ack_error;
logic en_reg;

always_ff @(posedge i_clk or negedge i_rstn) begin
  if(!i_rstn) begin
    state <= ST_IDLE;
    sda_out <= 1'b1;
    addr_rw_reg <= 8'd0;
    rw_reg <= 1'b0;
    data_reg <= 8'b0;
    bit_cntr <= 3'd0;
    done_int <= 1'b0;
    ack_error <= 1'b0;
    i2c_clk_en <= 1'b0;
    en_reg <= 1'b0;
  end else begin
    case (state)
      ST_IDLE: begin
        done_int <= 1'b0;
        ack_error <= 1'b0;
        if(i_en) begin
          state <= ST_START;
          i2c_clk_en <= 1'b1;
          sda_out <= 1'b0;
          addr_rw_reg <= {i_dev_addr, i_rw};
          rw_reg <= i_rw;
          data_reg <= i_data;
        end
      end
      ST_START: begin
        if(scl_wr_tick) begin
          state <= ST_DEV_ADDR_RW;
          sda_out <= addr_rw_reg[7];
          //addr_rw_reg <= {addr_rw_reg[6:0],1'b0};
          bit_cntr <= 3'd0;
        end
      end
      ST_DEV_ADDR_RW: begin
        if(scl_wr_tick) begin
          if(bit_cntr == 3'd7) begin
            state <= ST_SLAVE_ACK_1;
            bit_cntr <= 3'd0;
          end else begin
            sda_out <= addr_rw_reg[6-bit_cntr];
            //addr_rw_reg <= {addr_rw_reg[6:0],1'b0};
            bit_cntr <= bit_cntr + 3'd1;
          end
        end
      end

      ST_SLAVE_ACK_1: begin
        if(scl_rd_tick) begin
          if(sda_in) begin
            // NACK received
            ack_error <= 1'b1;
            done_int <= 1'b1;
          end
        end

        if(scl_wr_tick) begin
          done_int <= 1'b0;
          if(ack_error) begin
            // NACK received, generate stop condition
            state <= ST_STOP;
            sda_out <= 1'b0; // Assert SDA low before generating stop condition
            ack_error <= 1'b0;
          end else if(rw_reg) begin // If read operation, go to read data state
            state <= ST_READ_DATA;
            data_reg <= 8'd0;
            bit_cntr <= 3'd0;
          end else begin
            state <= ST_REG_ADDR;
            sda_out <= data_reg[7];
            data_reg <= {data_reg[6:0],1'b0};
            bit_cntr <= 3'd0;
          end
        end
      end

      ST_REG_ADDR: begin
        if(scl_wr_tick) begin
          if(bit_cntr == 3'd7) begin
            state <= ST_SLAVE_ACK_2;
            bit_cntr <= 3'd0;
          end else begin
            sda_out <= data_reg[7];
            data_reg <= {data_reg[6:0],1'b0};
            bit_cntr <= bit_cntr + 3'd1;
          end
        end
      end

      ST_SLAVE_ACK_2: begin
        if(scl_rd_tick) begin
          done_int <= 1'b1;
          if(sda_in) begin
            // NACK received
            ack_error <= 1'b1;
          end
        end

        if(scl_wr_tick) begin
          done_int <= 1'b0;
          if(ack_error) begin
            // NACK received, generate stop condition
            state <= ST_STOP;
            sda_out <= 1'b0; // Assert SDA low before generating stop condition
            ack_error <= 1'b0;
          end else if(i_en && (addr_rw_reg == {i_dev_addr, i_rw})) begin // If the same device and operation is requested, continue writing (burst write)
            state <= ST_WRITE_DATA;
            sda_out <= i_data[7];
            data_reg <= {i_data[6:0],1'b0};
            bit_cntr <= 3'd0;
          end else if(i_en) begin // If a different device or operation is requested, generate repeated start condition
            state <= ST_RSTART;
            sda_out <= 1'b1; // Release SDA for repeated start
            addr_rw_reg <= {i_dev_addr, i_rw};
            rw_reg <= i_rw;
            data_reg <= i_data;
            bit_cntr <= 3'd0;
          end else begin // If no more data is to be written, generate stop condition
            state <= ST_STOP;
            sda_out <= 1'b0;
          end
        end
      end

      ST_READ_DATA: begin
        if(scl_rd_tick) begin
          data_reg <= {data_reg[6:0], sda_in};
          if(bit_cntr == 3'd7) begin
            done_int <= 1'b1;
            bit_cntr <= 3'd0;
          end else begin
            bit_cntr <= bit_cntr + 3'd1;
          end
        end

        if(scl_wr_tick) begin
          if(done_int) begin
            done_int <= 1'b0;
            en_reg <= i_en; // Store the value of i_en for the next cycle
            if(i_en && (addr_rw_reg == {i_dev_addr, i_rw})) begin // If the same device and operation is requested, continue reading (burst read), send the ACK to the slave
              state <= ST_MASTER_ACK;
              sda_out <= 1'b0; // Assert ACK
            end else if(i_en) begin // If a different device or operation is requested, generate repeated start condition after sending the NACK to the slave
              state <= ST_MASTER_ACK;
              addr_rw_reg <= {i_dev_addr, i_rw};
              rw_reg <= i_rw;
              data_reg <= i_data;
              sda_out <= 1'b1; // Assert NACK to indicate no more data is to be read
            end else begin // If no more data is to be read, send the NACK to the slave and generate stop condition
              state <= ST_MASTER_ACK;
              sda_out <= 1'b1; // Assert NACK
            end
          end
        end
      end

      ST_WRITE_DATA: begin
        if(scl_wr_tick) begin
          if(bit_cntr == 3'd7) begin
            state <= ST_SLAVE_ACK_2;
            bit_cntr <= 3'd0;
          end else begin
            sda_out <= data_reg[7];
            data_reg <= {data_reg[6:0],1'b0};
            bit_cntr <= bit_cntr + 3'd1;
          end
        end
      end
      
      ST_MASTER_ACK: begin
        if(scl_wr_tick) begin
          en_reg <= 1'b0; // Clear the stored value of i_en after processing
          if(en_reg && !sda_out) begin // If the same device and operation is requested, continue reading (burst read),
            state <= ST_READ_DATA;
          end else if(en_reg) begin // If a different device or operation is requested, generate repeated start condition
            state <= ST_RSTART;
            sda_out <= 1'b1; // Release SDA for repeated start
          end else begin // If no more data is to be read, generate stop condition
            state <= ST_STOP;
            sda_out <= 1'b0;
          end
        end
      end
      
      ST_STOP: begin
        if(scl_rd_tick) begin
          state <= ST_IDLE;
          sda_out <= 1'b1; // Release SDA to generate stop condition
          i2c_clk_en <= 1'b0; // Disable I2C clock generation
          ack_error <= 1'b0;
        end
      end

      ST_RSTART: begin
        if(scl_rd_tick) begin
          state <= ST_START;
          sda_out <= 1'b0; // Assert SDA for repeated start
          bit_cntr <= 3'd0;
        end
      end

      default: begin
        state <= ST_IDLE;
      end
    endcase
  end
end

assign sda_oe = (state == ST_START || state == ST_DEV_ADDR_RW || state == ST_REG_ADDR || state == ST_WRITE_DATA || state == ST_MASTER_ACK || state == ST_RSTART || state == ST_STOP);
assign o_data = data_reg;
assign o_done = done_int;
assign o_error = ack_error;

endmodule