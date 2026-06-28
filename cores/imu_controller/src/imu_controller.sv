module imu_controller 
  import imu_controller_pkg::*;
(
  input  logic i_clk,
  input  logic i_rstn,

  // SPI interface
  output logic o_ncs, // MPU9250
  output logic o_csb, // BMP280
  output logic o_mosi,
  input  logic i_miso,
  output logic o_sclk,

  // Error flags
  output logic o_err_mpu9250_id,
  output logic o_err_bmp280_id
);

//state machine
typedef enum logic [3:0] {
  ST_IDLE,
  ST_WAIT_FOR_INIT,
  ST_READ_MPU9250_ID,
  ST_READ_BMP280_ID,
  ST_WRITE_MPU9250_CONFIG,
  ST_WRITE_BMP280_CONFIG,
  ST_READ_MPU9250,
  ST_READ_BMP280,
  ST_WAIT_SPI_READY
} imu_state_t;

// Internal signals for SPI communication
logic [7:0] spi_wrt_data;
logic [7:0] spi_rd_data;
logic spi_trig, spi_rd_valid, spi_busy;
logic [1:0] sclk_sel;
logic spi_cs_sel;
logic [1:0] spi_cs;
logic spi_mosi, spi_miso, spi_sclk;

assign o_ncs = spi_cs[SPI_CS_MPU9250];
assign o_csb = spi_cs[SPI_CS_BMP280];
assign o_mosi = spi_mosi;
assign o_sclk = spi_sclk;
assign spi_miso = i_miso;

// Internal state machine for IMU controller
imu_state_t state;
logic [9:0] power_on_counter;
logic [3:0] spi_byte_num;

always_ff @(posedge i_clk or negedge i_rstn) begin
  if (!i_rstn) begin
    state <= ST_IDLE;
    power_on_counter <= 0;
    spi_byte_num <= 0;
    spi_trig <= 1'b0;
  end else begin
    case (state)
      ST_IDLE: begin
        state <= ST_WAIT_FOR_INIT;
        power_on_counter <= 0;
        spi_byte_num <= 0;
        spi_trig <= 1'b0;
        o_err_mpu9250_id <= 1'b0;
        o_err_bmp280_id <= 1'b0;
      end
      ST_WAIT_FOR_INIT: begin
        if(power_on_counter == 10'd1023) begin
          state <= ST_READ_MPU9250_ID;
          spi_trig <= 1'b1;
          spi_wrt_data <= {SPI_READ_CMD, MPU9250_WHO_AM_I_ADDR};
          power_on_counter <= 0;
        end else begin
          state <= ST_WAIT_FOR_INIT;
          power_on_counter <= power_on_counter + 1;
        end
      end
      ST_READ_MPU9250_ID: begin
        if(spi_rd_valid) begin
          if(spi_byte_num == 1) begin
            if(spi_rd_data == MPU9250_WHO_AM_I_DATA) begin
              state <= ST_READ_BMP280_ID;
              spi_trig <= 1'b1;
              spi_wrt_data <= {SPI_READ_CMD, BMP280_CHIP_ID_ADDR};
              spi_byte_num <= 0;
              o_err_mpu9250_id <= 1'b0;
            end else begin
              state <= ST_IDLE; // Error handling: go back to idle if ID doesn't match
              o_err_mpu9250_id <= 1'b1; // Set error flag for MPU9250 ID mismatch
            end
          end else begin
            state <= ST_READ_MPU9250_ID; // Wait for SPI read to complete
            spi_byte_num <= spi_byte_num + 1;
            spi_trig <= 1'b0;
          end
        end else begin
          state <= ST_READ_MPU9250_ID; // Wait for SPI read to complete
          spi_trig <= 1'b0;
        end
      end
      ST_READ_BMP280_ID: begin
        if(spi_rd_valid) begin
          if(spi_byte_num == 1) begin
            if(spi_rd_data == BMP280_CHIP_ID_DATA) begin
              state <= ST_WRITE_MPU9250_CONFIG;
              spi_trig <= 1'b1;
              spi_wrt_data <= {SPI_WRITE_CMD, MPU9250_CFG_BASE_ADDR, MPU9250_SMPLRT_DIV_DATA, MPU9250_CONFIG_DATA, MPU9250_GYRO_CONFIG_DATA, MPU9250_ACCEL_CONFIG_DATA, MPU9250_ACCEL_CONFIG2_DATA};
              o_err_bmp280_id <= 1'b0;
            end else begin
              state <= ST_IDLE; // Error handling: go back to idle if ID doesn't match
              o_err_bmp280_id <= 1'b1; // Set error flag for BMP280 ID mismatch
            end
          end else begin
            state <= ST_READ_BMP280_ID; // Wait for SPI read to complete
            spi_byte_num <= spi_byte_num + 1;
            spi_trig <= 1'b0;
          end
        end else begin
          state <= ST_READ_BMP280_ID; // Wait for SPI read to complete
          spi_trig <= 1'b0;
        end
      end
      ST_WRITE_MPU9250_CONFIG: begin
        state <= ST_WRITE_BMP280_CONFIG;
      end
      ST_WRITE_BMP280_CONFIG: begin
        state <= ST_READ_MPU9250;
      end
      ST_READ_MPU9250: begin
        state <= ST_READ_BMP280;
      end
      ST_READ_BMP280: begin
        state <= ST_IDLE; // Loop back to idle for continuous reading
      end
      default: begin
        state <= ST_IDLE;
      end
    endcase
  end
end

spi_master  spi_master_inst (
  .i_clk(i_clk),
  .i_rstn(i_rstn),
  .i_valid(spi_trig),
  .i_sclk_sel(sclk_sel),
  .i_cs_sel(spi_cs_sel),
  .i_data(spi_wrt_data),
  .o_valid(spi_rd_valid),
  .o_busy(spi_busy),
  .o_data(spi_rd_data),
  .o_cs(spi_cs),
  .o_mosi(spi_mosi),
  .i_miso(spi_miso),
  .o_sclk(spi_sclk)
);

endmodule