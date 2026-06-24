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
  output logic o_sclk
);

//state machine
typedef enum logic [2:0] {
  ST_IDLE,
  ST_INIT_MPU9250,
  ST_INIT_BMP280,
  ST_READ_MPU9250,
  ST_READ_BMP280,
  ST_WRITE
} imu_state_t;

imu_state_t state;

endmodule