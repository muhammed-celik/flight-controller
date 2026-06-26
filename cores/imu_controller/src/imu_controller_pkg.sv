package imu_controller_pkg;
  // Define the MPU9250 Register Addresses
  localparam bit [6:0] MPU9250_WHO_AM_I = 7'h75;
  localparam bit [6:0] MPU9250_CFG_BASE = 7'h19;
  localparam bit [6:0] MPU9250_SENSOR_DATA_BASE = 7'h3B;
  localparam bit [6:0] MPU9250_GYRO_OFFSET_BASE = 7'h13;
  localparam bit [6:0] MPU9250_ACCEL_OFFSET_BASE = 7'h77;

  // Define the MPU9250 Register Data
  localparam bit [7:0] MPU9250_WHO_AM_I_DATA = 8'h71; // Expected WHO_AM_I response for MPU9250
  localparam bit [7:0] MPU9250_SMPLRT_DIV_DATA = 8'h00; // Sample Rate Divider
  localparam bit [7:0] MPU9250_CONFIG_DATA = 8'h03; // Configuration Register
  localparam bit [7:0] MPU9250_GYRO_CONFIG_DATA = 8'h18; // Gyroscope Configuration Register
  localparam bit [7:0] MPU9250_ACCEL_CONFIG_DATA = 8'h08; // Accelerometer Configuration Register
  localparam bit [7:0] MPU9250_ACCEL_CONFIG2_DATA = 8'h00; // Accelerometer Configuration 2 Register

  // Define the BMP280 Register Addresses
  localparam bit [6:0] BMP280_CHIP_ID = 7'h50;
  localparam bit [6:0] BMP280_RESET = 7'h60;
  localparam bit [6:0] BMP280_CTRL_MEAS = 7'h74;
  localparam bit [6:0] BMP280_CONFIG = 7'h75;
  localparam bit [6:0] BMP280_DATA_BASE = 7'h77;

  // Define the BMP280 Register Data
  localparam bit [7:0] BMP280_CHIP_ID_DATA = 8'h58; // Expected CHIP_ID response for BMP280
  localparam bit [7:0] BMP280_RESET_DATA = 8'hB6; // Reset Command
  localparam bit [7:0] BMP280_CTRL_MEAS_DATA = 8'h27; // Control and Measurement Register
  localparam bit [7:0] BMP280_CONFIG_DATA = 8'h00; // Configuration Register

endpackage