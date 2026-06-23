package spi_master_pkg;
  localparam int unsigned SYS_CLK_FREQ = 100_000_000;
  localparam bit          CPOL         = 1;
  localparam bit          CPHA         = 0;
  localparam int unsigned CS_CNT       = 2;

  localparam int unsigned SCLK_FREQ_0 = 1_000_000;
  localparam int unsigned SCLK_FREQ_1 = 2_000_000;
  localparam int unsigned SCLK_FREQ_2 = 5_000_000;
  localparam int unsigned SCLK_FREQ_3 = 10_000_000;

  localparam int unsigned SCLK_FREQ_0_LIM = SYS_CLK_FREQ/SCLK_FREQ_0;
  localparam int unsigned SCLK_FREQ_1_LIM = SYS_CLK_FREQ/SCLK_FREQ_1;
  localparam int unsigned SCLK_FREQ_2_LIM = SYS_CLK_FREQ/SCLK_FREQ_2;
  localparam int unsigned SCLK_FREQ_3_LIM = SYS_CLK_FREQ/SCLK_FREQ_3;

  localparam int unsigned SEL_MPU9250 = 0;
  localparam int unsigned SEL_BMP280  = 1;

  localparam int unsigned WAIT_CNTR_LIM = 2;

  // Timing values are stored as integer nanoseconds so that the SPI master can
  // convert them to i_clk cycles using ns_to_cycles_ceil(). Frequencies are Hz.
  // Both devices support SPI mode 0 and SPI mode 3.
  localparam logic [3:0] SPI_SENSOR_MODE_MASK = 4'b1001;

  // --------------------------------------------------------------------------
  // BMP280
  // Source: BST-BMP280-DS001-18, rev. 1.18, Table 28 (page 34).
  // These limits apply to both 3-wire and 4-wire SPI. This project uses 4-wire.
  // --------------------------------------------------------------------------
  localparam longint unsigned BMP280_SCLK_MAX_HZ            = 10_000_000;

  localparam int unsigned BMP280_SCLK_LOW_MIN_NS            = 20;
  localparam int unsigned BMP280_SCLK_HIGH_MIN_NS           = 20;
  localparam int unsigned BMP280_SDI_SETUP_MIN_NS           = 20;
  localparam int unsigned BMP280_SDI_HOLD_MIN_NS            = 20;
  localparam int unsigned BMP280_CSB_SETUP_MIN_NS           = 20;
  localparam int unsigned BMP280_CSB_HOLD_MIN_NS            = 20;

  // Maximum SDO delay for a 25 pF load. The GY-91 is operated at 3.3 V, so the
  // VDDIO >= 1.6 V value applies. The 1.2 V value is retained for completeness.
  localparam int unsigned BMP280_SDO_DELAY_MAX_NS           = 30;
  localparam int unsigned BMP280_SDO_DELAY_MAX_1V2_NS       = 40;

  // --------------------------------------------------------------------------
  // MPU-9250: standard register access
  // Source: PS-MPU-9250A-01, rev. 1.1, Table 7 (page 16).
  // Use this timing for writes and for reads that are not explicitly eligible
  // for the high-speed sensor/interrupt-register read mode.
  // --------------------------------------------------------------------------
  localparam longint unsigned MPU9250_SCLK_STD_MAX_HZ       = 1_000_000;

  localparam int unsigned MPU9250_SCLK_STD_LOW_MIN_NS       = 400;
  localparam int unsigned MPU9250_SCLK_STD_HIGH_MIN_NS      = 400;
  localparam int unsigned MPU9250_CS_STD_SETUP_MIN_NS       = 8;
  localparam int unsigned MPU9250_CS_STD_HOLD_MIN_NS        = 500;
  localparam int unsigned MPU9250_SDI_STD_SETUP_MIN_NS      = 11;
  localparam int unsigned MPU9250_SDI_STD_HOLD_MIN_NS       = 7;
  localparam int unsigned MPU9250_SDO_STD_VALID_MAX_NS      = 100;
  localparam int unsigned MPU9250_SDO_STD_HOLD_MIN_NS       = 4;
  localparam int unsigned MPU9250_SDO_STD_DISABLE_MAX_NS    = 50;

  // --------------------------------------------------------------------------
  // MPU-9250: high-speed reads
  // Source: PS-MPU-9250A-01, rev. 1.1, Table 8 (pages 16-17).
  // This mode is only for reading sensor and interrupt registers. Do not use it
  // for configuration writes or unrestricted register reads.
  // --------------------------------------------------------------------------
  localparam longint unsigned MPU9250_SCLK_FAST_MIN_HZ      = 900_000;
  localparam longint unsigned MPU9250_SCLK_FAST_MAX_HZ      = 20_000_000;

  // Table 8 does not specify separate minimum SCLK-high/SCLK-low pulse widths.
  localparam int unsigned MPU9250_CS_FAST_SETUP_MIN_NS      = 1;
  localparam int unsigned MPU9250_CS_FAST_HOLD_MIN_NS       = 1;
  localparam int unsigned MPU9250_SDI_FAST_SETUP_MIN_NS     = 0;
  localparam int unsigned MPU9250_SDI_FAST_HOLD_MIN_NS      = 1;
  localparam int unsigned MPU9250_SDO_FAST_VALID_MAX_NS     = 25;
  localparam int unsigned MPU9250_SDO_FAST_DISABLE_MAX_NS   = 25;

  // Convert a datasheet time in nanoseconds to an integer number of system
  // clock cycles, rounding upward. The result is suitable for constant/local
  // parameter calculations in synthesizable modules.
  function automatic longint unsigned ns_to_cycles_ceil(
    input longint unsigned time_ns,
    input longint unsigned clk_freq_hz
  );
    ns_to_cycles_ceil =
      ((time_ns * clk_freq_hz) + 1_000_000_000 - 1) / 1_000_000_000;
  endfunction

  // Number of system-clock cycles in one SPI half-period, rounded upward so
  // the generated SPI frequency never exceeds the requested frequency.
  function automatic longint unsigned spi_half_period_cycles_ceil(
    input longint unsigned sys_clk_freq_hz,
    input longint unsigned spi_clk_freq_hz
  );
    spi_half_period_cycles_ceil =
      (sys_clk_freq_hz + (2 * spi_clk_freq_hz) - 1) /
      (2 * spi_clk_freq_hz);
  endfunction

endpackage : spi_master_pkg
