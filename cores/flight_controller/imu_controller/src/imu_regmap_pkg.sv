`ifndef IMU_REGMAP_PKG_SV
`define IMU_REGMAP_PKG_SV

// =============================================================================
// imu_regmap_pkg
//   Register map for the SparkFun 9DoF IMU breakout:
//     - ISM330DHCX  : 3-axis accelerometer + 3-axis gyroscope (SPI or I2C)
//     - MMC5983MA   : 3-axis magnetometer (I2C only)
//
//   Sources:
//     - ST ism330dhcx_reg.h / ism330dhcx_reg.c (STMems standard C driver)
//     - ST ISM330DHCX datasheet (DS13079)
//     - MEMSIC MMC5983MA datasheet Rev A
//     - SparkFun MMC5983MA Arduino library constants
//
//   Addresses are the raw 7-bit/8-bit register addresses (SPI read flag is
//   applied separately via ism_spi_read_addr()).
// =============================================================================

package imu_regmap_pkg;

  // ===========================================================================
  // ISM330DHCX - accelerometer + gyroscope
  // ===========================================================================

  // I2C slave address, selected by the SA0 pin strap (7-bit form).
  localparam logic [6:0] ISM_I2C_ADDR_SA0_LOW  = 7'h6A;  // SA0 = 0
  localparam logic [6:0] ISM_I2C_ADDR_SA0_HIGH = 7'h6B;  // SA0 = 1 (default)
  // WHO_AM_I (0x0F) fixed contents.
  localparam logic [7:0] ISM_WHO_AM_I_VALUE    = 8'h6B;
  // SPI: OR this into the register address byte for a read; write uses 0x00.
  localparam logic [7:0] ISM_SPI_READ_MASK     = 8'h80;

  // ---- Register addresses ---------------------------------------------------
  localparam logic [7:0]
    ISM_FIFO_CTRL1       = 8'h07,  // FIFO watermark threshold, bits [7:0]
    ISM_FIFO_CTRL2       = 8'h08,  // FIFO mode, watermark [10:8], timers
    ISM_FIFO_CTRL3       = 8'h09,  // FIFO decimation: gyro (bdr_g) / accel (bdr_xl)
    ISM_FIFO_CTRL4       = 8'h0A,  // FIFO decimation, stop-on-FTH, ODR-change, etc.
    ISM_COUNTER_BDR_REG1 = 8'h0B,  // Counter BDR trigger mode / counter BDR [10:8]
    ISM_COUNTER_BDR_REG2 = 8'h0C,  // Counter BDR [7:0]
    ISM_INT1_CTRL        = 8'h0D,  // Route data-ready / FIFO / boot events to INT1 pin
    ISM_INT2_CTRL        = 8'h0E,  // Route data-ready / FIFO / temp events to INT2 pin
    ISM_WHO_AM_I         = 8'h0F,  // Device identification, reads 0x6B
    ISM_CTRL1_XL         = 8'h10,  // Accelerometer ODR, full-scale, LPF2 enable
    ISM_CTRL2_G          = 8'h11,  // Gyroscope ODR, full-scale
    ISM_CTRL3_C          = 8'h12,  // SW reset, auto-increment, BDU, BOOT, INT polarity
    ISM_CTRL4_C          = 8'h13,  // Gyro LPF1 select, I2C disable, DRDY mask, sleep
    ISM_CTRL5_C          = 8'h14,  // Accelerometer / gyro self-test, output rounding
    ISM_CTRL6_C          = 8'h15,  // Accel LPF1 bandwidth, user-offset weight, DEN mode
    ISM_CTRL7_G          = 8'h16,  // Gyro high-pass filter, OIS, user offset on output
    ISM_CTRL8_XL         = 8'h17,  // Accel LPF2 / HP filter, 6D low-pass, fast settling
    ISM_CTRL9_XL         = 8'h18,  // device_conf, data-enable (DEN) per axis
    ISM_CTRL10_C         = 8'h19,  // Timestamp enable
    ISM_ALL_INT_SRC      = 8'h1A,  // All interrupt-source flags (read-only)
    ISM_STATUS_REG       = 8'h1E,  // Data-ready flags: XLDA / GDA / TDA
    ISM_OUT_TEMP_L       = 8'h20,  // Temperature output, low byte
    ISM_OUT_TEMP_H       = 8'h21,  // Temperature output, high byte
    ISM_OUTX_L_G         = 8'h22,  // Gyro X output, low byte
    ISM_OUTX_H_G         = 8'h23,  // Gyro X output, high byte
    ISM_OUTY_L_G         = 8'h24,  // Gyro Y output, low byte
    ISM_OUTY_H_G         = 8'h25,  // Gyro Y output, high byte
    ISM_OUTZ_L_G         = 8'h26,  // Gyro Z output, low byte
    ISM_OUTZ_H_G         = 8'h27,  // Gyro Z output, high byte
    ISM_OUTX_L_A         = 8'h28,  // Accel X output, low byte
    ISM_OUTX_H_A         = 8'h29,  // Accel X output, high byte
    ISM_OUTY_L_A         = 8'h2A,  // Accel Y output, low byte
    ISM_OUTY_H_A         = 8'h2B,  // Accel Y output, high byte
    ISM_OUTZ_L_A         = 8'h2C,  // Accel Z output, low byte
    ISM_OUTZ_H_A         = 8'h2D,  // Accel Z output, high byte
    ISM_FIFO_STATUS1     = 8'h3A,  // FIFO fill level, bits [7:0]
    ISM_FIFO_STATUS2     = 8'h3B,  // FIFO full/overrun/watermark, fill [10:8]
    ISM_FIFO_DATA_OUT_TAG= 8'h78,  // FIFO data tag (identifies sensor + count)
    ISM_FIFO_DATA_OUT_X_L= 8'h79,  // FIFO X output, low byte
    ISM_FIFO_DATA_OUT_X_H= 8'h7A,  // FIFO X output, high byte
    ISM_FIFO_DATA_OUT_Y_L= 8'h7B,  // FIFO Y output, low byte
    ISM_FIFO_DATA_OUT_Y_H= 8'h7C,  // FIFO Y output, high byte
    ISM_FIFO_DATA_OUT_Z_L= 8'h7D,  // FIFO Z output, low byte
    ISM_FIFO_DATA_OUT_Z_H= 8'h7E;  // FIFO Z output, high byte

  // Extended / advanced registers. NOTE: PAGE_SEL (0x02) aliases PIN_CTRL /
  // SENSOR_HUB_1, PAGE_ADDRESS (0x08) aliases FIFO_CTRL2, PAGE_VALUE (0x09)
  // aliases FIFO_CTRL3, and PAGE_RW (0x17) aliases CTRL8_XL. The mapping
  // changes when FUNC_CFG_ACCESS.reg_access is set.
  localparam logic [7:0]
    ISM_FUNC_CFG_ACCESS    = 8'h01,  // Enable embedded-function / sensor-hub bank access
    ISM_PAGE_SEL           = 8'h02,  // Embedded-function page select (see alias note)
    ISM_PAGE_ADDRESS       = 8'h08,  // Embedded-function page address (see alias note)
    ISM_PAGE_VALUE         = 8'h09,  // Embedded-function page value (see alias note)
    ISM_PAGE_RW            = 8'h17,  // Page read/write control (see alias note)
    ISM_WAKE_UP_SRC        = 8'h1B,  // Wake-up / free-fall event source (read-only)
    ISM_TAP_SRC            = 8'h1C,  // Single/double tap event source (read-only)
    ISM_D6D_SRC            = 8'h1D,  // 6D orientation event source (read-only)
    ISM_TIMESTAMP0         = 8'h40,  // Timestamp counter, byte 0 (LSB)
    ISM_TIMESTAMP1         = 8'h41,  // Timestamp counter, byte 1
    ISM_TIMESTAMP2         = 8'h42,  // Timestamp counter, byte 2
    ISM_TIMESTAMP3         = 8'h43,  // Timestamp counter, byte 3 (MSB)
    ISM_MD1_CFG            = 8'h5E,  // Route functional events to INT1
    ISM_MD2_CFG            = 8'h5F,  // Route functional events to INT2
    ISM_INTERNAL_FREQ_FINE = 8'h63,  // Internal clock frequency fine value (read-only)
    ISM_X_OFS_USR          = 8'h73,  // Signed 8-bit user offset, X axis
    ISM_Y_OFS_USR          = 8'h74,  // Signed 8-bit user offset, Y axis
    ISM_Z_OFS_USR          = 8'h75;  // Signed 8-bit user offset, Z axis

  // ---- Register field bit positions (LSB index) -----------------------------
  localparam int unsigned
    // CTRL1_XL (0x10): accelerometer control
    ISM_CTRL1_XL_ODR_POS      = 4,  // [7:4] output data rate
    ISM_CTRL1_XL_FS_POS       = 2,  // [3:2] full-scale selection
    ISM_CTRL1_XL_LPF2_POS     = 1,  // [1]   enable LPF2 on accelerometer
    // CTRL2_G (0x11): gyroscope control
    ISM_CTRL2_G_ODR_POS       = 4,  // [7:4] output data rate
    ISM_CTRL2_G_FS_POS        = 0,  // [3:0] full-scale selection
    // CTRL3_C (0x12): global control
    ISM_CTRL3_C_SW_RESET_POS  = 0,  // [0]   software reset (self-clearing)
    ISM_CTRL3_C_IF_INC_POS    = 2,  // [2]   auto-increment address on multi-byte access
    ISM_CTRL3_C_SIM_POS       = 3,  // [3]   SPI 3-wire / 4-wire select
    ISM_CTRL3_C_PP_OD_POS     = 4,  // [4]   push-pull / open-drain interrupt pins
    ISM_CTRL3_C_H_LACTIVE_POS = 5,  // [5]   interrupt pin active level
    ISM_CTRL3_C_BDU_POS       = 6,  // [6]   block data update (no torn L/H reads)
    ISM_CTRL3_C_BOOT_POS      = 7,  // [7]   reboot memory content (self-clearing)
    // CTRL4_C (0x13)
    ISM_CTRL4_C_LPF1_SEL_G_POS= 1,  // [1]   gyro LPF1 output select
    ISM_CTRL4_C_I2C_DISABLE_POS=2,  // [2]   disable I2C interface
    ISM_CTRL4_C_DRDY_MASK_POS = 3,  // [3]   mask data-ready / disable DRDY signal
    ISM_CTRL4_C_INT2_ON_INT1_POS=5, // [5]   route INT2 to INT1 pin
    ISM_CTRL4_C_SLEEP_G_POS   = 6,  // [6]   gyro sleep mode
    // CTRL5_C (0x14): self-test + rounding
    ISM_CTRL5_C_ST_XL_POS     = 0,  // [1:0] accelerometer self-test
    ISM_CTRL5_C_ST_G_POS      = 2,  // [3:2] gyroscope self-test
    ISM_CTRL5_C_ROUNDING_POS  = 5,  // [6:5] output rounding
    // CTRL6_C (0x15)
    ISM_CTRL6_C_FTYPE_POS     = 0,  // [2:0] accelerometer LPF1 bandwidth type
    ISM_CTRL6_C_USR_OFF_W_POS = 3,  // [3]   user-offset weight (1mg / 16mg per LSB)
    ISM_CTRL6_C_XL_HM_MODE_POS= 4,  // [4]   accel high-perf mode disable (1 = low power)
    ISM_CTRL6_C_DEN_MODE_POS  = 5,  // [7:5] DEN pin trigger/level mode
    // CTRL7_G (0x16)
    ISM_CTRL7_G_OIS_ON_POS    = 0,  // [0]   OIS / auxiliary SPI active
    ISM_CTRL7_G_USR_OFF_ON_OUT_POS=1,// [1]  apply user offset to output
    ISM_CTRL7_G_OIS_ON_EN_POS = 2,  // [2]   enable OIS
    ISM_CTRL7_G_HPM_G_POS     = 4,  // [5:4] gyro high-pass filter cutoff
    ISM_CTRL7_G_HP_EN_G_POS   = 6,  // [6]   gyro high-pass filter enable
    ISM_CTRL7_G_G_HM_MODE_POS = 7,  // [7]   gyro high-perf mode disable (1 = low power)
    // CTRL8_XL (0x17)
    ISM_CTRL8_XL_LP_6D_POS    = 0,  // [0]   low-pass filter on 6D interrupt
    ISM_CTRL8_XL_HP_SLOPE_POS = 2,  // [2]   slope / high-pass filter on accel
    ISM_CTRL8_XL_FASTSETTL_POS= 3,  // [3]   fast-settling mode
    ISM_CTRL8_XL_HP_REF_POS   = 4,  // [4]   high-pass filter reference mode
    ISM_CTRL8_XL_HPCF_POS     = 5,  // [7:5] HP / LPF2 cutoff
    // CTRL9_XL (0x18)
    ISM_CTRL9_XL_DEVICE_CONF_POS=1, // [1]   enable device configuration / DEN config
    ISM_CTRL9_XL_DEN_LH_POS   = 2,  // [2]   DEN active level
    ISM_CTRL9_XL_DEN_XL_G_POS = 3,  // [4:3] DEN source select
    ISM_CTRL9_XL_DEN_Z_POS    = 5,  // [5]   data-enable Z axis
    ISM_CTRL9_XL_DEN_Y_POS    = 6,  // [6]   data-enable Y axis
    ISM_CTRL9_XL_DEN_X_POS    = 7,  // [7]   data-enable X axis
    // CTRL10_C (0x19)
    ISM_CTRL10_C_TIMESTAMP_POS= 5,  // [5]   timestamp counter enable
    // INT1_CTRL (0x0D): route events to INT1
    ISM_INT1_DRDY_XL_POS      = 0,  // [0]   accel data-ready
    ISM_INT1_DRDY_G_POS       = 1,  // [1]   gyro data-ready
    ISM_INT1_BOOT_POS         = 2,  // [2]   boot end
    ISM_INT1_FIFO_TH_POS      = 3,  // [3]   FIFO watermark threshold
    ISM_INT1_FIFO_OVR_POS     = 4,  // [4]   FIFO overrun
    ISM_INT1_FIFO_FULL_POS    = 5,  // [5]   FIFO full
    ISM_INT1_CNT_BDR_POS      = 6,  // [6]   counter BDR
    ISM_INT1_DEN_DRDY_POS     = 7,  // [7]   DEN data-ready (when DEN active)
    // INT2_CTRL (0x0E): route events to INT2
    ISM_INT2_DRDY_XL_POS      = 0,  // [0]   accel data-ready
    ISM_INT2_DRDY_G_POS       = 1,  // [1]   gyro data-ready
    ISM_INT2_DRDY_TEMP_POS    = 2,  // [2]   temperature data-ready
    ISM_INT2_FIFO_TH_POS      = 3,  // [3]   FIFO watermark threshold
    ISM_INT2_FIFO_OVR_POS     = 4,  // [4]   FIFO overrun
    ISM_INT2_FIFO_FULL_POS    = 5,  // [5]   FIFO full
    ISM_INT2_CNT_BDR_POS      = 6,  // [6]   counter BDR
    // STATUS_REG (0x1E): data-ready flags (read-only)
    ISM_STATUS_XLDA_POS       = 0,  // [0]   accel new data available
    ISM_STATUS_GDA_POS        = 1,  // [1]   gyro new data available
    ISM_STATUS_TDA_POS        = 2,  // [2]   temperature new data available
    // FUNC_CFG_ACCESS (0x01)
    ISM_FUNC_CFG_SHUB_POS     = 6,  // [6]   sensor-hub register bank access
    ISM_FUNC_CFG_EMB_POS      = 7,  // [7]   embedded-function register bank access
    // PAGE_SEL (0x02)
    ISM_PAGE_SEL_POS          = 4,  // [7:4] embedded-function page number
    // PAGE_RW (0x17)
    ISM_PAGE_RW_RW_POS        = 5,  // [6:5] page read/write (write + read enables)
    ISM_PAGE_RW_LIR_POS       = 7,  // [7]   latched interrupt request
    // WAKE_UP_SRC (0x1B): read-only event flags
    ISM_WU_Z_POS              = 0,  // [0]   Z-axis wake-up event
    ISM_WU_Y_POS              = 1,  // [1]   Y-axis wake-up event
    ISM_WU_X_POS              = 2,  // [2]   X-axis wake-up event
    ISM_WU_IA_POS             = 3,  // [3]   wake-up interrupt active
    ISM_WU_SLEEP_STATE_POS    = 4,  // [4]   sleep state
    ISM_WU_FF_IA_POS          = 5,  // [5]   free-fall interrupt active
    ISM_WU_SLEEP_CHG_POS      = 6,  // [6]   sleep-state change
    // TAP_SRC (0x1C): read-only event flags
    ISM_TAP_Z_POS             = 0,  // [0]   Z-axis tap
    ISM_TAP_Y_POS             = 1,  // [1]   Y-axis tap
    ISM_TAP_X_POS             = 2,  // [2]   X-axis tap
    ISM_TAP_SIGN_POS          = 3,  // [3]   tap direction sign
    ISM_TAP_DOUBLE_POS        = 4,  // [4]   double tap
    ISM_TAP_SINGLE_POS        = 5,  // [5]   single tap
    ISM_TAP_IA_POS            = 6,  // [6]   tap interrupt active
    // D6D_SRC (0x1D): read-only 6D orientation
    ISM_D6D_XL_POS            = 0,  // [0]   X low
    ISM_D6D_XH_POS            = 1,  // [1]   X high
    ISM_D6D_YL_POS            = 2,  // [2]   Y low
    ISM_D6D_YH_POS            = 3,  // [3]   Y high
    ISM_D6D_ZL_POS            = 4,  // [4]   Z low
    ISM_D6D_ZH_POS            = 5,  // [5]   Z high
    ISM_D6D_IA_POS            = 6,  // [6]   6D interrupt active
    ISM_D6D_DEN_DRDY_POS      = 7,  // [7]   DEN data-ready
    // MD1_CFG (0x5E): route functional events to INT1
    ISM_MD1_SHUB_POS          = 0,  // [0]   sensor-hub event
    ISM_MD1_EMB_FUNC_POS      = 1,  // [1]   embedded-function event
    ISM_MD1_6D_POS            = 2,  // [2]   6D orientation event
    ISM_MD1_DOUBLE_TAP_POS    = 3,  // [3]   double-tap event
    ISM_MD1_FF_POS            = 4,  // [4]   free-fall event
    ISM_MD1_WU_POS            = 5,  // [5]   wake-up event
    ISM_MD1_SINGLE_TAP_POS    = 6,  // [6]   single-tap event
    ISM_MD1_SLEEP_CHG_POS     = 7,  // [7]   sleep-change event
    // MD2_CFG (0x5F): route functional events to INT2
    ISM_MD2_TIMESTAMP_POS     = 0,  // [0]   timestamp event
    ISM_MD2_EMB_FUNC_POS      = 1,  // [1]   embedded-function event
    ISM_MD2_6D_POS            = 2,  // [2]   6D orientation event
    ISM_MD2_DOUBLE_TAP_POS    = 3,  // [3]   double-tap event
    ISM_MD2_FF_POS            = 4,  // [4]   free-fall event
    ISM_MD2_WU_POS            = 5,  // [5]   wake-up event
    ISM_MD2_SINGLE_TAP_POS    = 6,  // [6]   single-tap event
    ISM_MD2_SLEEP_CHG_POS     = 7;  // [7]   sleep-change event

  // ---- Register field masks (same bit ranges as the positions above) -------
  localparam logic [7:0]
    ISM_CTRL1_XL_ODR_MSK      = 8'hF0,  // [7:4] ODR
    ISM_CTRL1_XL_FS_MSK       = 8'h0C,  // [3:2] full-scale
    ISM_CTRL1_XL_LPF2_MSK     = 8'h02,  // [1]   LPF2 enable
    ISM_CTRL2_G_ODR_MSK       = 8'hF0,  // [7:4] ODR
    ISM_CTRL2_G_FS_MSK        = 8'h0F,  // [3:0] full-scale
    ISM_CTRL3_C_SW_RESET_MSK  = 8'h01,  // [0]   SW reset
    ISM_CTRL3_C_IF_INC_MSK    = 8'h04,  // [2]   auto-increment
    ISM_CTRL3_C_SIM_MSK       = 8'h08,  // [3]   SPI mode
    ISM_CTRL3_C_PP_OD_MSK     = 8'h10,  // [4]   push-pull/open-drain
    ISM_CTRL3_C_H_LACTIVE_MSK = 8'h20,  // [5]   INT active level
    ISM_CTRL3_C_BDU_MSK       = 8'h40,  // [6]   block data update
    ISM_CTRL3_C_BOOT_MSK      = 8'h80,  // [7]   reboot
    ISM_CTRL4_C_LPF1_SEL_G_MSK= 8'h02,  // [1]   gyro LPF1 select
    ISM_CTRL4_C_I2C_DISABLE_MSK=8'h04,  // [2]   I2C disable
    ISM_CTRL4_C_DRDY_MASK_MSK = 8'h08,  // [3]   DRDY mask
    ISM_CTRL4_C_INT2_ON_INT1_MSK=8'h20, // [5]   INT2 on INT1
    ISM_CTRL4_C_SLEEP_G_MSK   = 8'h40,  // [6]   gyro sleep
    ISM_CTRL5_C_ST_XL_MSK     = 8'h03,  // [1:0] accel self-test
    ISM_CTRL5_C_ST_G_MSK      = 8'h0C,  // [3:2] gyro self-test
    ISM_CTRL5_C_ROUNDING_MSK  = 8'h60,  // [6:5] rounding
    ISM_CTRL6_C_FTYPE_MSK     = 8'h07,  // [2:0] accel LPF1 bandwidth
    ISM_CTRL6_C_USR_OFF_W_MSK = 8'h08,  // [3]   user-offset weight
    ISM_CTRL6_C_XL_HM_MODE_MSK= 8'h10,  // [4]   accel low-power
    ISM_CTRL6_C_DEN_MODE_MSK  = 8'hE0,  // [7:5] DEN mode
    ISM_CTRL7_G_OIS_ON_MSK    = 8'h01,  // [0]   OIS active
    ISM_CTRL7_G_USR_OFF_ON_OUT_MSK=8'h02,// [1] user offset on output
    ISM_CTRL7_G_OIS_ON_EN_MSK = 8'h04,  // [2]   OIS enable
    ISM_CTRL7_G_HPM_G_MSK     = 8'h30,  // [5:4] gyro HP cutoff
    ISM_CTRL7_G_HP_EN_G_MSK   = 8'h40,  // [6]   gyro HP enable
    ISM_CTRL7_G_G_HM_MODE_MSK = 8'h80,  // [7]   gyro low-power
    ISM_CTRL8_XL_LP_6D_MSK    = 8'h01,  // [0]   LPF on 6D
    ISM_CTRL8_XL_HP_SLOPE_MSK = 8'h04,  // [2]   HP/slope
    ISM_CTRL8_XL_FASTSETTL_MSK= 8'h08,  // [3]   fast settling
    ISM_CTRL8_XL_HP_REF_MSK   = 8'h10,  // [4]   HP reference
    ISM_CTRL8_XL_HPCF_MSK     = 8'hE0,  // [7:5] HP/LPF2 cutoff
    ISM_CTRL9_XL_DEVICE_CONF_MSK=8'h02, // [1]   device configuration
    ISM_CTRL9_XL_DEN_LH_MSK   = 8'h04,  // [2]   DEN level
    ISM_CTRL9_XL_DEN_XL_G_MSK = 8'h18,  // [4:3] DEN source
    ISM_CTRL9_XL_DEN_Z_MSK    = 8'h20,  // [5]   DEN Z
    ISM_CTRL9_XL_DEN_Y_MSK    = 8'h40,  // [6]   DEN Y
    ISM_CTRL9_XL_DEN_X_MSK    = 8'h80,  // [7]   DEN X
    ISM_CTRL10_C_TIMESTAMP_MSK= 8'h20,  // [5]   timestamp enable
    ISM_INT1_DRDY_XL_MSK      = 8'h01,  // [0]   accel DRDY to INT1
    ISM_INT1_DRDY_G_MSK       = 8'h02,  // [1]   gyro DRDY to INT1
    ISM_INT1_BOOT_MSK         = 8'h04,  // [2]   boot to INT1
    ISM_INT1_FIFO_TH_MSK      = 8'h08,  // [3]   FIFO threshold to INT1
    ISM_INT1_FIFO_OVR_MSK     = 8'h10,  // [4]   FIFO overrun to INT1
    ISM_INT1_FIFO_FULL_MSK    = 8'h20,  // [5]   FIFO full to INT1
    ISM_INT1_CNT_BDR_MSK      = 8'h40,  // [6]   counter BDR to INT1
    ISM_INT1_DEN_DRDY_MSK     = 8'h80,  // [7]   DEN DRDY to INT1
    ISM_INT2_DRDY_XL_MSK      = 8'h01,  // [0]   accel DRDY to INT2
    ISM_INT2_DRDY_G_MSK       = 8'h02,  // [1]   gyro DRDY to INT2
    ISM_INT2_DRDY_TEMP_MSK    = 8'h04,  // [2]   temp DRDY to INT2
    ISM_INT2_FIFO_TH_MSK      = 8'h08,  // [3]   FIFO threshold to INT2
    ISM_INT2_FIFO_OVR_MSK     = 8'h10,  // [4]   FIFO overrun to INT2
    ISM_INT2_FIFO_FULL_MSK    = 8'h20,  // [5]   FIFO full to INT2
    ISM_INT2_CNT_BDR_MSK      = 8'h40,  // [6]   counter BDR to INT2
    ISM_STATUS_XLDA_MSK       = 8'h01,  // [0]   accel data ready
    ISM_STATUS_GDA_MSK        = 8'h02,  // [1]   gyro data ready
    ISM_STATUS_TDA_MSK        = 8'h04,  // [2]   temp data ready
    ISM_FUNC_CFG_SHUB_MSK     = 8'h40,  // [6]   sensor-hub access
    ISM_FUNC_CFG_EMB_MSK      = 8'h80,  // [7]   embedded-function access
    ISM_PAGE_SEL_MSK          = 8'hF0,  // [7:4] page number
    ISM_PAGE_RW_RW_MSK        = 8'h60,  // [6:5] page read/write
    ISM_PAGE_RW_LIR_MSK       = 8'h80,  // [7]   latched interrupt
    ISM_WU_Z_MSK              = 8'h01,  // [0]   Z wake-up
    ISM_WU_Y_MSK              = 8'h02,  // [1]   Y wake-up
    ISM_WU_X_MSK              = 8'h04,  // [2]   X wake-up
    ISM_WU_IA_MSK             = 8'h08,  // [3]   wake-up IA
    ISM_WU_SLEEP_STATE_MSK    = 8'h10,  // [4]   sleep state
    ISM_WU_FF_IA_MSK          = 8'h20,  // [5]   free-fall IA
    ISM_WU_SLEEP_CHG_MSK      = 8'h40,  // [6]   sleep change
    ISM_TAP_Z_MSK             = 8'h01,  // [0]   Z tap
    ISM_TAP_Y_MSK             = 8'h02,  // [1]   Y tap
    ISM_TAP_X_MSK             = 8'h04,  // [2]   X tap
    ISM_TAP_SIGN_MSK          = 8'h08,  // [3]   tap sign
    ISM_TAP_DOUBLE_MSK        = 8'h10,  // [4]   double tap
    ISM_TAP_SINGLE_MSK        = 8'h20,  // [5]   single tap
    ISM_TAP_IA_MSK            = 8'h40,  // [6]   tap IA
    ISM_D6D_XL_MSK            = 8'h01,  // [0]   X low
    ISM_D6D_XH_MSK            = 8'h02,  // [1]   X high
    ISM_D6D_YL_MSK            = 8'h04,  // [2]   Y low
    ISM_D6D_YH_MSK            = 8'h08,  // [3]   Y high
    ISM_D6D_ZL_MSK            = 8'h10,  // [4]   Z low
    ISM_D6D_ZH_MSK            = 8'h20,  // [5]   Z high
    ISM_D6D_IA_MSK            = 8'h40,  // [6]   6D IA
    ISM_D6D_DEN_DRDY_MSK      = 8'h80,  // [7]   DEN DRDY
    ISM_MD1_SHUB_MSK          = 8'h01,  // [0]   sensor hub to INT1
    ISM_MD1_EMB_FUNC_MSK      = 8'h02,  // [1]   embedded function to INT1
    ISM_MD1_6D_MSK            = 8'h04,  // [2]   6D to INT1
    ISM_MD1_DOUBLE_TAP_MSK    = 8'h08,  // [3]   double tap to INT1
    ISM_MD1_FF_MSK            = 8'h10,  // [4]   free fall to INT1
    ISM_MD1_WU_MSK            = 8'h20,  // [5]   wake-up to INT1
    ISM_MD1_SINGLE_TAP_MSK    = 8'h40,  // [6]   single tap to INT1
    ISM_MD1_SLEEP_CHG_MSK     = 8'h80,  // [7]   sleep change to INT1
    ISM_MD2_TIMESTAMP_MSK     = 8'h01,  // [0]   timestamp to INT2
    ISM_MD2_EMB_FUNC_MSK      = 8'h02,  // [1]   embedded function to INT2
    ISM_MD2_6D_MSK            = 8'h04,  // [2]   6D to INT2
    ISM_MD2_DOUBLE_TAP_MSK    = 8'h08,  // [3]   double tap to INT2
    ISM_MD2_FF_MSK            = 8'h10,  // [4]   free fall to INT2
    ISM_MD2_WU_MSK            = 8'h20,  // [5]   wake-up to INT2
    ISM_MD2_SINGLE_TAP_MSK    = 8'h40,  // [6]   single tap to INT2
    ISM_MD2_SLEEP_CHG_MSK     = 8'h80;  // [7]   sleep change to INT2

  // ---- Field value encodings ------------------------------------------------
  // Accelerometer output data rate (CTRL1_XL.odr_xl).
  typedef enum logic [3:0] {
    ISM_XL_ODR_OFF    = 4'd0,   // power-down
    ISM_XL_ODR_12HZ5  = 4'd1,
    ISM_XL_ODR_26HZ   = 4'd2,
    ISM_XL_ODR_52HZ   = 4'd3,
    ISM_XL_ODR_104HZ  = 4'd4,
    ISM_XL_ODR_208HZ  = 4'd5,
    ISM_XL_ODR_416HZ  = 4'd6,
    ISM_XL_ODR_833HZ  = 4'd7,
    ISM_XL_ODR_1666HZ = 4'd8,
    ISM_XL_ODR_3332HZ = 4'd9,
    ISM_XL_ODR_6667HZ = 4'd10,
    ISM_XL_ODR_1HZ6   = 4'd11   // low-power only
  } ism_xl_odr_e;

  // Gyroscope output data rate (CTRL2_G.odr_g).
  typedef enum logic [3:0] {
    ISM_G_ODR_OFF    = 4'd0,   // power-down
    ISM_G_ODR_12HZ5  = 4'd1,
    ISM_G_ODR_26HZ   = 4'd2,
    ISM_G_ODR_52HZ   = 4'd3,
    ISM_G_ODR_104HZ  = 4'd4,
    ISM_G_ODR_208HZ  = 4'd5,
    ISM_G_ODR_416HZ  = 4'd6,
    ISM_G_ODR_833HZ  = 4'd7,
    ISM_G_ODR_1666HZ = 4'd8,
    ISM_G_ODR_3332HZ = 4'd9,
    ISM_G_ODR_6667HZ = 4'd10
  } ism_g_odr_e;

  // Accelerometer full-scale (CTRL1_XL.fs_xl); comment = sensitivity.
  typedef enum logic [1:0] {
    ISM_XL_FS_2G  = 2'd0,  // 0.061 mg/LSB
    ISM_XL_FS_16G = 2'd1,  // 0.488 mg/LSB
    ISM_XL_FS_4G  = 2'd2,  // 0.122 mg/LSB
    ISM_XL_FS_8G  = 2'd3   // 0.244 mg/LSB
  } ism_xl_fs_e;

  // Gyroscope full-scale (CTRL2_G.fs_g); comment = sensitivity.
  // Encodings are non-monotonic (fs_4000/fs_125 share the field).
  typedef enum logic [3:0] {
    ISM_G_FS_250DPS  = 4'h0,  // 8.75  mdps/LSB
    ISM_G_FS_125DPS  = 4'h2,  // 4.375 mdps/LSB
    ISM_G_FS_500DPS  = 4'h4,  // 17.50 mdps/LSB
    ISM_G_FS_1000DPS = 4'h8,  // 35    mdps/LSB
    ISM_G_FS_2000DPS = 4'hC,  // 70    mdps/LSB
    ISM_G_FS_4000DPS = 4'h1   // 140   mdps/LSB
  } ism_g_fs_e;

  // Accelerometer self-test (CTRL5_C.st_xl).
  typedef enum logic [1:0] {
    ISM_XL_ST_DISABLE  = 2'd0,
    ISM_XL_ST_POSITIVE = 2'd1,
    ISM_XL_ST_NEGATIVE = 2'd2
  } ism_xl_st_e;

  // Gyroscope self-test (CTRL5_C.st_g).
  typedef enum logic [1:0] {
    ISM_G_ST_DISABLE  = 2'd0,
    ISM_G_ST_POSITIVE = 2'd1,
    ISM_G_ST_NEGATIVE = 2'd3
  } ism_g_st_e;

  // User-offset weight (CTRL6_C.usr_off_w).
  typedef enum logic {
    ISM_USR_OFF_W_1MG  = 1'b0,  // ~1 mg per LSB
    ISM_USR_OFF_W_16MG = 1'b1   // ~16 mg per LSB
  } ism_usr_off_w_e;

  // Axis selector for helper functions.
  typedef enum logic [1:0] {
    ISM_AXIS_X = 2'd0,
    ISM_AXIS_Y = 2'd1,
    ISM_AXIS_Z = 2'd2
  } ism_axis_e;

  // ===========================================================================
  // MMC5983MA - magnetometer (I2C only)
  // WARNING: INT_CTRL_0 (0x09), INT_CTRL_2 (0x0B), INT_CTRL_3 (0x0C) are
  //          WRITE-ONLY; INT_CTRL_1 (0x0A) reads reserved bits as '1'.
  //          Keep a shadow byte in RTL and write full bytes (no read-modify-write).
  // ===========================================================================

  localparam logic [6:0] MMC_I2C_ADDR      = 7'h30;  // fixed 7-bit I2C address
  localparam logic [7:0] MMC_PROD_ID_VALUE = 8'h30;  // PROD_ID (0x2F) contents

  // ---- Register addresses ---------------------------------------------------
  localparam logic [7:0]
    MMC_X_OUT_0   = 8'h00,  // X output [17:10]
    MMC_X_OUT_1   = 8'h01,  // X output [9:2]
    MMC_Y_OUT_0   = 8'h02,  // Y output [17:10]
    MMC_Y_OUT_1   = 8'h03,  // Y output [9:2]
    MMC_Z_OUT_0   = 8'h04,  // Z output [17:10]
    MMC_Z_OUT_1   = 8'h05,  // Z output [9:2]
    MMC_XYZ_OUT_2 = 8'h06,  // X/Y/Z output [1:0] (bits [7:6]/[5:4]/[3:2])
    MMC_T_OUT     = 8'h07,  // Temperature output, ~0.8 degC/LSB, 0 = -75 degC
    MMC_STATUS    = 8'h08,  // Measurement / OTP status flags
    MMC_INT_CTRL_0= 8'h09,  // Measurement trigger, SET/RESET, auto-SR, OTP (write-only)
    MMC_INT_CTRL_1= 8'h0A,  // Software reset, bandwidth, channel inhibit (write-only*)
    MMC_INT_CTRL_2= 8'h0B,  // Continuous mode: CM_FREQ, CMM_EN, periodic SET (write-only)
    MMC_INT_CTRL_3= 8'h0C,  // Self-test enable, SPI 3-wire (write-only)
    MMC_PROD_ID   = 8'h2F;  // Product ID, reads 0x30

  // ---- Register field bit positions -----------------------------------------
  localparam int unsigned
    // STATUS (0x08): read/write flags (writing 1 clears the interrupt)
    MMC_STATUS_MEAS_M_DONE_POS  = 0,  // [0] magnetic measurement complete
    MMC_STATUS_MEAS_T_DONE_POS  = 1,  // [1] temperature measurement complete
    MMC_STATUS_OTP_READ_DONE_POS= 4,  // [4] OTP memory read complete
    // INT_CTRL_0 (0x09): write-only
    MMC_IC0_TM_M_POS            = 0,  // [0] trigger magnetic measurement (self-clearing)
    MMC_IC0_TM_T_POS            = 1,  // [1] trigger temperature measurement (self-clearing)
    MMC_IC0_INT_MEAS_DONE_EN_POS= 2,  // [2] enable measurement-done interrupt
    MMC_IC0_SET_POS             = 3,  // [3] SET operation (self-clearing, ~500 ns)
    MMC_IC0_RESET_POS           = 4,  // [4] RESET operation (self-clearing, ~500 ns)
    MMC_IC0_AUTO_SR_EN_POS      = 5,  // [5] enable automatic SET/RESET
    MMC_IC0_OTP_READ_POS        = 6,  // [6] re-read OTP (self-clearing)
    // INT_CTRL_1 (0x0A): write-only, reads back reserved bits as 1
    MMC_IC1_BW0_POS             = 0,  // [0] filter bandwidth select, bit 0
    MMC_IC1_BW1_POS             = 1,  // [1] filter bandwidth select, bit 1
    MMC_IC1_X_INHIBIT_POS       = 2,  // [2] disable X channel
    MMC_IC1_YZ_INHIBIT_POS      = 3,  // [4:3] disable Y and Z channels
    MMC_IC1_SW_RST_POS          = 7,  // [7] software reset (10 ms power-on time)
    // INT_CTRL_2 (0x0B): write-only
    MMC_IC2_CM_FREQ_POS         = 0,  // [2:0] continuous-measurement frequency
    MMC_IC2_CMM_EN_POS          = 3,  // [3] enable continuous measurement mode
    MMC_IC2_PRD_SET_POS         = 4,  // [6:4] periodic SET interval (number of samples)
    MMC_IC2_EN_PRD_SET_POS      = 7,  // [7] enable periodic SET
    // INT_CTRL_3 (0x0C): write-only
    MMC_IC3_ST_ENP_POS          = 1,  // [1] self-test positive (extra coil current)
    MMC_IC3_ST_ENM_POS          = 2,  // [2] self-test negative (extra coil current)
    MMC_IC3_SPI_3W_POS          = 6;  // [6] enable SPI 3-wire mode

  // ---- Register field masks -------------------------------------------------
  localparam logic [7:0]
    MMC_STATUS_MEAS_M_DONE_MSK  = 8'h01,  // [0] magnetic done
    MMC_STATUS_MEAS_T_DONE_MSK  = 8'h02,  // [1] temperature done
    MMC_STATUS_OTP_READ_DONE_MSK= 8'h10,  // [4] OTP read done
    MMC_IC0_TM_M_MSK            = 8'h01,  // [0] start mag measurement
    MMC_IC0_TM_T_MSK            = 8'h02,  // [1] start temp measurement
    MMC_IC0_INT_MEAS_DONE_EN_MSK= 8'h04,  // [2] measurement-done IRQ enable
    MMC_IC0_SET_MSK             = 8'h08,  // [3] SET
    MMC_IC0_RESET_MSK           = 8'h10,  // [4] RESET
    MMC_IC0_AUTO_SR_EN_MSK      = 8'h20,  // [5] auto SET/RESET enable
    MMC_IC0_OTP_READ_MSK        = 8'h40,  // [6] OTP re-read
    MMC_IC1_BW0_MSK             = 8'h01,  // [0] bandwidth bit 0
    MMC_IC1_BW1_MSK             = 8'h02,  // [1] bandwidth bit 1
    MMC_IC1_X_INHIBIT_MSK       = 8'h04,  // [2] X inhibit
    MMC_IC1_YZ_INHIBIT_MSK      = 8'h18,  // [4:3] Y/Z inhibit
    MMC_IC1_SW_RST_MSK          = 8'h80,  // [7] software reset
    MMC_IC2_CM_FREQ_MSK         = 8'h07,  // [2:0] continuous frequency
    MMC_IC2_CMM_EN_MSK          = 8'h08,  // [3] continuous mode enable
    MMC_IC2_PRD_SET_MSK         = 8'h70,  // [6:4] periodic SET samples
    MMC_IC2_EN_PRD_SET_MSK      = 8'h80,  // [7] periodic SET enable
    MMC_IC3_ST_ENP_MSK          = 8'h02,  // [1] self-test positive
    MMC_IC3_ST_ENM_MSK          = 8'h04,  // [2] self-test negative
    MMC_IC3_SPI_3W_MSK          = 8'h40;  // [6] SPI 3-wire

  // ---- Field value encodings ------------------------------------------------
  // Filter bandwidth (INT_CTRL_1.BW1:BW0); comment = measurement time.
  typedef enum logic [1:0] {
    MMC_BW_100HZ = 2'b00,  // 8 ms
    MMC_BW_200HZ = 2'b01,  // 4 ms
    MMC_BW_400HZ = 2'b10,  // 2 ms
    MMC_BW_800HZ = 2'b11   // 0.5 ms
  } mmc_bw_e;

  // Continuous-measurement frequency (INT_CTRL_2.CM_FREQ). Values measured
  // assuming BW[1:0] = 00; CMM_EN requires CM_FREQ != 000.
  typedef enum logic [2:0] {
    MMC_CM_0HZ    = 3'b000,  // continuous measurement mode off
    MMC_CM_1HZ    = 3'b001,
    MMC_CM_10HZ   = 3'b010,
    MMC_CM_20HZ   = 3'b011,
    MMC_CM_50HZ   = 3'b100,
    MMC_CM_100HZ  = 3'b101,
    MMC_CM_200HZ  = 3'b110,
    MMC_CM_1000HZ = 3'b111
  } mmc_cm_freq_e;

  // Periodic SET interval (INT_CTRL_2.PRD_SET) = number of measurements between
  // automatic SET pulses. Requires AUTO_SR_EN and CMM_EN both set.
  typedef enum logic [2:0] {
    MMC_PRD_SET_1    = 3'b000,
    MMC_PRD_SET_25   = 3'b001,
    MMC_PRD_SET_75   = 3'b010,
    MMC_PRD_SET_100  = 3'b011,
    MMC_PRD_SET_250  = 3'b100,
    MMC_PRD_SET_500  = 3'b101,
    MMC_PRD_SET_1000 = 3'b110,
    MMC_PRD_SET_2000 = 3'b111
  } mmc_prd_set_e;

  // Axis selector for helper functions.
  typedef enum logic [1:0] {
    MMC_AXIS_X = 2'd0,
    MMC_AXIS_Y = 2'd1,
    MMC_AXIS_Z = 2'd2
  } mmc_axis_e;

  // 18-bit output is unsigned; this is the nominal zero point (2^17).
  localparam logic [17:0] MMC_MID_SCALE      = 18'd131072;
  // Byte index of XYZ_OUT_2 within a 7-byte burst read starting at 0x00.
  localparam logic [2:0]  MMC_IDX_XYZ_OUT_2  = 3'd6;
  // Sensitivity at +/-8 G full scale: 2^17 / 8 = 16384 counts per Gauss.
  localparam int unsigned MMC_COUNTS_PER_GAUSS = 16384;

  // ===========================================================================
  // Common types + helper functions
  // ===========================================================================

  // Signed 16-bit tri-axis sample (ISM accel / gyro / temp).
  typedef struct packed { logic signed [15:0] x, y, z; } ism_vec16_t;
  // Signed 19-bit tri-axis sample (MMC 18-bit centered at 0).
  typedef struct packed { logic signed [18:0] x, y, z; } mmc_vec19_t;

  // Add the SPI read flag to a register address.
  function automatic logic [7:0] ism_spi_read_addr(input logic [7:0] addr);
    return addr | ISM_SPI_READ_MASK;
  endfunction

  // Clear the SPI read flag to form a write address.
  function automatic logic [7:0] ism_spi_write_addr(input logic [7:0] addr);
    return addr & ~ISM_SPI_READ_MASK;
  endfunction

  // Combine two little-endian output bytes into a signed 16-bit value.
  function automatic logic signed [15:0] ism_word(
    input logic [7:0] lo,
    input logic [7:0] hi
  );
    return $signed({hi, lo});
  endfunction

  // Assemble one 18-bit MMC axis from a 7-byte burst (d[0..6] = regs 0x00..0x06).
  function automatic logic [17:0] mmc_axis_raw(
    input logic [7:0] d [7],
    input mmc_axis_e  axis
  );
    case (axis)
      MMC_AXIS_X: return {d[0], d[1], d[MMC_IDX_XYZ_OUT_2][7:6]};
      MMC_AXIS_Y: return {d[2], d[3], d[MMC_IDX_XYZ_OUT_2][5:4]};
      default:    return {d[4], d[5], d[MMC_IDX_XYZ_OUT_2][3:2]};
    endcase
  endfunction

  // Convert an unsigned 18-bit MMC axis to a signed value centered at 0.
  function automatic logic signed [18:0] mmc_axis_signed(
    input logic [17:0] raw
  );
    return $signed({1'b0, raw}) - 19'sd131072;
  endfunction

endpackage

`endif
