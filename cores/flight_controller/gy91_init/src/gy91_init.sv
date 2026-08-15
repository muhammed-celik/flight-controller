module gy91_init #(
  parameter int unsigned POWER_UP_CYCLES          = 10_000_000,
  parameter int unsigned MPU_RESET_CYCLES         = 10_000_000,
  parameter int unsigned MPU_WAKE_CYCLES          = 3_500_000,
  parameter int unsigned AK_MODE_CYCLES           = 10_000,
  parameter int unsigned BMP_POLL_INTERVAL_CYCLES = 50_000,
  parameter int unsigned BMP_POLL_LIMIT           = 20,
  parameter int unsigned RETRY_DELAY_CYCLES       = 100_000,
  parameter int unsigned MAX_TRANSACTION_RETRIES  = 2
) (
  input  logic       clk,
  input  logic       rst_n,

  input  logic       reinitialize,
  output logic       reinitialize_ready,

  output logic       req_valid,
  input  logic       req_ready,
  output logic [6:0] req_address,
  output logic [7:0] req_register,
  output logic       req_write,
  output logic [7:0] req_write_data,
  output logic [7:0] req_read_count,

  input  logic       rx_valid,
  output logic       rx_ready,
  input  logic [7:0] rx_data,
  input  logic [7:0] rx_index,

  input  logic       rsp_done,
  input  logic       rsp_error,
  input  logic [3:0] rsp_error_code,

  output logic       initializing,
  output logic       ready,
  output logic       failed,

  output logic       mpu_present,
  output logic       mpu_ready,
  output logic       mpu_failed,
  output logic       ak_present,
  output logic       ak_ready,
  output logic       ak_failed,
  output logic       bmp_present,
  output logic       bmp_ready,
  output logic       bmp_failed,

  output logic [6:0] mpu_address,
  output logic [6:0] bmp_address,
  output logic [23:0]  ak_asa,
  output logic [191:0] bmp_calibration,

  output logic [15:0] mpu_error_count,
  output logic [15:0] ak_error_count,
  output logic [15:0] bmp_error_count,
  output logic [3:0]  last_i2c_error,
  output logic [1:0]  last_failed_device,
  output logic [7:0]  last_failed_step,
  output logic [15:0] init_sequence
);

  localparam logic [1:0] DEVICE_NONE = 2'd0;
  localparam logic [1:0] DEVICE_MPU  = 2'd1;
  localparam logic [1:0] DEVICE_AK   = 2'd2;
  localparam logic [1:0] DEVICE_BMP  = 2'd3;

  typedef enum logic [1:0] {
    CONTROL_DELAY,
    CONTROL_REQUEST,
    CONTROL_RESPONSE,
    CONTROL_COMPLETE
  } control_state_t;

  typedef enum logic [7:0] {
    STEP_MPU_PROBE       = 8'd1,
    STEP_MPU_RESET       = 8'd2,
    STEP_MPU_POST_ID     = 8'd3,
    STEP_MPU_WAKE        = 8'd4,
    STEP_MPU_PWR2        = 8'd5,
    STEP_MPU_CONFIG      = 8'd6,
    STEP_MPU_VERIFY      = 8'd7,
    STEP_AK_RESET        = 8'd20,
    STEP_AK_ID           = 8'd21,
    STEP_AK_POWER_DOWN_1 = 8'd22,
    STEP_AK_FUSE         = 8'd23,
    STEP_AK_ASA          = 8'd24,
    STEP_AK_POWER_DOWN_2 = 8'd25,
    STEP_AK_CONTINUOUS   = 8'd26,
    STEP_AK_VERIFY       = 8'd27,
    STEP_BMP_PROBE       = 8'd40,
    STEP_BMP_RESET       = 8'd41,
    STEP_BMP_STATUS      = 8'd42,
    STEP_BMP_POST_ID     = 8'd43,
    STEP_BMP_CALIBRATION = 8'd44,
    STEP_BMP_SLEEP       = 8'd45,
    STEP_BMP_CONFIG      = 8'd46,
    STEP_BMP_VERIFY_CFG  = 8'd47,
    STEP_BMP_NORMAL      = 8'd48,
    STEP_BMP_VERIFY_CTRL = 8'd49
  } step_t;

  control_state_t control_state;
  step_t step;
  step_t delay_next_step;
  logic [191:0] read_buffer;
  logic mpu_candidate;
  logic bmp_candidate;
  logic [3:0] table_index;
  int unsigned delay_count;
  int unsigned delay_limit;
  int unsigned retry_count;
  int unsigned bmp_poll_count;

  initial begin
    if ((POWER_UP_CYCLES == 0) || (MPU_RESET_CYCLES == 0) ||
        (MPU_WAKE_CYCLES == 0) || (AK_MODE_CYCLES == 0) ||
        (BMP_POLL_INTERVAL_CYCLES == 0) || (BMP_POLL_LIMIT == 0) ||
        (RETRY_DELAY_CYCLES == 0) || (MAX_TRANSACTION_RETRIES == 0)) begin
      $fatal(1, "gy91_init timing, polling, and retry parameters must be nonzero");
    end
  end

  function automatic logic [7:0] mpu_config_register(input logic [3:0] index);
    unique case (index)
      4'd0: mpu_config_register = 8'h19;
      4'd1: mpu_config_register = 8'h1a;
      4'd2: mpu_config_register = 8'h1b;
      4'd3: mpu_config_register = 8'h1c;
      4'd4: mpu_config_register = 8'h1d;
      4'd5: mpu_config_register = 8'h6a;
      default: mpu_config_register = 8'h37;
    endcase
  endfunction

  function automatic logic [7:0] mpu_config_value(input logic [3:0] index);
    unique case (index)
      4'd0: mpu_config_value = 8'h00;
      4'd1: mpu_config_value = 8'h02;
      4'd2: mpu_config_value = 8'h18;
      4'd3: mpu_config_value = 8'h10;
      4'd4: mpu_config_value = 8'h02;
      4'd5: mpu_config_value = 8'h00;
      default: mpu_config_value = 8'h02;
    endcase
  endfunction

  function automatic logic [7:0] mpu_verify_register(input logic [3:0] index);
    unique case (index)
      4'd0: mpu_verify_register = 8'h75;
      4'd1: mpu_verify_register = 8'h6b;
      4'd2: mpu_verify_register = 8'h6c;
      4'd3: mpu_verify_register = 8'h19;
      4'd4: mpu_verify_register = 8'h1a;
      4'd5: mpu_verify_register = 8'h1b;
      4'd6: mpu_verify_register = 8'h1c;
      4'd7: mpu_verify_register = 8'h1d;
      4'd8: mpu_verify_register = 8'h6a;
      default: mpu_verify_register = 8'h37;
    endcase
  endfunction

  function automatic logic [7:0] mpu_verify_expected(input logic [3:0] index);
    unique case (index)
      4'd0: mpu_verify_expected = 8'h71;
      4'd1: mpu_verify_expected = 8'h01;
      4'd2: mpu_verify_expected = 8'h00;
      4'd3: mpu_verify_expected = 8'h00;
      4'd4: mpu_verify_expected = 8'h02;
      4'd5: mpu_verify_expected = 8'h18;
      4'd6: mpu_verify_expected = 8'h10;
      4'd7: mpu_verify_expected = 8'h02;
      4'd8: mpu_verify_expected = 8'h00;
      default: mpu_verify_expected = 8'h02;
    endcase
  endfunction

  function automatic logic [7:0] mpu_verify_mask(input logic [3:0] index);
    unique case (index)
      4'd0: mpu_verify_mask = 8'hff;
      4'd1: mpu_verify_mask = 8'h7f;
      4'd2: mpu_verify_mask = 8'h3f;
      4'd3: mpu_verify_mask = 8'hff;
      4'd4: mpu_verify_mask = 8'h7f;
      4'd5: mpu_verify_mask = 8'hfb;
      4'd6: mpu_verify_mask = 8'hf8;
      4'd7: mpu_verify_mask = 8'h0f;
      4'd8: mpu_verify_mask = 8'h30;
      default: mpu_verify_mask = 8'h02;
    endcase
  endfunction

  function automatic logic [1:0] step_device(input step_t selected_step);
    if (selected_step <= STEP_MPU_VERIFY) begin
      step_device = DEVICE_MPU;
    end else if ((selected_step >= STEP_AK_RESET) &&
                 (selected_step <= STEP_AK_VERIFY)) begin
      step_device = DEVICE_AK;
    end else begin
      step_device = DEVICE_BMP;
    end
  endfunction

  function automatic logic [15:0] increment_saturating(input logic [15:0] value);
    increment_saturating = (&value) ? value : value + 1'b1;
  endfunction

`define GY91_BEGIN_DELAY(CYCLES, NEXT_STEP) \
  begin \
    delay_count     <= 0; \
    delay_limit     <= CYCLES; \
    delay_next_step <= NEXT_STEP; \
    control_state   <= CONTROL_DELAY; \
  end

`define GY91_COMPLETE(SUCCESSFUL) \
  begin \
    initializing  <= 1'b0; \
    ready         <= SUCCESSFUL; \
    failed        <= !(SUCCESSFUL); \
    init_sequence <= increment_saturating(init_sequence); \
    control_state <= CONTROL_COMPLETE; \
  end

`define GY91_RECORD_FAILURE(DEVICE, FAILED_STEP) \
  begin \
    last_failed_device <= DEVICE; \
    last_failed_step   <= FAILED_STEP; \
    unique case (DEVICE) \
      DEVICE_MPU: mpu_error_count <= increment_saturating(mpu_error_count); \
      DEVICE_AK:  ak_error_count  <= increment_saturating(ak_error_count); \
      default:    bmp_error_count <= increment_saturating(bmp_error_count); \
    endcase \
  end

  always_comb begin
    req_valid      = 1'b0;
    req_address    = 7'h00;
    req_register   = 8'h00;
    req_write      = 1'b0;
    req_write_data = 8'h00;
    req_read_count = 8'd1;
    rx_ready       = (control_state == CONTROL_RESPONSE);
    reinitialize_ready = !initializing;

    if (control_state == CONTROL_REQUEST) begin
      req_valid = 1'b1;
      unique case (step)
        STEP_MPU_PROBE: begin
          req_address  = mpu_candidate ? 7'h69 : 7'h68;
          req_register = 8'h75;
        end
        STEP_MPU_RESET: begin
          req_address = mpu_address; req_register = 8'h6b;
          req_write = 1'b1; req_write_data = 8'h80;
        end
        STEP_MPU_POST_ID: begin
          req_address = mpu_address; req_register = 8'h75;
        end
        STEP_MPU_WAKE: begin
          req_address = mpu_address; req_register = 8'h6b;
          req_write = 1'b1; req_write_data = 8'h01;
        end
        STEP_MPU_PWR2: begin
          req_address = mpu_address; req_register = 8'h6c;
          req_write = 1'b1; req_write_data = 8'h00;
        end
        STEP_MPU_CONFIG: begin
          req_address = mpu_address;
          req_register = mpu_config_register(table_index);
          req_write = 1'b1; req_write_data = mpu_config_value(table_index);
        end
        STEP_MPU_VERIFY: begin
          req_address = mpu_address;
          req_register = mpu_verify_register(table_index);
        end
        STEP_AK_RESET: begin
          req_address = 7'h0c; req_register = 8'h0b;
          req_write = 1'b1; req_write_data = 8'h01;
        end
        STEP_AK_ID: begin
          req_address = 7'h0c; req_register = 8'h00;
        end
        STEP_AK_POWER_DOWN_1, STEP_AK_POWER_DOWN_2: begin
          req_address = 7'h0c; req_register = 8'h0a;
          req_write = 1'b1; req_write_data = 8'h00;
        end
        STEP_AK_FUSE: begin
          req_address = 7'h0c; req_register = 8'h0a;
          req_write = 1'b1; req_write_data = 8'h0f;
        end
        STEP_AK_ASA: begin
          req_address = 7'h0c; req_register = 8'h10;
          req_read_count = 8'd3;
        end
        STEP_AK_CONTINUOUS: begin
          req_address = 7'h0c; req_register = 8'h0a;
          req_write = 1'b1; req_write_data = 8'h16;
        end
        STEP_AK_VERIFY: begin
          req_address = 7'h0c; req_register = 8'h0a;
        end
        STEP_BMP_PROBE: begin
          req_address = bmp_candidate ? 7'h77 : 7'h76;
          req_register = 8'hd0;
        end
        STEP_BMP_RESET: begin
          req_address = bmp_address; req_register = 8'he0;
          req_write = 1'b1; req_write_data = 8'hb6;
        end
        STEP_BMP_STATUS: begin
          req_address = bmp_address; req_register = 8'hf3;
        end
        STEP_BMP_POST_ID: begin
          req_address = bmp_address; req_register = 8'hd0;
        end
        STEP_BMP_CALIBRATION: begin
          req_address = bmp_address; req_register = 8'h88;
          req_read_count = 8'd24;
        end
        STEP_BMP_SLEEP: begin
          req_address = bmp_address; req_register = 8'hf4;
          req_write = 1'b1; req_write_data = 8'h2c;
        end
        STEP_BMP_CONFIG: begin
          req_address = bmp_address; req_register = 8'hf5;
          req_write = 1'b1; req_write_data = 8'h08;
        end
        STEP_BMP_VERIFY_CFG: begin
          req_address = bmp_address; req_register = 8'hf5;
        end
        STEP_BMP_NORMAL: begin
          req_address = bmp_address; req_register = 8'hf4;
          req_write = 1'b1; req_write_data = 8'h2f;
        end
        STEP_BMP_VERIFY_CTRL: begin
          req_address = bmp_address; req_register = 8'hf4;
        end
        default: req_valid = 1'b0;
      endcase
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      control_state      <= CONTROL_DELAY;
      step               <= STEP_MPU_PROBE;
      delay_next_step    <= STEP_MPU_PROBE;
      read_buffer        <= '0;
      mpu_candidate      <= 1'b0;
      bmp_candidate      <= 1'b0;
      table_index        <= '0;
      delay_count        <= 0;
      delay_limit        <= POWER_UP_CYCLES;
      retry_count        <= 0;
      bmp_poll_count     <= 0;
      initializing       <= 1'b1;
      ready              <= 1'b0;
      failed             <= 1'b0;
      mpu_present        <= 1'b0;
      mpu_ready          <= 1'b0;
      mpu_failed         <= 1'b0;
      ak_present         <= 1'b0;
      ak_ready           <= 1'b0;
      ak_failed          <= 1'b0;
      bmp_present        <= 1'b0;
      bmp_ready          <= 1'b0;
      bmp_failed         <= 1'b0;
      mpu_address        <= '0;
      bmp_address        <= '0;
      ak_asa             <= '0;
      bmp_calibration    <= '0;
      mpu_error_count    <= '0;
      ak_error_count     <= '0;
      bmp_error_count    <= '0;
      last_i2c_error     <= '0;
      last_failed_device <= DEVICE_NONE;
      last_failed_step   <= '0;
      init_sequence      <= '0;
    end else begin
      unique case (control_state)
        CONTROL_DELAY: begin
          if (delay_count == (delay_limit - 1)) begin
            delay_count   <= 0;
            step          <= delay_next_step;
            read_buffer   <= '0;
            control_state <= CONTROL_REQUEST;
          end else begin
            delay_count <= delay_count + 1;
          end
        end

        CONTROL_REQUEST: begin
          if (req_valid && req_ready) begin
            read_buffer   <= '0;
            control_state <= CONTROL_RESPONSE;
          end else if (!req_valid) begin
            mpu_failed         <= 1'b1;
            ak_failed          <= 1'b1;
            bmp_failed         <= 1'b1;
            last_failed_device <= step_device(step);
            last_failed_step   <= step;
            `GY91_COMPLETE(1'b0)
          end
        end

        CONTROL_RESPONSE: begin
          if (rx_valid && (rx_index < 8'd24)) begin
            read_buffer[rx_index * 8 +: 8] <= rx_data;
          end

          if (rsp_done) begin
            if (rsp_error) begin
              last_i2c_error <= rsp_error_code;
              unique case (step_device(step))
                DEVICE_MPU: mpu_error_count <= increment_saturating(mpu_error_count);
                DEVICE_AK:  ak_error_count  <= increment_saturating(ak_error_count);
                default:    bmp_error_count <= increment_saturating(bmp_error_count);
              endcase

              if (retry_count < MAX_TRANSACTION_RETRIES) begin
                retry_count <= retry_count + 1;
                `GY91_BEGIN_DELAY(RETRY_DELAY_CYCLES, step)
              end else begin
                retry_count        <= 0;
                last_failed_device <= step_device(step);
                last_failed_step   <= step;
                if ((step == STEP_MPU_PROBE) && !mpu_candidate) begin
                  mpu_candidate <= 1'b1;
                  `GY91_BEGIN_DELAY(RETRY_DELAY_CYCLES, STEP_MPU_PROBE)
                end else if (step_device(step) == DEVICE_MPU) begin
                  mpu_failed <= 1'b1;
                  ak_failed  <= 1'b1;
                  step       <= STEP_BMP_PROBE;
                  read_buffer <= '0;
                  control_state <= CONTROL_REQUEST;
                end else if (step_device(step) == DEVICE_AK) begin
                  ak_failed <= 1'b1;
                  step      <= STEP_BMP_PROBE;
                  read_buffer <= '0;
                  control_state <= CONTROL_REQUEST;
                end else if ((step == STEP_BMP_PROBE) && !bmp_candidate) begin
                  bmp_candidate <= 1'b1;
                  `GY91_BEGIN_DELAY(RETRY_DELAY_CYCLES, STEP_BMP_PROBE)
                end else begin
                  bmp_failed <= 1'b1;
                  `GY91_COMPLETE(1'b0)
                end
              end
            end else begin
              retry_count <= 0;
              unique case (step)
                STEP_MPU_PROBE: begin
                  if (read_buffer[7:0] == 8'h71) begin
                    mpu_present <= 1'b1;
                    mpu_address <= mpu_candidate ? 7'h69 : 7'h68;
                    step <= STEP_MPU_RESET; control_state <= CONTROL_REQUEST;
                  end else begin
                    `GY91_RECORD_FAILURE(DEVICE_MPU, step)
                    if (!mpu_candidate) begin
                      mpu_candidate <= 1'b1;
                      step <= STEP_MPU_PROBE; control_state <= CONTROL_REQUEST;
                    end else begin
                      mpu_failed <= 1'b1; ak_failed <= 1'b1;
                      step <= STEP_BMP_PROBE; control_state <= CONTROL_REQUEST;
                    end
                  end
                end
                STEP_MPU_RESET: begin
                  `GY91_BEGIN_DELAY(MPU_RESET_CYCLES, STEP_MPU_POST_ID)
                end
                STEP_MPU_POST_ID: begin
                  if (read_buffer[7:0] == 8'h71) begin
                    step <= STEP_MPU_WAKE; control_state <= CONTROL_REQUEST;
                  end else begin
                    `GY91_RECORD_FAILURE(DEVICE_MPU, step)
                    mpu_failed <= 1'b1; ak_failed <= 1'b1;
                    step <= STEP_BMP_PROBE; control_state <= CONTROL_REQUEST;
                  end
                end
                STEP_MPU_WAKE: begin
                  step <= STEP_MPU_PWR2; control_state <= CONTROL_REQUEST;
                end
                STEP_MPU_PWR2: begin
                  `GY91_BEGIN_DELAY(MPU_WAKE_CYCLES, STEP_MPU_CONFIG)
                  table_index <= 0;
                end
                STEP_MPU_CONFIG: begin
                  if (table_index == 6) begin
                    table_index <= 0;
                    step <= STEP_MPU_VERIFY; control_state <= CONTROL_REQUEST;
                  end else begin
                    table_index <= table_index + 1'b1;
                    step <= STEP_MPU_CONFIG; control_state <= CONTROL_REQUEST;
                  end
                end
                STEP_MPU_VERIFY: begin
                  if ((read_buffer[7:0] & mpu_verify_mask(table_index)) ==
                      mpu_verify_expected(table_index)) begin
                    if (table_index == 9) begin
                      mpu_ready <= 1'b1;
                      step <= STEP_AK_RESET; control_state <= CONTROL_REQUEST;
                    end else begin
                      table_index <= table_index + 1'b1;
                      step <= STEP_MPU_VERIFY; control_state <= CONTROL_REQUEST;
                    end
                  end else begin
                    `GY91_RECORD_FAILURE(DEVICE_MPU, step)
                    mpu_failed <= 1'b1; ak_failed <= 1'b1;
                    step <= STEP_BMP_PROBE; control_state <= CONTROL_REQUEST;
                  end
                end
                STEP_AK_RESET: begin
                  `GY91_BEGIN_DELAY(AK_MODE_CYCLES, STEP_AK_ID)
                end
                STEP_AK_ID: begin
                  if (read_buffer[7:0] == 8'h48) begin
                    ak_present <= 1'b1;
                    step <= STEP_AK_POWER_DOWN_1; control_state <= CONTROL_REQUEST;
                  end else begin
                    `GY91_RECORD_FAILURE(DEVICE_AK, step)
                    ak_failed <= 1'b1;
                    step <= STEP_BMP_PROBE; control_state <= CONTROL_REQUEST;
                  end
                end
                STEP_AK_POWER_DOWN_1: begin
                  `GY91_BEGIN_DELAY(AK_MODE_CYCLES, STEP_AK_FUSE)
                end
                STEP_AK_FUSE: begin
                  `GY91_BEGIN_DELAY(AK_MODE_CYCLES, STEP_AK_ASA)
                end
                STEP_AK_ASA: begin
                  if ((read_buffer[23:0] != 24'h000000) &&
                      (read_buffer[23:0] != 24'hffffff)) begin
                    ak_asa <= read_buffer[23:0];
                    step <= STEP_AK_POWER_DOWN_2; control_state <= CONTROL_REQUEST;
                  end else begin
                    `GY91_RECORD_FAILURE(DEVICE_AK, step)
                    ak_failed <= 1'b1;
                    step <= STEP_BMP_PROBE; control_state <= CONTROL_REQUEST;
                  end
                end
                STEP_AK_POWER_DOWN_2: begin
                  `GY91_BEGIN_DELAY(AK_MODE_CYCLES, STEP_AK_CONTINUOUS)
                end
                STEP_AK_CONTINUOUS: begin
                  `GY91_BEGIN_DELAY(AK_MODE_CYCLES, STEP_AK_VERIFY)
                end
                STEP_AK_VERIFY: begin
                  if ((read_buffer[7:0] & 8'h1f) == 8'h16) begin
                    ak_ready <= 1'b1;
                  end else begin
                    `GY91_RECORD_FAILURE(DEVICE_AK, step)
                    ak_failed <= 1'b1;
                  end
                  step <= STEP_BMP_PROBE; control_state <= CONTROL_REQUEST;
                end
                STEP_BMP_PROBE: begin
                  if (read_buffer[7:0] == 8'h58) begin
                    bmp_present <= 1'b1;
                    bmp_address <= bmp_candidate ? 7'h77 : 7'h76;
                    step <= STEP_BMP_RESET; control_state <= CONTROL_REQUEST;
                  end else begin
                    `GY91_RECORD_FAILURE(DEVICE_BMP, step)
                    if (!bmp_candidate) begin
                      bmp_candidate <= 1'b1;
                      step <= STEP_BMP_PROBE; control_state <= CONTROL_REQUEST;
                    end else begin
                      bmp_failed <= 1'b1;
                      `GY91_COMPLETE(1'b0)
                    end
                  end
                end
                STEP_BMP_RESET: begin
                  bmp_poll_count <= 0;
                  `GY91_BEGIN_DELAY(BMP_POLL_INTERVAL_CYCLES, STEP_BMP_STATUS)
                end
                STEP_BMP_STATUS: begin
                  if ((read_buffer[7:0] & 8'h09) == 0) begin
                    step <= STEP_BMP_POST_ID; control_state <= CONTROL_REQUEST;
                  end else if ((bmp_poll_count + 1) >= BMP_POLL_LIMIT) begin
                    `GY91_RECORD_FAILURE(DEVICE_BMP, step)
                    bmp_failed <= 1'b1;
                    `GY91_COMPLETE(1'b0)
                  end else begin
                    bmp_poll_count <= bmp_poll_count + 1;
                    `GY91_BEGIN_DELAY(BMP_POLL_INTERVAL_CYCLES, STEP_BMP_STATUS)
                  end
                end
                STEP_BMP_POST_ID: begin
                  if (read_buffer[7:0] == 8'h58) begin
                    step <= STEP_BMP_CALIBRATION; control_state <= CONTROL_REQUEST;
                  end else begin
                    `GY91_RECORD_FAILURE(DEVICE_BMP, step)
                    bmp_failed <= 1'b1;
                    `GY91_COMPLETE(1'b0)
                  end
                end
                STEP_BMP_CALIBRATION: begin
                  if ((read_buffer != 192'h0) && (read_buffer != {192{1'b1}}) &&
                      (read_buffer[15:0] != 16'h0000) &&
                      (read_buffer[63:48] != 16'h0000)) begin
                    bmp_calibration <= read_buffer;
                    step <= STEP_BMP_SLEEP; control_state <= CONTROL_REQUEST;
                  end else begin
                    `GY91_RECORD_FAILURE(DEVICE_BMP, step)
                    bmp_failed <= 1'b1;
                    `GY91_COMPLETE(1'b0)
                  end
                end
                STEP_BMP_SLEEP: begin
                  step <= STEP_BMP_CONFIG; control_state <= CONTROL_REQUEST;
                end
                STEP_BMP_CONFIG: begin
                  step <= STEP_BMP_VERIFY_CFG; control_state <= CONTROL_REQUEST;
                end
                STEP_BMP_VERIFY_CFG: begin
                  if ((read_buffer[7:0] & 8'hfd) == 8'h08) begin
                    step <= STEP_BMP_NORMAL; control_state <= CONTROL_REQUEST;
                  end else begin
                    `GY91_RECORD_FAILURE(DEVICE_BMP, step)
                    bmp_failed <= 1'b1;
                    `GY91_COMPLETE(1'b0)
                  end
                end
                STEP_BMP_NORMAL: begin
                  step <= STEP_BMP_VERIFY_CTRL; control_state <= CONTROL_REQUEST;
                end
                STEP_BMP_VERIFY_CTRL: begin
                  if (read_buffer[7:0] == 8'h2f) begin
                    bmp_ready <= 1'b1;
                    `GY91_COMPLETE(mpu_ready && ak_ready)
                  end else begin
                    `GY91_RECORD_FAILURE(DEVICE_BMP, step)
                    bmp_failed <= 1'b1;
                    `GY91_COMPLETE(1'b0)
                  end
                end
                default: begin
                  mpu_failed <= 1'b1; ak_failed <= 1'b1; bmp_failed <= 1'b1;
                  last_failed_device <= step_device(step);
                  last_failed_step <= step;
                  `GY91_COMPLETE(1'b0)
                end
              endcase
            end
          end
        end

        CONTROL_COMPLETE: begin
          if (reinitialize) begin
            initializing       <= 1'b1;
            ready              <= 1'b0;
            failed             <= 1'b0;
            mpu_present        <= 1'b0;
            mpu_ready          <= 1'b0;
            mpu_failed         <= 1'b0;
            ak_present         <= 1'b0;
            ak_ready           <= 1'b0;
            ak_failed          <= 1'b0;
            bmp_present        <= 1'b0;
            bmp_ready          <= 1'b0;
            bmp_failed         <= 1'b0;
            mpu_address        <= '0;
            bmp_address        <= '0;
            ak_asa             <= '0;
            bmp_calibration    <= '0;
            last_i2c_error     <= '0;
            last_failed_device <= DEVICE_NONE;
            last_failed_step   <= '0;
            mpu_candidate      <= 1'b0;
            bmp_candidate      <= 1'b0;
            table_index        <= '0;
            retry_count        <= 0;
            bmp_poll_count     <= 0;
            `GY91_BEGIN_DELAY(POWER_UP_CYCLES, STEP_MPU_PROBE)
          end
        end

        default: begin
          mpu_failed         <= 1'b1;
          ak_failed          <= 1'b1;
          bmp_failed         <= 1'b1;
          last_failed_device <= DEVICE_NONE;
          last_failed_step   <= '0;
          `GY91_COMPLETE(1'b0)
        end
      endcase
    end
  end

`undef GY91_BEGIN_DELAY
`undef GY91_COMPLETE
`undef GY91_RECORD_FAILURE

endmodule
