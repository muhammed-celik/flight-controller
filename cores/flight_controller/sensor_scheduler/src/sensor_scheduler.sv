module sensor_scheduler #(
  parameter int unsigned FIFO_DEPTH                    = 16,
  parameter int unsigned MPU_RETRY_DELAY_CYCLES        = 5_000,
  parameter int unsigned MPU_FRESH_CYCLES              = 200_000,
  parameter int unsigned MPU_HARD_STALE_CYCLES         = 500_000,
  parameter int unsigned AK_FRESH_CYCLES               = 3_000_000,
  parameter int unsigned BMP_FRESH_CYCLES              = 6_000_000,
  parameter int unsigned MPU_DEADLINE_CYCLES           = 100_000,
  parameter int unsigned AK_DEADLINE_CYCLES            = 1_000_000,
  parameter int unsigned BMP_DEADLINE_CYCLES           = 2_000_000,
  parameter int unsigned RUNTIME_FAULT_CONSECUTIVE     = 3
) (
  input  logic        clk,
  input  logic        rst_n,
  input  logic        init_ready,
  input  logic [6:0]  mpu_address,
  input  logic [6:0]  bmp_address,
  input  logic        enable_1khz,
  input  logic        enable_100hz,
  input  logic        enable_50hz,
  input  logic        enable_10hz,
  input  logic [63:0] timestamp_cycles,

  output logic        req_valid,
  input  logic        req_ready,
  output logic [6:0]  req_address,
  output logic [7:0]  req_register,
  output logic        req_write,
  output logic [7:0]  req_write_data,
  output logic [7:0]  req_read_count,
  output logic        req_fast_mode,
  input  logic        rx_valid,
  output logic        rx_ready,
  input  logic [7:0]  rx_data,
  input  logic [7:0]  rx_index,
  input  logic        rsp_done,
  input  logic        rsp_error,
  input  logic [3:0]  rsp_error_code,
  input  logic        i2c_busy,

  input  logic        snapshot_capture,
  input  logic        fifo_pop,

  output logic         mpu_sample,
  output logic         ak_sample,
  output logic         bmp_sample,
  output logic         mpu_sample_pulse,
  output logic         ak_sample_pulse,
  output logic         bmp_sample_pulse,
  output logic         mpu_valid,
  output logic         ak_valid,
  output logic         bmp_valid,
  output logic [255:0] mpu_latest_record,
  output logic [255:0] ak_latest_record,
  output logic [255:0] bmp_latest_record,
  output logic         mpu_fresh,
  output logic         ak_fresh,
  output logic         bmp_fresh,
  output logic         mpu_hard_stale,

  output logic [767:0] snapshot_records,
  output logic [31:0]  snapshot_status,
  output logic [31:0]  snapshot_sequence,

  output logic [255:0]                  fifo_head_data,
  output logic [$clog2(FIFO_DEPTH+1)-1:0] fifo_level,
  output logic                          fifo_empty,
  output logic                          fifo_full,
  output logic                          fifo_overflow_sticky,
  output logic [31:0]                   fifo_overflow_count,
  output logic [31:0]                   fifo_underflow_count,

  output logic [31:0] mpu_release_count,
  output logic [31:0] ak_release_count,
  output logic [31:0] bmp_release_count,
  output logic [31:0] mpu_accepted_sample_count,
  output logic [31:0] ak_accepted_sample_count,
  output logic [31:0] bmp_accepted_sample_count,
  output logic [31:0] mpu_missed_release_count,
  output logic [31:0] ak_missed_release_count,
  output logic [31:0] bmp_missed_release_count,
  output logic [31:0] mpu_missed_sample_count,
  output logic [31:0] ak_missed_sample_count,
  output logic [31:0] bmp_missed_sample_count,
  output logic [31:0] mpu_i2c_error_count,
  output logic [31:0] ak_i2c_error_count,
  output logic [31:0] bmp_i2c_error_count,
  output logic [31:0] mpu_invalid_sample_count,
  output logic [31:0] ak_invalid_sample_count,
  output logic [31:0] bmp_invalid_sample_count,
  output logic [31:0] mpu_duplicate_poll_count,
  output logic [31:0] ak_duplicate_poll_count,
  output logic [31:0] ak_overrun_count,
  output logic [31:0] mpu_retry_count,
  output logic [31:0] mpu_max_latency_cycles,
  output logic [31:0] ak_max_latency_cycles,
  output logic [31:0] bmp_max_latency_cycles,
  output logic [31:0] deadline_miss_count,
  output logic [31:0] bus_transaction_count,
  output logic [31:0] bus_error_count,
  output logic [63:0] bus_busy_cycles,
  output logic [31:0] bus_window_busy_cycles,
  output logic [3:0]  last_i2c_error,
  output logic        runtime_fault
);

  typedef enum logic [1:0] {SENSOR_MPU, SENSOR_AK, SENSOR_BMP} sensor_t;
  typedef enum logic [2:0] {STATE_IDLE, STATE_REQUEST, STATE_RESPONSE,
                            STATE_RETRY_WAIT} state_t;

  state_t state;
  sensor_t active_sensor;
  logic active_retry;
  logic mpu_pending, ak_pending, bmp_pending;
  logic [63:0] mpu_release_timestamp, ak_release_timestamp, bmp_release_timestamp;
  logic [31:0] mpu_release_sequence, ak_release_sequence, bmp_release_sequence;
  logic [31:0] mpu_pending_sequence, ak_pending_sequence, bmp_pending_sequence;
  logic [63:0] active_release_timestamp;
  logic [31:0] active_release_sequence;
  logic [6:0] active_address;
  logic [31:0] mpu_sample_sequence, ak_sample_sequence, bmp_sample_sequence;
  logic [119:0] read_buffer;
  logic [14:0] received_indices;
  logic [4:0] received_count;
  logic bad_receive_index;
  logic [63:0] retry_delay_count;
  logic [63:0] runtime_start_timestamp;
  logic runtime_started;
  logic [63:0] mpu_completion_timestamp, ak_completion_timestamp;
  logic [63:0] bmp_completion_timestamp;
  logic [31:0] mpu_consecutive_failures;
  logic [63:0] bus_window_accumulator;
  logic fifo_push_valid;
  logic [255:0] fifo_push_data;
  logic fifo_overflow, fifo_underflow;
  logic [255:0] completion_record;
  logic [31:0] live_snapshot_status;
  logic [63:0] active_latency;
  logic exact_receive_count;

  initial begin
    if ((MPU_RETRY_DELAY_CYCLES == 0) || (MPU_FRESH_CYCLES == 0) ||
        (MPU_HARD_STALE_CYCLES == 0) || (AK_FRESH_CYCLES == 0) ||
        (BMP_FRESH_CYCLES == 0) || (MPU_DEADLINE_CYCLES == 0) ||
        (AK_DEADLINE_CYCLES == 0) || (BMP_DEADLINE_CYCLES == 0) ||
        (RUNTIME_FAULT_CONSECUTIVE == 0)) begin
      $fatal(1, "sensor_scheduler timing and fault parameters must be nonzero");
    end
    if ((FIFO_DEPTH < 2) || ((FIFO_DEPTH & (FIFO_DEPTH - 1)) != 0)) begin
      $fatal(1, "sensor_scheduler FIFO_DEPTH must be a power of two and at least 2");
    end
  end

  function automatic logic [31:0] increment_saturating32(input logic [31:0] value);
    increment_saturating32 = (&value) ? value : value + 1'b1;
  endfunction

  function automatic logic [63:0] increment_saturating64(input logic [63:0] value);
    increment_saturating64 = (&value) ? value : value + 1'b1;
  endfunction

  function automatic logic [31:0] latency32(input logic [63:0] value);
    latency32 = (|value[63:32]) ? 32'hffff_ffff : value[31:0];
  endfunction

  always_comb begin
    req_valid      = init_ready && (state == STATE_REQUEST);
    req_address    = active_address;
    req_register   = 8'h3a;
    req_write      = 1'b0;
    req_write_data = 8'h00;
    req_read_count = 8'd15;
    req_fast_mode  = 1'b1;
    if (active_sensor == SENSOR_AK) begin
      req_register   = 8'h02;
      req_read_count = 8'd8;
    end else if (active_sensor == SENSOR_BMP) begin
      req_register   = 8'hf7;
      req_read_count = 8'd6;
    end
    rx_ready = init_ready && (state == STATE_RESPONSE);
    mpu_sample_pulse = mpu_sample;
    ak_sample_pulse = ak_sample;
    bmp_sample_pulse = bmp_sample;
  end

  always_comb begin
    active_latency = timestamp_cycles - active_release_timestamp;
    unique case (active_sensor)
      SENSOR_MPU: exact_receive_count = !bad_receive_index &&
                   (received_count == 5'd15) && (&received_indices);
      SENSOR_AK: exact_receive_count = !bad_receive_index &&
                  (received_count == 5'd8) && (&received_indices[7:0]);
      default: exact_receive_count = !bad_receive_index &&
                (received_count == 5'd6) && (&received_indices[5:0]);
    endcase
  end

  always_comb begin
    completion_record = '0;
    completion_record[1:0] = active_sensor;
    completion_record[2] = active_retry;
    completion_record[47:16] = (active_sensor == SENSOR_MPU) ?
      mpu_sample_sequence + 1'b1 : ((active_sensor == SENSOR_AK) ?
      ak_sample_sequence + 1'b1 : bmp_sample_sequence + 1'b1);
    completion_record[111:48] = timestamp_cycles;
    completion_record[255:224] = active_release_sequence;
    if (active_sensor == SENSOR_MPU) begin
      completion_record[3] = read_buffer[0];
      for (int unsigned i = 0; i < 14; i++) begin
        completion_record[112 + i*8 +: 8] = read_buffer[8 + i*8 +: 8];
      end
    end else if (active_sensor == SENSOR_AK) begin
      completion_record[3] = read_buffer[0];
      completion_record[4] = read_buffer[1];
      completion_record[5] = read_buffer[59];
      completion_record[6] = read_buffer[60];
      for (int unsigned i = 0; i < 6; i++) begin
        completion_record[112 + i*8 +: 8] = read_buffer[8 + i*8 +: 8];
      end
    end else begin
      completion_record[119:112] = read_buffer[7:0];
      completion_record[127:120] = read_buffer[15:8];
      completion_record[135:128] = {read_buffer[23:20], 4'h0};
      completion_record[143:136] = read_buffer[31:24];
      completion_record[151:144] = read_buffer[39:32];
      completion_record[159:152] = {read_buffer[47:44], 4'h0};
    end
  end

  always_comb begin
    mpu_fresh = mpu_valid &&
      ((timestamp_cycles - mpu_completion_timestamp) <= 64'(MPU_FRESH_CYCLES));
    ak_fresh = ak_valid &&
      ((timestamp_cycles - ak_completion_timestamp) <= 64'(AK_FRESH_CYCLES));
    bmp_fresh = bmp_valid &&
      ((timestamp_cycles - bmp_completion_timestamp) <= 64'(BMP_FRESH_CYCLES));
    mpu_hard_stale = runtime_started &&
      (mpu_valid ? ((timestamp_cycles - mpu_completion_timestamp) > 64'(MPU_HARD_STALE_CYCLES)) :
                   ((timestamp_cycles - runtime_start_timestamp) > 64'(MPU_HARD_STALE_CYCLES)));

    live_snapshot_status = '0;
    live_snapshot_status[0] = mpu_valid;
    live_snapshot_status[1] = ak_valid;
    live_snapshot_status[2] = bmp_valid;
    live_snapshot_status[3] = mpu_fresh;
    live_snapshot_status[4] = ak_fresh;
    live_snapshot_status[5] = bmp_fresh;
    live_snapshot_status[6] = mpu_hard_stale;
    live_snapshot_status[7] = runtime_fault;
    live_snapshot_status[8] = fifo_empty;
    live_snapshot_status[9] = fifo_full;
    live_snapshot_status[31:16] = 16'(fifo_level);
  end

  sensor_fifo #(
    .DATA_WIDTH(256),
    .DEPTH(FIFO_DEPTH)
  ) sample_fifo (
    .clk,
    .rst_n(rst_n && init_ready),
    .push_valid(fifo_push_valid),
    .push_data(fifo_push_data),
    .pop(fifo_pop),
    .head_data(fifo_head_data),
    .level(fifo_level),
    .empty(fifo_empty),
    .full(fifo_full),
    .overflow(fifo_overflow),
    .underflow(fifo_underflow)
  );

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      state <= STATE_IDLE;
      active_sensor <= SENSOR_MPU;
      active_retry <= 1'b0;
      mpu_pending <= 1'b0;
      ak_pending <= 1'b0;
      bmp_pending <= 1'b0;
      mpu_release_timestamp <= '0;
      ak_release_timestamp <= '0;
      bmp_release_timestamp <= '0;
      active_release_timestamp <= '0;
      active_address <= '0;
      mpu_release_sequence <= '0;
      ak_release_sequence <= '0;
      bmp_release_sequence <= '0;
      mpu_pending_sequence <= '0;
      ak_pending_sequence <= '0;
      bmp_pending_sequence <= '0;
      active_release_sequence <= '0;
      mpu_sample_sequence <= '0;
      ak_sample_sequence <= '0;
      bmp_sample_sequence <= '0;
      read_buffer <= '0;
      received_indices <= '0;
      received_count <= '0;
      bad_receive_index <= 1'b0;
      retry_delay_count <= '0;
      runtime_start_timestamp <= '0;
      runtime_started <= 1'b0;
      mpu_completion_timestamp <= '0;
      ak_completion_timestamp <= '0;
      bmp_completion_timestamp <= '0;
      mpu_latest_record <= '0;
      ak_latest_record <= '0;
      bmp_latest_record <= '0;
      mpu_valid <= 1'b0;
      ak_valid <= 1'b0;
      bmp_valid <= 1'b0;
      mpu_sample <= 1'b0;
      ak_sample <= 1'b0;
      bmp_sample <= 1'b0;
      fifo_push_valid <= 1'b0;
      fifo_push_data <= '0;
      fifo_overflow_sticky <= 1'b0;
      fifo_overflow_count <= '0;
      fifo_underflow_count <= '0;
      snapshot_records <= '0;
      snapshot_status <= '0;
      snapshot_sequence <= '0;
      mpu_release_count <= '0;
      ak_release_count <= '0;
      bmp_release_count <= '0;
      mpu_accepted_sample_count <= '0;
      ak_accepted_sample_count <= '0;
      bmp_accepted_sample_count <= '0;
      mpu_missed_release_count <= '0;
      ak_missed_release_count <= '0;
      bmp_missed_release_count <= '0;
      mpu_missed_sample_count <= '0;
      ak_missed_sample_count <= '0;
      bmp_missed_sample_count <= '0;
      mpu_i2c_error_count <= '0;
      ak_i2c_error_count <= '0;
      bmp_i2c_error_count <= '0;
      mpu_invalid_sample_count <= '0;
      ak_invalid_sample_count <= '0;
      bmp_invalid_sample_count <= '0;
      mpu_duplicate_poll_count <= '0;
      ak_duplicate_poll_count <= '0;
      ak_overrun_count <= '0;
      mpu_retry_count <= '0;
      mpu_max_latency_cycles <= '0;
      ak_max_latency_cycles <= '0;
      bmp_max_latency_cycles <= '0;
      deadline_miss_count <= '0;
      bus_transaction_count <= '0;
      bus_error_count <= '0;
      bus_busy_cycles <= '0;
      bus_window_accumulator <= '0;
      bus_window_busy_cycles <= '0;
      last_i2c_error <= '0;
      mpu_consecutive_failures <= '0;
      runtime_fault <= 1'b0;
    end else if (!init_ready) begin
      state <= STATE_IDLE;
      active_sensor <= SENSOR_MPU;
      active_retry <= 1'b0;
      mpu_pending <= 1'b0;
      ak_pending <= 1'b0;
      bmp_pending <= 1'b0;
      mpu_release_timestamp <= '0;
      ak_release_timestamp <= '0;
      bmp_release_timestamp <= '0;
      active_release_timestamp <= '0;
      active_address <= '0;
      mpu_release_sequence <= '0;
      ak_release_sequence <= '0;
      bmp_release_sequence <= '0;
      mpu_pending_sequence <= '0;
      ak_pending_sequence <= '0;
      bmp_pending_sequence <= '0;
      active_release_sequence <= '0;
      mpu_sample_sequence <= '0;
      ak_sample_sequence <= '0;
      bmp_sample_sequence <= '0;
      read_buffer <= '0;
      received_indices <= '0;
      received_count <= '0;
      bad_receive_index <= 1'b0;
      retry_delay_count <= '0;
      runtime_start_timestamp <= timestamp_cycles;
      runtime_started <= 1'b0;
      mpu_completion_timestamp <= '0;
      ak_completion_timestamp <= '0;
      bmp_completion_timestamp <= '0;
      mpu_latest_record <= '0;
      ak_latest_record <= '0;
      bmp_latest_record <= '0;
      mpu_valid <= 1'b0;
      ak_valid <= 1'b0;
      bmp_valid <= 1'b0;
      mpu_sample <= 1'b0;
      ak_sample <= 1'b0;
      bmp_sample <= 1'b0;
      fifo_push_valid <= 1'b0;
      fifo_push_data <= '0;
      fifo_overflow_sticky <= 1'b0;
      fifo_overflow_count <= '0;
      fifo_underflow_count <= '0;
      snapshot_records <= '0;
      snapshot_status <= '0;
      snapshot_sequence <= '0;
      mpu_release_count <= '0;
      ak_release_count <= '0;
      bmp_release_count <= '0;
      mpu_accepted_sample_count <= '0;
      ak_accepted_sample_count <= '0;
      bmp_accepted_sample_count <= '0;
      mpu_missed_release_count <= '0;
      ak_missed_release_count <= '0;
      bmp_missed_release_count <= '0;
      mpu_missed_sample_count <= '0;
      ak_missed_sample_count <= '0;
      bmp_missed_sample_count <= '0;
      mpu_i2c_error_count <= '0;
      ak_i2c_error_count <= '0;
      bmp_i2c_error_count <= '0;
      mpu_invalid_sample_count <= '0;
      ak_invalid_sample_count <= '0;
      bmp_invalid_sample_count <= '0;
      mpu_duplicate_poll_count <= '0;
      ak_duplicate_poll_count <= '0;
      ak_overrun_count <= '0;
      mpu_retry_count <= '0;
      mpu_max_latency_cycles <= '0;
      ak_max_latency_cycles <= '0;
      bmp_max_latency_cycles <= '0;
      deadline_miss_count <= '0;
      bus_transaction_count <= '0;
      bus_error_count <= '0;
      bus_busy_cycles <= '0;
      bus_window_accumulator <= '0;
      bus_window_busy_cycles <= '0;
      last_i2c_error <= '0;
      mpu_consecutive_failures <= '0;
      runtime_fault <= 1'b0;
    end else begin
      mpu_sample <= 1'b0;
      ak_sample <= 1'b0;
      bmp_sample <= 1'b0;
      fifo_push_valid <= 1'b0;

      if (!runtime_started) begin
        runtime_started <= 1'b1;
        runtime_start_timestamp <= timestamp_cycles;
      end
      if (mpu_hard_stale) begin
        runtime_fault <= 1'b1;
      end

      if (i2c_busy) begin
        bus_busy_cycles <= increment_saturating64(bus_busy_cycles);
      end
      if (enable_10hz) begin
        bus_window_busy_cycles <= latency32(bus_window_accumulator);
        bus_window_accumulator <= i2c_busy ? 64'd1 : 64'd0;
      end else if (i2c_busy) begin
        bus_window_accumulator <= increment_saturating64(bus_window_accumulator);
      end

      if (fifo_overflow) begin
        fifo_overflow_sticky <= 1'b1;
        fifo_overflow_count <= increment_saturating32(fifo_overflow_count);
      end
      if (fifo_underflow) begin
        fifo_underflow_count <= increment_saturating32(fifo_underflow_count);
      end

      if (snapshot_capture) begin
        snapshot_records <= {bmp_latest_record, ak_latest_record, mpu_latest_record};
        snapshot_status <= live_snapshot_status;
        snapshot_sequence <= snapshot_sequence + 1'b1;
      end

      if (enable_1khz) begin
        mpu_release_count <= increment_saturating32(mpu_release_count);
        mpu_release_sequence <= mpu_release_sequence + 1'b1;
        if (mpu_pending || ((state != STATE_IDLE) && (active_sensor == SENSOR_MPU))) begin
          mpu_missed_release_count <= increment_saturating32(mpu_missed_release_count);
          mpu_missed_sample_count <= increment_saturating32(mpu_missed_sample_count);
        end else begin
          mpu_pending <= 1'b1;
          mpu_release_timestamp <= timestamp_cycles;
          mpu_pending_sequence <= mpu_release_sequence + 1'b1;
        end
      end
      if (enable_100hz) begin
        ak_release_count <= increment_saturating32(ak_release_count);
        ak_release_sequence <= ak_release_sequence + 1'b1;
        if (ak_pending || ((state != STATE_IDLE) && (active_sensor == SENSOR_AK))) begin
          ak_missed_release_count <= increment_saturating32(ak_missed_release_count);
          ak_missed_sample_count <= increment_saturating32(ak_missed_sample_count);
        end else begin
          ak_pending <= 1'b1;
          ak_release_timestamp <= timestamp_cycles;
          ak_pending_sequence <= ak_release_sequence + 1'b1;
        end
      end
      if (enable_50hz) begin
        bmp_release_count <= increment_saturating32(bmp_release_count);
        bmp_release_sequence <= bmp_release_sequence + 1'b1;
        if (bmp_pending || ((state != STATE_IDLE) && (active_sensor == SENSOR_BMP))) begin
          bmp_missed_release_count <= increment_saturating32(bmp_missed_release_count);
          bmp_missed_sample_count <= increment_saturating32(bmp_missed_sample_count);
        end else begin
          bmp_pending <= 1'b1;
          bmp_release_timestamp <= timestamp_cycles;
          bmp_pending_sequence <= bmp_release_sequence + 1'b1;
        end
      end

      unique case (state)
        STATE_IDLE: begin
          if (mpu_pending) begin
            active_sensor <= SENSOR_MPU;
            active_retry <= 1'b0;
            active_release_timestamp <= mpu_release_timestamp;
            active_release_sequence <= mpu_pending_sequence;
            active_address <= mpu_address;
            mpu_pending <= 1'b0;
            state <= STATE_REQUEST;
          end else if (ak_pending) begin
            active_sensor <= SENSOR_AK;
            active_retry <= 1'b0;
            active_release_timestamp <= ak_release_timestamp;
            active_release_sequence <= ak_pending_sequence;
            active_address <= 7'h0c;
            ak_pending <= 1'b0;
            state <= STATE_REQUEST;
          end else if (bmp_pending) begin
            active_sensor <= SENSOR_BMP;
            active_retry <= 1'b0;
            active_release_timestamp <= bmp_release_timestamp;
            active_release_sequence <= bmp_pending_sequence;
            active_address <= bmp_address;
            bmp_pending <= 1'b0;
            state <= STATE_REQUEST;
          end
        end

        STATE_REQUEST: begin
          if (req_ready) begin
            bus_transaction_count <= increment_saturating32(bus_transaction_count);
            read_buffer <= '0;
            received_indices <= '0;
            received_count <= '0;
            bad_receive_index <= 1'b0;
            state <= STATE_RESPONSE;
          end
        end

        STATE_RESPONSE: begin
          if (rx_valid) begin
            if ((rx_index < 15) && !received_indices[rx_index[3:0]]) begin
              read_buffer[rx_index[3:0]*8 +: 8] <= rx_data;
              received_indices[rx_index[3:0]] <= 1'b1;
              received_count <= received_count + 1'b1;
            end else begin
              bad_receive_index <= 1'b1;
            end
          end

          if (rsp_done) begin
            state <= STATE_IDLE;
            if (rsp_error) begin
              bus_error_count <= increment_saturating32(bus_error_count);
              last_i2c_error <= rsp_error_code;
              unique case (active_sensor)
                SENSOR_MPU: begin
                  mpu_i2c_error_count <= increment_saturating32(mpu_i2c_error_count);
                  mpu_missed_sample_count <= increment_saturating32(mpu_missed_sample_count);
                  mpu_consecutive_failures <= increment_saturating32(mpu_consecutive_failures);
                  if (mpu_consecutive_failures >= RUNTIME_FAULT_CONSECUTIVE - 1) runtime_fault <= 1'b1;
                  if (latency32(active_latency) > mpu_max_latency_cycles) mpu_max_latency_cycles <= latency32(active_latency);
                  if (active_latency > 64'(MPU_DEADLINE_CYCLES)) deadline_miss_count <= increment_saturating32(deadline_miss_count);
                end
                SENSOR_AK: begin
                  ak_i2c_error_count <= increment_saturating32(ak_i2c_error_count);
                  ak_missed_sample_count <= increment_saturating32(ak_missed_sample_count);
                  if (latency32(active_latency) > ak_max_latency_cycles) ak_max_latency_cycles <= latency32(active_latency);
                  if (active_latency > 64'(AK_DEADLINE_CYCLES)) deadline_miss_count <= increment_saturating32(deadline_miss_count);
                end
                default: begin
                  bmp_i2c_error_count <= increment_saturating32(bmp_i2c_error_count);
                  bmp_missed_sample_count <= increment_saturating32(bmp_missed_sample_count);
                  if (latency32(active_latency) > bmp_max_latency_cycles) bmp_max_latency_cycles <= latency32(active_latency);
                  if (active_latency > 64'(BMP_DEADLINE_CYCLES)) deadline_miss_count <= increment_saturating32(deadline_miss_count);
                end
              endcase
            end else begin
              unique case (active_sensor)
                SENSOR_MPU: begin
                  if (!exact_receive_count) begin
                    mpu_invalid_sample_count <= increment_saturating32(mpu_invalid_sample_count);
                    mpu_missed_sample_count <= increment_saturating32(mpu_missed_sample_count);
                    mpu_consecutive_failures <= increment_saturating32(mpu_consecutive_failures);
                    if (mpu_consecutive_failures >= RUNTIME_FAULT_CONSECUTIVE - 1) runtime_fault <= 1'b1;
                  end else if (!read_buffer[0] && !active_retry) begin
                    mpu_duplicate_poll_count <= increment_saturating32(mpu_duplicate_poll_count);
                    mpu_retry_count <= increment_saturating32(mpu_retry_count);
                    retry_delay_count <= 64'(MPU_RETRY_DELAY_CYCLES);
                    active_retry <= 1'b1;
                    state <= STATE_RETRY_WAIT;
                  end else if (!read_buffer[0]) begin
                    mpu_duplicate_poll_count <= increment_saturating32(mpu_duplicate_poll_count);
                    mpu_invalid_sample_count <= increment_saturating32(mpu_invalid_sample_count);
                    mpu_missed_sample_count <= increment_saturating32(mpu_missed_sample_count);
                    mpu_consecutive_failures <= increment_saturating32(mpu_consecutive_failures);
                    if (mpu_consecutive_failures >= RUNTIME_FAULT_CONSECUTIVE - 1) runtime_fault <= 1'b1;
                  end else begin
                    mpu_latest_record <= completion_record;
                    mpu_valid <= 1'b1;
                    mpu_sample <= 1'b1;
                    mpu_sample_sequence <= mpu_sample_sequence + 1'b1;
                    mpu_completion_timestamp <= timestamp_cycles;
                    mpu_accepted_sample_count <= increment_saturating32(mpu_accepted_sample_count);
                    mpu_consecutive_failures <= '0;
                    fifo_push_valid <= 1'b1;
                    fifo_push_data <= completion_record;
                  end
                  if (!(!exact_receive_count || read_buffer[0] || active_retry)) begin
                    // The first no-data poll is not a completed sample attempt.
                  end else begin
                    if (latency32(active_latency) > mpu_max_latency_cycles) mpu_max_latency_cycles <= latency32(active_latency);
                    if (active_latency > 64'(MPU_DEADLINE_CYCLES)) deadline_miss_count <= increment_saturating32(deadline_miss_count);
                  end
                end
                SENSOR_AK: begin
                  if (exact_receive_count && read_buffer[0] && !read_buffer[59] && read_buffer[60]) begin
                    ak_latest_record <= completion_record;
                    ak_valid <= 1'b1;
                    ak_sample <= 1'b1;
                    ak_sample_sequence <= ak_sample_sequence + 1'b1;
                    ak_completion_timestamp <= timestamp_cycles;
                    ak_accepted_sample_count <= increment_saturating32(ak_accepted_sample_count);
                    fifo_push_valid <= 1'b1;
                    fifo_push_data <= completion_record;
                  end else begin
                    ak_invalid_sample_count <= increment_saturating32(ak_invalid_sample_count);
                    ak_missed_sample_count <= increment_saturating32(ak_missed_sample_count);
                  end
                  if (exact_receive_count && !read_buffer[0]) ak_duplicate_poll_count <= increment_saturating32(ak_duplicate_poll_count);
                  if (exact_receive_count && read_buffer[1]) begin
                    ak_overrun_count <= increment_saturating32(ak_overrun_count);
                    ak_missed_sample_count <= increment_saturating32(ak_missed_sample_count);
                  end
                  if (latency32(active_latency) > ak_max_latency_cycles) ak_max_latency_cycles <= latency32(active_latency);
                  if (active_latency > 64'(AK_DEADLINE_CYCLES)) deadline_miss_count <= increment_saturating32(deadline_miss_count);
                end
                default: begin
                  if (exact_receive_count &&
                      ({read_buffer[7:0], read_buffer[15:8], read_buffer[23:20]} != 20'h80000) &&
                      ({read_buffer[31:24], read_buffer[39:32], read_buffer[47:44]} != 20'h80000)) begin
                    bmp_latest_record <= completion_record;
                    bmp_valid <= 1'b1;
                    bmp_sample <= 1'b1;
                    bmp_sample_sequence <= bmp_sample_sequence + 1'b1;
                    bmp_completion_timestamp <= timestamp_cycles;
                    bmp_accepted_sample_count <= increment_saturating32(bmp_accepted_sample_count);
                    fifo_push_valid <= 1'b1;
                    fifo_push_data <= completion_record;
                  end else begin
                    bmp_invalid_sample_count <= increment_saturating32(bmp_invalid_sample_count);
                    bmp_missed_sample_count <= increment_saturating32(bmp_missed_sample_count);
                  end
                  if (latency32(active_latency) > bmp_max_latency_cycles) bmp_max_latency_cycles <= latency32(active_latency);
                  if (active_latency > 64'(BMP_DEADLINE_CYCLES)) deadline_miss_count <= increment_saturating32(deadline_miss_count);
                end
              endcase
            end
          end
        end

        STATE_RETRY_WAIT: begin
          if (retry_delay_count > 1) begin
            retry_delay_count <= retry_delay_count - 1'b1;
          end else begin
            retry_delay_count <= '0;
            state <= STATE_REQUEST;
          end
        end

        default: begin
          state <= STATE_IDLE;
          mpu_pending <= 1'b0;
          ak_pending <= 1'b0;
          bmp_pending <= 1'b0;
          runtime_fault <= 1'b1;
        end
      endcase
    end
  end

endmodule
