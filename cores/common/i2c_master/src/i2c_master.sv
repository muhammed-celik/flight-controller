module i2c_master #(
  parameter int unsigned FAST_LOW_CYCLES            = 130,
  parameter int unsigned FAST_HIGH_CYCLES           = 120,
  parameter int unsigned FAST_BUS_FREE_CYCLES       = 130,
  parameter int unsigned STANDARD_LOW_CYCLES        = 500,
  parameter int unsigned STANDARD_HIGH_CYCLES       = 500,
  parameter int unsigned STANDARD_BUS_FREE_CYCLES   = 470,
  parameter int unsigned STRETCH_TIMEOUT_CYCLES     = 100_000,
  parameter int unsigned TRANSACTION_TIMEOUT_CYCLES = 5_000_000
) (
  input  logic       clk,
  input  logic       rst_n,

  input  logic       cmd_valid,
  output logic       cmd_ready,
  input  logic [6:0] cmd_address,
  input  logic [7:0] cmd_write_count,
  input  logic [7:0] cmd_read_count,
  input  logic       cmd_fast_mode,

  input  logic       tx_valid,
  output logic       tx_ready,
  input  logic [7:0] tx_data,

  output logic       rx_valid,
  input  logic       rx_ready,
  output logic [7:0] rx_data,

  output logic       busy,
  output logic       done,
  output logic       error,
  output logic [3:0] error_code,
  output logic [7:0] error_byte_index,

  input  logic       scl_i,
  input  logic       sda_i,
  output logic       scl_drive_low,
  output logic       sda_drive_low
);

  // Completion error codes. Data NACK indices are zero based.
  localparam logic [3:0] ERROR_NONE                = 4'd0;
  localparam logic [3:0] ERROR_INVALID_COMMAND     = 4'd1;
  localparam logic [3:0] ERROR_ADDRESS_NACK        = 4'd2;
  localparam logic [3:0] ERROR_DATA_NACK           = 4'd3;
  localparam logic [3:0] ERROR_STRETCH_TIMEOUT     = 4'd4;
  localparam logic [3:0] ERROR_TRANSACTION_TIMEOUT = 4'd5;
  localparam logic [3:0] ERROR_RECOVERY_FAILED     = 4'd6;

  typedef enum logic [4:0] {
    IDLE,
    BUS_FREE,
    START_HOLD,
    START_SCL_LOW,
    RESTART_LOW,
    RESTART_RAISE_WAIT,
    RESTART_RAISE_HIGH,
    TX_LOW,
    TX_RAISE_WAIT,
    TX_HIGH,
    TX_ACK_LOW,
    TX_ACK_RAISE_WAIT,
    TX_ACK_HIGH,
    TX_WAIT,
    RX_LOW,
    RX_RAISE_WAIT,
    RX_HIGH,
    RX_PRESENT,
    RX_ACK_LOW,
    RX_ACK_RAISE_WAIT,
    RX_ACK_HIGH,
    STOP_LOW,
    STOP_RAISE_WAIT,
    STOP_HIGH,
    STOP_RELEASE,
    RECOVERY_LOW,
    RECOVERY_RAISE_WAIT,
    RECOVERY_HIGH,
    RECOVERY_STOP_LOW,
    RECOVERY_STOP_RAISE_WAIT,
    RECOVERY_STOP_HIGH,
    RECOVERY_STOP_RELEASE
  } state_t;

  state_t state;

  (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) logic [1:0] scl_sync_pipe;
  (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) logic [1:0] sda_sync_pipe;
  logic       scl_sync;
  logic       sda_sync;

  logic [6:0] address_reg;
  logic [7:0] write_count;
  logic [7:0] read_count;
  logic [7:0] write_index;
  logic [7:0] read_index;
  logic [7:0] tx_shift;
  logic [7:0] rx_shift;
  logic [2:0] bit_index;
  logic       tx_is_address;
  logic       address_is_read;

  logic [3:0] pending_error;
  logic [7:0] pending_error_index;
  logic       recovery_for_timeout;
  logic [3:0] recovery_pulse;
  logic [1:0] settle_count;

  int unsigned low_cycles;
  int unsigned high_cycles;
  int unsigned bus_free_cycles;
  int unsigned phase_count;
  int unsigned stretch_count;
  int unsigned transaction_count;

  initial begin
    if ((FAST_LOW_CYCLES == 0) || (FAST_HIGH_CYCLES == 0) ||
        (FAST_BUS_FREE_CYCLES == 0) ||
        (STANDARD_LOW_CYCLES == 0) || (STANDARD_HIGH_CYCLES == 0) ||
        (STANDARD_BUS_FREE_CYCLES == 0) || (STRETCH_TIMEOUT_CYCLES == 0) ||
        (TRANSACTION_TIMEOUT_CYCLES == 0)) begin
      $fatal(1, "i2c_master timing parameters must be nonzero");
    end
  end

  assign scl_sync = scl_sync_pipe[1];
  assign sda_sync = sda_sync_pipe[1];

  always_comb begin
    cmd_ready     = (state == IDLE);
    tx_ready      = (state == TX_WAIT);
    rx_valid      = (state == RX_PRESENT);
    rx_data       = rx_shift;
    busy          = (state != IDLE);
    scl_drive_low = 1'b0;
    sda_drive_low = 1'b0;

    unique case (state)
      START_HOLD: begin
        sda_drive_low = 1'b1;
      end

      START_SCL_LOW,
      STOP_LOW,
      RECOVERY_STOP_LOW: begin
        scl_drive_low = 1'b1;
        sda_drive_low = 1'b1;
      end

      RESTART_LOW: begin
        scl_drive_low = 1'b1;
      end

      TX_LOW: begin
        scl_drive_low = 1'b1;
        sda_drive_low = ~tx_shift[bit_index];
      end

      TX_RAISE_WAIT,
      TX_HIGH: begin
        sda_drive_low = ~tx_shift[bit_index];
      end

      TX_ACK_LOW,
      TX_WAIT,
      RX_LOW,
      RX_PRESENT: begin
        scl_drive_low = 1'b1;
      end

      RX_ACK_LOW: begin
        scl_drive_low = 1'b1;
        sda_drive_low = (read_index != (read_count - 1'b1));
      end

      RX_ACK_RAISE_WAIT,
      RX_ACK_HIGH: begin
        sda_drive_low = (read_index != (read_count - 1'b1));
      end

      STOP_RAISE_WAIT,
      STOP_HIGH,
      RECOVERY_STOP_RAISE_WAIT,
      RECOVERY_STOP_HIGH: begin
        sda_drive_low = 1'b1;
      end

      RECOVERY_LOW: begin
        scl_drive_low = 1'b1;
      end

      default: begin
        scl_drive_low = 1'b0;
        sda_drive_low = 1'b0;
      end
    endcase
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      scl_sync_pipe <= '1;
      sda_sync_pipe <= '1;
    end else begin
      scl_sync_pipe <= {scl_sync_pipe[0], scl_i};
      sda_sync_pipe <= {sda_sync_pipe[0], sda_i};
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      state                  <= IDLE;
      address_reg            <= '0;
      write_count            <= '0;
      read_count             <= '0;
      write_index            <= '0;
      read_index             <= '0;
      tx_shift               <= '0;
      rx_shift               <= '0;
      bit_index              <= '0;
      tx_is_address          <= 1'b0;
      address_is_read        <= 1'b0;
      pending_error          <= ERROR_NONE;
      pending_error_index    <= '0;
      recovery_for_timeout   <= 1'b0;
      recovery_pulse         <= '0;
      settle_count           <= '0;
      low_cycles             <= STANDARD_LOW_CYCLES;
      high_cycles            <= STANDARD_HIGH_CYCLES;
      bus_free_cycles        <= STANDARD_BUS_FREE_CYCLES;
      phase_count            <= 0;
      stretch_count          <= 0;
      transaction_count      <= 0;
      done                   <= 1'b0;
      error                  <= 1'b0;
      error_code             <= ERROR_NONE;
      error_byte_index       <= '0;
    end else begin
      done <= 1'b0;

      if (state == IDLE) begin
        transaction_count <= 0;
      end else if (!recovery_for_timeout) begin
        transaction_count <= transaction_count + 1;
      end

      unique case (state)
        IDLE: begin
          phase_count   <= 0;
          stretch_count <= 0;

          if (cmd_valid) begin
            address_reg          <= cmd_address;
            write_count          <= cmd_write_count;
            read_count           <= cmd_read_count;
            write_index          <= '0;
            read_index           <= '0;
            pending_error        <= ERROR_NONE;
            pending_error_index  <= '0;
            recovery_for_timeout <= 1'b0;
            settle_count         <= '0;
            error                <= 1'b0;
            error_code           <= ERROR_NONE;
            error_byte_index     <= '0;

            if (cmd_fast_mode) begin
              low_cycles      <= FAST_LOW_CYCLES;
              high_cycles     <= FAST_HIGH_CYCLES;
              bus_free_cycles <= FAST_BUS_FREE_CYCLES;
            end else begin
              low_cycles      <= STANDARD_LOW_CYCLES;
              high_cycles     <= STANDARD_HIGH_CYCLES;
              bus_free_cycles <= STANDARD_BUS_FREE_CYCLES;
            end

            if ((cmd_write_count == 0) && (cmd_read_count == 0)) begin
              done       <= 1'b1;
              error      <= 1'b1;
              error_code <= ERROR_INVALID_COMMAND;
            end else begin
              address_is_read <= (cmd_write_count == 0);
              tx_is_address   <= 1'b1;
              tx_shift        <= {cmd_address, (cmd_write_count == 0)};
              state           <= BUS_FREE;
            end
          end
        end

        BUS_FREE: begin
          if (settle_count != 2) begin
            settle_count <= settle_count + 1'b1;
          end else if (!scl_sync || !sda_sync) begin
            phase_count          <= 0;
            stretch_count        <= 0;
            recovery_pulse       <= 0;
            recovery_for_timeout <= 1'b0;
            state                <= RECOVERY_LOW;
          end else if (phase_count == (bus_free_cycles - 1)) begin
            phase_count <= 0;
            state       <= START_HOLD;
          end else begin
            phase_count <= phase_count + 1;
          end
        end

        START_HOLD: begin
          if (!scl_sync) begin
            if (stretch_count == (STRETCH_TIMEOUT_CYCLES - 1)) begin
              state            <= IDLE;
              done             <= 1'b1;
              error            <= 1'b1;
              error_code       <= ERROR_STRETCH_TIMEOUT;
              error_byte_index <= '0;
            end else begin
              stretch_count <= stretch_count + 1;
            end
          end else begin
            stretch_count <= 0;
            if (phase_count == (high_cycles - 1)) begin
              phase_count <= 0;
              state       <= START_SCL_LOW;
            end else begin
              phase_count <= phase_count + 1;
            end
          end
        end

        START_SCL_LOW: begin
          if (phase_count == (low_cycles - 1)) begin
            phase_count <= 0;
            bit_index   <= 3'd7;
            state       <= TX_LOW;
          end else begin
            phase_count <= phase_count + 1;
          end
        end

        RESTART_LOW: begin
          if (phase_count == (low_cycles - 1)) begin
            phase_count   <= 0;
            stretch_count <= 0;
            state         <= RESTART_RAISE_WAIT;
          end else begin
            phase_count <= phase_count + 1;
          end
        end

        RESTART_RAISE_WAIT: begin
          if (scl_sync) begin
            phase_count   <= 0;
            stretch_count <= 0;
            state         <= RESTART_RAISE_HIGH;
          end else if (stretch_count == (STRETCH_TIMEOUT_CYCLES - 1)) begin
            state            <= IDLE;
            done             <= 1'b1;
            error            <= 1'b1;
            error_code       <= ERROR_STRETCH_TIMEOUT;
            error_byte_index <= '0;
          end else begin
            stretch_count <= stretch_count + 1;
          end
        end

        RESTART_RAISE_HIGH: begin
          if (!scl_sync) begin
            if (stretch_count == (STRETCH_TIMEOUT_CYCLES - 1)) begin
              state            <= IDLE;
              done             <= 1'b1;
              error            <= 1'b1;
              error_code       <= ERROR_STRETCH_TIMEOUT;
              error_byte_index <= '0;
            end else begin
              stretch_count <= stretch_count + 1;
            end
          end else begin
            stretch_count <= 0;
            if (phase_count == (high_cycles - 1)) begin
              phase_count <= 0;
              state       <= START_HOLD;
            end else begin
              phase_count <= phase_count + 1;
            end
          end
        end

        TX_LOW: begin
          if (phase_count == (low_cycles - 1)) begin
            phase_count   <= 0;
            stretch_count <= 0;
            state         <= TX_RAISE_WAIT;
          end else begin
            phase_count <= phase_count + 1;
          end
        end

        TX_RAISE_WAIT: begin
          if (scl_sync) begin
            phase_count   <= 0;
            stretch_count <= 0;
            state         <= TX_HIGH;
          end else if (stretch_count == (STRETCH_TIMEOUT_CYCLES - 1)) begin
            state            <= IDLE;
            done             <= 1'b1;
            error            <= 1'b1;
            error_code       <= ERROR_STRETCH_TIMEOUT;
            error_byte_index <= '0;
          end else begin
            stretch_count <= stretch_count + 1;
          end
        end

        TX_HIGH: begin
          if (!scl_sync) begin
            if (stretch_count == (STRETCH_TIMEOUT_CYCLES - 1)) begin
              state            <= IDLE;
              done             <= 1'b1;
              error            <= 1'b1;
              error_code       <= ERROR_STRETCH_TIMEOUT;
              error_byte_index <= '0;
            end else begin
              stretch_count <= stretch_count + 1;
            end
          end else begin
            stretch_count <= 0;
            if (phase_count == (high_cycles - 1)) begin
              phase_count <= 0;
              if (bit_index == 0) begin
                state <= TX_ACK_LOW;
              end else begin
                bit_index <= bit_index - 1'b1;
                state     <= TX_LOW;
              end
            end else begin
              phase_count <= phase_count + 1;
            end
          end
        end

        TX_ACK_LOW: begin
          if (phase_count == (low_cycles - 1)) begin
            phase_count   <= 0;
            stretch_count <= 0;
            state         <= TX_ACK_RAISE_WAIT;
          end else begin
            phase_count <= phase_count + 1;
          end
        end

        TX_ACK_RAISE_WAIT: begin
          if (scl_sync) begin
            phase_count   <= 0;
            stretch_count <= 0;
            state         <= TX_ACK_HIGH;
          end else if (stretch_count == (STRETCH_TIMEOUT_CYCLES - 1)) begin
            state            <= IDLE;
            done             <= 1'b1;
            error            <= 1'b1;
            error_code       <= ERROR_STRETCH_TIMEOUT;
            error_byte_index <= '0;
          end else begin
            stretch_count <= stretch_count + 1;
          end
        end

        TX_ACK_HIGH: begin
          if (!scl_sync) begin
            if (stretch_count == (STRETCH_TIMEOUT_CYCLES - 1)) begin
              state            <= IDLE;
              done             <= 1'b1;
              error            <= 1'b1;
              error_code       <= ERROR_STRETCH_TIMEOUT;
              error_byte_index <= '0;
            end else begin
              stretch_count <= stretch_count + 1;
            end
          end else begin
            stretch_count <= 0;
            if (phase_count == (high_cycles - 1)) begin
              phase_count <= 0;
              if (sda_sync) begin
                pending_error       <= tx_is_address ? ERROR_ADDRESS_NACK : ERROR_DATA_NACK;
                pending_error_index <= tx_is_address ? 8'd0 : write_index;
                state               <= STOP_LOW;
              end else if (tx_is_address) begin
                if (address_is_read) begin
                  bit_index <= 3'd7;
                  rx_shift  <= '0;
                  state     <= RX_LOW;
                end else begin
                  tx_is_address <= 1'b0;
                  state         <= TX_WAIT;
                end
              end else if (write_index == (write_count - 1'b1)) begin
                if (read_count != 0) begin
                  address_is_read <= 1'b1;
                  tx_is_address   <= 1'b1;
                  tx_shift        <= {address_reg, 1'b1};
                  state           <= RESTART_LOW;
                end else begin
                  state <= STOP_LOW;
                end
              end else begin
                write_index <= write_index + 1'b1;
                state       <= TX_WAIT;
              end
            end else begin
              phase_count <= phase_count + 1;
            end
          end
        end

        TX_WAIT: begin
          if (tx_valid) begin
            tx_shift   <= tx_data;
            bit_index <= 3'd7;
            phase_count <= 0;
            state       <= TX_LOW;
          end
        end

        RX_LOW: begin
          if (phase_count == (low_cycles - 1)) begin
            phase_count   <= 0;
            stretch_count <= 0;
            state         <= RX_RAISE_WAIT;
          end else begin
            phase_count <= phase_count + 1;
          end
        end

        RX_RAISE_WAIT: begin
          if (scl_sync) begin
            phase_count   <= 0;
            stretch_count <= 0;
            state         <= RX_HIGH;
          end else if (stretch_count == (STRETCH_TIMEOUT_CYCLES - 1)) begin
            state            <= IDLE;
            done             <= 1'b1;
            error            <= 1'b1;
            error_code       <= ERROR_STRETCH_TIMEOUT;
            error_byte_index <= '0;
          end else begin
            stretch_count <= stretch_count + 1;
          end
        end

        RX_HIGH: begin
          if (!scl_sync) begin
            if (stretch_count == (STRETCH_TIMEOUT_CYCLES - 1)) begin
              state            <= IDLE;
              done             <= 1'b1;
              error            <= 1'b1;
              error_code       <= ERROR_STRETCH_TIMEOUT;
              error_byte_index <= '0;
            end else begin
              stretch_count <= stretch_count + 1;
            end
          end else begin
            stretch_count <= 0;
            if (phase_count == (high_cycles - 1)) begin
              rx_shift   <= {rx_shift[6:0], sda_sync};
              phase_count <= 0;
              if (bit_index == 0) begin
                state <= RX_PRESENT;
              end else begin
                bit_index <= bit_index - 1'b1;
                state     <= RX_LOW;
              end
            end else begin
              phase_count <= phase_count + 1;
            end
          end
        end

        RX_PRESENT: begin
          if (rx_ready) begin
            phase_count <= 0;
            state       <= RX_ACK_LOW;
          end
        end

        RX_ACK_LOW: begin
          if (phase_count == (low_cycles - 1)) begin
            phase_count   <= 0;
            stretch_count <= 0;
            state         <= RX_ACK_RAISE_WAIT;
          end else begin
            phase_count <= phase_count + 1;
          end
        end

        RX_ACK_RAISE_WAIT: begin
          if (scl_sync) begin
            phase_count   <= 0;
            stretch_count <= 0;
            state         <= RX_ACK_HIGH;
          end else if (stretch_count == (STRETCH_TIMEOUT_CYCLES - 1)) begin
            state            <= IDLE;
            done             <= 1'b1;
            error            <= 1'b1;
            error_code       <= ERROR_STRETCH_TIMEOUT;
            error_byte_index <= '0;
          end else begin
            stretch_count <= stretch_count + 1;
          end
        end

        RX_ACK_HIGH: begin
          if (!scl_sync) begin
            if (stretch_count == (STRETCH_TIMEOUT_CYCLES - 1)) begin
              state            <= IDLE;
              done             <= 1'b1;
              error            <= 1'b1;
              error_code       <= ERROR_STRETCH_TIMEOUT;
              error_byte_index <= '0;
            end else begin
              stretch_count <= stretch_count + 1;
            end
          end else begin
            stretch_count <= 0;
            if (phase_count == (high_cycles - 1)) begin
              phase_count <= 0;
              if (read_index == (read_count - 1'b1)) begin
                state <= STOP_LOW;
              end else begin
                read_index <= read_index + 1'b1;
                bit_index  <= 3'd7;
                rx_shift   <= '0;
                state      <= RX_LOW;
              end
            end else begin
              phase_count <= phase_count + 1;
            end
          end
        end

        STOP_LOW: begin
          if (phase_count == (low_cycles - 1)) begin
            phase_count   <= 0;
            stretch_count <= 0;
            state         <= STOP_RAISE_WAIT;
          end else begin
            phase_count <= phase_count + 1;
          end
        end

        STOP_RAISE_WAIT: begin
          if (scl_sync) begin
            phase_count   <= 0;
            stretch_count <= 0;
            state         <= STOP_HIGH;
          end else if (stretch_count == (STRETCH_TIMEOUT_CYCLES - 1)) begin
            state            <= IDLE;
            done             <= 1'b1;
            error            <= 1'b1;
            error_code       <= ERROR_STRETCH_TIMEOUT;
            error_byte_index <= '0;
          end else begin
            stretch_count <= stretch_count + 1;
          end
        end

        STOP_HIGH: begin
          if (!scl_sync) begin
            if (stretch_count == (STRETCH_TIMEOUT_CYCLES - 1)) begin
              state            <= IDLE;
              done             <= 1'b1;
              error            <= 1'b1;
              error_code       <= ERROR_STRETCH_TIMEOUT;
              error_byte_index <= '0;
            end else begin
              stretch_count <= stretch_count + 1;
            end
          end else begin
            stretch_count <= 0;
            if (phase_count == (high_cycles - 1)) begin
              phase_count <= 0;
              settle_count <= 0;
              state       <= STOP_RELEASE;
            end else begin
              phase_count <= phase_count + 1;
            end
          end
        end

        STOP_RELEASE: begin
          if (settle_count != 2) begin
            settle_count <= settle_count + 1'b1;
          end else if (!scl_sync) begin
            phase_count <= 0;
            if (stretch_count == (STRETCH_TIMEOUT_CYCLES - 1)) begin
              state            <= IDLE;
              done             <= 1'b1;
              error            <= 1'b1;
              error_code       <= ERROR_STRETCH_TIMEOUT;
              error_byte_index <= '0;
            end else begin
              stretch_count <= stretch_count + 1;
            end
          end else if (!sda_sync) begin
            phase_count <= 0;
            if (stretch_count == (bus_free_cycles - 1)) begin
              state            <= IDLE;
              done             <= 1'b1;
              error            <= 1'b1;
              error_code       <= ERROR_RECOVERY_FAILED;
              error_byte_index <= '0;
            end else begin
              stretch_count <= stretch_count + 1;
            end
          end else if (phase_count == (bus_free_cycles - 1)) begin
            state            <= IDLE;
            done             <= 1'b1;
            error            <= (pending_error != ERROR_NONE);
            error_code       <= pending_error;
            error_byte_index <= pending_error_index;
          end else begin
            stretch_count <= 0;
            phase_count   <= phase_count + 1;
          end
        end

        RECOVERY_LOW: begin
          if (phase_count == (low_cycles - 1)) begin
            phase_count   <= 0;
            stretch_count <= 0;
            state         <= RECOVERY_RAISE_WAIT;
          end else begin
            phase_count <= phase_count + 1;
          end
        end

        RECOVERY_RAISE_WAIT: begin
          if (scl_sync) begin
            phase_count   <= 0;
            stretch_count <= 0;
            state         <= RECOVERY_HIGH;
          end else if (stretch_count == (STRETCH_TIMEOUT_CYCLES - 1)) begin
            state            <= IDLE;
            done             <= 1'b1;
            error            <= 1'b1;
            error_code       <= recovery_for_timeout ? ERROR_TRANSACTION_TIMEOUT : ERROR_STRETCH_TIMEOUT;
            error_byte_index <= '0;
          end else begin
            stretch_count <= stretch_count + 1;
          end
        end

        RECOVERY_HIGH: begin
          if (!scl_sync) begin
            if (stretch_count == (STRETCH_TIMEOUT_CYCLES - 1)) begin
              state            <= IDLE;
              done             <= 1'b1;
              error            <= 1'b1;
              error_code       <= recovery_for_timeout ? ERROR_TRANSACTION_TIMEOUT : ERROR_STRETCH_TIMEOUT;
              error_byte_index <= '0;
            end else begin
              stretch_count <= stretch_count + 1;
            end
          end else begin
            stretch_count <= 0;
            if (phase_count == (high_cycles - 1)) begin
              phase_count <= 0;
              if (recovery_pulse == 4'd8) begin
                state <= RECOVERY_STOP_LOW;
              end else begin
                recovery_pulse <= recovery_pulse + 1'b1;
                state          <= RECOVERY_LOW;
              end
            end else begin
              phase_count <= phase_count + 1;
            end
          end
        end

        RECOVERY_STOP_LOW: begin
          if (phase_count == (low_cycles - 1)) begin
            phase_count   <= 0;
            stretch_count <= 0;
            state         <= RECOVERY_STOP_RAISE_WAIT;
          end else begin
            phase_count <= phase_count + 1;
          end
        end

        RECOVERY_STOP_RAISE_WAIT: begin
          if (scl_sync) begin
            phase_count   <= 0;
            stretch_count <= 0;
            state         <= RECOVERY_STOP_HIGH;
          end else if (stretch_count == (STRETCH_TIMEOUT_CYCLES - 1)) begin
            state            <= IDLE;
            done             <= 1'b1;
            error            <= 1'b1;
            error_code       <= recovery_for_timeout ? ERROR_TRANSACTION_TIMEOUT : ERROR_STRETCH_TIMEOUT;
            error_byte_index <= '0;
          end else begin
            stretch_count <= stretch_count + 1;
          end
        end

        RECOVERY_STOP_HIGH: begin
          if (!scl_sync) begin
            if (stretch_count == (STRETCH_TIMEOUT_CYCLES - 1)) begin
              state            <= IDLE;
              done             <= 1'b1;
              error            <= 1'b1;
              error_code       <= recovery_for_timeout ? ERROR_TRANSACTION_TIMEOUT : ERROR_STRETCH_TIMEOUT;
              error_byte_index <= '0;
            end else begin
              stretch_count <= stretch_count + 1;
            end
          end else begin
            stretch_count <= 0;
            if (phase_count == (high_cycles - 1)) begin
              phase_count <= 0;
              settle_count <= 0;
              state       <= RECOVERY_STOP_RELEASE;
            end else begin
              phase_count <= phase_count + 1;
            end
          end
        end

        RECOVERY_STOP_RELEASE: begin
          if (settle_count != 2) begin
            settle_count <= settle_count + 1'b1;
          end else if (!scl_sync) begin
            phase_count <= 0;
            if (stretch_count == (STRETCH_TIMEOUT_CYCLES - 1)) begin
              state            <= IDLE;
              done             <= 1'b1;
              error            <= 1'b1;
              error_code       <= recovery_for_timeout ? ERROR_TRANSACTION_TIMEOUT : ERROR_STRETCH_TIMEOUT;
              error_byte_index <= '0;
            end else begin
              stretch_count <= stretch_count + 1;
            end
          end else if (!sda_sync) begin
            phase_count <= 0;
            if (stretch_count == (bus_free_cycles - 1)) begin
              state            <= IDLE;
              done             <= 1'b1;
              error            <= 1'b1;
              error_code       <= recovery_for_timeout ? ERROR_TRANSACTION_TIMEOUT : ERROR_RECOVERY_FAILED;
              error_byte_index <= '0;
            end else begin
              stretch_count <= stretch_count + 1;
            end
          end else if (phase_count == (bus_free_cycles - 1)) begin
            phase_count   <= 0;
            stretch_count <= 0;
            if (recovery_for_timeout) begin
              state            <= IDLE;
              done             <= 1'b1;
              error            <= 1'b1;
              error_code       <= ERROR_TRANSACTION_TIMEOUT;
              error_byte_index <= '0;
            end else begin
              settle_count <= 0;
              state <= BUS_FREE;
            end
          end else begin
            stretch_count <= 0;
            phase_count   <= phase_count + 1;
          end
        end

        default: begin
          state            <= IDLE;
          done             <= 1'b1;
          error            <= 1'b1;
          error_code       <= ERROR_RECOVERY_FAILED;
          error_byte_index <= '0;
        end
      endcase

      if ((state != IDLE) && !recovery_for_timeout &&
          (transaction_count == (TRANSACTION_TIMEOUT_CYCLES - 1))) begin
        pending_error        <= ERROR_TRANSACTION_TIMEOUT;
        pending_error_index  <= '0;
        recovery_for_timeout <= 1'b1;
        recovery_pulse       <= 0;
        phase_count          <= 0;
        stretch_count        <= 0;
        state                <= RECOVERY_LOW;
      end
    end
  end

endmodule
