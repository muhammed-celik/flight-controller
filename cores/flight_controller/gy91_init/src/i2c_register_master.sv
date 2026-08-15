module i2c_register_master (
  input  logic       clk,
  input  logic       rst_n,

  input  logic       req_valid,
  output logic       req_ready,
  input  logic [6:0] req_address,
  input  logic [7:0] req_register,
  input  logic       req_write,
  input  logic [7:0] req_write_data,
  input  logic [7:0] req_read_count,

  output logic       rx_valid,
  input  logic       rx_ready,
  output logic [7:0] rx_data,
  output logic [7:0] rx_index,

  output logic       rsp_done,
  output logic       rsp_error,
  output logic [3:0] rsp_error_code,
  output logic [7:0] rsp_error_index,

  output logic       i2c_cmd_valid,
  input  logic       i2c_cmd_ready,
  output logic [6:0] i2c_cmd_address,
  output logic [7:0] i2c_cmd_write_count,
  output logic [7:0] i2c_cmd_read_count,
  output logic       i2c_cmd_fast_mode,

  output logic       i2c_tx_valid,
  input  logic       i2c_tx_ready,
  output logic [7:0] i2c_tx_data,

  input  logic       i2c_rx_valid,
  output logic       i2c_rx_ready,
  input  logic [7:0] i2c_rx_data,

  input  logic       i2c_done,
  input  logic       i2c_error,
  input  logic [3:0] i2c_error_code,
  input  logic [7:0] i2c_error_byte_index
);

  localparam logic [3:0] ERROR_INVALID_COMMAND = 4'd1;

  typedef enum logic [2:0] {
    IDLE,
    SEND_COMMAND,
    SEND_REGISTER,
    SEND_DATA,
    WAIT_RESPONSE,
    LOCAL_RESPONSE
  } state_t;

  state_t state;
  logic [6:0] address_reg;
  logic [7:0] register_reg;
  logic       write_reg;
  logic [7:0] write_data_reg;
  logic [7:0] read_count_reg;
  logic [7:0] read_index_reg;

  always_comb begin
    req_ready           = (state == IDLE);
    rx_valid            = (state == WAIT_RESPONSE) && i2c_rx_valid;
    rx_data             = i2c_rx_data;
    rx_index            = read_index_reg;

    i2c_cmd_valid       = (state == SEND_COMMAND);
    i2c_cmd_address     = address_reg;
    i2c_cmd_write_count = write_reg ? 8'd2 : 8'd1;
    i2c_cmd_read_count  = write_reg ? 8'd0 : read_count_reg;
    i2c_cmd_fast_mode   = 1'b0;

    i2c_tx_valid        = (state == SEND_REGISTER) || (state == SEND_DATA);
    i2c_tx_data         = (state == SEND_DATA) ? write_data_reg : register_reg;
    i2c_rx_ready        = (state == WAIT_RESPONSE) && rx_ready;
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      state             <= IDLE;
      address_reg       <= '0;
      register_reg      <= '0;
      write_reg         <= 1'b0;
      write_data_reg    <= '0;
      read_count_reg    <= '0;
      read_index_reg    <= '0;
      rsp_done          <= 1'b0;
      rsp_error         <= 1'b0;
      rsp_error_code    <= '0;
      rsp_error_index   <= '0;
    end else begin
      rsp_done <= 1'b0;

      unique case (state)
        IDLE: begin
          if (req_valid) begin
            address_reg    <= req_address;
            register_reg   <= req_register;
            write_reg      <= req_write;
            write_data_reg <= req_write_data;
            read_count_reg <= req_read_count;
            read_index_reg <= '0;

            if (!req_write && (req_read_count == 0)) begin
              rsp_error       <= 1'b1;
              rsp_error_code  <= ERROR_INVALID_COMMAND;
              rsp_error_index <= '0;
              state           <= LOCAL_RESPONSE;
            end else begin
              state <= SEND_COMMAND;
            end
          end
        end

        SEND_COMMAND: begin
          if (i2c_cmd_ready) begin
            state <= SEND_REGISTER;
          end
        end

        SEND_REGISTER: begin
          if (i2c_done) begin
            rsp_done        <= 1'b1;
            rsp_error       <= i2c_error;
            rsp_error_code  <= i2c_error_code;
            rsp_error_index <= i2c_error_byte_index;
            state           <= IDLE;
          end else if (i2c_tx_ready) begin
            state <= write_reg ? SEND_DATA : WAIT_RESPONSE;
          end
        end

        SEND_DATA: begin
          if (i2c_done) begin
            rsp_done        <= 1'b1;
            rsp_error       <= i2c_error;
            rsp_error_code  <= i2c_error_code;
            rsp_error_index <= i2c_error_byte_index;
            state           <= IDLE;
          end else if (i2c_tx_ready) begin
            state <= WAIT_RESPONSE;
          end
        end

        WAIT_RESPONSE: begin
          if (i2c_rx_valid && rx_ready) begin
            read_index_reg <= read_index_reg + 1'b1;
          end

          if (i2c_done) begin
            rsp_done        <= 1'b1;
            rsp_error       <= i2c_error;
            rsp_error_code  <= i2c_error_code;
            rsp_error_index <= i2c_error_byte_index;
            state           <= IDLE;
          end
        end

        LOCAL_RESPONSE: begin
          rsp_done <= 1'b1;
          state    <= IDLE;
        end

        default: begin
          state             <= IDLE;
          rsp_error         <= 1'b1;
          rsp_error_code    <= ERROR_INVALID_COMMAND;
          rsp_error_index   <= '0;
        end
      endcase
    end
  end

endmodule
