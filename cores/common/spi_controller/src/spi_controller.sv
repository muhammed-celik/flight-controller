module spi_controller (
  input logic i_clk,
  input logic i_rstn,
  //Top Level Interface
  input logic i_cmd_valid,
  input logic i_cmd_type, // 1: Read, 0: Write
  input logic [4:0] i_cmd_nbytes, // Number of bytes to transfer (minimum 1 (5'b00000), maximum 32 (5'b11111))
  input logic [7:0] i_cmd_addr, // Register Address
  input logic i_cmd_data_valid, // Register Data for Write is valid
  input logic [7:0] i_cmd_data, // Register Data for Write

  output logic o_cmd_ready, // Ready to accept new command
  output logic o_cmd_data_valid, // Register Data for Read is valid
  output logic [7:0] o_cmd_data, // Register Data for Read
  //SPI Pinout
  output logic o_cs,
  output logic o_sclk,
  output logic o_mosi,
  input  logic i_miso
);

localparam bit CPOL = 1;
localparam bit CPHA = 1;

logic spi_en;
logic [7:0] spi_data_in;
logic [7:0] spi_data_out;
logic spi_done;

spi_master # (
  .CPOL(CPOL),
  .CPHA(CPHA)
)
spi_master_inst (
  .i_clk(i_clk),
  .i_rstn(i_rstn),
  .i_en(spi_en),
  .i_data(spi_data_in),
  .o_done(spi_done),
  .o_data(spi_data_out),
  .o_cs(o_cs),
  .o_sclk(o_sclk),
  .o_mosi(o_mosi),
  .i_miso(i_miso)
);

typedef enum logic [1:0] {
  ST_IDLE,
  ST_SETUP,
  ST_CMD_READ,
  ST_CMD_WRITE
} state_t;

state_t state;
logic cmd_ready_int;
logic cmd_type_reg;
logic [4:0] cmd_nbytes_reg;

assign o_cmd_ready = cmd_ready_int;


always_ff @(posedge i_clk or negedge i_rstn) begin
  if(!i_rstn) begin
    state <= ST_IDLE;
    spi_en <= 1'b0;
    spi_data_in <= 8'h00;
    cmd_ready_int <= 1'b1;
    o_cmd_data_valid <= 1'b0;
    o_cmd_data <= 8'h00;
    cmd_type_reg <= 1'b0;
    cmd_nbytes_reg <= '0;
  end else begin
    case (state)
      ST_IDLE: begin
        if(i_cmd_valid && cmd_ready_int) begin
          state <= ST_SETUP;
          cmd_ready_int <= 1'b0;
          spi_data_in <= {i_cmd_type, i_cmd_addr[6:0]}; // Send the register address first and RW bit
          spi_en <= 1'b1;
          cmd_type_reg <= i_cmd_type;
          cmd_nbytes_reg <= i_cmd_nbytes;
        end else begin
          cmd_ready_int <= 1'b1;
          spi_en <= 1'b0;
          o_cmd_data_valid <= 1'b0;
        end
      end

      ST_SETUP: begin
        if(spi_done) begin
          if(cmd_type_reg) begin // Read operation
            state <= ST_CMD_READ;
          end else begin // Write operation
            state <= ST_CMD_WRITE;
          end
        end else begin
          spi_en <= 1'b0; // Wait for SPI transfer to complete
        end
      end

      ST_CMD_READ: begin
        if(spi_done) begin
          o_cmd_data_valid <= 1'b1;
          o_cmd_data <= spi_data_out;
          if(cmd_nbytes_reg == 0) begin
            state <= ST_IDLE;
            cmd_ready_int <= 1'b1;
          end else begin
            spi_data_in <= 8'h00; // Send dummy data to read
            spi_en <= 1'b1;
            cmd_nbytes_reg <= cmd_nbytes_reg - 1;
          end
        end else begin
          spi_en <= 1'b0; // Wait for SPI transfer to complete
        end
      end

      ST_CMD_WRITE

    endcase
  end
end

endmodule