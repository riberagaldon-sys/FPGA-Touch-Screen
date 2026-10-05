`timescale 1ns/1ps

module top_chapter6_touch #(
    parameter integer CLK_HZ   = 50_000_000,
    parameter integer POR_BITS = 21,
    parameter integer LED_MODE = 1,
    parameter integer UART_CLK_HZ = 100_000_000,
    parameter integer UART_BAUD = 115_200
)(
    input  wire       sys_clk,
    output wire       TOUCH_SCL,
    inout  wire       TOUCH_SDA,
    inout  wire       TOUCH_INT,
    output wire       TOUCH_RST,
    output wire       UART_TXD,
    output wire [4:0] led
);

    wire rst_n;

    por_reset #(
        .COUNTER_BITS(POR_BITS)
    ) u_por_reset (
        .clk   (sys_clk),
        .rst_n (rst_n)
    );

    (* MARK_DEBUG = "TRUE" *) wire [31:0] touch_data;
    (* MARK_DEBUG = "TRUE" *) wire i2c_ack;
    (* MARK_DEBUG = "TRUE" *) wire i2c_done;
    (* MARK_DEBUG = "TRUE" *) wire once_byte_done;
    (* MARK_DEBUG = "TRUE" *) wire touch_valid_raw;
    (* MARK_DEBUG = "TRUE" *) wire ft_flag;
    (* MARK_DEBUG = "TRUE" *) wire [7:0] i2c_data_r;
    (* MARK_DEBUG = "TRUE" *) wire [15:0] chip_version;
    (* MARK_DEBUG = "TRUE" *) wire [15:0] touch_x_raw;
    (* MARK_DEBUG = "TRUE" *) wire [15:0] touch_y_raw;
    (* MARK_DEBUG = "TRUE" *) wire [6:0] touch_state;

    touch_top_ch34 #(
        .CLK_FREQ    (CLK_HZ),
        .I2C_FREQ    (250_000),
        .REG_NUM_WID (8)
    ) u_touch_top (
        .clk                (sys_clk),
        .rst_n              (rst_n),
        .touch_rst_n        (TOUCH_RST),
        .touch_int          (TOUCH_INT),
        .touch_scl          (TOUCH_SCL),
        .touch_sda          (TOUCH_SDA),
        .data               (touch_data),
        .dbg_i2c_ack        (i2c_ack),
        .dbg_i2c_done       (i2c_done),
        .dbg_once_byte_done (once_byte_done),
        .dbg_i2c_data_r     (i2c_data_r),
        .dbg_touch_valid    (touch_valid_raw),
        .dbg_ft_flag        (ft_flag),
        .dbg_chip_version   (chip_version),
        .dbg_x              (touch_x_raw),
        .dbg_y              (touch_y_raw),
        .dbg_state          (touch_state)
    );

    localparam [6:0] ST_CHECK_TOUCH  = 7'b001_0000;
    localparam [6:0] ST_GET_COORD    = 7'b010_0000;
    localparam [6:0] ST_COORD_HANDLE = 7'b100_0000;

    (* MARK_DEBUG = "TRUE" *) reg [6:0] touch_state_d;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)
            touch_state_d <= 7'b000_0001;
        else
            touch_state_d <= touch_state;
    end

    (* MARK_DEBUG = "TRUE" *) wire coord_update_pulse;

    assign coord_update_pulse =
        (touch_state_d == ST_COORD_HANDLE) &&
        (touch_state   == ST_CHECK_TOUCH);

    wire driver_ready;

    assign driver_ready =
        (touch_state == 7'b000_1000) ||
        (touch_state == ST_CHECK_TOUCH) ||
        (touch_state == ST_GET_COORD) ||
        (touch_state == ST_COORD_HANDLE);

    wire touch_hold;

    pulse_stretcher #(
        .HOLD_CYCLES(CLK_HZ / 2)
    ) u_hold (
        .clk       (sys_clk),
        .rst_n     (rst_n),
        .pulse_in  (coord_update_pulse),
        .level_out (touch_hold)
    );

    (* MARK_DEBUG = "TRUE" *) reg touch_seen;
    (* MARK_DEBUG = "TRUE" *) reg last_right;
    (* MARK_DEBUG = "TRUE" *) reg last_lower;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            touch_seen <= 1'b0;
            last_right <= 1'b0;
            last_lower <= 1'b0;
        end
        else if (coord_update_pulse) begin
            touch_seen <= 1'b1;
            last_right <= (touch_x_raw >= 16'd512);
            last_lower <= (touch_y_raw >= 16'd300);
        end
    end

    wire i2c_activity_hold;

    pulse_stretcher #(
        .HOLD_CYCLES(CLK_HZ / 10)
    ) u_i2c_hold (
        .clk       (sys_clk),
        .rst_n     (rst_n),
        .pulse_in  (i2c_done),
        .level_out (i2c_activity_hold)
    );

    wire [4:0] led_diag;
    assign led_diag = {
        i2c_activity_hold,
        touch_seen,
        touch_hold,
        (driver_ready && ft_flag),
        driver_ready
    };

    wire [4:0] led_coord;
    assign led_coord = {
        touch_seen,
        (driver_ready && ft_flag),
        last_lower,
        last_right,
        touch_hold
    };

    assign led = (LED_MODE == 0) ? led_diag : led_coord;

    // A line is sent only after a complete, valid coordinate has been read.
    // No UART data is produced while the screen is idle.
    wire uart_busy;

    touch_coord_uart #(
        // Calculate UART baud from the measured 100 MHz W19 clock while
        // retaining the already verified touch configuration above.
        .CLK_HZ (UART_CLK_HZ),
        .BAUD   (UART_BAUD)
    ) u_touch_coord_uart (
        .clk         (sys_clk),
        .rst_n       (rst_n),
        .coord_valid (coord_update_pulse),
        .coord_x     (touch_x_raw),
        .coord_y     (touch_y_raw),
        .uart_txd    (UART_TXD),
        .busy        (uart_busy)
    );

endmodule
