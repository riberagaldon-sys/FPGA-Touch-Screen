`timescale 1ns/1ps

// Sends one ASCII line for each completed touch coordinate:
// X=0123,Y=0456\r\n
module touch_coord_uart #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115_200
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        coord_valid,
    input  wire [15:0] coord_x,
    input  wire [15:0] coord_y,
    output wire        uart_txd,
    output wire        busy
);

    reg        tx_start;
    reg [7:0]  tx_data;
    wire       tx_busy;
    wire       tx_done;

    reg        message_active;
    reg        byte_inflight;
    reg [3:0]  char_index;

    reg [3:0] x_thousands;
    reg [3:0] x_hundreds;
    reg [3:0] x_tens;
    reg [3:0] x_ones;
    reg [3:0] y_thousands;
    reg [3:0] y_hundreds;
    reg [3:0] y_tens;
    reg [3:0] y_ones;

    // Keep the most recent coordinate if a new sample arrives while a line is
    // still being transmitted.
    reg        pending_valid;
    reg [15:0] pending_x;
    reg [15:0] pending_y;

    assign busy = message_active | pending_valid | tx_busy;

    function [7:0] message_char;
        input [3:0] index;
        begin
            case (index)
                4'd0:  message_char = "X";
                4'd1:  message_char = "=";
                4'd2:  message_char = 8'h30 + x_thousands;
                4'd3:  message_char = 8'h30 + x_hundreds;
                4'd4:  message_char = 8'h30 + x_tens;
                4'd5:  message_char = 8'h30 + x_ones;
                4'd6:  message_char = ",";
                4'd7:  message_char = "Y";
                4'd8:  message_char = "=";
                4'd9:  message_char = 8'h30 + y_thousands;
                4'd10: message_char = 8'h30 + y_hundreds;
                4'd11: message_char = 8'h30 + y_tens;
                4'd12: message_char = 8'h30 + y_ones;
                4'd13: message_char = 8'h0d;
                4'd14: message_char = 8'h0a;
                default: message_char = 8'h20;
            endcase
        end
    endfunction

    task latch_digits;
        input [15:0] x_value;
        input [15:0] y_value;
        begin
            x_thousands <= (x_value / 1000) % 10;
            x_hundreds  <= (x_value / 100)  % 10;
            x_tens      <= (x_value / 10)   % 10;
            x_ones      <=  x_value         % 10;
            y_thousands <= (y_value / 1000) % 10;
            y_hundreds  <= (y_value / 100)  % 10;
            y_tens      <= (y_value / 10)   % 10;
            y_ones      <=  y_value         % 10;
        end
    endtask

    uart_tx_byte #(
        .CLK_HZ (CLK_HZ),
        .BAUD   (BAUD)
    ) u_uart_tx_byte (
        .clk     (clk),
        .rst_n   (rst_n),
        .start   (tx_start),
        .data_in (tx_data),
        .tx      (uart_txd),
        .busy    (tx_busy),
        .done    (tx_done)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_start       <= 1'b0;
            tx_data        <= 8'h00;
            message_active <= 1'b0;
            byte_inflight  <= 1'b0;
            char_index     <= 4'd0;
            x_thousands    <= 4'd0;
            x_hundreds     <= 4'd0;
            x_tens         <= 4'd0;
            x_ones         <= 4'd0;
            y_thousands    <= 4'd0;
            y_hundreds     <= 4'd0;
            y_tens         <= 4'd0;
            y_ones         <= 4'd0;
            pending_valid  <= 1'b0;
            pending_x      <= 16'd0;
            pending_y      <= 16'd0;
        end
        else begin
            tx_start <= 1'b0;

            if (coord_valid) begin
                if (!message_active && !byte_inflight) begin
                    latch_digits(coord_x, coord_y);
                    message_active <= 1'b1;
                    char_index     <= 4'd0;
                end
                else begin
                    pending_x     <= coord_x;
                    pending_y     <= coord_y;
                    pending_valid <= 1'b1;
                end
            end

            if (message_active && !byte_inflight && !tx_busy) begin
                tx_data       <= message_char(char_index);
                tx_start      <= 1'b1;
                byte_inflight <= 1'b1;
            end

            if (tx_done) begin
                byte_inflight <= 1'b0;

                if (char_index == 4'd14) begin
                    if (pending_valid) begin
                        latch_digits(pending_x, pending_y);
                        pending_valid  <= 1'b0;
                        message_active <= 1'b1;
                        char_index     <= 4'd0;
                    end
                    else begin
                        message_active <= 1'b0;
                        char_index     <= 4'd0;
                    end
                end
                else begin
                    char_index <= char_index + 4'd1;
                end
            end
        end
    end

endmodule
