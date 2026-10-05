`timescale 1ns/1ps

// 8-N-1 UART byte transmitter.
module uart_tx_byte #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,
    input  wire [7:0] data_in,
    output reg        tx,
    output reg        busy,
    output reg        done
);

    localparam integer CLKS_PER_BIT = CLK_HZ / BAUD;

    reg [15:0] baud_count;
    reg [3:0]  bit_index;
    reg [9:0]  shift_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx         <= 1'b1;
            busy       <= 1'b0;
            done       <= 1'b0;
            baud_count <= 16'd0;
            bit_index  <= 4'd0;
            shift_reg  <= 10'h3ff;
        end
        else begin
            done <= 1'b0;

            if (!busy) begin
                tx         <= 1'b1;
                baud_count <= 16'd0;
                bit_index  <= 4'd0;

                if (start) begin
                    // {stop bit, data[7:0], start bit}
                    shift_reg <= {1'b1, data_in, 1'b0};
                    tx        <= 1'b0;
                    busy      <= 1'b1;
                end
            end
            else if (baud_count == CLKS_PER_BIT - 1) begin
                baud_count <= 16'd0;

                if (bit_index == 4'd9) begin
                    tx        <= 1'b1;
                    busy      <= 1'b0;
                    done      <= 1'b1;
                    bit_index <= 4'd0;
                end
                else begin
                    shift_reg <= {1'b1, shift_reg[9:1]};
                    tx        <= shift_reg[1];
                    bit_index <= bit_index + 4'd1;
                end
            end
            else begin
                baud_count <= baud_count + 16'd1;
            end
        end
    end

endmodule
