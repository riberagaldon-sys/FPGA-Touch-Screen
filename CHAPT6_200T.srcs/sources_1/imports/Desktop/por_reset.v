`timescale 1ns/1ps
module por_reset #(parameter integer COUNTER_BITS=21)(input wire clk,output wire rst_n);
reg [COUNTER_BITS-1:0] cnt={COUNTER_BITS{1'b0}};
always @(posedge clk) if(!(&cnt)) cnt <= cnt + {{(COUNTER_BITS-1){1'b0}},1'b1};
assign rst_n=&cnt;
endmodule
