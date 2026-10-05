`timescale 1ns/1ps
// Chapter 7: capacitive touch coordinates, region LEDs and coordinate UART.
// Project: chapter6_touch_complete_200t; simulation top: tb_chapter7_all.
// Add THIS file to Simulation Sources only. The model below is included here.
// Use the effective top_chapter6_touch.v from imports/chapter6_uart_upgrade.
//
// REQUIRED original-source correction in i2c_dri.v:
//   old: if(reg_cnt == reg_cnt - 1'b1)
//   new: if(reg_cnt == reg_num - 1'b1)
// The supplied i2c_dri_chapter7_fixed.v contains ONLY this one-line fix.
// Replace the effective i2c_dri source; do not compile both copies together.
// The testbench checks real bus ACK/NACK. It does not hide the original error.
//
// Run: restart; log_wave /tb_chapter7_all/*; run 25 ms.
// Expected: CH7 PASS, checks_done=1, checks_pass=1, errors=0,
//           model_protocol_errors=0, coord_events=6, uart_lines=6.
// No simulation-ending task is used: waveforms remain available afterwards.
//
// Simulation-only settings (do NOT copy these settings to the board design):
//   sys_clk=50 MHz; CLK_HZ=UART_CLK_HZ=50_000_000; UART=115200, 8-N-1.
//   POR_BITS=4; I2C_FREQ override=6_250_000 accelerates the fixed delay counts.
//   I2C timing here is a functional stimulus, NOT the board's measured rate.
//   Touch update hold=50000 system cycles (1 ms), diagnostic hold=10000 cycles.
//   Peripheral model is FT at 7-bit address 0x38, version 0x3003. GT is NOT
//   claimed verified. The active driver swaps FT register X/Y; the model
//   encodes those registers so the requested horizontal output is x/y below.
//
// Screenshot groups (all aliases are at the testbench root):
// Fig.7-1(a), 4800..4870 us: TOUCH_SCL, TOUCH_SDA, slave_addr, i2c_addr, i2c_rh_wl,
//   i2c_data_r, once_byte_done, i2c_done, touch_state, chip_version, ft_flag,
//   driver_ready, model_protocol_errors. Identification/configuration details.
// Fig.7-1(b), 8070..8150 us: TOUCH_SCL, TOUCH_SDA, i2c_addr, i2c_data_r, once_byte_done,
//   touch_x_raw, touch_y_raw, coord_update_pulse. First complete coordinate.
// Fig.7-2, 8050..14450 us: touch_x_raw, touch_y_raw, coord_update_pulse, coord_events,
//   touch_seen, last_right, last_lower, touch_hold, led, touch_valid_raw.
//   Four regions: (200,100), (800,100), (200,500), (800,500).
//   With hold active, led=11001,11011,11101,11111 respectively.
//   Later cases (512,300) and (511,299) check the exact region thresholds.
// Optional UART detail, 8130..9450 us: UART_TXD, uart_busy, uart_rx_byte, uart_rx_valid,
//   uart_lines; first decoded line is X=0200,Y=0100 followed by CR LF.
// Hex: addresses, byte data, chip_version, touch_state. Unsigned decimal:
// coordinates/event counters. Binary: LEDs and one-bit flags. UART byte: ASCII.

module tb_chapter7_all;
    localparam integer SYS_CLK_HZ = 50_000_000;
    localparam integer UART_BAUD = 115_200;
    localparam integer UART_BIT_NS = (SYS_CLK_HZ / UART_BAUD) * 20;
    reg sys_clk = 1'b0;
    always #10 sys_clk = ~sys_clk;

    wire TOUCH_SCL;
    tri1 TOUCH_SDA;
    tri1 TOUCH_INT;
    wire TOUCH_RST;
    wire UART_TXD;
    wire [4:0] led;
    top_chapter6_touch #(
        .CLK_HZ(SYS_CLK_HZ), .POR_BITS(4), .LED_MODE(1),
        .UART_CLK_HZ(SYS_CLK_HZ), .UART_BAUD(UART_BAUD)
    ) dut (
        .sys_clk(sys_clk), .TOUCH_SCL(TOUCH_SCL), .TOUCH_SDA(TOUCH_SDA),
        .TOUCH_INT(TOUCH_INT), .TOUCH_RST(TOUCH_RST),
        .UART_TXD(UART_TXD), .led(led)
    );
    defparam dut.u_touch_top.u_i2c_dri.I2C_FREQ = 6_250_000;
    defparam dut.u_hold.HOLD_CYCLES = 50_000;
    defparam dut.u_i2c_hold.HOLD_CYCLES = 10_000;

    wire rst_n = dut.rst_n;
    wire [6:0] touch_state = dut.touch_state;
    wire driver_ready = dut.driver_ready;
    wire ft_flag = dut.ft_flag;
    wire [15:0] chip_version = dut.chip_version;
    wire [6:0] slave_addr = dut.u_touch_top.slave_addr;
    wire [15:0] i2c_addr = dut.u_touch_top.i2c_addr;
    wire i2c_rh_wl = dut.u_touch_top.i2c_rh_wl;
    wire i2c_exec = dut.u_touch_top.i2c_exec;
    wire i2c_ack = dut.i2c_ack; // RTL convention: ACK=0, NACK=1.
    wire i2c_done = dut.i2c_done;
    wire once_byte_done = dut.once_byte_done;
    wire [7:0] i2c_data_r = dut.i2c_data_r;
    wire [15:0] touch_x_raw = dut.touch_x_raw;
    wire [15:0] touch_y_raw = dut.touch_y_raw;
    wire touch_valid_raw = dut.touch_valid_raw;
    wire coord_update_pulse = dut.coord_update_pulse;
    wire touch_seen = dut.touch_seen;
    wire last_right = dut.last_right;
    wire last_lower = dut.last_lower;
    wire touch_hold = dut.touch_hold;
    wire uart_busy = dut.uart_busy;

    reg model_touch_active = 1'b0;
    reg [15:0] model_x = 16'd200;
    reg [15:0] model_y = 16'd100;
    wire [3:0] model_config_flags;
    wire [7:0] model_reg_addr;
    wire [7:0] model_bus_rx_byte;
    wire [7:0] model_bus_tx_byte;
    wire [31:0] model_status_reads;
    wire [31:0] model_coord_reads;
    wire [31:0] model_protocol_errors;
    wire [31:0] model_nack_events;

    ch7_ft_i2c_model touch_model (
        .rst_n(TOUCH_RST), .scl(TOUCH_SCL), .sda(TOUCH_SDA),
        .touch_active(model_touch_active), .out_x(model_x), .out_y(model_y),
        .config_flags(model_config_flags), .reg_addr(model_reg_addr),
        .bus_rx_byte(model_bus_rx_byte), .bus_tx_byte(model_bus_tx_byte),
        .status_reads(model_status_reads), .coord_reads(model_coord_reads),
        .protocol_errors(model_protocol_errors), .nack_events(model_nack_events)
    );

    integer errors = 0;
    integer coord_events = 0;
    integer sample_case = 0;
    integer uart_bytes = 0;
    integer uart_lines = 0;
    integer uart_char_index = 0;
    integer expected_x [0:5];
    integer expected_y [0:5];
    reg [7:0] uart_rx_byte = 8'd0;
    reg uart_rx_valid = 1'b0;
    reg checks_done = 1'b0;
    reg checks_pass = 1'b0;
    reg previous_update = 1'b0;

    task automatic expect_eq;
        input [8*96-1:0] label;
        input [31:0] actual;
        input [31:0] expected;
        begin
            if (actual !== expected) begin
                errors = errors + 1;
                $display("CH7 ERROR at %0.3f us: %0s actual=%h expected=%h",
                         $realtime / 1000.0, label, actual, expected);
            end
        end
    endtask

    // Coordinates must be complete before the root update event is emitted.
    always @(posedge sys_clk) begin
        #0.001;
        if (rst_n) begin
            if (coord_update_pulse) begin
                if (previous_update) expect_eq("Coordinate pulse width", 1, 0);
                if (coord_events < 6) begin
                    expect_eq("Complete coordinate X", touch_x_raw, expected_x[coord_events]);
                    expect_eq("Complete coordinate Y", touch_y_raw, expected_y[coord_events]);
                end else expect_eq("Unexpected extra coordinate update", 1, 0);
                coord_events = coord_events + 1;
                $display("CH7 coordinate %0d at %0.3f us: x=%0d y=%0d",
                         coord_events, $realtime / 1000.0, touch_x_raw, touch_y_raw);
            end
            previous_update = coord_update_pulse;
        end
    end

    function [7:0] expected_uart_char;
        input integer line_no;
        input integer char_no;
        integer x_value;
        integer y_value;
        begin
            x_value = expected_x[line_no];
            y_value = expected_y[line_no];
            case (char_no)
                 0: expected_uart_char = "X";
                 1: expected_uart_char = "=";
                 2: expected_uart_char = 8'h30 + (x_value / 1000) % 10;
                 3: expected_uart_char = 8'h30 + (x_value / 100) % 10;
                 4: expected_uart_char = 8'h30 + (x_value / 10) % 10;
                 5: expected_uart_char = 8'h30 + x_value % 10;
                 6: expected_uart_char = ",";
                 7: expected_uart_char = "Y";
                 8: expected_uart_char = "=";
                 9: expected_uart_char = 8'h30 + (y_value / 1000) % 10;
                10: expected_uart_char = 8'h30 + (y_value / 100) % 10;
                11: expected_uart_char = 8'h30 + (y_value / 10) % 10;
                12: expected_uart_char = 8'h30 + y_value % 10;
                13: expected_uart_char = 8'h0D;
                14: expected_uart_char = 8'h0A;
                default: expected_uart_char = 8'hFF;
            endcase
        end
    endfunction

    // Decode the ACTUAL UART_TXD pin, not the DUT's internal tx_data register.
    initial begin : uart_pin_receiver
        integer i;
        reg [7:0] received;
        wait (rst_n === 1'b1);
        forever begin
            @(negedge UART_TXD);
            #(UART_BIT_NS / 2);
            expect_eq("UART start bit", UART_TXD, 0);
            for (i = 0; i < 8; i = i + 1) begin
                #(UART_BIT_NS);
                received[i] = UART_TXD;
            end
            #(UART_BIT_NS);
            expect_eq("UART stop bit", UART_TXD, 1);
            uart_rx_byte = received;
            uart_rx_valid = 1;
            uart_bytes = uart_bytes + 1;
            if (uart_lines < 6)
                expect_eq("UART coordinate text", received,
                          expected_uart_char(uart_lines, uart_char_index));
            else expect_eq("Unexpected extra UART line", 1, 0);
            if (uart_char_index == 14) begin
                uart_lines = uart_lines + 1;
                uart_char_index = 0;
                $display("CH7 UART line %0d received at %0.3f us",
                         uart_lines, $realtime / 1000.0);
            end else uart_char_index = uart_char_index + 1;
            #20;
            uart_rx_valid = 0;
        end
    end

    task automatic do_touch;
        input integer case_no;
        input integer x_value;
        input integer y_value;
        input [4:0] expected_led;
        integer before_count;
        begin
            sample_case = case_no;
            before_count = coord_events;
            model_x = x_value;
            model_y = y_value;
            model_touch_active = 1;
            wait (coord_events > before_count);
            #500;
            expect_eq("One coordinate for requested sample", coord_events, before_count + 1);
            expect_eq("Touch X output", touch_x_raw, x_value);
            expect_eq("Touch Y output", touch_y_raw, y_value);
            expect_eq("Combined coordinate data", dut.touch_data,
                      {touch_x_raw, touch_y_raw});
            expect_eq("Region LED including active hold", led, expected_led);
            expect_eq("Touch has been seen", touch_seen, 1);
            expect_eq("Right region latch", last_right, x_value >= 512);
            expect_eq("Lower region latch", last_lower, y_value >= 300);
            expect_eq("Update has been stretched", touch_hold, 1);
            wait (uart_lines >= case_no);
        end
    endtask

    initial begin : stimulus
        integer updates_before_release;
        integer idle_reads_before;
        expected_x[0]=200; expected_y[0]=100;
        expected_x[1]=800; expected_y[1]=100;
        expected_x[2]=200; expected_y[2]=500;
        expected_x[3]=800; expected_y[3]=500;
        expected_x[4]=512; expected_y[4]=300;
        expected_x[5]=511; expected_y[5]=299;

        wait (touch_state === 7'b0010000 && model_config_flags === 4'b1111);
        $display("CH7 FT configured at %0.3f us", $realtime / 1000.0);
        expect_eq("FT address", slave_addr, 7'h38);
        expect_eq("FT version", chip_version, 16'h3003);
        expect_eq("FT identified", ft_flag, 1);
        expect_eq("Driver ready", driver_ready, 1);
        wait (model_status_reads >= 1);
        #5000;
        expect_eq("No false coordinate while idle", coord_events, 0);
        expect_eq("No false seen flag while idle", touch_seen, 0);
        expect_eq("No idle UART bytes", uart_bytes, 0);
        expect_eq("Idle LED FT-ready only", led, 5'b01000);

        do_touch(1,200,100,5'b11001);
        do_touch(2,800,100,5'b11011);
        do_touch(3,200,500,5'b11101);
        do_touch(4,800,500,5'b11111);
        do_touch(5,512,300,5'b11111);
        do_touch(6,511,299,5'b11001);

        sample_case = 7;
        model_touch_active = 0;
        updates_before_release = coord_events;
        idle_reads_before = model_status_reads;
        wait (model_status_reads > idle_reads_before);
        #50000;
        wait (!uart_busy);
        expect_eq("Release has no extra coordinate", coord_events, updates_before_release);
        expect_eq("Release clears raw touch indication", touch_valid_raw, 0);
        expect_eq("Seen flag persists after release", touch_seen, 1);
        expect_eq("Hold expires after release", touch_hold, 0);
        expect_eq("Last X persists after release", touch_x_raw, 511);
        expect_eq("Last Y persists after release", touch_y_raw, 299);
        expect_eq("Released LED keeps latest region", led, 5'b11000);
        expect_eq("Six complete coordinates", coord_events, 6);
        expect_eq("Six UART lines", uart_lines, 6);
        expect_eq("Ninety UART bytes", uart_bytes, 90);
        expect_eq("I2C protocol check", model_protocol_errors, 0);
        expect_eq("FT path remains selected", ft_flag, 1);
        checks_pass = (errors == 0);
        checks_done = 1;
        if (checks_pass)
            $display("CH7 PASS at %0.3f us: coordinates=%0d UART lines=%0d bytes=%0d I2C errors=%0d",
                     $realtime / 1000.0, coord_events, uart_lines,
                     uart_bytes, model_protocol_errors);
        else $display("CH7 FAIL: errors=%0d", errors);
    end

    initial begin : watchdog
        #24_000_000;
        if (!checks_done) begin
            errors = errors + 1;
            checks_done = 1;
            checks_pass = 0;
            $display("CH7 FAIL: timeout; state=%h coord=%0d UART lines=%0d I2C errors=%0d",
                     touch_state, coord_events, uart_lines, model_protocol_errors);
        end
    end
endmodule

// Behavioral FT register model. Bus input comes solely from SCL/SDA pins.
// Receives address/register writes, supports repeated START and sequential
// reads, validates every master's ACK and the final NACK, and releases SDA
// for STOP. No hierarchical DUT writes, forced events or fake bus waveforms.
module ch7_ft_i2c_model (
    input wire rst_n,
    input wire scl,
    inout wire sda,
    input wire touch_active,
    input wire [15:0] out_x,
    input wire [15:0] out_y,
    output reg [3:0] config_flags = 4'b0000,
    output reg [7:0] reg_addr = 8'd0,
    output reg [7:0] bus_rx_byte = 8'd0,
    output reg [7:0] bus_tx_byte = 8'd0,
    output reg [31:0] status_reads = 0,
    output reg [31:0] coord_reads = 0,
    output reg [31:0] protocol_errors = 0,
    output reg [31:0] nack_events = 0
);
    localparam IDLE=0, RX=1, SLAVE_ACK_PREP=2, SLAVE_ACK=3,
               TX=4, MASTER_ACK_PREP=5, MASTER_ACK=6, WAIT_STOP=7;
    integer state = IDLE;
    integer bit_count = 0;
    integer read_count = 0;
    integer expected_read_count = 0;
    reg [7:0] rx_shift = 0;
    reg [7:0] read_start = 0;
    reg drive_low = 0;
    reg address_phase = 1;
    reg register_phase = 0;
    reg selected = 0;
    reg read_mode = 0;
    reg ack_clock_seen = 0;
    reg master_ack = 1;
    assign sda = drive_low ? 1'b0 : 1'bz;

    function [7:0] read_register;
        input [7:0] address;
        begin
            case (address)
                8'hA1: read_register = 8'h30;
                8'hA2: read_register = 8'h03;
                8'h02: read_register = touch_active ? 8'h01 : 8'h00;
                // The active driver swaps FT X/Y for horizontal output.
                8'h03: read_register = {4'b0000,out_y[11:8]};
                8'h04: read_register = out_y[7:0];
                8'h05: read_register = {4'b0000,out_x[11:8]};
                8'h06: read_register = out_x[7:0];
                default: read_register = 8'hFF;
            endcase
        end
    endfunction

    task automatic write_register;
        input [7:0] address;
        input [7:0] value;
        begin
            case (address)
                8'h00: if (value == 0) config_flags[0] = 1;
                8'hA4: if (value == 0) config_flags[1] = 1;
                8'h80: if (value == 22) config_flags[2] = 1;
                8'h88: if (value == 12) config_flags[3] = 1;
                // Status clear is acknowledged; a held simulated finger
                // remains available on the next polling cycle.
                8'h02: begin end
                default: begin
                    protocol_errors = protocol_errors + 1;
                    $display("CH7 MODEL unexpected register write %h=%h",address,value);
                end
            endcase
        end
    endtask

    always @(negedge rst_n) begin
        state = IDLE;
        drive_low = 0;
        config_flags = 0;
        reg_addr = 0;
        bit_count = 0;
        read_count = 0;
        status_reads = 0;
        coord_reads = 0;
        protocol_errors = 0;
        nack_events = 0;
    end

    // START/repeated START. The register pointer survives repeated START.
    always @(negedge sda) begin
        if (rst_n === 1'b1 && scl === 1'b1) begin
            state = RX;
            drive_low = 0;
            bit_count = 0;
            rx_shift = 0;
            address_phase = 1;
            register_phase = 0;
            selected = 0;
            read_mode = 0;
            ack_clock_seen = 0;
        end
    end

    always @(posedge sda) begin
        if (rst_n === 1'b1 && scl === 1'b1) begin
            state = IDLE;
            drive_low = 0;
        end
    end

    always @(posedge scl) begin
        if (rst_n === 1'b1) begin
            case (state)
                RX: begin
                    rx_shift = {rx_shift[6:0],sda};
                    bit_count = bit_count + 1;
                    if (bit_count == 8) begin
                        bus_rx_byte = rx_shift;
                        if (address_phase) begin
                            selected = (rx_shift[7:1] == 7'h38);
                            read_mode = rx_shift[0];
                            address_phase = 0;
                            register_phase = !read_mode;
                            if (selected && read_mode) begin
                                read_count = 0;
                                read_start = reg_addr;
                                case (reg_addr)
                                    8'hA1: expected_read_count = 2;
                                    8'h02: expected_read_count = 1;
                                    8'h03: expected_read_count = 4;
                                    default: expected_read_count = 0;
                                endcase
                            end
                        end else if (selected) begin
                            if (register_phase) begin
                                reg_addr = rx_shift;
                                register_phase = 0;
                            end else begin
                                write_register(reg_addr,rx_shift);
                                reg_addr = reg_addr + 1'b1;
                            end
                        end
                        state = SLAVE_ACK_PREP;
                    end
                end
                SLAVE_ACK: ack_clock_seen = 1;
                TX: begin
                    bit_count = bit_count + 1;
                    if (bit_count == 8) state = MASTER_ACK_PREP;
                end
                MASTER_ACK: begin
                    ack_clock_seen = 1;
                    master_ack = sda;
                    read_count = read_count + 1;
                    if (expected_read_count != 0) begin
                        if ((read_count == expected_read_count && sda !== 1'b1) ||
                            (read_count < expected_read_count && sda !== 1'b0)) begin
                            protocol_errors = protocol_errors + 1;
                            $display("CH7 MODEL ACK/NACK error at %0.3f us: reg=%h byte=%0d/%0d SDA=%b",
                                     $realtime/1000.0,read_start,read_count,expected_read_count,sda);
                        end
                    end
                    if (read_count == expected_read_count) begin
                        if (read_start == 8'h02) status_reads = status_reads + 1;
                        if (read_start == 8'h03) coord_reads = coord_reads + 1;
                    end
                    if (sda === 1'b1) nack_events = nack_events + 1;
                end
                default: begin end
            endcase
        end
    end

    always @(negedge scl) begin
        if (rst_n === 1'b1) begin
            case (state)
                SLAVE_ACK_PREP: begin
                    drive_low = selected;
                    ack_clock_seen = 0;
                    state = SLAVE_ACK;
                end
                SLAVE_ACK: begin
                    if (ack_clock_seen) begin
                        drive_low = 0;
                        bit_count = 0;
                        rx_shift = 0;
                        if (selected && read_mode) begin
                            bus_tx_byte = read_register(reg_addr);
                            drive_low = !bus_tx_byte[7];
                            state = TX;
                        end else if (selected) state = RX;
                        else state = WAIT_STOP;
                    end
                end
                TX: drive_low = !bus_tx_byte[7-bit_count];
                MASTER_ACK_PREP: begin
                    drive_low = 0;
                    ack_clock_seen = 0;
                    state = MASTER_ACK;
                end
                MASTER_ACK: begin
                    if (ack_clock_seen) begin
                        drive_low = 0;
                        if (master_ack === 1'b0) begin
                            reg_addr = reg_addr + 1'b1;
                            bus_tx_byte = read_register(reg_addr);
                            bit_count = 0;
                            drive_low = !bus_tx_byte[7];
                            state = TX;
                        end else state = WAIT_STOP;
                    end
                end
                default: begin end
            endcase
        end
    end
endmodule
