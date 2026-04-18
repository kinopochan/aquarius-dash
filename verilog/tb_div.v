`include "timescale.v"
`include "defines.v"

module tb_div;

    reg CLK, RST;
    reg SLOT;
    reg MULCOM1;
    reg [7:0] MULCOM2;
    reg MAC_S;
    reg WRMACH, WRMACL;
    reg [31:0] MACIN1, MACIN2;
    wire [31:0] MACH, MACL;
    wire MAC_BUSY;

    mult DUT (
        .CLK(CLK), .RST(RST),
        .SLOT(SLOT),
        .MULCOM1(MULCOM1), .MULCOM2(MULCOM2),
        .MAC_S(MAC_S),
        .WRMACH(WRMACH), .WRMACL(WRMACL),
        .MACIN1(MACIN1), .MACIN2(MACIN2),
        .MACH(MACH), .MACL(MACL),
        .MAC_BUSY(MAC_BUSY)
    );

    // Clock generation
    initial CLK = 0;
    always #(`HALF_CYCLE) CLK = ~CLK;

    integer pass_count = 0;
    integer fail_count = 0;
    integer test_num = 0;

    task issue_div;
        input [7:0] cmd;      // 8'hB1=DIVU, 8'hB9=DIVS
        input [31:0] dividend;
        input [31:0] divisor;
        integer timeout;
        begin
            @(posedge CLK);
            SLOT     <= 1'b1;
            MULCOM1  <= 1'b1;
            MULCOM2  <= cmd;
            MACIN1   <= dividend;
            MACIN2   <= divisor;
            @(posedge CLK);
            SLOT     <= 1'b1;
            MULCOM1  <= 1'b0;
            MULCOM2  <= 8'h00;
            MACIN1   <= 32'd0;
            MACIN2   <= 32'd0;
            // Wait for MAC_BUSY to assert then deassert
            @(posedge CLK); // let state machine enter DIVOP
            timeout = 0;
            while (MAC_BUSY == 1'b1 && timeout < 100) begin
                @(posedge CLK);
                timeout = timeout + 1;
            end
            if (timeout >= 100) begin
                $display("TIMEOUT waiting for MAC_BUSY deassert");
                $finish;
            end
            @(posedge CLK); // div_write cycle
            @(posedge CLK); // ensure MACH/MACL latched
        end
    endtask

    task check;
        input [31:0] exp_quot;
        input [31:0] exp_rem;
        input [255:0] name;  // 32-char test name (padded)
        begin
            test_num = test_num + 1;
            if (MACH === exp_quot && MACL === exp_rem) begin
                $display("PASS test %0d: %0s  MACH=%08X MACL=%08X",
                         test_num, name, MACH, MACL);
                pass_count = pass_count + 1;
            end else begin
                $display("FAIL test %0d: %0s  MACH=%08X (exp %08X)  MACL=%08X (exp %08X)",
                         test_num, name, MACH, exp_quot, MACL, exp_rem);
                fail_count = fail_count + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("tb_div.vcd");
        $dumpvars(0, tb_div);

        RST      = 1'b1;
        SLOT     = 1'b0;
        MULCOM1  = 1'b0;
        MULCOM2  = 8'h00;
        MAC_S    = 1'b0;
        WRMACH   = 1'b0;
        WRMACL   = 1'b0;
        MACIN1   = 32'd0;
        MACIN2   = 32'd0;

        #(`CYCLE * 3);
        RST = 1'b0;
        #(`CYCLE * 2);

        //===========================================
        // DIVU tests (unsigned)
        //===========================================
        $display("--- DIVU (unsigned division) ---");

        // Test 1: 100 / 7 = 14 rem 2
        issue_div(8'hB1, 32'd100, 32'd7);
        check(32'd14, 32'd2, "100 / 7");

        // Test 2: 0 / 5 = 0 rem 0
        issue_div(8'hB1, 32'd0, 32'd5);
        check(32'd0, 32'd0, "0 / 5");

        // Test 3: 7 / 7 = 1 rem 0
        issue_div(8'hB1, 32'd7, 32'd7);
        check(32'd1, 32'd0, "7 / 7");

        // Test 4: 1 / 3 = 0 rem 1
        issue_div(8'hB1, 32'd1, 32'd3);
        check(32'd0, 32'd1, "1 / 3");

        // Test 5: 0xFFFFFFFF / 2 = 0x7FFFFFFF rem 1
        issue_div(8'hB1, 32'hFFFFFFFF, 32'd2);
        check(32'h7FFFFFFF, 32'd1, "FFFFFFFF / 2");

        // Test 6: 0xFFFFFFFF / 1 = 0xFFFFFFFF rem 0
        issue_div(8'hB1, 32'hFFFFFFFF, 32'd1);
        check(32'hFFFFFFFF, 32'd0, "FFFFFFFF / 1");

        // Test 7: Zero division - DIVU
        issue_div(8'hB1, 32'd42, 32'd0);
        check(32'hFFFFFFFF, 32'd42, "42 / 0 (zero div)");

        // Test 8: Large dividend
        issue_div(8'hB1, 32'h80000000, 32'd3);
        check(32'h2AAAAAAA, 32'd2, "80000000 / 3");

        //===========================================
        // DIVS tests (signed)
        //===========================================
        $display("--- DIVS (signed division) ---");

        // Test 9: 100 / 7 = 14 rem 2  (positive / positive)
        issue_div(8'hB9, 32'd100, 32'd7);
        check(32'd14, 32'd2, "+100 / +7");

        // Test 10: -100 / 7 = -14 rem -2  (negative / positive)
        issue_div(8'hB9, -32'sd100, 32'd7);
        check(-32'sd14, -32'sd2, "-100 / +7");

        // Test 11: 100 / -7 = -14 rem 2  (positive / negative)
        issue_div(8'hB9, 32'd100, -32'sd7);
        check(-32'sd14, 32'd2, "+100 / -7");

        // Test 12: -100 / -7 = 14 rem -2  (negative / negative)
        issue_div(8'hB9, -32'sd100, -32'sd7);
        check(32'd14, -32'sd2, "-100 / -7");

        // Test 13: Signed zero division
        issue_div(8'hB9, -32'sd42, 32'd0);
        check(32'hFFFFFFFF, -32'sd42, "-42 / 0 (zero div)");

        // Test 14: Signed overflow: MIN / -1
        issue_div(8'hB9, 32'h80000000, 32'hFFFFFFFF);
        check(32'h80000000, 32'd0, "MIN / -1 (overflow)");

        // Test 15: -1 / 1 = -1 rem 0
        issue_div(8'hB9, 32'hFFFFFFFF, 32'd1);
        check(32'hFFFFFFFF, 32'd0, "-1 / 1");

        // Test 16: 1 / -1 = -1 rem 0
        issue_div(8'hB9, 32'd1, 32'hFFFFFFFF);
        check(32'hFFFFFFFF, 32'd0, "1 / -1");

        //===========================================
        // Summary
        //===========================================
        $display("");
        $display("=== Results: %0d passed, %0d failed ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("SOME TESTS FAILED");

        #(`CYCLE * 5);
        $finish;
    end

endmodule
