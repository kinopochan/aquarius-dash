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
        // Additional edge case tests
        //===========================================
        $display("--- Additional edge cases ---");

        // Powers of 2
        issue_div(8'hB1, 32'hFFFFFFFF, 32'h80000000);
        check(32'd1, 32'h7FFFFFFF, "FFFFFFFF / 80000000");

        issue_div(8'hB1, 32'h80000000, 32'h80000000);
        check(32'd1, 32'd0, "80000000 / 80000000");

        issue_div(8'hB1, 32'd1, 32'hFFFFFFFF);
        check(32'd0, 32'd1, "1 / FFFFFFFF");

        issue_div(8'hB1, 32'hFFFFFFFF, 32'hFFFFFFFF);
        check(32'd1, 32'd0, "FFFFFFFF / FFFFFFFF");

        issue_div(8'hB1, 32'hFFFFFFFE, 32'hFFFFFFFF);
        check(32'd0, 32'hFFFFFFFE, "FFFFFFFE / FFFFFFFF");

        // Small values
        issue_div(8'hB1, 32'd1, 32'd1);
        check(32'd1, 32'd0, "1 / 1");

        issue_div(8'hB1, 32'd2, 32'd3);
        check(32'd0, 32'd2, "2 / 3");

        issue_div(8'hB1, 32'd255, 32'd16);
        check(32'd15, 32'd15, "255 / 16");

        // Large quotient
        issue_div(8'hB1, 32'hFFFFFFFF, 32'd3);
        check(32'h55555555, 32'd0, "FFFFFFFF / 3");

        issue_div(8'hB1, 32'hFFFFFFFF, 32'd7);
        check(32'h24924924, 32'd3, "FFFFFFFF / 7");

        issue_div(8'hB1, 32'hFFFFFFFF, 32'd10);
        check(32'h19999999, 32'd5, "FFFFFFFF / 10");

        // Signed edge cases
        issue_div(8'hB9, 32'h80000000, 32'd1);
        check(32'h80000000, 32'd0, "MIN / 1 (signed)");

        issue_div(8'hB9, 32'h80000000, 32'd2);
        check(32'hC0000000, 32'd0, "MIN / 2 (signed)");

        issue_div(8'hB9, 32'h80000001, 32'hFFFFFFFF);
        check(32'h7FFFFFFF, 32'd0, "MIN+1 / -1 (signed)");

        //===========================================
        // Random unsigned tests
        //===========================================
        $display("--- Random unsigned tests (1000 cases) ---");
        begin : random_unsigned
            integer i;
            reg [31:0] a, b;
            reg [31:0] exp_q, exp_r;
            reg random_fail;
            random_fail = 0;
            for (i = 0; i < 1000; i = i + 1) begin
                a = $random;
                b = $random;
                if (b == 0) b = 32'd1; // avoid zero div in random tests
                exp_q = a / b;
                exp_r = a % b;
                issue_div(8'hB1, a, b);
                test_num = test_num + 1;
                if (MACH === exp_q && MACL === exp_r) begin
                    pass_count = pass_count + 1;
                end else begin
                    $display("FAIL test %0d: DIVU %08X / %08X = %08X r %08X (exp %08X r %08X)",
                             test_num, a, b, MACH, MACL, exp_q, exp_r);
                    fail_count = fail_count + 1;
                    random_fail = 1;
                end
            end
            if (!random_fail)
                $display("  All 1000 random unsigned tests passed");
        end

        //===========================================
        // Random signed tests
        //===========================================
        $display("--- Random signed tests (1000 cases) ---");
        begin : random_signed
            integer i;
            reg signed [31:0] a, b;
            reg signed [31:0] exp_q, exp_r;
            reg random_fail;
            random_fail = 0;
            for (i = 0; i < 1000; i = i + 1) begin
                a = $random;
                b = $random;
                if (b == 0) b = 32'sd1;
                if (a == 32'sh80000000 && b == -32'sd1) b = 32'sd1; // avoid overflow
                exp_q = a / b;
                exp_r = a % b;
                issue_div(8'hB9, a, b);
                test_num = test_num + 1;
                if (MACH === exp_q && MACL === exp_r) begin
                    pass_count = pass_count + 1;
                end else begin
                    $display("FAIL test %0d: DIVS %08X / %08X = %08X r %08X (exp %08X r %08X)",
                             test_num, a, b, MACH, MACL, exp_q, exp_r);
                    fail_count = fail_count + 1;
                    random_fail = 1;
                end
            end
            if (!random_fail)
                $display("  All 1000 random signed tests passed");
        end

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
