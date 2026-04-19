`include "timescale.v"
`include "defines.v"

module tb_mult;

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

    initial CLK = 0;
    always #(`HALF_CYCLE) CLK = ~CLK;

    integer pass_count = 0;
    integer fail_count = 0;
    integer test_num = 0;

    // Issue a multiply/MAC command and wait for completion
    task issue;
        input [7:0] cmd;
        input [31:0] a;
        input [31:0] b;
        integer timeout;
        begin
            @(posedge CLK);
            SLOT     <= 1'b1;
            MULCOM1  <= 1'b1;
            MULCOM2  <= cmd;
            MACIN1   <= a;
            MACIN2   <= b;
            @(posedge CLK);
            SLOT     <= 1'b1;
            MULCOM1  <= 1'b0;
            MULCOM2  <= 8'h00;
            MACIN1   <= 32'd0;
            MACIN2   <= 32'd0;
            @(posedge CLK);
            timeout = 0;
            while (MAC_BUSY == 1'b1 && timeout < 40) begin
                @(posedge CLK);
                timeout = timeout + 1;
            end
            if (timeout >= 40) begin
                $display("TIMEOUT waiting for MAC_BUSY deassert");
                $finish;
            end
            @(posedge CLK); // final latch settle
            @(posedge CLK);
        end
    endtask

    // Issue MAC.L / MAC.W (2-stage: op1 -> M1 first, then op2 -> M2 + engage)
    // MAC instructions use A=MB (= M1 latched in previous cycle), B=M2
    task issue_mac;
        input [7:0] cmd;
        input [31:0] op1;  // goes to M1 then MB
        input [31:0] op2;  // goes to M2
        integer timeout;
        begin
            @(posedge CLK);
            SLOT     <= 1'b1;
            MULCOM1  <= 1'b1;
            MULCOM2  <= 8'h00;
            MACIN1   <= op1;
            MACIN2   <= 32'd0;
            @(posedge CLK);
            SLOT     <= 1'b1;
            MULCOM1  <= 1'b0;
            MULCOM2  <= cmd;
            MACIN1   <= 32'd0;
            MACIN2   <= op2;
            @(posedge CLK);
            SLOT     <= 1'b1;
            MULCOM1  <= 1'b0;
            MULCOM2  <= 8'h00;
            MACIN1   <= 32'd0;
            MACIN2   <= 32'd0;
            @(posedge CLK);
            timeout = 0;
            while (MAC_BUSY == 1'b1 && timeout < 40) begin
                @(posedge CLK);
                timeout = timeout + 1;
            end
            if (timeout >= 40) begin
                $display("TIMEOUT waiting for MAC_BUSY (MAC)");
                $finish;
            end
            @(posedge CLK);
            @(posedge CLK);
        end
    endtask

    // Pre-set MACH and MACL (for MAC.L/MAC.W accumulation tests)
    task set_mac;
        input [31:0] init_mach;
        input [31:0] init_macl;
        begin
            @(posedge CLK);
            SLOT    <= 1'b1;
            WRMACH  <= 1'b1;
            WRMACL  <= 1'b0;
            MACIN1  <= init_mach;
            MACIN2  <= 32'd0;
            @(posedge CLK);
            SLOT    <= 1'b1;
            WRMACH  <= 1'b0;
            WRMACL  <= 1'b1;
            MACIN1  <= 32'd0;
            MACIN2  <= init_macl;
            @(posedge CLK);
            SLOT    <= 1'b0;
            WRMACH  <= 1'b0;
            WRMACL  <= 1'b0;
            MACIN1  <= 32'd0;
            MACIN2  <= 32'd0;
            @(posedge CLK);
        end
    endtask

    task check;
        input [31:0] exp_mach;
        input [31:0] exp_macl;
        input [255:0] name;
        begin
            test_num = test_num + 1;
            if (MACH === exp_mach && MACL === exp_macl) begin
                $display("PASS test %0d: %0s  MACH=%08X MACL=%08X",
                         test_num, name, MACH, MACL);
                pass_count = pass_count + 1;
            end else begin
                $display("FAIL test %0d: %0s  MACH=%08X (exp %08X)  MACL=%08X (exp %08X)",
                         test_num, name, MACH, exp_mach, MACL, exp_macl);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // Check MACL only (ignore MACH for MUL.L/MULSW/MULUW style)
    task check_macl;
        input [31:0] exp_macl;
        input [255:0] name;
        begin
            test_num = test_num + 1;
            if (MACL === exp_macl) begin
                $display("PASS test %0d: %0s  MACL=%08X", test_num, name, MACL);
                pass_count = pass_count + 1;
            end else begin
                $display("FAIL test %0d: %0s  MACL=%08X (exp %08X)",
                         test_num, name, MACL, exp_macl);
                fail_count = fail_count + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("tb_mult.vcd");
        $dumpvars(0, tb_mult);

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
        // DMULS.L (8'hBD): signed 32x32 -> MACH:MACL
        //===========================================
        $display("--- DMULS.L (signed 32x32 -> 64bit) ---");
        issue(8'hBD, 32'd100, 32'd7);
        check(32'd0, 32'd700, "100 * 7");

        issue(8'hBD, -32'sd100, 32'd7);
        check(32'hFFFFFFFF, -32'sd700, "-100 * 7");

        issue(8'hBD, 32'd100, -32'sd7);
        check(32'hFFFFFFFF, -32'sd700, "100 * -7");

        issue(8'hBD, -32'sd100, -32'sd7);
        check(32'd0, 32'd700, "-100 * -7");

        issue(8'hBD, 32'h7FFFFFFF, 32'h7FFFFFFF);
        check(32'h3FFFFFFF, 32'h00000001, "INT32_MAX^2");

        issue(8'hBD, 32'h80000000, 32'h80000000);
        check(32'h40000000, 32'd0, "INT32_MIN^2");

        issue(8'hBD, 32'hFFFFFFFF, 32'hFFFFFFFF);
        check(32'd0, 32'd1, "-1 * -1");

        issue(8'hBD, 32'h80000000, 32'hFFFFFFFF);
        check(32'd0, 32'h80000000, "INT32_MIN * -1");

        issue(8'hBD, 32'd0, 32'hDEADBEEF);
        check(32'd0, 32'd0, "0 * x");

        //===========================================
        // DMULU.L (8'hB5): unsigned 32x32 -> MACH:MACL
        //===========================================
        $display("--- DMULU.L (unsigned 32x32 -> 64bit) ---");
        issue(8'hB5, 32'd100, 32'd7);
        check(32'd0, 32'd700, "100 * 7");

        issue(8'hB5, 32'hFFFFFFFF, 32'hFFFFFFFF);
        check(32'hFFFFFFFE, 32'h00000001, "FFFFFFFF * FFFFFFFF");

        issue(8'hB5, 32'h80000000, 32'd2);
        check(32'h00000001, 32'h00000000, "80000000 * 2");

        issue(8'hB5, 32'h12345678, 32'h9ABCDEF0);
        check(32'h0B00EA4E, 32'h242D2080, "12345678 * 9ABCDEF0");

        issue(8'hB5, 32'hFFFFFFFF, 32'd2);
        check(32'd1, 32'hFFFFFFFE, "FFFFFFFF * 2");

        //===========================================
        // MUL.L (8'h87): signed 32x32 -> MACL only (MACH preserved)
        //===========================================
        $display("--- MUL.L (32x32 -> MACL lower 32bit) ---");
        // Prior instruction preserved MACH. Clear via DMULU.L dummy first.
        issue(8'hB5, 32'd0, 32'd0);  // MACH=0, MACL=0
        issue(8'h87, 32'd100, 32'd7);
        check_macl(32'd700, "100 * 7 (MACL)");

        issue(8'h87, 32'hFFFFFFFF, 32'hFFFFFFFF);
        check_macl(32'd1, "-1 * -1 (lower 32)");

        issue(8'h87, 32'h12345678, 32'h9ABCDEF0);
        check_macl(32'h242D2080, "12345678 * 9ABCDEF0 (low)");

        //===========================================
        // MULS.W (8'hAF): signed 16x16 -> MACL
        //===========================================
        $display("--- MULS.W (signed 16x16 -> 32bit) ---");
        issue(8'hB5, 32'd0, 32'd0);  // clear
        issue(8'hAF, 32'd100, 32'd7);
        check_macl(32'd700, "100 * 7");

        issue(8'hAF, 32'hFFFFFF9C, 32'd7);   // lower 16: 0xFF9C = -100 (signed)
        check_macl(-32'sd700, "-100 * 7 (lower 16)");

        issue(8'hAF, 32'hFFFF8000, 32'hFFFFFFFE);  // -32768 * -2 = 65536
        check_macl(32'h00010000, "-32768 * -2");

        issue(8'hAF, 32'hFFFF7FFF, 32'hFFFF7FFF);  // 32767 * 32767 = 1073676289
        check_macl(32'h3FFF0001, "32767^2");

        issue(8'hAF, 32'hFFFF8000, 32'hFFFF8000);  // -32768 * -32768 = 0x40000000
        check_macl(32'h40000000, "-32768^2");

        //===========================================
        // MULU.W (8'hAE): unsigned 16x16 -> MACL
        //===========================================
        $display("--- MULU.W (unsigned 16x16 -> 32bit) ---");
        issue(8'hAE, 32'd100, 32'd7);
        check_macl(32'd700, "100 * 7 (unsigned)");

        issue(8'hAE, 32'hFFFFFFFF, 32'hFFFFFFFF);   // 0xFFFF * 0xFFFF
        check_macl(32'hFFFE0001, "FFFF * FFFF");

        issue(8'hAE, 32'hFFFF8000, 32'hFFFF0002);   // 0x8000 * 2 = 0x10000
        check_macl(32'h00010000, "0x8000 * 2");

        //===========================================
        // MAC.L (8'h8F): signed 32x32 + MACH:MACL accumulate
        //===========================================
        $display("--- MAC.L (signed 32x32 + MACH:MACL) ---");
        set_mac(32'd0, 32'd100);  // MACH:MACL = 0:100
        issue_mac(8'h8F, 32'd10, 32'd20);  // add 10*20=200
        check(32'd0, 32'd300, "0:100 + 10*20");

        set_mac(32'd0, 32'd0);
        issue_mac(8'h8F, -32'sd100, 32'd7);  // add -700
        check(32'hFFFFFFFF, -32'sd700, "0 + -100*7");

        set_mac(32'h00000001, 32'h00000000);  // 2^32
        issue_mac(8'h8F, 32'd2, 32'hFFFFFFFF);    // add 2 * -1 = -2
        check(32'h00000000, 32'hFFFFFFFE, "2^32 + 2*-1");

        //===========================================
        // MAC.W (8'hCF): signed 16x16 + MACL (MAC_S=0: 32bit add to MACL)
        //===========================================
        $display("--- MAC.W (signed 16x16 + MACL, MAC_S=0) ---");
        set_mac(32'd0, 32'd100);
        issue_mac(8'hCF, 32'd10, 32'd20);
        check(32'd0, 32'd300, "0:100 + 10*20 (MACW)");

        set_mac(32'd0, 32'd1000);
        issue_mac(8'hCF, 32'hFFFFFF9C, 32'd7);  // -100 * 7 = -700
        check(32'd0, 32'd300, "1000 + -100*7 (MACW)");

        //===========================================
        // Random DMULS.L
        //===========================================
        $display("--- Random DMULS.L (500) ---");
        begin : rand_dmulsl
            integer i;
            reg signed [31:0] a, b;
            reg signed [63:0] exp;
            reg rfail;
            rfail = 0;
            for (i = 0; i < 500; i = i + 1) begin
                a = $random;
                b = $random;
                exp = a * b;
                issue(8'hBD, a, b);
                test_num = test_num + 1;
                if (MACH === exp[63:32] && MACL === exp[31:0]) begin
                    pass_count = pass_count + 1;
                end else begin
                    $display("FAIL random DMULSL %0d: %08X * %08X = %08X:%08X (exp %08X:%08X)",
                             i, a, b, MACH, MACL, exp[63:32], exp[31:0]);
                    fail_count = fail_count + 1;
                    rfail = 1;
                end
            end
            if (!rfail) $display("  All 500 DMULS.L random PASS");
        end

        //===========================================
        // Random DMULU.L
        //===========================================
        $display("--- Random DMULU.L (500) ---");
        begin : rand_dmulul
            integer i;
            reg [31:0] a, b;
            reg [63:0] exp;
            reg rfail;
            rfail = 0;
            for (i = 0; i < 500; i = i + 1) begin
                a = $random;
                b = $random;
                exp = {32'd0, a} * {32'd0, b};
                issue(8'hB5, a, b);
                test_num = test_num + 1;
                if (MACH === exp[63:32] && MACL === exp[31:0]) begin
                    pass_count = pass_count + 1;
                end else begin
                    $display("FAIL random DMULUL %0d: %08X * %08X = %08X:%08X (exp %08X:%08X)",
                             i, a, b, MACH, MACL, exp[63:32], exp[31:0]);
                    fail_count = fail_count + 1;
                    rfail = 1;
                end
            end
            if (!rfail) $display("  All 500 DMULU.L random PASS");
        end

        //===========================================
        // Random MULS.W
        //===========================================
        $display("--- Random MULS.W (300) ---");
        begin : rand_mulsw
            integer i;
            reg [31:0] a, b;
            reg signed [15:0] al, bl;
            reg signed [31:0] exp;
            reg rfail;
            rfail = 0;
            issue(8'hB5, 32'd0, 32'd0);  // clear MACH
            for (i = 0; i < 300; i = i + 1) begin
                a = $random;
                b = $random;
                al = a[15:0];
                bl = b[15:0];
                exp = al * bl;
                issue(8'hAF, a, b);
                test_num = test_num + 1;
                if (MACL === exp) begin
                    pass_count = pass_count + 1;
                end else begin
                    $display("FAIL random MULSW %0d: %08X * %08X -> MACL=%08X (exp %08X)",
                             i, a, b, MACL, exp);
                    fail_count = fail_count + 1;
                    rfail = 1;
                end
            end
            if (!rfail) $display("  All 300 MULS.W random PASS");
        end

        //===========================================
        // Random MULU.W
        //===========================================
        $display("--- Random MULU.W (300) ---");
        begin : rand_muluw
            integer i;
            reg [31:0] a, b;
            reg [31:0] exp;
            reg rfail;
            rfail = 0;
            for (i = 0; i < 300; i = i + 1) begin
                a = $random;
                b = $random;
                exp = a[15:0] * b[15:0];
                issue(8'hAE, a, b);
                test_num = test_num + 1;
                if (MACL === exp) begin
                    pass_count = pass_count + 1;
                end else begin
                    $display("FAIL random MULUW %0d: %08X * %08X -> MACL=%08X (exp %08X)",
                             i, a, b, MACL, exp);
                    fail_count = fail_count + 1;
                    rfail = 1;
                end
            end
            if (!rfail) $display("  All 300 MULU.W random PASS");
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
