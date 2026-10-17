`timescale 1ns/1ps
`include "timescale.v"
`include "defines.v"

// Integration test for CAS.L Rm,Rn,@R0 (2nm3), J-core compatible:
//   tmp = @R0; if (tmp == Rm) { @R0 = Rn; T=1; } else T=0; Rn = tmp;
// Runs hand-assembled machine code through the full cpu (decode+datapath+
// register file+mem) with a simple Wishbone memory model, exactly like
// tb_cpu_custom.v.
module tb_cas;

    reg CLK, RST;
    wire CYC, STB, WE;
    reg  ACK;
    wire [31:0] ADR, DATO;
    reg  [31:0] DATI;
    wire [3:0] SEL;
    wire EVENT_ACK, SLP;

    cpu CPU (
        .CLK(CLK), .RST(RST),
        .CYC_O(CYC), .STB_O(STB), .ACK_I(ACK),
        .ADR_O(ADR), .DAT_I(DATI), .DAT_O(DATO),
        .WE_O(WE), .SEL_O(SEL),
        .TAG0_I(1'b0),
        .EVENT_REQ_I(`NO_EVT),
        .EVENT_ACK_O(EVENT_ACK),
        .EVENT_INFO_I(12'h000),
        .SLP_O(SLP)
    );

    always @(*) ACK = STB;

    // Word-addressable 32-bit memory for the "shared variable" area, plus
    // 16-bit instruction memory mirrored into both halves like tb_cpu_custom.
    reg [15:0] IMEM [0:1023];
    reg [31:0] DMEM [0:255]; // word addr = ADR[9:2], byte range 0x800-0xBFF

    wire in_dmem = (ADR >= 32'h00000800) && (ADR < 32'h00000C00);
    always @(*) begin
        if (ADR == 32'h00000000) DATI = 32'h00000010;      // reset vector: initial PC
        else if (ADR == 32'h00000004) DATI = 32'h00001000; // reset vector: initial R15
        else if (in_dmem) DATI = DMEM[ADR[9:2]];
        else DATI = {IMEM[ADR[10:1]], IMEM[ADR[10:1]]};
    end

    always @(posedge CLK) begin
        if (STB && WE && in_dmem)
            DMEM[ADR[9:2]] <= DATO;
    end

    // Atomicity check: once CAS.L's read of the shared word is on the bus,
    // CYC must stay asserted (read-modify-write cycle) until the next
    // instruction fetch begins -- the arbiter relies on this to not hand the
    // bus to another master in the middle.
    reg in_rmw;
    integer cyc_drops;
    initial begin in_rmw = 1'b0; cyc_drops = 0; end
    always @(posedge CLK) begin
        if (in_rmw && !CYC) cyc_drops = cyc_drops + 1;
        if (STB && !WE && ADR == 32'h00000800) in_rmw <= 1'b1;
        else if (STB && !WE && ADR != 32'h00000800) in_rmw <= 1'b0;
    end

    initial CLK <= 1'b0;
    always #(`HALF_CYCLE) CLK <= ~CLK;

    localparam NOP = 16'h0009;
    function [15:0] mov_imm;
        input [3:0] n;
        input [7:0] imm;
        mov_imm = 16'hE000 | (n << 8) | imm;
    endfunction
    function [15:0] cas_l;
        input [3:0] n, m; // Rn=new value / old value out, Rm=expected; R0=address
        cas_l = 16'h2003 | (n << 8) | (m << 4);
    endfunction

    integer i;
    integer test2_start;
    integer test3_start;
    integer test3_end;
    // Layout:
    //   0x800: shared variable (long), address kept in R0 for all tests
    //          (0x800 = 0x08 << 8: mov #8,r0 / shll8 r0)
    initial begin
        for (i = 0; i < 1024; i = i + 1) IMEM[i] = NOP;
        for (i = 0; i < 256; i = i + 1) DMEM[i] = 32'h00000000;

        i = 8;
        // ---- Test 1: CAS match (mem == Rm) ----
        IMEM[i]=mov_imm(0, 8'h08);  i=i+1; // r0 = 8
        IMEM[i]=16'h4018;           i=i+1; // shll8 r0   -> r0 = 0x800 (address)
        IMEM[i]=mov_imm(1, 8'h11); i=i+1; // r1 = 0x11 (expected, matches DMEM preset)
        IMEM[i]=mov_imm(2, 8'h55); i=i+1; // r2 = 0x55 (new value to store on match)
        IMEM[i]=cas_l(2,1);        i=i+1; // CAS.L r1,r2,@r0
        IMEM[i]=NOP;                i=i+1;
        IMEM[i]=NOP;                i=i+1;
        IMEM[i]=NOP;                i=i+1;
        IMEM[i]=NOP;                i=i+1;
        IMEM[i]=NOP;                i=i+1;
        test2_start = i*2;          // byte address where test2 begins

        // ---- Test 2: CAS mismatch (mem != Rm) ----
        IMEM[i]=mov_imm(3, 8'h7F); i=i+1; // r3 = 0x7F (deliberately wrong expectation)
        IMEM[i]=mov_imm(4, 8'h33); i=i+1; // r4 = 0x33 (would-be new value, should NOT be written)
        IMEM[i]=cas_l(4,3);        i=i+1; // CAS.L r3,r4,@r0
        IMEM[i]=NOP;                i=i+1;
        IMEM[i]=NOP;                i=i+1;
        IMEM[i]=NOP;                i=i+1;
        IMEM[i]=NOP;                i=i+1;
        IMEM[i]=NOP;                i=i+1;
        test3_start = i*2;

        // ---- Test 3: n == m, register value matches mem ----
        IMEM[i]=mov_imm(5, 8'h55); i=i+1; // r5 = 0x55 (== mem after test1)
        IMEM[i]=cas_l(5,5);        i=i+1; // CAS.L r5,r5,@r0
        IMEM[i]=NOP;                i=i+1;
        IMEM[i]=NOP;                i=i+1;
        IMEM[i]=NOP;                i=i+1;
        IMEM[i]=NOP;                i=i+1;
        IMEM[i]=NOP;                i=i+1;
        test3_end = i*2;

        DMEM[0] = 32'h00000011; // matches r1=0x11 for test1
    end

    integer pass_count = 0;
    integer fail_count = 0;
    task check32;
        input [200:0] name;
        input [31:0] got, exp;
        begin
            if (got !== exp) begin
                fail_count = fail_count + 1;
                $display("FAIL %0s got=%08h exp=%08h", name, got, exp);
            end else begin
                pass_count = pass_count + 1;
                $display("pass %0s got=%08h", name, got);
            end
        end
    endtask

    initial begin
        RST = 1;
        #(`CYCLE * 4);
        RST = 0;

        wait (CPU.DATAPATH.PC == test2_start);
        #(`CYCLE * 2); // let the pipeline fully settle past the boundary
        // ---- check test1 (match): mem := Rn(0x55), Rn := old(0x11), R0/Rm unchanged, T=1 ----
        check32("test1 mem[@0x800] after match", DMEM[0], 32'h00000055);
        check32("test1 Rn gets old value",       CPU.DATAPATH.REGISTER.REG[2], 32'h00000011);
        check32("test1 Rm unchanged",            CPU.DATAPATH.REGISTER.REG[1], 32'h00000011);
        check32("test1 R0 unchanged",            CPU.DATAPATH.REGISTER.REG[0], 32'h00000800);
        check32("test1 T bit set on match",      {31'b0, CPU.DATAPATH.SR[0]}, 32'h00000001);

        wait (CPU.DATAPATH.PC == test3_start);
        #(`CYCLE * 2);
        // ---- check test2 (mismatch): mem unchanged (0x55), Rn := old(0x55), R0 unchanged, T=0 ----
        check32("test2 mem[@0x800] unchanged on mismatch", DMEM[0], 32'h00000055);
        check32("test2 Rn gets old value",                 CPU.DATAPATH.REGISTER.REG[4], 32'h00000055);
        check32("test2 Rm unchanged",                      CPU.DATAPATH.REGISTER.REG[3], 32'h0000007F);
        check32("test2 R0 unchanged",                      CPU.DATAPATH.REGISTER.REG[0], 32'h00000800);
        check32("test2 T bit clear on mismatch",           {31'b0, CPU.DATAPATH.SR[0]}, 32'h00000000);

        wait (CPU.DATAPATH.PC == test3_end);
        #(`CYCLE * 2);
        // ---- check test3 (n == m, match): mem and the register unchanged, T=1 ----
        check32("test3 mem[@0x800] unchanged", DMEM[0], 32'h00000055);
        check32("test3 Rn(=Rm) unchanged",     CPU.DATAPATH.REGISTER.REG[5], 32'h00000055);
        check32("test3 R0 unchanged",          CPU.DATAPATH.REGISTER.REG[0], 32'h00000800);
        check32("test3 T bit set on match",    {31'b0, CPU.DATAPATH.SR[0]}, 32'h00000001);

        check32("CYC_O held through all RMW cycles (drops)", cyc_drops, 32'h00000000);

        $display("---- cas.l test: %0d pass, %0d fail ----", pass_count, fail_count);
        if (fail_count != 0) $display("*** CAS.L TEST FAILED ***");
        else $display("*** CAS.L TEST PASSED ***");
        $finish;
    end
endmodule
