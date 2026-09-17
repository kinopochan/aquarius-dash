`timescale 1ns/1ps
`include "timescale.v"
`include "defines.v"

// Integration test for CAS.L Rm,Rn,@R0 (2nm3):
//   if (@Rn == R0) { *Rn = Rm; T=1; } else { R0 = @Rn; T=0; }
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

    initial CLK <= 1'b0;
    always #(`HALF_CYCLE) CLK <= ~CLK;

    localparam NOP = 16'h0009;
    function [15:0] mov_imm;
        input [3:0] n;
        input [7:0] imm;
        mov_imm = 16'hE000 | (n << 8) | imm;
    endfunction
    function [15:0] cas_l;
        input [3:0] n, m; // Rn=addr, Rm=new value; R0 implicit
        cas_l = 16'h2003 | (n << 8) | (m << 4);
    endfunction

    integer i;
    integer test2_start;
    integer test2_end;
    // --- Simpler, explicit plan built directly as opcodes ---
    // Layout:
    //   0x800: shared variable (long)
    //   main:
    //     mov #0x20,r1     ; r1 = 0x20
    //     shll8 r1         ; r1 = 0x2000        -> not what we want; instead:
    // We just want r1 = 0x800. 0x800 = 0x20 << 6. No single shift-by-6 op.
    // Easiest: 0x800 = 0x08 << 8 (shll8, *256). 0x08*256=0x800. exact!
    initial begin
        for (i = 0; i < 1024; i = i + 1) IMEM[i] = NOP;
        for (i = 0; i < 256; i = i + 1) DMEM[i] = 32'h00000000;

        i = 8;
        // ---- Test 1: CAS match (R0 == mem) ----
        IMEM[i]=mov_imm(1, 8'h08);  i=i+1; // r1 = 8
        IMEM[i]=16'h4118;           i=i+1; // shll8 r1   -> r1 = 0x800 (address)
        IMEM[i]=mov_imm(0, 8'h11); i=i+1; // r0 = 0x11 (expected, matches DMEM preset low byte pattern below)
        IMEM[i]=mov_imm(2, 8'h55); i=i+1; // r2 = 0x55 (new value to store on match)
        IMEM[i]=cas_l(1,2);        i=i+1; // CAS.L r2,r1,@r0  -> compares @r1 vs r0
        IMEM[i]=NOP;                i=i+1;
        IMEM[i]=NOP;                i=i+1;
        IMEM[i]=NOP;                i=i+1;
        IMEM[i]=NOP;                i=i+1;
        IMEM[i]=NOP;                i=i+1;
        test2_start = i*2;          // byte address where test2 begins

        // ---- Test 2: CAS mismatch (R0 != mem) ----
        IMEM[i]=mov_imm(3, 8'h08);  i=i+1; // r3 = 8
        IMEM[i]=16'h4318;           i=i+1; // shll8 r3 -> r3 = 0x800 (reuse same word; still holds 0x55 from test1 if matched)
        IMEM[i]=mov_imm(0, 8'h7F); i=i+1; // r0 = 0x7F (deliberately wrong expectation)
        IMEM[i]=mov_imm(4, 8'h33); i=i+1; // r4 = 0x33 (would-be new value, should NOT be written)
        IMEM[i]=cas_l(3,4);        i=i+1; // CAS.L r4,r3,@r0
        IMEM[i]=NOP;                i=i+1;
        IMEM[i]=NOP;                i=i+1;
        IMEM[i]=NOP;                i=i+1;
        IMEM[i]=NOP;                i=i+1;
        IMEM[i]=NOP;                i=i+1;
        test2_end = i*2; // byte address right after test2's trailing nops

        DMEM[0] = 32'h00000011; // matches r0=0x11 for test1
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
        // ---- check test1 (match): mem := 0x55, R0 unchanged (0x11), T=1 ----
        check32("test1 mem[@0x800] after match", DMEM[0], 32'h00000055);
        check32("test1 R0 unchanged on match",   CPU.DATAPATH.REGISTER.REG[0], 32'h00000011);
        check32("test1 T bit set on match",      {31'b0, CPU.DATAPATH.SR[0]}, 32'h00000001);

        wait (CPU.DATAPATH.PC == test2_end);
        #(`CYCLE * 2);
        // ---- check test2 (mismatch): mem unchanged (still 0x55), R0 := 0x55, T=0 ----
        check32("test2 mem[@0x800] unchanged on mismatch", DMEM[0], 32'h00000055);
        check32("test2 R0 becomes loaded value",           CPU.DATAPATH.REGISTER.REG[0], 32'h00000055);
        check32("test2 T bit clear on mismatch",           {31'b0, CPU.DATAPATH.SR[0]}, 32'h00000000);

        $display("---- cas.l test: %0d pass, %0d fail ----", pass_count, fail_count);
        if (fail_count != 0) $display("*** CAS.L TEST FAILED ***");
        else $display("*** CAS.L TEST PASSED ***");
        $finish;
    end
endmodule
