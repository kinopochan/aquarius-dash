`timescale 1ns/1ps
`include "timescale.v"
`include "defines.v"

// Integration smoke test: run hand-assembled SH machine code through the
// full `cpu` (decode + datapath + register file + mem pipeline stage) to
// confirm the new SHAD/SHLD/ROTLV/ROTRV opcodes decode correctly and their
// results land in the right register via the real pipeline/forwarding,
// not just the isolated shifter expression (see tb_shift_unit.v for that).
module tb_cpu_custom;

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
        .TAG0_I(1'b0),               // 16bit-wide instruction space -> simplest mem.v path
        .EVENT_REQ_I(`NO_EVT),
        .EVENT_ACK_O(EVENT_ACK),
        .EVENT_INFO_I(12'h000),
        .SLP_O(SLP)
    );

    // 0-wait combinational ack, always ready
    always @(*) ACK = STB;

    // 16-bit-word instruction memory, mirrored into both halves of the
    // 32-bit data bus so mem.v's ADR[1] byte-select always lands on the
    // right word regardless of address parity.
    reg [15:0] IMEM [0:1023];
    always @(*) begin
        if (ADR == 32'h00000000) DATI = 32'h00000010; // reset vector: initial PC
        else if (ADR == 32'h00000004) DATI = 32'h00001000; // reset vector: initial R15
        else DATI = {IMEM[ADR[10:1]], IMEM[ADR[10:1]]};
    end

    initial CLK <= 1'b0;
    always #(`HALF_CYCLE) CLK <= ~CLK;

    // ---- program ----
    // r1 = 5; r2 = 3; SHAD r2,r1      -> r1 = 5<<3 = 40 (0x28)
    // r3 = -8 (0xFFFFFFF8); r4 = -1 (0xFFFFFFFF, amt=32-31=1); SHAD r4,r3 -> r3 = 0xFFFFFFFC
    // r5 = -8; r6 = -1; SHLD r6,r5    -> r5 = 0x7FFFFFFC (logical, top bit cleared)
    // r0 = 4; r7 = 1; ROTLV r0,r7     -> r7 = 0x00000010
    // r0 = 4; r8 = 0x10 (via mov #16); ROTRV r0,r8 -> r8 = 0x00000001
    // infinite loop (bra self) to park the core
    localparam MOV   = 16'hE000; // MOV #imm,Rn  : E n ii
    function [15:0] mov_imm;
        input [3:0] n;
        input [7:0] imm;
        mov_imm = MOV | (n << 8) | imm;
    endfunction
    function [15:0] shad;
        input [3:0] n, m;
        shad = 16'h4000 | (n << 8) | (m << 4) | 4'hC;
    endfunction
    function [15:0] shld;
        input [3:0] n, m;
        shld = 16'h4000 | (n << 8) | (m << 4) | 4'hD;
    endfunction
    function [15:0] rotlv;
        input [3:0] n;
        rotlv = 16'h4000 | (n << 8) | 8'h34;
    endfunction
    function [15:0] rotrv;
        input [3:0] n;
        rotrv = 16'h4000 | (n << 8) | 8'h35;
    endfunction
    localparam NOP = 16'h0009;

    integer i;
    initial begin
        for (i = 0; i < 1024; i = i + 1) IMEM[i] = NOP;
        i = 8; // byte address 0x10 (reset vector points here)
        IMEM[i]=mov_imm(1, 8'd5);    i=i+1;
        IMEM[i]=mov_imm(2, 8'd3);    i=i+1;
        IMEM[i]=shad(1,2);           i=i+1;  // r1 = 5<<3
        IMEM[i]=NOP;                 i=i+1;
        IMEM[i]=NOP;                 i=i+1;

        IMEM[i]=mov_imm(3, -8);      i=i+1;
        IMEM[i]=mov_imm(4, -1);      i=i+1;
        IMEM[i]=shad(3,4);           i=i+1;  // r3 = 0xFFFFFFF8 >>> 1
        IMEM[i]=NOP;                 i=i+1;
        IMEM[i]=NOP;                 i=i+1;

        IMEM[i]=mov_imm(5, -8);      i=i+1;
        IMEM[i]=mov_imm(6, -1);      i=i+1;
        IMEM[i]=shld(5,6);           i=i+1;  // r5 = 0xFFFFFFF8 >> 1 (logical)
        IMEM[i]=NOP;                 i=i+1;
        IMEM[i]=NOP;                 i=i+1;

        IMEM[i]=mov_imm(0, 8'd4);    i=i+1;
        IMEM[i]=mov_imm(7, 8'd1);    i=i+1;
        IMEM[i]=rotlv(7);            i=i+1;  // r7 = rotl(1,4) = 0x10
        IMEM[i]=NOP;                 i=i+1;
        IMEM[i]=NOP;                 i=i+1;

        IMEM[i]=mov_imm(0, 8'd4);    i=i+1;
        IMEM[i]=mov_imm(8, 8'd16);   i=i+1;
        IMEM[i]=rotrv(8);            i=i+1;  // r8 = rotr(0x10,4) = 1
        IMEM[i]=NOP;                 i=i+1;
        IMEM[i]=NOP;                 i=i+1;
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

        // enough cycles for ~25 instructions to retire through the pipeline
        #(`CYCLE * 400);

        check32("SHAD r1,r2 (5<<3)",        CPU.DATAPATH.REGISTER.REG[1], 32'h00000028);
        check32("SHAD r3,r4 (neg amt=1)",   CPU.DATAPATH.REGISTER.REG[3], 32'hFFFFFFFC);
        check32("SHLD r5,r6 (neg amt=1)",   CPU.DATAPATH.REGISTER.REG[5], 32'h7FFFFFFC);
        check32("ROTLV r7 (1 rol 4)",       CPU.DATAPATH.REGISTER.REG[7], 32'h00000010);
        check32("ROTRV r8 (0x10 ror 4)",    CPU.DATAPATH.REGISTER.REG[8], 32'h00000001);

        $display("---- cpu integration: %0d pass, %0d fail ----", pass_count, fail_count);
        if (fail_count != 0) $display("*** CPU INTEGRATION TEST FAILED ***");
        else $display("*** CPU INTEGRATION TEST PASSED ***");
        $finish;
    end
endmodule
