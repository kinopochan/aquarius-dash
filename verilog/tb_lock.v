`timescale 1ns/1ps
`include "timescale.v"
`include "defines.v"

// LOCK_O test: LOCK_O must cover every read-modify-write (CAS.L / TAS.B)
// so that a multi-master arbiter can keep this master's grant from the
// read through the write, and must stay low for ordinary accesses.
//
// Rule (what an arbiter relies on):
//   (a) LOCK_O = 1 in every cycle of the RMW read
//   (b) after the read is acked, LOCK_O = 1 in every cycle until the next
//       access starts (the write, or for CAS.L mismatch the next fetch)
//       -- the arbiter registers LOCK_O, so the cycle before the write
//       must have LOCK_O = 1 and that keeps the grant through the write
//   (c) LOCK_O = 0 outside RMW instructions. The only allowed tail is the
//       first access after the read, until it is acked: LOCK_O is
//       NEXT_KEEP_CYC, which is cleared at the end of an access, so after a
//       CAS.L mismatch (no write) it stays 1 through the following fetch
//
// Parameters:
//   IFWAIT / DWAIT : wait states for instruction fetch / data access
//   FAULT          : 1 = force LOCK_O to 0 for one cycle right after the
//                    TAS.B read (checks that this bench does notice a gap:
//                    exactly 1 violation is expected)
//
//   iverilog -g2005 -o tb_lock tb_lock.v cpu.v decode.v datapath.v mem.v mult.v register.v
//   vvp tb_lock                 (or -P tb_lock.DWAIT=2 etc.)
module tb_lock;
    parameter IFWAIT = 0;
    parameter DWAIT  = 0;
    parameter FAULT  = 0;

    reg CLK, RST;
    wire CYC, STB, WE, ACK, LOCK;
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
        .SLP_O(SLP),
        .LOCK_O(LOCK)
    );

    // memory: 0x800 = CAS.L word, 0x804 = ordinary word, 0x808 = TAS.B byte
    reg [15:0] IMEM [0:1023];
    reg [31:0] DMEM [0:255];
    wire in_dmem = (ADR >= 32'h00000800) && (ADR < 32'h00000C00);
    wire is_rmw_adr = in_dmem && ((ADR[11:2] == 10'h200) || (ADR[11:2] == 10'h202));

    integer wcnt;
    wire [31:0] need = in_dmem ? DWAIT : IFWAIT;
    assign ACK = STB && (wcnt >= need);
    always @(posedge CLK)
        if (!STB || ACK) wcnt <= 0; else wcnt <= wcnt + 1;
    initial wcnt = 0;

    always @(*) begin
        if (ADR == 32'h00000000) DATI = 32'h00000010;
        else if (ADR == 32'h00000004) DATI = 32'h00001000;
        else if (in_dmem) DATI = DMEM[ADR[9:2]];
        else DATI = {IMEM[ADR[10:1]], IMEM[ADR[10:1]]};
    end
    always @(posedge CLK)
        if (STB && WE && ACK && in_dmem) begin
            if (SEL[3]) DMEM[ADR[9:2]][31:24] <= DATO[31:24];
            if (SEL[2]) DMEM[ADR[9:2]][23:16] <= DATO[23:16];
            if (SEL[1]) DMEM[ADR[9:2]][15: 8] <= DATO[15: 8];
            if (SEL[0]) DMEM[ADR[9:2]][ 7: 0] <= DATO[ 7: 0];
        end

    initial CLK <= 1'b0;
    always #(`HALF_CYCLE) CLK <= ~CLK;

    localparam NOP = 16'h0009;
    function [15:0] mov_imm; input [3:0] n; input [7:0] imm;
        mov_imm = 16'hE000 | (n << 8) | imm; endfunction
    function [15:0] cas_l; input [3:0] n, m;
        cas_l = 16'h2003 | (n << 8) | (m << 4); endfunction

    integer i, k, prog_end;
    initial begin
        for (i = 0; i < 1024; i = i + 1) IMEM[i] = NOP;
        for (i = 0; i < 256; i = i + 1) DMEM[i] = 32'h0;
        DMEM[0] = 32'h00000011;
        DMEM[1] = 32'h12345678;
        i = 8;
        IMEM[i]=mov_imm(0,8'h08); i=i+1;
        IMEM[i]=16'h4018;         i=i+1;      // r0 = 0x800
        IMEM[i]=16'h6703;         i=i+1;      // mov r0,r7
        IMEM[i]=16'h7708;         i=i+1;      // add #8,r7  -> 0x808
        IMEM[i]=16'h6803;         i=i+1;      // mov r0,r8
        IMEM[i]=16'h7804;         i=i+1;      // add #4,r8  -> 0x804
        IMEM[i]=mov_imm(1,8'h11); i=i+1;
        IMEM[i]=mov_imm(2,8'h55); i=i+1;
        IMEM[i]=cas_l(2,1);       i=i+1;      // CAS.L match    (0x11 -> 0x55)
        for (k=0;k<4;k=k+1) begin IMEM[i]=NOP; i=i+1; end
        IMEM[i]=mov_imm(3,8'h7F); i=i+1;
        IMEM[i]=mov_imm(4,8'h33); i=i+1;
        IMEM[i]=cas_l(4,3);       i=i+1;      // CAS.L mismatch (stays 0x55)
        for (k=0;k<4;k=k+1) begin IMEM[i]=NOP; i=i+1; end
        IMEM[i]=16'h471B;         i=i+1;      // TAS.B @r7      (0x00 -> 0x80, T=1)
        for (k=0;k<4;k=k+1) begin IMEM[i]=NOP; i=i+1; end
        IMEM[i]=16'h471B;         i=i+1;      // TAS.B @r7      (0x80 stays, T=0)
        for (k=0;k<4;k=k+1) begin IMEM[i]=NOP; i=i+1; end
        IMEM[i]=16'h6682;         i=i+1;      // mov.l @r8,r6   (ordinary)
        IMEM[i]=16'h2862;         i=i+1;      // mov.l r6,@r8
        for (k=0;k<8;k=k+1) begin IMEM[i]=NOP; i=i+1; end
        prog_end = i*2;
    end

    // ---- checker ----
    integer viol, n_rmw_rd, n_rmw_wr, cyc_n;
    reg pending, tail, rd_seen;
    initial begin viol = 0; n_rmw_rd = 0; n_rmw_wr = 0; cyc_n = 0;
                  pending = 0; tail = 0; rd_seen = 0; end

    always @(posedge CLK) if (!RST) begin
        cyc_n <= cyc_n + 1;
        // (a) RMW read
        if (STB && !WE && is_rmw_adr) begin
            if (!LOCK) begin viol = viol + 1;
                $display("VIOL (a) cycle %0d: LOCK=0 during RMW read adr=%h", cyc_n, ADR); end
            if (!rd_seen) n_rmw_rd = n_rmw_rd + 1;
            rd_seen = 1;
            if (ACK) begin pending <= 1; rd_seen = 0; end
        end
        // (b) between read and the next access
        else if (pending) begin
            if (STB) begin
                pending <= 0;
                tail <= !ACK;    // first access after the read, until acked
                if (WE && is_rmw_adr) n_rmw_wr = n_rmw_wr + 1;
            end else if (!LOCK) begin viol = viol + 1;
                $display("VIOL (b) cycle %0d: LOCK=0 between RMW read and next access", cyc_n); end
        end
        // (c) ordinary
        else if (tail) begin
            if (ACK) tail <= 0;
        end
        else if (LOCK) begin viol = viol + 1;
            $display("VIOL (c) cycle %0d: LOCK=1 outside RMW adr=%h stb=%b we=%b", cyc_n, ADR, STB, WE); end
    end

    // fault injection: one-cycle gap right after the first TAS.B read is acked
    generate if (FAULT) begin : g_fault
        reg done;
        initial done = 0;
        always @(posedge CLK)
            if (!done && STB && !WE && ACK && in_dmem && ADR[11:2] == 10'h202) begin
                done <= 1;
                @(negedge CLK) force CPU.LOCK = 1'b0;
                @(negedge CLK) release CPU.LOCK;
            end
    end endgenerate

    integer fails;
    initial begin
        fails = 0;
        RST = 1; #(`CYCLE * 4); RST = 0;
        wait (CPU.DATAPATH.PC == prog_end);
        #(`CYCLE * 4);
        if (DMEM[0] !== 32'h00000055) begin fails = fails + 1; $display("FAIL CAS word %08h", DMEM[0]); end
        if (DMEM[1] !== 32'h12345678) begin fails = fails + 1; $display("FAIL ordinary word %08h", DMEM[1]); end
        if (DMEM[2] !== 32'h80000000) begin fails = fails + 1; $display("FAIL TAS byte %08h", DMEM[2]); end
        // 2 CAS.L + 2 TAS.B reads; writes: CAS match + 2 TAS.B = 3
        if (n_rmw_rd != 4) begin fails = fails + 1; $display("FAIL RMW reads seen %0d (exp 4)", n_rmw_rd); end
        if (n_rmw_wr != 3) begin fails = fails + 1; $display("FAIL RMW writes seen %0d (exp 3)", n_rmw_wr); end
        $display("IFWAIT=%0d DWAIT=%0d FAULT=%0d : violations %0d (exp %0d), other fails %0d",
                 IFWAIT, DWAIT, FAULT, viol, FAULT ? 1 : 0, fails);
        if (fails == 0 && viol == (FAULT ? 1 : 0)) $display("*** LOCK TEST PASSED ***");
        else $display("*** LOCK TEST FAILED ***");
        $finish;
    end
endmodule
