`timescale 1ns/1ps
`include "defines.v"

// Extracted verbatim from datapath.v (SHAD/SHLD/ROTLV/ROTRV portion) so we
// exercise the exact RTL expressions that were added, without dragging in
// the whole datapath/register-file/decode machinery.
module sft_unit (
    input      [4:0]  SFTFUNC,
    input      [31:0] XBUS,
    input      [31:0] YBUS,
    output reg [31:0] SFTOUT
);
    wire        [4:0]  DSFT_N  = YBUS[4:0];
    wire signed [31:0] XBUS_S  = XBUS;
    wire        [31:0] DSFT_L  = XBUS << DSFT_N;
    wire        [31:0] DSFT_AR = (XBUS_S >>> 1) >>> (~DSFT_N);
    wire        [31:0] DSFT_LR = (XBUS   >>  1) >>  (~DSFT_N);
    wire        [31:0] DSFT_R  = XBUS >> DSFT_N;
    wire        [31:0] DSFT_RL = (XBUS << 1) << (~DSFT_N);

    always @*
    begin
        case (SFTFUNC)
            `SHAD   : SFTOUT <= YBUS[31] ? DSFT_AR : DSFT_L;
            `SHLD   : SFTOUT <= YBUS[31] ? DSFT_LR : DSFT_L;
            `ROTLV  : SFTOUT <= DSFT_L | DSFT_LR;
            `ROTRV  : SFTOUT <= DSFT_R | DSFT_RL;
            default : SFTOUT <= 32'hxxxxxxxx;
        endcase
    end
endmodule

module tb_shift_unit;
    reg  [4:0]  SFTFUNC;
    reg  [31:0] XBUS, YBUS;
    wire [31:0] SFTOUT;

    sft_unit DUT (.SFTFUNC(SFTFUNC), .XBUS(XBUS), .YBUS(YBUS), .SFTOUT(SFTOUT));

    integer i;
    integer pass_count = 0;
    integer fail_count = 0;

    // Independent reference model, written straight from the SH-3
    // SHAD/SHLD/ROTLV/ROTRV spec (aquarius-dash-plan.md section 3/4),
    // deliberately NOT sharing expressions with the RTL above.
    function [31:0] ref_shad;
        input [31:0] rn;
        input [31:0] rm;
        integer amt;
        begin
            amt = rm[4:0];
            if (!rm[31])
                ref_shad = rn << amt;
            else if (amt == 0)
                ref_shad = rn[31] ? 32'hFFFFFFFF : 32'h00000000; // fill with sign
            else
                ref_shad = $signed(rn) >>> (32 - amt);
        end
    endfunction

    function [31:0] ref_shld;
        input [31:0] rn;
        input [31:0] rm;
        integer amt;
        begin
            amt = rm[4:0];
            if (!rm[31])
                ref_shld = rn << amt;
            else if (amt == 0)
                ref_shld = 32'h00000000;
            else
                ref_shld = rn >> (32 - amt);
        end
    endfunction

    function [31:0] ref_rotlv;
        input [31:0] rn;
        input [31:0] r0;
        integer amt;
        begin
            amt = r0[4:0];
            ref_rotlv = (rn << amt) | (rn >> (32 - amt)); // amt=0 -> rn>>32 == 0 in real HW; handled by case below
            if (amt == 0) ref_rotlv = rn;
        end
    endfunction

    function [31:0] ref_rotrv;
        input [31:0] rn;
        input [31:0] r0;
        integer amt;
        begin
            amt = r0[4:0];
            ref_rotrv = (rn >> amt) | (rn << (32 - amt));
            if (amt == 0) ref_rotrv = rn;
        end
    endfunction

    task check;
        input [200:0] name;
        input [31:0] got;
        input [31:0] exp;
        begin
            if (got !== exp) begin
                fail_count = fail_count + 1;
                $display("FAIL %0s  XBUS=%08h YBUS=%08h  got=%08h exp=%08h",
                          name, XBUS, YBUS, got, exp);
            end else begin
                pass_count = pass_count + 1;
            end
        end
    endtask

    reg [31:0] rn, rm;
    initial begin
        // ---- boundary cases: amt = 0 and amt = 31, both signs of rm/r0 ----
        rn = 32'h80000000; rm = 32'h00000000; // amt=0, positive
        XBUS=rn; YBUS=rm; SFTFUNC=`SHAD;  #1; check("SHAD amt0 pos", SFTOUT, ref_shad(rn,rm));
        SFTFUNC=`SHLD;  #1; check("SHLD amt0 pos", SFTOUT, ref_shld(rn,rm));

        rn = 32'h80000001; rm = 32'hFFFFFFE0; // amt=0 (low5=0), negative
        XBUS=rn; YBUS=rm; SFTFUNC=`SHAD;  #1; check("SHAD amt0 neg", SFTOUT, ref_shad(rn,rm));
        SFTFUNC=`SHLD;  #1; check("SHLD amt0 neg", SFTOUT, ref_shld(rn,rm));

        rn = 32'h00000001; rm = 32'hFFFFFFFF; // amt=31, negative -> shift right 1
        XBUS=rn; YBUS=rm; SFTFUNC=`SHAD;  #1; check("SHAD amt31 neg", SFTOUT, ref_shad(rn,rm));
        SFTFUNC=`SHLD;  #1; check("SHLD amt31 neg", SFTOUT, ref_shld(rn,rm));

        rn = 32'h80000000; rm = 32'h0000001F; // amt=31, positive -> shift left 31
        XBUS=rn; YBUS=rm; SFTFUNC=`SHAD;  #1; check("SHAD amt31 pos", SFTOUT, ref_shad(rn,rm));
        SFTFUNC=`SHLD;  #1; check("SHLD amt31 pos", SFTOUT, ref_shld(rn,rm));

        rn = 32'h12345678; rm = 32'h00000000; XBUS=rn; YBUS=rm;
        SFTFUNC=`ROTLV; #1; check("ROTLV amt0", SFTOUT, ref_rotlv(rn,rm));
        SFTFUNC=`ROTRV; #1; check("ROTRV amt0", SFTOUT, ref_rotrv(rn,rm));

        rn = 32'h12345678; rm = 32'h0000001F; XBUS=rn; YBUS=rm;
        SFTFUNC=`ROTLV; #1; check("ROTLV amt31", SFTOUT, ref_rotlv(rn,rm));
        SFTFUNC=`ROTRV; #1; check("ROTRV amt31", SFTOUT, ref_rotrv(rn,rm));

        // ---- random sweep: rn random, rm/r0 sweeping amt 0..31 with both signs, plus random garbage in upper bits ----
        for (i = 0; i < 300000; i = i + 1) begin
            rn = {$random, $random};
            rm = {$random, $random};
            XBUS = rn; YBUS = rm;
            SFTFUNC = `SHAD; #1; check("SHAD rand", SFTOUT, ref_shad(rn, rm));
            SFTFUNC = `SHLD; #1; check("SHLD rand", SFTOUT, ref_shld(rn, rm));
        end
        for (i = 0; i < 200000; i = i + 1) begin
            rn = {$random, $random};
            rm = {$random, $random};
            XBUS = rn; YBUS = rm;
            SFTFUNC = `ROTLV; #1; check("ROTLV rand", SFTOUT, ref_rotlv(rn, rm));
            SFTFUNC = `ROTRV; #1; check("ROTRV rand", SFTOUT, ref_rotrv(rn, rm));
        end

        $display("---- shift unit: %0d pass, %0d fail ----", pass_count, fail_count);
        if (fail_count != 0) $display("*** SHIFT UNIT TEST FAILED ***");
        else $display("*** SHIFT UNIT TEST PASSED ***");
        $finish;
    end
endmodule
