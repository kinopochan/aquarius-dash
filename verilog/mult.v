//======================================================
// Aquarius Project
//    SuperH-2 ISA Compatible RISC CPU
//------------------------------------------------------
// Module      : Multiplier Unit
//------------------------------------------------------
// File        : mult.v
// Library     : none
// Description : Multiplier Unit in CPU.
// Simulator   : Icarus Verilog (Cygwin)
// Synthesizer : Xilinx XST (Windows XP)
// Author      : Thorn Aitch
//------------------------------------------------------
// Revision Number : 1
// Date of Change  : 19th August 2002
// Creator         : Thorn Aitch
// Description     : Initial Design				  
//------------------------------------------------------
// Revision Number : 2
// Date of Change  : 30th April 2003
// Modifier        : Thorn Aitch
// Description     : Release Version 1.0
//======================================================
// Copyright (C) 2002-2003, Thorn Aitch
//
// Designs can be altered while keeping list of
// modifications "the same as in GNU" No money can
// be earned by selling the designs themselves, but
// anyone can get money by selling the implementation
// of the design, such as ICs based on some cores, 
// boards based on some schematics or Layouts, and
// even GUI interfaces to text mode drivers.
// "The same as GPL SW" Any update to the design
// should be documented and returned to the design. 
// Any derivative work based on the IP should be free
// under OpenIP License. Derivative work means any
// update, change or improvement on the design. 
// Any work based on the design can be either made
// free under OpenIP license or protected by any other
// license. Work based on the design means any work uses
// the OpenIP Licensed core as a building black without
// changing anything on it with any other blocks to
// produce larger design.  There is NO WARRANTY on the
// functionality or performance of the design on the
// real hardware implementation.
// On the other hand, the SuperH-2 ISA (Instruction Set
// Architecture) executed by Aquarius is rigidly
// the property of Renesas Corp. Then you have all 
// responsibility to judge if there are not any 
// infringements to Renesas's rights regarding your 
// Aquarius adoption into your design. 
// By adopting Aquarius, the user assumes all 
// responsibility for its use.
// This project may cause any damages around you, for 
// example, loss of properties, data, money, profits,
// life, or business etc. By adopting this source, 
// the user assumes all responsibility for its use.
//======================================================

`include "timescale.v"
`include "defines.v"

//****************************
// Multiply Unit Specification
//****************************
// This unit handles following multiplier related operations. 
//   DMULS.L
//   DMULU.L
//   MAC.L
//   MAC.W
//   MUL.L
//   MULS.W
//   MULU.W
// Physical multiplier size is 32bit*16bit->48bit.
// Then, 32x32 operations are executed in 2 cycles.

//*************************************************
// Module Definition
//*************************************************
module mult(
    // system signal
    CLK, RST,
    // command
    SLOT, MULCOM1, MULCOM2, MAC_S, WRMACH, WRMACL,
    // input data
    MACIN1, MACIN2,
    // output data
    MACH, MACL,
    // busy signal
    MAC_BUSY
    );

//-------------------
// Module I/O Signals
//-------------------
    input  CLK;           // clock
    input  RST;           // reset
    input  SLOT;          // cpu pipe slot
    input  MULCOM1;       // M1 latch command
    input  [7:0] MULCOM2; // M2 latch and mult engage command
                          // NOP      0 0000000 00
                          // DMULS.L  1 0111101 BD
                          // DMULU.L  1 0110101 B5
                          // MAC.L    1 0001111 8F
                          // MAC.W    1 1001111 CF
                          // MUL.L    1 0000111 87
                          // MULS.W   1 0101111 AF
                          // MULU.W   1 0101110 AE
    input  MAC_S;         // S-bit in SR
    input  WRMACH, WRMACL;// write MACH and MACL directly from data path
    input  [31:0] MACIN1; // input data 1
    input  [31:0] MACIN2; // input data 2
    output [31:0] MACH;   // output MACH
    output [31:0] MACL;   // output MACL
    output MAC_BUSY;      // busy signal (negate at final operation state)

//-----------------
// Internal Signals
//-----------------
    reg  [31:0] M1;   // input data1 latch
    reg  [31:0] M2;   // input data2 latch
    reg  [31:0] MB;   // input data 2 buffer to implement continuous MAC.L instruction
    reg  SELA;        // 0:A=M1, 1:A=MB
    reg  [31:0] A;    // A=M1 or A=MB (selected by SELA)
    reg  [31:0] B;    // B=M2
    reg  SHIFT;       // use lower(0)/upper(1) 16bit of B; use unshifted(0)/16bit-shifted(1) PM
    reg  SIGN;        // 0:unsigned, 1:signed (multipier operation)
    reg  SIZE;        // if 32*32 then 1, 16*16 then 0
    reg  [30:0] AH;   // lower 31bit of A
    reg  [15:0] BH;   // upper 16bit of B(0) or lower 16bit of B(1)
    reg  [46:0] ABH;  // output of Multiplier(31x16) (=A * BH) (calculated as unsigned)
    reg  [32:0] ABH2; // modified ABH
    reg  [31:0] P2;   // if signed32*32, ~SHIFT&A[31]&B[31:0], if signed16*16, ~SHIFT&A[31]&{B[15:0]:16'h0000}
    reg  [31:0] P3;   // if signed32*32, ~SHIFT&B[31]&A[31:0], if signed16*16, ~SHIFT&B[15]&A[31:0]
    reg  [31:0] P23;  // P2 + P3
    reg  [32:0] P23S; // if SIGN, ~P23, else P23
    reg  [47:0] PM;   // multiplier output (partial result) with sign
    reg  [63:0] C;    // one of the adder inputs
    reg  ZH;          // if final result is 16bit, adder input from MACH is forced to zero 
    reg  [1:0] ADD;   // 00:ADD, 10:ADDS48, 11:ADDS32 (adder functions regarding saturation)
    reg  [63:0] ADDRESULT;  // pure adder result
    reg  [63:0] ADDRESULT2; // saturated result
    reg  SAT;         // whether saturation has occured or not (to or 0001 to MACH)
    reg  LATMACH;     // latch signal of MACH by state machine
    reg  LATMACL;     // latch signal of MACL by state machine
    reg  [31:0] MACH; // actual MACH
    reg  [31:0] MACL; // actual MACL
    reg  MAC_BUSY;    // busy signal (negate at final operation state)
    reg  [3:0] STATE;     // control state
    reg  [3:0] NEXTSTATE; // next state
    reg  MAC_DISPATCH;    // mult can accept next new operation

//-----------------
// Division Signals (DSP-based Newton-Raphson reciprocal)
//-----------------
    reg  [32:0] div_remainder;
    reg  [31:0] div_quotient;
    reg  [5:0]  div_counter;   // 25=setup, 24..1=NR+verify, 0=done
    reg         div_signed_op; // 0=DIVU, 1=DIVS
    reg         div_neg_q;     // negate quotient at end
    reg         div_neg_r;     // negate remainder at end
    wire        div_write;     // write MACH/MACL with division result

    reg  [31:0] div_d_norm;    // normalized divisor (MSB=1)
    reg  [31:0] div_recip;     // reciprocal estimate (Q2.30)
    reg  [31:0] div_dividend;  // dividend (absolute value)
    reg  [31:0] div_original_d;// original divisor (absolute value)
    reg  [4:0]  div_shift;     // CLZ result
    reg         div_load_m1;   // load M1 from division logic
    reg         div_load_m2;   // load M2 from division logic
    reg  [31:0] div_m1_val;    // value to load into M1
    reg  [31:0] div_m2_val;    // value to load into M2
    reg         ZL;            // zero MACL input to adder (for chained multiplies)

    // Reciprocal LUT ROM - 256 entries indexed by d_norm[30:23]
    // LUT[i] = round(2^39 / (256 + i)), gives 1/d in Q2.30
    reg  [31:0] recip_rom [0:255];

    // CLZ (count leading zeros) - combinational
    function [4:0] clz32;
        input [31:0] val;
        reg [4:0] n;
        begin
            n = 5'd0;
            if (val[31:16] == 16'd0) begin n = n + 5'd16; val = val << 16; end
            if (val[31:24] == 8'd0)  begin n = n + 5'd8;  val = val << 8;  end
            if (val[31:28] == 4'd0)  begin n = n + 5'd4;  val = val << 4;  end
            if (val[31:30] == 2'd0)  begin n = n + 5'd2;  val = val << 2;  end
            if (val[31]    == 1'd0)  begin n = n + 5'd1;  end
            clz32 = n;
        end
    endfunction

    initial begin
        recip_rom[0] = 32'd2147483648;
        recip_rom[1] = 32'd2139127680;
        recip_rom[2] = 32'd2130836488;
        recip_rom[3] = 32'd2122609320;
        recip_rom[4] = 32'd2114445438;
        recip_rom[5] = 32'd2106344115;
        recip_rom[6] = 32'd2098304633;
        recip_rom[7] = 32'd2090326289;
        recip_rom[8] = 32'd2082408386;
        recip_rom[9] = 32'd2074550241;
        recip_rom[10] = 32'd2066751180;
        recip_rom[11] = 32'd2059010539;
        recip_rom[12] = 32'd2051327664;
        recip_rom[13] = 32'd2043701910;
        recip_rom[14] = 32'd2036132644;
        recip_rom[15] = 32'd2028619239;
        recip_rom[16] = 32'd2021161080;
        recip_rom[17] = 32'd2013757560;
        recip_rom[18] = 32'd2006408080;
        recip_rom[19] = 32'd1999112051;
        recip_rom[20] = 32'd1991868891;
        recip_rom[21] = 32'd1984678028;
        recip_rom[22] = 32'd1977538899;
        recip_rom[23] = 32'd1970450946;
        recip_rom[24] = 32'd1963413621;
        recip_rom[25] = 32'd1956426384;
        recip_rom[26] = 32'd1949488702;
        recip_rom[27] = 32'd1942600049;
        recip_rom[28] = 32'd1935759908;
        recip_rom[29] = 32'd1928967768;
        recip_rom[30] = 32'd1922223125;
        recip_rom[31] = 32'd1915525484;
        recip_rom[32] = 32'd1908874354;
        recip_rom[33] = 32'd1902269252;
        recip_rom[34] = 32'd1895709703;
        recip_rom[35] = 32'd1889195237;
        recip_rom[36] = 32'd1882725390;
        recip_rom[37] = 32'd1876299706;
        recip_rom[38] = 32'd1869917734;
        recip_rom[39] = 32'd1863579030;
        recip_rom[40] = 32'd1857283155;
        recip_rom[41] = 32'd1851029676;
        recip_rom[42] = 32'd1844818167;
        recip_rom[43] = 32'd1838648207;
        recip_rom[44] = 32'd1832519380;
        recip_rom[45] = 32'd1826431275;
        recip_rom[46] = 32'd1820383490;
        recip_rom[47] = 32'd1814375623;
        recip_rom[48] = 32'd1808407283;
        recip_rom[49] = 32'd1802478078;
        recip_rom[50] = 32'd1796587627;
        recip_rom[51] = 32'd1790735550;
        recip_rom[52] = 32'd1784921474;
        recip_rom[53] = 32'd1779145029;
        recip_rom[54] = 32'd1773405851;
        recip_rom[55] = 32'd1767703582;
        recip_rom[56] = 32'd1762037865;
        recip_rom[57] = 32'd1756408351;
        recip_rom[58] = 32'd1750814694;
        recip_rom[59] = 32'd1745256552;
        recip_rom[60] = 32'd1739733588;
        recip_rom[61] = 32'd1734245470;
        recip_rom[62] = 32'd1728791868;
        recip_rom[63] = 32'd1723372457;
        recip_rom[64] = 32'd1717986918;
        recip_rom[65] = 32'd1712634934;
        recip_rom[66] = 32'd1707316192;
        recip_rom[67] = 32'd1702030384;
        recip_rom[68] = 32'd1696777203;
        recip_rom[69] = 32'd1691556350;
        recip_rom[70] = 32'd1686367527;
        recip_rom[71] = 32'd1681210440;
        recip_rom[72] = 32'd1676084798;
        recip_rom[73] = 32'd1670990316;
        recip_rom[74] = 32'd1665926709;
        recip_rom[75] = 32'd1660893698;
        recip_rom[76] = 32'd1655891006;
        recip_rom[77] = 32'd1650918360;
        recip_rom[78] = 32'd1645975491;
        recip_rom[79] = 32'd1641062131;
        recip_rom[80] = 32'd1636178018;
        recip_rom[81] = 32'd1631322890;
        recip_rom[82] = 32'd1626496491;
        recip_rom[83] = 32'd1621698566;
        recip_rom[84] = 32'd1616928864;
        recip_rom[85] = 32'd1612187138;
        recip_rom[86] = 32'd1607473140;
        recip_rom[87] = 32'd1602786629;
        recip_rom[88] = 32'd1598127366;
        recip_rom[89] = 32'd1593495113;
        recip_rom[90] = 32'd1588889636;
        recip_rom[91] = 32'd1584310703;
        recip_rom[92] = 32'd1579758086;
        recip_rom[93] = 32'd1575231558;
        recip_rom[94] = 32'd1570730897;
        recip_rom[95] = 32'd1566255880;
        recip_rom[96] = 32'd1561806289;
        recip_rom[97] = 32'd1557381909;
        recip_rom[98] = 32'd1552982525;
        recip_rom[99] = 32'd1548607926;
        recip_rom[100] = 32'd1544257904;
        recip_rom[101] = 32'd1539932252;
        recip_rom[102] = 32'd1535630765;
        recip_rom[103] = 32'd1531353242;
        recip_rom[104] = 32'd1527099483;
        recip_rom[105] = 32'd1522869291;
        recip_rom[106] = 32'd1518662469;
        recip_rom[107] = 32'd1514478826;
        recip_rom[108] = 32'd1510318170;
        recip_rom[109] = 32'd1506180312;
        recip_rom[110] = 32'd1502065065;
        recip_rom[111] = 32'd1497972245;
        recip_rom[112] = 32'd1493901668;
        recip_rom[113] = 32'd1489853154;
        recip_rom[114] = 32'd1485826524;
        recip_rom[115] = 32'd1481821601;
        recip_rom[116] = 32'd1477838209;
        recip_rom[117] = 32'd1473876177;
        recip_rom[118] = 32'd1469935331;
        recip_rom[119] = 32'd1466015504;
        recip_rom[120] = 32'd1462116526;
        recip_rom[121] = 32'd1458238233;
        recip_rom[122] = 32'd1454380460;
        recip_rom[123] = 32'd1450543045;
        recip_rom[124] = 32'd1446725826;
        recip_rom[125] = 32'd1442928645;
        recip_rom[126] = 32'd1439151345;
        recip_rom[127] = 32'd1435393770;
        recip_rom[128] = 32'd1431655765;
        recip_rom[129] = 32'd1427937179;
        recip_rom[130] = 32'd1424237860;
        recip_rom[131] = 32'd1420557659;
        recip_rom[132] = 32'd1416896428;
        recip_rom[133] = 32'd1413254020;
        recip_rom[134] = 32'd1409630292;
        recip_rom[135] = 32'd1406025099;
        recip_rom[136] = 32'd1402438301;
        recip_rom[137] = 32'd1398869755;
        recip_rom[138] = 32'd1395319325;
        recip_rom[139] = 32'd1391786871;
        recip_rom[140] = 32'd1388272257;
        recip_rom[141] = 32'd1384775350;
        recip_rom[142] = 32'd1381296015;
        recip_rom[143] = 32'd1377834120;
        recip_rom[144] = 32'd1374389535;
        recip_rom[145] = 32'd1370962129;
        recip_rom[146] = 32'd1367551776;
        recip_rom[147] = 32'd1364158347;
        recip_rom[148] = 32'd1360781718;
        recip_rom[149] = 32'd1357421763;
        recip_rom[150] = 32'd1354078359;
        recip_rom[151] = 32'd1350751385;
        recip_rom[152] = 32'd1347440720;
        recip_rom[153] = 32'd1344146244;
        recip_rom[154] = 32'd1340867839;
        recip_rom[155] = 32'd1337605387;
        recip_rom[156] = 32'd1334358772;
        recip_rom[157] = 32'd1331127879;
        recip_rom[158] = 32'd1327912594;
        recip_rom[159] = 32'd1324712805;
        recip_rom[160] = 32'd1321528399;
        recip_rom[161] = 32'd1318359266;
        recip_rom[162] = 32'd1315205296;
        recip_rom[163] = 32'd1312066382;
        recip_rom[164] = 32'd1308942414;
        recip_rom[165] = 32'd1305833287;
        recip_rom[166] = 32'd1302738895;
        recip_rom[167] = 32'd1299659134;
        recip_rom[168] = 32'd1296593901;
        recip_rom[169] = 32'd1293543092;
        recip_rom[170] = 32'd1290506605;
        recip_rom[171] = 32'd1287484342;
        recip_rom[172] = 32'd1284476201;
        recip_rom[173] = 32'd1281482084;
        recip_rom[174] = 32'd1278501893;
        recip_rom[175] = 32'd1275535531;
        recip_rom[176] = 32'd1272582903;
        recip_rom[177] = 32'd1269643912;
        recip_rom[178] = 32'd1266718465;
        recip_rom[179] = 32'd1263806469;
        recip_rom[180] = 32'd1260907830;
        recip_rom[181] = 32'd1258022457;
        recip_rom[182] = 32'd1255150260;
        recip_rom[183] = 32'd1252291148;
        recip_rom[184] = 32'd1249445032;
        recip_rom[185] = 32'd1246611823;
        recip_rom[186] = 32'd1243791434;
        recip_rom[187] = 32'd1240983779;
        recip_rom[188] = 32'd1238188770;
        recip_rom[189] = 32'd1235406323;
        recip_rom[190] = 32'd1232636354;
        recip_rom[191] = 32'd1229878778;
        recip_rom[192] = 32'd1227133513;
        recip_rom[193] = 32'd1224400476;
        recip_rom[194] = 32'd1221679586;
        recip_rom[195] = 32'd1218970763;
        recip_rom[196] = 32'd1216273925;
        recip_rom[197] = 32'd1213588993;
        recip_rom[198] = 32'd1210915890;
        recip_rom[199] = 32'd1208254536;
        recip_rom[200] = 32'd1205604855;
        recip_rom[201] = 32'd1202966770;
        recip_rom[202] = 32'd1200340205;
        recip_rom[203] = 32'd1197725085;
        recip_rom[204] = 32'd1195121335;
        recip_rom[205] = 32'd1192528880;
        recip_rom[206] = 32'd1189947649;
        recip_rom[207] = 32'd1187377568;
        recip_rom[208] = 32'd1184818564;
        recip_rom[209] = 32'd1182270568;
        recip_rom[210] = 32'd1179733506;
        recip_rom[211] = 32'd1177207310;
        recip_rom[212] = 32'd1174691910;
        recip_rom[213] = 32'd1172187236;
        recip_rom[214] = 32'd1169693221;
        recip_rom[215] = 32'd1167209796;
        recip_rom[216] = 32'd1164736894;
        recip_rom[217] = 32'd1162274448;
        recip_rom[218] = 32'd1159822392;
        recip_rom[219] = 32'd1157380661;
        recip_rom[220] = 32'd1154949189;
        recip_rom[221] = 32'd1152527912;
        recip_rom[222] = 32'd1150116765;
        recip_rom[223] = 32'd1147715687;
        recip_rom[224] = 32'd1145324612;
        recip_rom[225] = 32'd1142943480;
        recip_rom[226] = 32'd1140572228;
        recip_rom[227] = 32'd1138210795;
        recip_rom[228] = 32'd1135859120;
        recip_rom[229] = 32'd1133517142;
        recip_rom[230] = 32'd1131184802;
        recip_rom[231] = 32'd1128862041;
        recip_rom[232] = 32'd1126548799;
        recip_rom[233] = 32'd1124245018;
        recip_rom[234] = 32'd1121950641;
        recip_rom[235] = 32'd1119665609;
        recip_rom[236] = 32'd1117389866;
        recip_rom[237] = 32'd1115123355;
        recip_rom[238] = 32'd1112866020;
        recip_rom[239] = 32'd1110617806;
        recip_rom[240] = 32'd1108378657;
        recip_rom[241] = 32'd1106148519;
        recip_rom[242] = 32'd1103927337;
        recip_rom[243] = 32'd1101715058;
        recip_rom[244] = 32'd1099511628;
        recip_rom[245] = 32'd1097316994;
        recip_rom[246] = 32'd1095131103;
        recip_rom[247] = 32'd1092953904;
        recip_rom[248] = 32'd1090785345;
        recip_rom[249] = 32'd1088625374;
        recip_rom[250] = 32'd1086473940;
        recip_rom[251] = 32'd1084330994;
        recip_rom[252] = 32'd1082196484;
        recip_rom[253] = 32'd1080070361;
        recip_rom[254] = 32'd1077952576;
        recip_rom[255] = 32'd1075843080;
    end

//-------------------
// Main State Machine
//-------------------
    // state machine F/F
    always @(posedge CLK or posedge RST)
    begin
        if (RST == 1'b1)
            STATE <= `NOP;
        else if (MAC_DISPATCH & SLOT)
            begin                
                case(MULCOM2)
                    8'h00   : STATE <= `NOP;
                    8'hBD   : STATE <= `DMULSL;
                    8'hB5   : STATE <= `DMULUL;
                    8'h8F   : if (MAC_S == 1'b0)
                                  STATE <= `MACL0;
                              else
                                  STATE <= `MACLS;
                    8'hCF   : if (MAC_S == 1'b0)
                                  STATE <= `MACW;
                              else
                                  STATE <= `MACWS;
                    8'h87   : STATE <= `MULL;
                    8'hAF   : STATE <= `MULSW;
                    8'hAE   : STATE <= `MULUW;
                    8'hB1   : STATE <= `DIVOP; // DIVU (3nm1)
                    8'hB9   : STATE <= `DIVOP; // DIVS (3nm9)
                    default : STATE <= `NOP;
                endcase
            end
        else if (MAC_DISPATCH & ~SLOT)
            STATE <= `NOP;
        else if (~MAC_DISPATCH)
            STATE <= NEXTSTATE;
    end

//------------------
// State Transistion
//------------------
// NOP     : A=M1, BH=LowerB, unsign MULT, C=PM,                > NOP
//
// DMULSL  : A=M1, BH=LowerB, signed MULT, C=PM,       MAC<=ADD > DMULSL2
// DMULSL2 : A=M1, BH=upperB, signed MULT, C=(PM<<16), MAC<=ADD > NOP
//
// DMULUL  : A=M1, BH=LowerB, unsign MULT, C=PM,       MAC<=ADD > DMULUL2
// DMULUL2 : A=M1, BH=upperB, unsign MULT, C=(PM<<16), MAC<=ADD > NOP
//
// MACL0   : A=MB, BH=LowerB, signed MULT, C=PM,       MAC<=ADD > MACL2
// MACL2   : A=MB, BH=upperB, signed MULT, C=(PM<<16), MAC<=ADD > NOP

// MACLS   : A=MB, BH=LowerB, signed MULT, C=PM,       MAC<=ADDS48 > MACL2
// MACLS2  : A=MB, BH=upperB, signed MULT, C=(PM<<16), MAC<=ADDS48 > NOP

// MACW    : A=M1, BH=LowerB, signed MULT, C=PM,       MAC<=ADD > NOP

// MACWS   : A=M1, BH=LowerB, signed MULT, C=PM,       MACL<=ADDS32 > NOP
//                                                               if saturate, MACH|=0001

// MULL    : A=M1, BH=LowerB, signed MULT, C=PM,       MACL<=ADD > MULL2
// MULL2   : A=M1, BH=upperB, signed MULT, C=(PM<<16), MACL<=ADD > NOP

// MULSW   : A=M1, BH=LowerB, signed MULT, C=PM,       MACL<=ADD > NOP

// MULUW   : A=M1, BH=LowerB, unsign MULT, C=PM,       MACL<=ADD > NOP

    always @(STATE or SLOT or MULCOM2 or MAC_S or div_counter)
    begin
        ZL <= 1'b0;
        case (STATE)
            `NOP    :begin
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_000_00_000;
                      MAC_BUSY <= 1'b0;
                      MAC_DISPATCH <= 1'b1;
                      NEXTSTATE <= `NOP;
                     end
            `DMULSL :begin
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_011_00_110;
                      MAC_BUSY <= 1'b1;
                      MAC_DISPATCH <= 1'b0;
                      NEXTSTATE <= `DMULSL2;
                     end
            `DMULSL2:begin
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_111_00_110;
                      MAC_BUSY <= 1'b0;
                      MAC_DISPATCH <= 1'b1;
                      NEXTSTATE <= `NOP;
                     end
            `DMULUL :begin
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_001_00_110;
                      MAC_BUSY <= 1'b1;
                      MAC_DISPATCH <= 1'b0;
                      NEXTSTATE <= `DMULUL2;
                     end
            `DMULUL2:begin
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_101_00_110;
                      MAC_BUSY <= 1'b0;    
                      MAC_DISPATCH <= 1'b1;
                      NEXTSTATE <= `NOP;
                     end
            `MACL0  :begin
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b1_011_00_110;
                      MAC_BUSY <= 1'b1;     
                      MAC_DISPATCH <= 1'b0;
                      NEXTSTATE <= `MACL2;
                     end
            `MACL2  :begin
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b1_111_00_110;
                      MAC_BUSY <= 1'b0;    
                      MAC_DISPATCH <= 1'b1;
                      NEXTSTATE <= `NOP;
                     end
            `MACLS  :begin
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b1_011_10_110;
                      MAC_BUSY <= 1'b1;     
                      MAC_DISPATCH <= 1'b0;
                      NEXTSTATE <= `MACLS2;
                     end
            `MACLS2 :begin
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b1_111_10_110;
                      MAC_BUSY <= 1'b0;    
                      MAC_DISPATCH <= 1'b1;
                      NEXTSTATE <= `NOP;
                     end
            `MACW   :begin
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_010_00_110;
                      MAC_BUSY <= 1'b1;    
                      MAC_DISPATCH <= 1'b1;
                      NEXTSTATE <= `NOP;
                     end
            `MACWS  :begin
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_010_11_011;
                      MAC_BUSY <= 1'b0;     
                      MAC_DISPATCH <= 1'b1;
                      NEXTSTATE <= `NOP;
                     end
            `MULL   :begin
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_011_00_011;
                      MAC_BUSY <= 1'b1;     
                      MAC_DISPATCH <= 1'b0;
                      NEXTSTATE <= `MULL2;
                     end
            `MULL2  :begin
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_111_00_011;
                      MAC_BUSY <= 1'b0;    
                      MAC_DISPATCH <= 1'b1;
                      NEXTSTATE <= `NOP;
                     end
            `MULSW  :begin
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_010_00_011;
                      MAC_BUSY <= 1'b0;     
                      MAC_DISPATCH <= 1'b1;
                      NEXTSTATE <= `NOP;
                     end
            `MULUW  :begin
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_000_00_011;
                      MAC_BUSY <= 1'b0;     
                      MAC_DISPATCH <= 1'b1;
                      NEXTSTATE <= `NOP;
                     end
            `DIVOP  :begin
                      // DSP multiply control based on div_counter
                      case (div_counter)
                          // Multiply step 1 (unsigned 31x16 lower half, fresh)
                          6'd23, 6'd19, 6'd15, 6'd11, 6'd7, 6'd3: begin
                              {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_001_00_111;
                              ZL <= 1'b1;
                          end
                          // Multiply step 2 (upper half, accumulate)
                          6'd22, 6'd18, 6'd14, 6'd10, 6'd6, 6'd2: begin
                              {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_101_00_110;
                          end
                          // All other cycles: no multiply
                          default: begin
                              {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_000_00_000;
                          end
                      endcase
                      if (div_counter == 6'd0) begin
                          MAC_BUSY <= 1'b0;
                          MAC_DISPATCH <= 1'b1;
                          NEXTSTATE <= `NOP;
                      end else begin
                          MAC_BUSY <= 1'b1;
                          MAC_DISPATCH <= 1'b0;
                          NEXTSTATE <= `DIVOP;
                      end
                     end
            default : begin
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_000_00_000;
                      MAC_BUSY <= 1'b0;
                      MAC_DISPATCH <= 1'b1;
                      NEXTSTATE <= `NOP;
                     end
        endcase
    end

//-----------------------------------------
// Data Path
//-----------------------------------------

//---
// M1
//---
    always @(posedge CLK)
    begin
        if (SLOT & MULCOM1)
	   begin
            M1 <= MACIN1;
        end
        else if (div_load_m1)
        begin
            M1 <= div_m1_val;
        end
    end

//-------
// M2
// B (=M2)
//-------
    always @(posedge CLK)
    begin
        if (SLOT & MULCOM2[7])
	   begin
            M2 <= MACIN2;
        end
        else if (div_load_m2)
        begin
            M2 <= div_m2_val;
        end
    end

    always @(M2) B <= M2;

//---
// MB
//---
// if SLOT and MULCOM2=MACL, do latch M1.
    always @(posedge CLK)
    begin
        if (SLOT & MULCOM2[7] & (MULCOM2[5:0] == 6'b001111))
	   begin
            MB <= M1;
        end
    end

//--------
//Select A
//--------
    always @(SELA or M1 or MB)
    begin
        if (SELA)
            A <= MB;
         else
            A <= M1;
    end

//---------------------------------------
// lower 31bit of A	(input to Multiplier)
//---------------------------------------
    always @(A or SIZE)
    begin
	   if (SIZE == 1'b0)
	       AH <= {16'h0000,A[14:0]};
	   else
	       AH <= A[30:0];
    end

//---------------------------------------
// upper/lower of B (input to Multiplier)
//---------------------------------------
    always @(B or SHIFT or SIZE)
    begin
        if (SIZE == 1'b0)
		  BH <= {1'b0,B[14:0]};
        else if (SHIFT == 1'b0)
            BH <= B[15:0];
        else
            BH <= {1'b0,B[30:16]};
    end

//-----------
// Multiplier
//----------
    always @(AH or BH)
    begin
        ABH[46:0] <= AH[30:0] * BH[15:0]; // 31bit * 16bit -> 47bit
    end

//---
// PM
//---
    always @(SHIFT or SIZE or A or B)
    begin
        if (SHIFT)
	       begin
			 P2 <= {1'b0, (A[31])? B[30:0]:31'h00000000};
			 P3 <= {1'b0, (B[31])? A[30:0]:31'h00000000};
		  end
	   else if(~SIZE)
	       begin
			 P2 <= {17'h00000, (A[15])? B[14:0]:15'h0000};
			 P3 <= {17'h00000, (B[15])? A[14:0]:15'h0000};
		  end
	   else
	       begin
			 P2 <= 32'h00000000;
			 P3 <= 32'h00000000;
		  end
    end

    always @(P2 or P3)
    begin
        P23 <= P2 + P3;
    end

    always @(ABH or SHIFT or SIZE or A or B)
    begin
        if (SIZE == 1'b0)
	       ABH2 <= {17'h00000,(A[15] & B[15]),ABH[29:15]};
        else if (SHIFT == 1'b0)
            ABH2 <= {1'b0, ABH[46:15]};
	   else
	       ABH2 <= {1'b0,(A[31] & B[31]),ABH[45:15]};
    end

    always @(P23 or SIGN)
    begin
        if (SIGN == 1'b0)
	       P23S <= {1'b0, P23};
	   else
	       P23S <= {1'b1,~P23};
    end

    always @(P23S or ABH or ABH2 or SIGN)
    begin
	   PM[47:15] <= ABH2[32:0] + P23S + SIGN;
	   PM[14: 0] <= ABH[14: 0];
    end

//---------
// Select C
//---------
    always @(PM or SHIFT or SIZE)
    begin
        if (SHIFT == 1'b0)
            if (~SIZE & PM[47])
                C <= {16'hffff, PM};
            else 
                C <= {16'h0000, PM};
        else
            C <= {PM, 16'h0000};
    end

//-------------------------------------
// 64bit ADDER with Satulating function
//-------------------------------------
//
// Essential of Saturate Operation [ MAC[64] + C[64] ]
//
//   +S     |             Left plane shows value of MAC.
//     \    |             "+S" is + side saturate value (ex.00007FFF).
//      \  <P>            "-S" is - side saturate value (ex.FFFF8000).
//  <P'> \  |             Addition of plus C rotates MAC value counterclockwise.
//        \ |             Addition of minus C rotates MAC value clockwise.                                                     
// 7F..    \|     00..
// -------------------    Region<M'> : MAC=80000000~FFFF7FFF
// 80..    /|     FF..    Region<M > : MAC=FFFF8000~FFFFFFFF
//        / |             Region<P > : MAC=00000000~00007FFF
//  <M'> /  |             Region<P'> : MAC=00008000~7FFFFFFF
//      /  <M>  
//     /    |             Note that initial MAC value may be in any region.
//   -S     |             And value C may also be 00000000~FFFFFFFF.
//                                            
// ===========================================
// Initial_MAC  Rotation(C)  MAC+C  Result_MAC
// ===========================================
//     P            +          P     OK      
//     P            +          P'    00007FFF
//     P            +          M'    00007FFF
//     P            +          M     00007FFF 
// -------------------------------------------
//     P'           +          P     00007FFF Impossible      
//     P'           +          P'    00007FFF
//     P'           +          M'    00007FFF
//     P'           +          M     00007FFF 
// -------------------------------------------
//     M'           +          P     OK      
//     M'           +          P'    00007FFF Imposible
//     M'           +          M'    FFFF8000
//     M'           +          M     OK       
// -------------------------------------------
//     M            +          P     OK      
//     M            +          P'    00007FFF
//     M            +          M'    00007FFF Impossible
//     M            +          M     OK       
// ===========================================
//     P            -          P     OK      
//     P            -          P'    FFFF8000 Impossible
//     P            -          M'    FFFF8000
//     P            -          M     OK 
// -------------------------------------------
//     P'           -          P     OK      
//     P'           -          P'    00007FFF
//     P'           -          M'    FFFF8000 Impossible
//     P'           -          M     OK 
// -------------------------------------------
//     M'           -          P     FFFF8000      
//     M'           -          P'    FFFF8000
//     M'           -          M'    FFFF8000
//     M'           -          M     FFFF8000 Impossible       
// -------------------------------------------
//     M            -          P     FFFF8000      
//     M            -          P'    FFFF8000
//     M            -          M'    FFFF8000
//     M            -          M     OK       
// ===========================================

// Again, Compactly...                               
// ===========================================
// Initial_MAC  Rotation(C)  MAC+C  Result_MAC
// ===========================================
//     P /M        +/-        P /M  OK      
//     P /M        +/-        P'/M' 00007FFF/FFFF8000
//     P /M        +/-        M'/P' 00007FFF/FFFF8000
//     P /M        +/-        M /P  00007FFF/FFFF8000
// -------------------------------------------
//     P'/M'       +/-        P /M  Impossible = Don't care      
//     P'/M'       +/-        P'/M' 00007FFF/FFFF8000
//     P'/M'       +/-        M'/P' 00007FFF/FFFF8000
//     P'/M'       +/-        M /P  00007FFF/FFFF8000
// -------------------------------------------
//     M'/P'       +/-        P /M  OK      
//     M'/P'       +/-        P'/M' Impossible = Don't care
//     M'/P'       +/-        M'/P' FFFF8000/00007FFF <--caution !
//     M'/P'       +/-        M /P  OK       
// -------------------------------------------
//     M /P        +/-        P /M  OK      
//     M /P        +/-        P'/M' 00007FFF/FFFF8000
//     M /P        +/-        M'/P' Impossible = Don't care
//     M /P        +/-        M /P  OK       
// ===========================================

// Again, Much Compactly...                                  
// ===========================================
// Initial_MAC  Rotation(C)  MAC+C  Result_MAC
// ===========================================
//     P /M        +/-        P /M  OK      
//     P /M        +/-        P'/M' 00007FFF/FFFF8000
//     P /M        +/-        - /+  00007FFF/FFFF8000
// -------------------------------------------
//     P'/M'       +/-        P /M  Impossible = Don't care      
//     P'/M'       +/-        P'/M' 00007FFF/FFFF8000
//     P'/M'       +/-        - /+  00007FFF/FFFF8000
// -------------------------------------------
//     M'/P'       +/-        P /M  OK 
//     - /+        +/-        M'/P' FFFF8000/00007FFF <--caution !
//     M'/P'       +/-        M /P  OK       
// -------------------------------------------
//     M /P        +/-        P /M  OK      
//     - /+        +/-        P'/M' 00007FFF/FFFF8000
//     M /P        +/-        M /P  OK       
// ===========================================

// Again, Much Compactly...                                  
// ===========================================
// Initial_MAC  Rotation(C)  MAC+C  Result_MAC
// ===========================================
//     + /-        +/-        P /M  OK      
//     + /-        +/-        P'/M' 00007FFF/FFFF8000
//     + /-        +/-        - /+  00007FFF/FFFF8000
// -------------------------------------------
//     - /+        +/-        P /M  OK 
//     - /+        +/-        M'/P' FFFF8000/00007FFF <--caution !
//     - /+        +/-        M /P  OK       
//     - /+        +/-        P'/M' 00007FFF/FFFF8000
// ===========================================

// Again, Much Compactly...                                  
// ===========================================
// Initial_MAC  Rotation(C)  MAC+C  Result_MAC
// ===========================================
//     * /*        +/-        P /M  OK      
//     * /*        +/-        P'/M' 00007FFF/FFFF8000
//     + /-        +/-        - /+  00007FFF/FFFF8000
//     - /+        +/-        M'/P' FFFF8000/00007FFF <--caution !
//     - /+        +/-        M /P  OK       
// ===========================================

    always @(C or MACH or MACL or ZH or ZL)
    begin
        ADDRESULT <= C + {((ZH == 1'b0) ? MACH : 32'h00000000),
                          ((ZL == 1'b0) ? MACL : 32'h00000000)};
    end

    reg [1:0] RESULT_REGION48; //00:P, 01:P', 10:M, 11:M'
    reg       RESULT_REGION32; //0:P, 1:M, No P'/M' region in case of 32bit saturation.
    always @(ADDRESULT)
    begin
        RESULT_REGION48[1] <=  ADDRESULT[63];
        RESULT_REGION32    <=  ADDRESULT[31];
        if (ADDRESULT[63] == 1'b0)
          //RESULT_REGION48[0] <= (ADDRESULT[63:47] >= 17'h00001);
		  RESULT_REGION48[0] <= (ADDRESULT[63:47] != 17'h00000);
        else
          //RESULT_REGION48[0] <= (ADDRESULT[63:47] <= 17'hFFFFE);
		  RESULT_REGION48[0] <= (ADDRESULT[63:47] != 17'hFFFFF);
    end

    always @(ADDRESULT or C or MACH or MACL or ADD or RESULT_REGION48 or RESULT_REGION32)
    begin
        case(ADD)
            2'b00   : begin // ADD
                          ADDRESULT2 <= ADDRESULT;
                          SAT <= 1'b0;
                      end
            2'b10   : begin // ADDS48
                          if (~C[63]) // + rotation
                              case ({MACH[31], RESULT_REGION48})
                                  3'b000 : {ADDRESULT2,SAT} <= {ADDRESULT,1'b0};            //+,P
                                  3'b001 : {ADDRESULT2,SAT} <= {64'h00007FFFFFFFFFFF,1'b1}; //+,P'
                                  3'b010 : {ADDRESULT2,SAT} <= {64'h00007FFFFFFFFFFF,1'b1}; //+,M
                                  3'b011 : {ADDRESULT2,SAT} <= {64'h00007FFFFFFFFFFF,1'b1}; //+,M'
                                  3'b100 : {ADDRESULT2,SAT} <= {ADDRESULT,1'b0};            //-,P
                                  3'b101 : {ADDRESULT2,SAT} <= {64'h00007FFFFFFFFFFF,1'b1}; //-,P'
                                  3'b110 : {ADDRESULT2,SAT} <= {ADDRESULT,1'b0};            //-,M
                                  3'b111 : {ADDRESULT2,SAT} <= {64'hFFFF800000000000,1'b1}; //-,M'
                                  default: {ADDRESULT2,SAT} <= {64'hxxxxxxxxxxxxxxxx,1'bx};
                              endcase
                          else        // - rotation
                              case ({MACH[31], RESULT_REGION48})
                                  3'b000 : {ADDRESULT2,SAT} <= {ADDRESULT,1'b0};            //+,P
                                  3'b001 : {ADDRESULT2,SAT} <= {64'h00007FFFFFFFFFFF,1'b1}; //+,P'
                                  3'b010 : {ADDRESULT2,SAT} <= {ADDRESULT,1'b0};            //+,M
                                  3'b011 : {ADDRESULT2,SAT} <= {64'hFFFF800000000000,1'b1}; //+,M'
                                  3'b100 : {ADDRESULT2,SAT} <= {64'hFFFF800000000000,1'b1}; //-,P
                                  3'b101 : {ADDRESULT2,SAT} <= {64'hFFFF800000000000,1'b1}; //-,P'
                                  3'b110 : {ADDRESULT2,SAT} <= {ADDRESULT,1'b0};            //-,M
                                  3'b111 : {ADDRESULT2,SAT} <= {64'hFFFF800000000000,1'b1}; //-,M'
                                  default: {ADDRESULT2,SAT} <= {64'hxxxxxxxxxxxxxxxx,1'bx};
                              endcase
                      end
            2'b11   : begin // ADDS32
                          if (~C[31]) // + rotation
                              case ({MACL[31], RESULT_REGION32})
                                  2'b00  : {ADDRESULT2,SAT} <= {ADDRESULT,1'b0};            //+,P
                                  2'b01  : {ADDRESULT2,SAT} <= {64'h000000007FFFFFFF,1'b1}; //+,M
                                  2'b10  : {ADDRESULT2,SAT} <= {ADDRESULT,1'b0};            //-,P
                                  2'b11  : {ADDRESULT2,SAT} <= {ADDRESULT,1'b0};            //-,M
                                  default: {ADDRESULT2,SAT} <= {64'hxxxxxxxxxxxxxxxx,1'bx};
                              endcase
                          else        // - rotation
                              case ({MACL[31], RESULT_REGION32})
                                  2'b00  : {ADDRESULT2,SAT} <= {ADDRESULT,1'b0};            //+,P
                                  2'b01  : {ADDRESULT2,SAT} <= {ADDRESULT,1'b0};            //+,M
                                  2'b10  : {ADDRESULT2,SAT} <= {64'hFFFFFFFF80000000,1'b1}; //-,P
                                  2'b11  : {ADDRESULT2,SAT} <= {ADDRESULT,1'b0};            //-,M
                                  default: {ADDRESULT2,SAT} <= {64'hxxxxxxxxxxxxxxxx,1'bx};
                              endcase
                      end
            default : begin 
                          ADDRESULT2 <= 64'hxxxxxxxxxxxxxxxx;
                          SAT <= 1'b0;
                      end
        endcase
    end

//-----
// MACH
//-----
// Clear condition of MACH: following command do not clear MACH
                          // MAC.L    1 0001111 8F
                          // MAC.W    1 1001111 CF
                          // MUL.L    1 0000111 87
                          // MULS.W   1 0101111 AF
                          // MULU.W   1 0101110 AE
// Should do saturating operation 
    always @(posedge CLK)
    begin
        if (SLOT & WRMACH)
            MACH <= MACIN1;
        else if (SLOT & MULCOM2[7] 
                 & (MULCOM2[5:0] != 6'b001111) 
                 & (MULCOM2[6:0] != 7'b0000111)
                 & (MULCOM2[6:0] != 7'b0101111) 
                 & (MULCOM2[6:0] != 7'b0101110)
           )
            begin
                MACH <= 32'h00000000;
            end
        else if ((STATE == `MACWS) && (SAT == 1'b1))
            begin
                MACH <= MACH | 32'h00000001;
            end
        else if (LATMACH == 1'b1)
            begin
                MACH <= ADDRESULT2[63:32];
            end
        else if (div_write)
            begin
                MACH <= div_quotient;
            end
    end

//-----
// MACL
//-----
// Clear condition of MACL: following command do not clear MACL
                          // MAC.L    1 0001111 8F
                          // MAC.W    1 1001111 CF
    always @(posedge CLK)
    begin
        if (SLOT & WRMACL)
            MACL <= MACIN2;
        else if (SLOT & MULCOM2[7] & (MULCOM2[5:0] != 6'b001111))
            begin
                MACL <= 32'h00000000;
            end
        else if (LATMACL == 1'b1)
            begin
                MACL <= ADDRESULT2[31:0];
            end
        else if (div_write)
            begin
                MACL <= div_remainder[31:0];
            end
    end

//***************************
// Division Unit (DIVU/DIVS) - DSP Newton-Raphson
//***************************
// Uses existing DSP multiplier for Newton-Raphson reciprocal division.
// Algorithm: LUT(8-bit) -> 2x NR iterations -> multiply -> verify+correct
// DIVU (3nm1): unsigned Rn / Rm -> MACH=quotient, MACL=remainder
// DIVS (3nm9): signed   Rn / Rm -> MACH=quotient, MACL=remainder
//
// div_counter usage:
//   25    : SETUP (CLZ, normalize, LUT, abs, special cases)
//   24    : LOAD (M1=d_norm, M2=r0)
//   23-22 : NR1 multiply (d_norm * r0)
//   21    : NR1 error + LOAD (M1=r0, M2=eps)
//   20    : LOAD settle
//   19-18 : NR1 correction multiply (r0 * eps)
//   17    : Store r1 + LOAD (M1=d_norm, M2=r1)
//   16    : LOAD settle
//   15-14 : NR2 multiply (d_norm * r1)
//   13    : NR2 error + LOAD (M1=r1, M2=eps2)
//   12    : LOAD settle
//   11-10 : NR2 correction multiply (r1 * eps2)
//    9    : Store r2 + LOAD (M1=dividend, M2=r2)
//    8    : LOAD settle
//    7-6  : Quotient multiply (dividend * r2)
//    5    : Extract q + LOAD (M1=q, M2=divisor)
//    4    : LOAD settle
//    3-2  : Verify multiply (q * divisor)
//    1    : Correction + sign -> counter=0
//    0    : DONE, div_write asserts
//
// Total: ~26 cycles (special cases: 2 cycles)

    assign div_write = (STATE == `DIVOP) & (div_counter == 6'd0);

    // Setup combinational wires (CLZ, normalize, LUT lookup)
    wire [31:0] div_setup_abs_d = (div_signed_op && M2[31]) ? (~M2 + 32'd1) : M2;
    wire [4:0]  div_setup_clz   = clz32(div_setup_abs_d);
    wire [31:0] div_setup_norm  = div_setup_abs_d << div_setup_clz;
    wire [31:0] div_setup_recip = recip_rom[div_setup_norm[30:23]];

    // NR correction extraction: (r * eps) >> 30 from MACH:MACL
    wire [31:0] div_nr_new_r = {MACH[29:0], MACL[31:30]};

    // Quotient correction combinational logic (for counter=1)
    wire [31:0] div_raw_rem_w = div_dividend - MACL;
    wire        div_q_ovf_w   = (MACH != 32'd0) || (MACL > div_dividend);
    wire        div_q_unf_w   = !div_q_ovf_w && (div_raw_rem_w >= div_original_d);
    wire [31:0] div_adj_q_w   = div_q_ovf_w ? (div_quotient - 32'd1) :
                                 div_q_unf_w ? (div_quotient + 32'd1) :
                                 div_quotient;
    wire [31:0] div_adj_r_w   = div_q_ovf_w ? (div_raw_rem_w + div_original_d) :
                                 div_q_unf_w ? (div_raw_rem_w - div_original_d) :
                                 div_raw_rem_w;

    // Division data path
    always @(posedge CLK or posedge RST)
    begin
        if (RST) begin
            div_counter    <= 6'd0;
            div_signed_op  <= 1'b0;
            div_remainder  <= 33'd0;
            div_quotient   <= 32'd0;
            div_neg_q      <= 1'b0;
            div_neg_r      <= 1'b0;
            div_d_norm     <= 32'd0;
            div_recip      <= 32'd0;
            div_dividend   <= 32'd0;
            div_original_d <= 32'd0;
            div_shift      <= 5'd0;
            div_load_m1    <= 1'b0;
            div_load_m2    <= 1'b0;
            div_m1_val     <= 32'd0;
            div_m2_val     <= 32'd0;
        end else if (MAC_DISPATCH & SLOT) begin
            case (MULCOM2)
                8'hB1: begin  // DIVU
                    div_counter    <= 6'd25;
                    div_signed_op  <= 1'b0;
                    div_neg_q      <= 1'b0;
                    div_neg_r      <= 1'b0;
                    div_load_m1    <= 1'b0;
                    div_load_m2    <= 1'b0;
                end
                8'hB9: begin  // DIVS
                    div_counter    <= 6'd25;
                    div_signed_op  <= 1'b1;
                    div_neg_q      <= 1'b0;
                    div_neg_r      <= 1'b0;
                    div_load_m1    <= 1'b0;
                    div_load_m2    <= 1'b0;
                end
                default: ;
            endcase
        end else if (STATE == `DIVOP) begin
            div_load_m1 <= 1'b0;  // default: clear load signals
            div_load_m2 <= 1'b0;

            case (div_counter)
                //---------------------------
                // SETUP (counter=25)
                //---------------------------
                6'd25: begin
                    if (M2 == 32'd0) begin
                        // Zero division: q=all-1s, r=dividend
                        div_quotient  <= 32'hFFFFFFFF;
                        div_remainder <= {1'b0, M1};
                        div_counter   <= 6'd0;
                    end else if (div_signed_op && M1 == 32'h80000000 && M2 == 32'hFFFFFFFF) begin
                        // Signed overflow: MIN / -1
                        div_quotient  <= 32'h80000000;
                        div_remainder <= 33'd0;
                        div_counter   <= 6'd0;
                    end else begin
                        // Normal case: setup operands and NR initial estimate
                        if (div_signed_op) begin
                            div_neg_q      <= M1[31] ^ M2[31];
                            div_neg_r      <= M1[31];
                            div_dividend   <= M1[31] ? (~M1 + 32'd1) : M1;
                            div_original_d <= M2[31] ? (~M2 + 32'd1) : M2;
                        end else begin
                            div_dividend   <= M1;
                            div_original_d <= M2;
                        end
                        div_shift  <= div_setup_clz;
                        div_d_norm <= div_setup_norm;
                        div_recip  <= div_setup_recip;
                        // Load M1=d_norm, M2=r0 for NR1 multiply
                        div_load_m1 <= 1'b1;
                        div_load_m2 <= 1'b1;
                        div_m1_val  <= div_setup_norm;
                        div_m2_val  <= div_setup_recip;
                        div_counter <= 6'd24;
                    end
                end

                //---------------------------
                // NR1: error compute (counter=21)
                // MACH = (d_norm * r0) >> 32
                // eps = 0x80000000 - MACH  (= (2 - d*r) in Q2.30)
                // Load M1=r0, M2=eps for correction multiply
                //---------------------------
                6'd21: begin
                    div_load_m1 <= 1'b1;
                    div_load_m2 <= 1'b1;
                    div_m1_val  <= div_recip;
                    div_m2_val  <= 32'h80000000 - MACH;
                    div_counter <= div_counter - 6'd1;
                end

                //---------------------------
                // NR1: store r1 (counter=17)
                // MACH:MACL = r0 * eps
                // r1 = (r0 * eps) >> 30 = {MACH[29:0], MACL[31:30]}
                // Load M1=d_norm, M2=r1 for NR2 multiply
                //---------------------------
                6'd17: begin
                    div_recip   <= div_nr_new_r;
                    div_load_m1 <= 1'b1;
                    div_load_m2 <= 1'b1;
                    div_m1_val  <= div_d_norm;
                    div_m2_val  <= div_nr_new_r;
                    div_counter <= div_counter - 6'd1;
                end

                //---------------------------
                // NR2: error compute (counter=13)
                // MACH = (d_norm * r1) >> 32
                // eps2 = 0x80000000 - MACH
                // Load M1=r1, M2=eps2
                //---------------------------
                6'd13: begin
                    div_load_m1 <= 1'b1;
                    div_load_m2 <= 1'b1;
                    div_m1_val  <= div_recip;
                    div_m2_val  <= 32'h80000000 - MACH;
                    div_counter <= div_counter - 6'd1;
                end

                //---------------------------
                // NR2: store r2 (counter=9)
                // r2 = (r1 * eps2) >> 30
                // Load M1=dividend, M2=r2
                //---------------------------
                6'd9: begin
                    div_recip   <= div_nr_new_r;
                    div_load_m1 <= 1'b1;
                    div_load_m2 <= 1'b1;
                    div_m1_val  <= div_dividend;
                    div_m2_val  <= div_nr_new_r;
                    div_counter <= div_counter - 6'd1;
                end

                //---------------------------
                // Quotient extract (counter=5)
                // MACH:MACL = dividend * r2
                // q = product >> (62 - shift)
                // Load M1=q, M2=divisor for verification
                //---------------------------
                6'd5: begin
                    if (div_shift == 5'd31)
                        div_quotient <= {MACH[30:0], MACL[31]};
                    else
                        div_quotient <= MACH >> (5'd30 - div_shift);
                    // Load for verify multiply
                    div_load_m1 <= 1'b1;
                    div_load_m2 <= 1'b1;
                    if (div_shift == 5'd31)
                        div_m1_val <= {MACH[30:0], MACL[31]};
                    else
                        div_m1_val <= MACH >> (5'd30 - div_shift);
                    div_m2_val  <= div_original_d;
                    div_counter <= div_counter - 6'd1;
                end

                //---------------------------
                // Correction + sign (counter=1)
                // MACH:MACL = q * divisor
                // Adjust q by ±1 if needed, compute remainder
                //---------------------------
                6'd1: begin
                    div_quotient  <= div_neg_q ? (~div_adj_q_w + 32'd1) : div_adj_q_w;
                    div_remainder <= div_neg_r ? {1'b0, ~div_adj_r_w + 32'd1} : {1'b0, div_adj_r_w};
                    div_counter   <= 6'd0;
                end

                //---------------------------
                // Default: decrement counter (LOAD, multiply steps, etc.)
                //---------------------------
                default: begin
                    if (div_counter != 6'd0)
                        div_counter <= div_counter - 6'd1;
                end
            endcase
        end
    end

//======================================================
  endmodule
//======================================================