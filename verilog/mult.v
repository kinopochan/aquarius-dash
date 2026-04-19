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
    reg  SHIFT;       // (kept for 9-bit control word compatibility; unused after 32x32 rewrite)
    reg  SIGN;        // 0:unsigned, 1:signed (multipier operation)
    reg  SIZE;        // if 32*32 then 1, 16*16 then 0
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
// State Transistion (32x32 single-cycle multiplier)
//------------------
// Each multiply/MAC instruction now completes in 1 cycle.
// The 2-cycle states (DMULSL2, DMULUL2, MACL2, MACLS2, MULL2) are kept in
// defines.v for backward compatibility but are unused here.
//
// NOP     : idle.
// DMULSL  : signed 32x32 -> MACH:MACL (was 2 cycles)
// DMULUL  : unsigned 32x32 -> MACH:MACL (was 2 cycles)
// MACL0   : A=MB, signed 32x32 add to MACH:MACL (was 2 cycles)
// MACLS   : same with ADDS48 saturation
// MACW    : A=M1, signed 16x16 add to MACH:MACL
// MACWS   : signed 16x16 add with ADDS32 saturation
// MULL    : signed 32x32 lower 32bit -> MACL (was 2 cycles)
// MULSW   : signed 16x16 -> MACL
// MULUW   : unsigned 16x16 -> MACL

    always @(STATE or SLOT or MULCOM2 or MAC_S or div_counter)
    begin
        case (STATE)
            `NOP    :begin
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_000_00_000;
                      MAC_BUSY <= 1'b0;
                      MAC_DISPATCH <= 1'b1;
                      NEXTSTATE <= `NOP;
                     end
            `DMULSL :begin
                      // signed 32x32, latch both MACH and MACL
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_011_00_110;
                      MAC_BUSY <= 1'b0;
                      MAC_DISPATCH <= 1'b1;
                      NEXTSTATE <= `NOP;
                     end
            `DMULUL :begin
                      // unsigned 32x32, latch both MACH and MACL
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_001_00_110;
                      MAC_BUSY <= 1'b0;
                      MAC_DISPATCH <= 1'b1;
                      NEXTSTATE <= `NOP;
                     end
            `MACL0  :begin
                      // signed 32x32 accumulate (A=MB), ADD=00
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b1_011_00_110;
                      MAC_BUSY <= 1'b0;
                      MAC_DISPATCH <= 1'b1;
                      NEXTSTATE <= `NOP;
                     end
            `MACLS  :begin
                      // signed 32x32 accumulate with ADDS48 saturation
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b1_011_10_110;
                      MAC_BUSY <= 1'b0;
                      MAC_DISPATCH <= 1'b1;
                      NEXTSTATE <= `NOP;
                     end
            `MACW   :begin
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_010_00_110;
                      MAC_BUSY <= 1'b0;
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
                      // signed 32x32, write lower 32bit to MACL (ZH=1 masks MACH)
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_011_00_011;
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
                      // Division does not touch MACH/MACL via the adder path.
                      // div_write directly writes quotient/remainder at counter=0.
                      // SELA=0, SIZE=1, SIGN=0 (dividend/divisor are already
                      // converted to absolute values during SETUP).
                      // {SELA, SHIFT, SIGN, SIZE, ADD[1:0], LATMACH, LATMACL, ZH}
                      //  0     0      0     1     00        0        0        0
                      {SELA,SHIFT,SIGN,SIZE,ADD,LATMACH,LATMACL,ZH}<=9'b0_001_00_000;
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

//-----------
// 32x32 Multiplier (unified, case B-1)
//-----------
// Combinational 32x32 signed/unsigned multiply producing 64-bit result.
// Handles all SIZE/SIGN combinations:
//   SIZE=0, SIGN=0: unsigned 16x16 -> result in lower 32 bits
//   SIZE=0, SIGN=1: signed   16x16 -> result in lower 32 bits (sign-extended to 64)
//   SIZE=1, SIGN=0: unsigned 32x32 -> result in full 64 bits
//   SIZE=1, SIGN=1: signed   32x32 -> result in full 64 bits
//
// Quartus infers a DSP-based 33x33 signed multiplier (Cyclone V: 2 DSP blocks).
    wire [31:0] mul_a_pre = SIZE ? A :
                            (SIGN ? {{16{A[15]}}, A[15:0]} : {16'd0, A[15:0]});
    wire [31:0] mul_b_pre = SIZE ? B :
                            (SIGN ? {{16{B[15]}}, B[15:0]} : {16'd0, B[15:0]});
    wire signed [32:0] mul_a_ext = SIGN ? {mul_a_pre[31], mul_a_pre}
                                        : {1'b0,         mul_a_pre};
    wire signed [32:0] mul_b_ext = SIGN ? {mul_b_pre[31], mul_b_pre}
                                        : {1'b0,         mul_b_pre};
    wire signed [65:0] MULT_RAW  = mul_a_ext * mul_b_ext;
    wire        [63:0] MULT_RESULT = MULT_RAW[63:0];

//---------
// Select C
//---------
// For 16x16 signed, lower 32 bits of the product is the 16x16 result;
// sign-extend to 64 bits so the 48-bit saturation adder sees the right value.
// For 32x32 (signed or unsigned), use the full 64-bit product.
    always @(MULT_RESULT or SIZE or SIGN)
    begin
        if (SIZE == 1'b0 && SIGN == 1'b1)
            C <= {{32{MULT_RESULT[31]}}, MULT_RESULT[31:0]};
        else if (SIZE == 1'b0)
            C <= {32'h00000000, MULT_RESULT[31:0]};
        else
            C <= MULT_RESULT;
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

    always @(C or MACH or MACL or ZH)
    begin
        ADDRESULT <= C + {((ZH == 1'b0) ? MACH : 32'h00000000), MACL};
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
// Division Unit (DIVU/DIVS) - DSP Newton-Raphson (B-1 revision)
//***************************
// With the new combinational 32x32 multiplier, MULT_RESULT tracks
// M1*M2 in a single cycle. Each NR step now takes exactly 1 cycle
// (the previous LOAD-settle cycles are no longer needed).
//
// Because M1/M2 get updated at the NBA region after div_load is asserted,
// the combinational MULT_RESULT = M1*M2 does not reflect the new operands
// until the cycle AFTER the load request. Each multiply therefore spans
// two DIVOP cycles: one LOAD cycle (sets div_load_m*) and one USE cycle
// (reads MULT_RESULT and issues the next load).
//
// div_counter usage:
//   14 : SETUP (CLZ, normalize, LUT, abs-value, special cases)
//        Loads M1=d_norm, M2=r0 for the first multiply.
//   13 : settle (M1/M2 updating to d_norm / r0)
//   12 : MUL1 use (MULT_RESULT=d*r). eps = 0x80000000 - upper32.
//        Load M1=r0, M2=eps for MUL2.
//   11 : settle
//   10 : MUL2 use (MULT_RESULT=r*eps). r1 = upper >>30. Load d_norm/r1.
//    9 : settle
//    8 : MUL3 use (MULT_RESULT=d*r1). eps2. Load r1/eps2.
//    7 : settle
//    6 : MUL4 use (MULT_RESULT=r*eps2). r2. Load dividend/r2.
//    5 : settle
//    4 : MUL5 use (MULT_RESULT=dividend*r2). Extract q. Load q/divisor.
//    3 : settle
//    2 : MUL6 use (MULT_RESULT=q*divisor). Correction and sign.
//    1 : (skip via default decrement)
//    0 : DONE, div_write asserts, quotient/remainder written to MACH/MACL.
//
// Total busy cycles: 14 (counter 14..0).
// Special cases (zero div / signed overflow) complete in 2 busy cycles.

    assign div_write = (STATE == `DIVOP) & (div_counter == 6'd0);

    // Setup combinational wires (CLZ, normalize, LUT lookup)
    wire [31:0] div_setup_abs_d = (div_signed_op && M2[31]) ? (~M2 + 32'd1) : M2;
    wire [4:0]  div_setup_clz   = clz32(div_setup_abs_d);
    wire [31:0] div_setup_norm  = div_setup_abs_d << div_setup_clz;
    wire [31:0] div_setup_recip = recip_rom[div_setup_norm[30:23]];

    // Current multiply result (combinational MULT_RESULT on M1, M2).
    wire [31:0] div_mul_hi = MULT_RESULT[63:32];
    wire [31:0] div_mul_lo = MULT_RESULT[31:0];

    // NR reciprocal extraction: (r * eps) >> 30 from MULT_RESULT
    wire [31:0] div_nr_new_r = MULT_RESULT[61:30];

    // Quotient extraction from dividend*r2 result.
    //   q = MULT_RESULT >> (62 - shift)
    // For shift in [0,30]: MULT_RESULT[63:32] >> (30 - shift)
    // For shift == 31   : MULT_RESULT[62:31]    (one extra bit from low half)
    wire [31:0] div_q_from_mul = (div_shift == 5'd31) ? MULT_RESULT[62:31]
                                                      : (div_mul_hi >> (5'd30 - div_shift));

    // Quotient correction combinational logic (for counter=6)
    // MULT_RESULT = q * divisor (unsigned).
    wire [31:0] div_raw_rem_w = div_dividend - div_mul_lo;
    wire        div_q_ovf_w   = (div_mul_hi != 32'd0) || (div_mul_lo > div_dividend);
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
                    div_counter    <= 6'd14;
                    div_signed_op  <= 1'b0;
                    div_neg_q      <= 1'b0;
                    div_neg_r      <= 1'b0;
                    div_load_m1    <= 1'b0;
                    div_load_m2    <= 1'b0;
                end
                8'hB9: begin  // DIVS
                    div_counter    <= 6'd14;
                    div_signed_op  <= 1'b1;
                    div_neg_q      <= 1'b0;
                    div_neg_r      <= 1'b0;
                    div_load_m1    <= 1'b0;
                    div_load_m2    <= 1'b0;
                end
                default: ;
            endcase
        end else if (STATE == `DIVOP) begin
            div_load_m1 <= 1'b0;
            div_load_m2 <= 1'b0;

            case (div_counter)
                //---------------------------
                // SETUP (counter=14)
                //---------------------------
                6'd14: begin
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
                        // Kick MUL1: M1=d_norm, M2=r0 (takes effect at next edge)
                        div_load_m1 <= 1'b1;
                        div_load_m2 <= 1'b1;
                        div_m1_val  <= div_setup_norm;
                        div_m2_val  <= div_setup_recip;
                        div_counter <= 6'd13;
                    end
                end

                //---------------------------
                // MUL1 use (counter=12): MULT_RESULT = d*r.
                // eps = 0x80000000 - upper32.
                // Load M1=r0, M2=eps for MUL2.
                //---------------------------
                6'd12: begin
                    div_load_m1 <= 1'b1;
                    div_load_m2 <= 1'b1;
                    div_m1_val  <= div_recip;
                    div_m2_val  <= 32'h80000000 - div_mul_hi;
                    div_counter <= 6'd11;
                end

                //---------------------------
                // MUL2 use (counter=10): MULT_RESULT = r0*eps.
                // r1 = MULT_RESULT[61:30]. Store div_recip.
                // Load M1=d_norm, M2=r1 for MUL3.
                //---------------------------
                6'd10: begin
                    div_recip   <= div_nr_new_r;
                    div_load_m1 <= 1'b1;
                    div_load_m2 <= 1'b1;
                    div_m1_val  <= div_d_norm;
                    div_m2_val  <= div_nr_new_r;
                    div_counter <= 6'd9;
                end

                //---------------------------
                // MUL3 use (counter=8): MULT_RESULT = d*r1.
                // eps2 = 0x80000000 - upper32.
                // Load M1=r1, M2=eps2 for MUL4.
                //---------------------------
                6'd8: begin
                    div_load_m1 <= 1'b1;
                    div_load_m2 <= 1'b1;
                    div_m1_val  <= div_recip;
                    div_m2_val  <= 32'h80000000 - div_mul_hi;
                    div_counter <= 6'd7;
                end

                //---------------------------
                // MUL4 use (counter=6): MULT_RESULT = r1*eps2.
                // r2 = MULT_RESULT[61:30]. Store div_recip.
                // Load M1=dividend, M2=r2 for MUL5.
                //---------------------------
                6'd6: begin
                    div_recip   <= div_nr_new_r;
                    div_load_m1 <= 1'b1;
                    div_load_m2 <= 1'b1;
                    div_m1_val  <= div_dividend;
                    div_m2_val  <= div_nr_new_r;
                    div_counter <= 6'd5;
                end

                //---------------------------
                // MUL5 use (counter=4): MULT_RESULT = dividend*r2.
                // Extract q = MULT_RESULT >> (62 - shift).
                // Load M1=q, M2=divisor for verify MUL6.
                //---------------------------
                6'd4: begin
                    div_quotient <= div_q_from_mul;
                    div_load_m1  <= 1'b1;
                    div_load_m2  <= 1'b1;
                    div_m1_val   <= div_q_from_mul;
                    div_m2_val   <= div_original_d;
                    div_counter  <= 6'd3;
                end

                //---------------------------
                // MUL6 use (counter=2): MULT_RESULT = q*divisor.
                // Apply ±1 correction, restore sign, compute remainder.
                //---------------------------
                6'd2: begin
                    div_quotient  <= div_neg_q ? (~div_adj_q_w + 32'd1) : div_adj_q_w;
                    div_remainder <= div_neg_r ? {1'b0, ~div_adj_r_w + 32'd1} : {1'b0, div_adj_r_w};
                    div_counter   <= 6'd0;
                end

                //---------------------------
                // Default: decrement non-zero counter (settle cycles 13,11,9,7,5,3,1)
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