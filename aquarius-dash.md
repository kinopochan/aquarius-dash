# aquarius-dash — 改造内容まとめ

OpenCores の Aquarius (SH-2 互換コア) SVN trunk をベースに、本リポジトリで
加えた改造内容の一覧。オリジナルからの差分 **のみ** を記録する。

## ベース

- 起点: `c435e79` OpenCores SVN trunk 丸ごと import (42,280 行)
- 由来: https://opencores.org/projects/aquarius
- License: Public domain (original) + **OpenIP License** (derivatives) — 典拠 `doc/Aquarius.pdf` p.2 "Copyright"。詳細は `LICENSE` 参照。
- ターゲット: 元は Xilinx Spartan + UCF、現プロジェクトでは Cyclone V へ

## 改造コミット 3 本

| commit | 日付 | 主題 |
|--------|------|------|
| `ee554b9` | 2026-04-18 | DIVU/DIVS 追加 (非復元法, 35 cyc) |
| `f78d17f` | 2026-04-19 | 除算を DSP Newton-Raphson に差し替え (25 cyc) |
| `6b9d47b` | 2026-04-19 | 乗算器 31×16 → 32×32 単サイクル化 & DIVOP 14 cyc へ |

(`83e9efc` は tb_mult 追加のテストベンチのみ)

---

## 1. 追加命令: DIVU / DIVS

### エンコーディング

SH-2 の `3xxx` オペコード群は `3xx1` と `3xx9` が未定義スロット。ここを
使っている:

| オペコード | 命令 | 意味 |
|-----------|------|------|
| `0x3nm1` | `DIVU Rm, Rn` | 符号なし `Rn ÷ Rm` |
| `0x3nm9` | `DIVS Rm, Rn` | 符号付き `Rn ÷ Rm` |

- n = 被除数レジスタ (bit [11:8])
- m = 除数レジスタ (bit [7:4])
- 結果: **商 → MACH**, **剰余 → MACL**

### 特殊ケース (RISC-V 準拠)

| 条件 | 商 | 剰余 |
|------|-----|------|
| 除数 == 0 | `0xFFFFFFFF` | 被除数 |
| 符号付きオーバーフロー (`0x80000000 / -1`) | `0x80000000` | `0` |

### 実装 (decode.v / defines.v / mult.v)

- `decode.v`: `0x3xxx` デコード内に `4'b?001` パターンを追加。
  - `EX_MULCOM2 <= {1'b1, INSTR[14:12], INSTR[3:0]}` なので
    DIVU → `8'hB1`, DIVS → `8'hB9` (上位 bit=1 で既存 MULxxx と分離)
- `defines.v`: `DIVOP = 4'b1111` を追加
- `mult.v`: 専用ステートマシン `DIVOP` を新設

### アルゴリズムとサイクル数 (最終形)

| 世代 | コミット | 方式 | ビジー cyc |
|------|---------|------|----------|
| v1 | `ee554b9` | 非復元法 32 反復 | 35 |
| v2 | `f78d17f` | DSP + 256-entry LUT + NR 2 反復 | 25 |
| v3 | `6b9d47b` | v2 + 32×32 単サイクル乗算 | **14** |

v3 のフロー (`div_counter` 14→0):

```
14  SETUP  : CLZ → 正規化 → LUT 引き → abs → 特殊ケース判定
13  settle
12  MUL1   : d_norm * r0 → eps = 2 - d*r
11  settle
10  MUL2   : r0 * eps   → r1
 9  settle
 8  MUL3   : d_norm * r1 → eps2
 7  settle
 6  MUL4   : r1 * eps2  → r2
 5  settle
 4  MUL5   : dividend * r2 → 商候補 q
 3  settle
 2  MUL6   : q * divisor → 検算 → ±1 補正 & 符号付与
 1  (skip)
 0  DONE   : div_write で MACH/MACL 更新
```

- 乗算ごとに 2 cyc かかる理由は `div_load_m1/m2` が NBA 経由で M1/M2 を
  書くため、次サイクルでないと `MULT_RESULT` に反映されないから。
- 特殊ケース (0 除算 / オーバーフロー) は 2 cyc で完了。

### 追加された reg / wire

- `div_counter[5:0]`, `div_signed_op`, `div_neg_q/div_neg_r`
- `div_remainder[32:0]`, `div_quotient[31:0]`, `div_dividend`,
  `div_original_d`, `div_d_norm`, `div_recip`, `div_shift[4:0]`
- `div_load_m1/m2`, `div_m1_val`, `div_m2_val` (M1/M2 への割込書込)
- `recip_rom[0:255]` (Q2.30 逆数 LUT, `initial` で焼き込み)

### 検証

- `verilog/tb_div.v` : 2030 ケース (30 手書き + 2000 ランダム) PASS
- 剰余/商/ゼロ除算/符号オーバーフローを全網羅

---

## 2. 乗算の高速化 (`6b9d47b` "Case B-1")

### 変更前

- 31×16 部分積ベースの手組み乗算器 (`AH, BH, ABH, ABH2, P2, P3, P23, P23S, PM`
  と逐次パイプラインを組んで 32×32 を実現)
- 結果、以下の命令は **2 サイクル** (lower/upper 2 回発行):
  - `DMULS.L`, `DMULU.L`, `MUL.L`, `MAC.L`, `MAC.LS`

### 変更後

```verilog
wire signed [32:0] mul_a_ext = SIGN ? {A[31], A} : {1'b0, A};
wire signed [32:0] mul_b_ext = SIGN ? {B[31], B} : {1'b0, B};
wire signed [65:0] MULT_RAW  = mul_a_ext * mul_b_ext;   // 33×33 signed
wire        [63:0] MULT_RESULT = MULT_RAW[63:0];
```

- 32×32 結合乗算器に置換 (33×33 signed, Quartus が Cyclone V DSP 2 個に推論)
- 以下が **1 サイクル** 化:
  - `DMULS.L`, `DMULU.L`, `MUL.L`, `MAC.L`, `MAC.LS`
- `MULS.W`, `MULU.W`, `MAC.W` は元々 1 サイクル (変化なし)
- 16×16 のときは `A/B` の下位 16bit を extract (signed/unsigned 判別して符号拡張)
- C 選択ロジックで 16×16 結果は sign-extend、32×32 はフル 64bit

### 削除された内部信号

`AH, BH, ABH, ABH2, P2, P3, P23, P23S, PM, ZL, SHIFT パイプ`
(`SHIFT` レジスタ名は 9-bit 制御ワード互換のため残置、未使用)

### 残したもの

- 制御ワード `{SELA, SHIFT, SIGN, SIZE, ADD, LATMACH, LATMACL, ZH}`
- `MB` ラッチ (MAC.L/MAC.W の前回被乗数保持)
- 48bit / 32bit saturation adder パス (MAC.LS / MAC.WS)

### 2 サイクルだった state の扱い

`DMULSL2`, `DMULUL2`, `MACL2`, `MACLS2`, `MULL2` は `defines.v` に
定数としては残るが遷移先にしていない (後方互換のための残置)。

### 検証

- `verilog/tb_mult.v` : 1630 ケース (30 手書き + 1600 ランダム) PASS
  - `DMULS.L / DMULU.L / MUL.L / MULS.W / MULU.W / MAC.L / MAC.W` 網羅
  - MAC 系は `MB ← 前サイクルの M1` の 2-stage issue で SH-2 セマンティクス再現
- `verilog/tb_div.v` : 2030/2030 PASS (回帰確認)

---

## 3. 変更ファイル一覧

| ファイル | 変更 | 備考 |
|---------|------|------|
| `verilog/defines.v` | +1 行 | `DIVOP = 4'b1111` |
| `verilog/decode.v` | +22 行 | `3nm1/3nm9` デコード追加 |
| `verilog/mult.v` | 大改造 | 除算ユニット追加 + 乗算器置換 |
| `verilog/tb_div.v` | 新規 | 除算テストベンチ (2030 ケース) |
| `verilog/tb_mult.v` | 新規 | 乗算テストベンチ (1630 ケース) |
| `README.md` | 新規 | OpenCores 由来の明記 |

他のファイル (cpu.v, datapath.v, memory*.v, top.v, sys.v 等) は
SVN trunk **ノータッチ**。

---

## 4. コンパイラから叩くには

GCC の SH-2 用インラインアセンブリで `.word` 直書き。

```c
static inline uint32_t divu(uint32_t n, uint32_t m) {
    register uint32_t _n __asm__("r0") = n;
    register uint32_t _m __asm__("r1") = m;
    uint32_t q;
    __asm__ volatile (
        ".word 0x3011"      // DIVU R1, R0  → opcode 0x3nm1, n=0, m=1
        "\n\tsts mach, %0"
        : "=r"(q) : "r"(_n), "r"(_m) : "mach", "macl"
    );
    return q;
}
```

(n=Rn=被除数, m=Rm=除数 なので `0x3{n}{m}1`)

---

## 5. 注意点

- **DIV1 (0x3nm4) は従来どおり残っている**。既存の 64bit 除算ヘルパー
  (GCC libgcc の `__udivsi3` 等) は DIV1 を使うので壊れない。
- DIVU/DIVS は **MACH/MACL を上書き** する。MAC 系命令と併用するときは
  保存が必要 (`sts mach,Rx` / `lds Rx,mach`)。
- Cyclone V で 32×32 combinational 乗算は DSP 2 個 + 周辺ロジック。
  Fmax への影響はタイミングレポートで確認のこと。

---

## 6. 今後の候補 (未実装)

- `FADD / FSUB / FMUL / FDIV` 系は本改造に含まれない (SH-2 自体に FPU 命令がないため)
- DIVOP のレイテンシはさらに詰められる (settle を消すには M1/M2 を
  assign 化 or 同サイクル更新化が必要)
