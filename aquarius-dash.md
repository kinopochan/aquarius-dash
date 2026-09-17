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

**[branch: `feature/custom-shift-rotate-ops`, 未マージ]** 上記に加え、
`verilog/datapath.v` (+16 行, バレルシフタ配線)、`verilog/defines.v` (+4 行)、
`verilog/decode.v` (+39 行, SHAD/SHLD/ROTLV/ROTRV デコード) を変更。
`verilog/tb_shift_unit.v` / `verilog/tb_cpu_custom.v` を新規追加。
詳細は本ファイル 7 章参照。

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

## 7. [branch: `feature/custom-shift-rotate-ops`, 未マージ] 追加命令: SHAD / SHLD / ROTLV / ROTRV

**この章は `main` にはまだ入っていない。** ブランチ `feature/custom-shift-rotate-ops`
(commit `ad9739c`) の内容。マージ後は章番号を含めて見直すこと。

### 背景

`hobicom-artix` プロジェクトの CPU コア更新 (Artix-7 移植) に合わせて追加。
Artix-7 対応自体 (RAM の `$readmemh` 化等) は `hobicom-artix` リポジトリ側の
作業で、本リポジトリの改造点はここに記す独自命令のみ。

### エンコーディング

| 命令 | オペコード | 意味 |
|------|-----------|------|
| `SHAD Rm,Rn` | `4nmC` | Rm≥0: `Rn <<= Rm[4:0]` / Rm<0: `Rn >>>= 32-Rm[4:0]` (算術, 符号拡張) |
| `SHLD Rm,Rn` | `4nmD` | Rm≥0: `Rn <<= Rm[4:0]` / Rm<0: `Rn >>= 32-Rm[4:0]` (論理) |
| `ROTLV R0,Rn` | `4n34` | `Rn` を `R0[4:0]` ビット左ローテート |
| `ROTRV R0,Rn` | `4n35` | `Rn` を `R0[4:0]` ビット右ローテート |

- `SHAD`/`SHLD` は**実物の SH-3 のオペコードと完全にビット互換**(このコア自体は
  SH-2 のまま、対象命令だけ拡張)。回転量 0 のときの符号拡張/ゼロ埋めも実機仕様通り。
- `ROTLV`/`ROTRV` は SH-1〜4 のどのファミリにも存在しない完全な独自命令。
  4xxx シフト/ローテート系の空きスロット (`4n34`/`4n35`) に配置。
  回転量は R0 固定 (1 レジスタ形式の制約)。
- いずれも **T ビットは変化しない**。

### 実装 (decode.v / datapath.v / defines.v)

- `defines.v`: `SFTFUNC` に `SHAD/SHLD/ROTLV/ROTRV` の4値を追加 (既存 14 種の続き)
- `decode.v`: `4xxx` casex に `6'b??110?` (SHAD/SHLD) と `6'b11010?` (ROTLV/ROTRV,
  `INSTR_STATE[7:6]==00` で他スロットとの衝突回避) を追加
- `datapath.v`: バレルシフタ配線 (`DSFT_L/DSFT_R/DSFT_AR/DSFT_LR/DSFT_RL`、
  シフト量は `YBUS[4:0]`) を追加し、`SFTOUT` の `case` に4パターン追加。
  シフタの `always` 文は `YBUS` 依存が増えたため `@(SFTFUNC or XBUS or SR)` から
  `@*` に変更 (感度リスト漏れ対策)。

### 検証

- `verilog/tb_shift_unit.v`: バレルシフタ式を独立実装のリファレンスモデルと
  突き合わせ。境界値 (シフト量 0/31、正負) + ランダム 50 万件、不一致 0。
- `verilog/tb_cpu_custom.v`: `cpu` モジュール全体 (decode+datapath+register+mem)
  に簡易 Wishbone メモリを繋ぎ、手組み機械語を実行する結合テスト。
  SHAD/SHLD (正負シフト量) / ROTLV/ROTRV、計 5 ケース PASS。

### hobicom-artix (xc7a50t) での合成結果

- DSP48E1: 追加後の `CPU/DATAPATH` 配下は **0 DSP** (シフタは全て LUT 実装)。
  全体の DSP 増分 (+2, 7→9) はこの命令追加とは無関係で、乗算器 (`mult.v`,
  33×33 単サイクル化) 由来と判明 (`report_utilization -hierarchical` で確認)。
- Timing: `clk_cpu` 46.15MHz で当初 WNS -1.47ns (194 endpoints 違反) だったが、
  ワーストパスは `DECODE.TEMP`→`SR[0]`→`FSM_sequential_STATE`→`MEM.ADR` の
  既存制御パス (26 段) であり、追加した SHAD/SHLD/ROTLV/ROTRV のシフタ経路
  ではなかった。Vivado impl strategy を `Performance_ExplorePostRoutePhysOpt`
  に変更して timing closure (この経路はマージン薄めなので今後注意)。

### コンパイラから叩くには

SHAD/SHLD は GCC (`-m3`) が出す可能性があるが、既存アプリの Makefile は
すべて `-m2` のまま (要変更・未対応)。ROTLV/ROTRV はコンパイラが知らないので
`.word` 直書きかアセンブリ関数化が必須 (呼び出し規約は `divu`/`divs` の例と同様)。

**`-m3` 単体では足りない。`-fno-delayed-branch` も必須。** 追加命令自体とは
無関係な、SH-1/SH-2 と SH-3/SH-4 の `RTE` 実装差に由来する既知のコンパイラ制約
なので明記しておく。

- `-m3` にすると GCC は `interrupt_handler` 属性関数のエピローグで、
  `rte` の遅延スロットに `lds.l @r15+,mach` を詰め込む最適化を行うことがある。
- **原因 (確定):** GCC SH バックエンド (`gcc/config/sh/sh.md`) に明記されている。

  > On the SH1\*/SH2\*, the rte instruction reads the return pc from the
  > stack, and thus a pop instruction cannot be put in its delay slot.
  > On the SH3\*/SH4\*, the rte instruction does not use the stack, so a
  > pop instruction can go in the delay slot, unless it references a
  > banked register.

  SH-1/SH-2 の `RTE` は戻り先 PC を **自分で `R15` 経由でスタックから読む**
  実装 (このコアもその系譜、`decode.v` の `RTE (002B)` ハンドラが
  `EX_RDREG_X=R15`→`ALU_INCX4`→スタック読み出しを2回行う多サイクル
  シーケンス)。SH-3/SH-4 は戻り先 PC をバンクレジスタ相当に保持し
  スタックを使わないため、遅延スロットに `@Rn+`/`@-Rn` のようなスタック
  操作命令を置いても安全。**`-m3` は SH-3/SH-4 の RTE 実装を前提に
  遅延スロットへの pop 命令のスケジューリングを許可してしまうが、
  このコアの `RTE` は SH-1/SH-2 のスタックベース実装のまま**なので、
  GCC の前提とハードウェアの実装が食い違い、`rte` 自身のスタックアクセスと
  遅延スロットの pop 命令のスタックアクセスが衝突する。
  (`-m2` が常に遅延スロットに NOP を置くのは、GCC がこの制約を承知の上で
  安全側に倒しているだけで、コアの実装を検証した結果ではない。)
- `irq_init()` が `main()` の先頭付近でタイマ割込みを即有効化する構成だと、
  起動から 1ms 以内に最初の `RTE` (この新パターン込み) が実行されてしまい、
  **起動直後に固まる**。
- 症状は決定的ではなく **リセット10回中2〜3回しか起動しない**、という
  不安定な挙動になる (割込みタイミング依存で毎回このパターンを踏むとは
  限らないため)。追加命令のデコード衝突を疑いたくなるが無関係だった。
- シミュレーションでの追加確認: 単発の NMI + `rte`/`lds.l @r15+,mach` 単体
  では `tb_cpu_custom.v` 系の簡易テストでハングを再現できなかった
  (別途 `tb_rte_nmi_hang.v` で確認、リポジトリ未収録)。実機での不安定な
  再現率と整合的で、`RTE` 自身の内部スタックアクセスシーケンスと遅延スロット
  命令のタイミングが噛み合う特定の位相でのみ顕在化するとみられる。
  上記 GCC 側のコメントがアーキテクチャレベルの確定原因である以上、
  RTL 側のピンポイントな再現・修正は本改造の対象外とする。
- 回避策: `-m3` と同時に **`-fno-delayed-branch` を必ず付ける**。

---

## 8. 今後の候補 (未実装)

- `FADD / FSUB / FMUL / FDIV` 系は本改造に含まれない (SH-2 自体に FPU 命令がないため)
- DIVOP のレイテンシはさらに詰められる (settle を消すには M1/M2 を
  assign 化 or 同サイクル更新化が必要)
