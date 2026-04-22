# Aquarius SH-2 CPU Core (aquarius-dash)

This repository starts from an SVN mirror of the **Aquarius** SH-2 compatible CPU core
originally published on OpenCores (branch `original`, commit `c435e79`), and applies a
minimal set of modifications on top (branch `main`, aka **Aquarius'** / *aquarius-dash*).

- **Branch `original`**: untouched OpenCores SVN trunk snapshot.
- **Branch `main`**: modified version with faster multiplier and added DIVU/DIVS.
- `git diff original..main` shows exactly what was changed.
- See [`aquarius-dash.md`](aquarius-dash.md) for a detailed list of modifications,
  including cycle counts and verification status.

## Source

- **Origin**: OpenCores SVN repository
- **Original project page**: https://opencores.org/projects/aquarius
- **Author**: Thorn Aitch
- **License**: Public domain, with **OpenIP License** applying to derivative works
  (see `doc/Aquarius.pdf` p.2 "Copyright" section for authoritative terms).
  Any derivative work based on this IP must be free under OpenIP License,
  with a documented list of modifications ("the same as in GNU").
- **SH-2 ISA**: The SuperH-2 Instruction Set Architecture is the property of
  Renesas Technology Corp. Users adopting this core are responsible for
  judging potential infringements regarding Renesas's rights.
- **No warranty** on functionality or performance in real hardware.

## Contents

| Directory | Description |
|-----------|-------------|
| `verilog/` | CPU core Verilog HDL source files |
| `fpga/` | FPGA-specific files (Xilinx UCF, RAM generator) |
| `application/` | Sample applications (SH-2 C/assembly) with crt0 and linker scripts |
| `verification/` | Test sources and simulation tools |
| `doc/` | Original documentation (PDF/DOC) |

## Notes

- The `original` branch is a verbatim mirror of the SVN trunk content.
- The CPU implements a subset of the SH-2 instruction set.
- FPGA target in the original project is Xilinx Spartan. For Cyclone V usage, see the parent project.
- Modifications on `main` affect only `verilog/decode.v`, `verilog/defines.v`, `verilog/mult.v`, plus new testbenches `verilog/tb_mult.v` and `verilog/tb_div.v`. All other files are untouched.

## About the modifications in this fork

kinopochan makes no claim of rights over the modifications in the `main`
branch.  When using this code, please follow the original upstream
license (OpenIP License, see above) and include the attribution terms
it requires, with a documented list of modifications (this repository's
`aquarius-dash.md` serves as such a list).

この fork の改造差分 (`main` ブランチ) について、きのぽは特に権利
主張をしません。利用時はオリジナル (上記 OpenIP License) のライセンス
に従った表記 + 改造内容の記録を含めてください (本リポジトリの
`aquarius-dash.md` が改造内容の記録として使えます)。
