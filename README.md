# Aquarius SH-2 CPU Core

This repository is a mirror of the **Aquarius** SH-2 compatible CPU core, originally published on OpenCores.

## Source

- **Origin**: OpenCores SVN repository
- **Original project page**: https://opencores.org/projects/aquarius
- **License**: See individual source files (LGPL)

## Contents

| Directory | Description |
|-----------|-------------|
| `verilog/` | CPU core Verilog HDL source files |
| `fpga/` | FPGA-specific files (Xilinx UCF, RAM generator) |
| `application/` | Sample applications (SH-2 C/assembly) with crt0 and linker scripts |
| `verification/` | Test sources and simulation tools |
| `doc/` | Original documentation (PDF/DOC) |

## Notes

- This is a mirror of the SVN trunk content.
- The CPU implements a subset of the SH-2 instruction set.
- FPGA target in the original project is Xilinx Spartan. For Cyclone V usage, see the parent project.
