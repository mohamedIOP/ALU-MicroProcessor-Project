# ⚙️ ALU MicroProcessor — Verilog HDL Project
> A fully-featured 16-bit Arithmetic Logic Unit (ALU) implemented in Verilog, with a complete testbench and simulation waveforms.

---

## 📂 Repository Structure

```
ALU-MicroProcessor-Project/
│
├── ALU.v                          ← ALU module + testbench (main source)
├── ALU.v.out                      ← Simulation output log
├── ALU_TESTBENCH.vcd              ← VCD waveform dump (open with GTKWave)
├── ALU_Project.mpf                ← ModelSim project file
├── ALU_Project.cr.mti             ← ModelSim compile record
├── vsim.wlf                       ← ModelSim waveform log
├── transcript                     ← ModelSim simulation transcript
│
├── WF_0_100.jpg                   ← Waveform screenshot (0–100 ps)
├── WF_100_200.jpg                 ← Waveform screenshot (100–200 ps)
├── WF_200_300.jpg                 ← Waveform screenshot (200–300 ps)
├── WF_300_400.jpg                 ← Waveform screenshot (300–400 ps)
│
├── Assignment student version.pdf ← Original assignment spec
├── Final_Requirement.pdf          ← Final project requirements
└── work/                          ← ModelSim compiled library
```

---

## 📖 About

This project implements a **16-bit ALU** in Verilog HDL, supporting 18 operations across arithmetic, logical, and shift/rotate categories. It also computes a **6-bit status register** with full flag support.

The design was developed and simulated using **ModelSim**, with waveforms captured across the full testbench run.

---

## ⚡ Supported Operations

### Arithmetic

| Opcode (`F`) | Operation | Description |
|---|---|---|
| `5'b00001` | `INC` | Increment A by 1 |
| `5'b00011` | `DEC` | Decrement A by 1 |
| `5'b00100` | `ADD` | A + B |
| `5'b00101` | `ADD_CARRY` | A + B + Cin |
| `5'b00110` | `SUB` | A − B |
| `5'b00111` | `SUB_BORROW` | A − B − Cin |

### Logical

| Opcode (`F`) | Operation | Description |
|---|---|---|
| `5'b01000` | `AND` | A & B |
| `5'b01001` | `OR` | A \| B |
| `5'b01010` | `XOR` | A ^ B |
| `5'b01011` | `NOT` | ~A |

### Shift & Rotate

| Opcode (`F`) | Operation | Description |
|---|---|---|
| `5'b10000` | `SHL` | Logical shift left |
| `5'b10001` | `SHR` | Logical shift right |
| `5'b10010` | `SAL` | Arithmetic shift left |
| `5'b10011` | `SAR` | Arithmetic shift right (sign-extended) |
| `5'b10100` | `ROL` | Rotate left |
| `5'b10101` | `ROR` | Rotate right |
| `5'b10110` | `RCL` | Rotate left through carry |
| `5'b10111` | `RCR` | Rotate right through carry |

---

## 🚦 Status Register (6-bit)

| Bit | Flag | Description |
|---|---|---|
| `[5]` | **Carry (CF)** | Carry/borrow out of MSB; also set by shift/rotate ops |
| `[4]` | **Zero (ZF)** | Set when `Result == 0` |
| `[3]` | **Negative (NF)** | Set when `Result[15] == 1` (MSB) |
| `[2]` | **Overflow (OF)** | Set on signed arithmetic overflow |
| `[1]` | **Parity (PF)** | Set when result has even number of 1s |
| `[0]` | **Aux Carry (AF)** | Carry/borrow between nibbles (bits 3→4), used for BCD |

---

## 🔌 Module Interface

```verilog
module ALU (
    output reg [15:0] Result,   // 16-bit operation result
    output reg [5:0]  Status,   // 6-bit status/flags register
    input      [15:0] A, B,     // 16-bit operands
    input      [4:0]  F,        // 5-bit function select
    input             Cin       // Carry-in
);
```

---

## 🛠️ Tools & Requirements

This project was built and simulated with **ModelSim**. You can also use the free open-source flow:

- **[ModelSim / QuestaSim](https://www.intel.com/content/www/us/en/software/programmable/quartus-prime/model-sim.html)** — used for this project (`.mpf`, `.wlf` files included)
- **[Icarus Verilog (iverilog)](http://iverilog.icarus.com/)** — free alternative simulator
- **[GTKWave](http://gtkwave.sourceforge.net/)** — open the included `.vcd` file to inspect waveforms

### Simulate with Icarus Verilog

```bash
# Compile
iverilog -o alu_sim ALU.v

# Run (generates ALU_TESTBENCH.vcd)
vvp alu_sim

# View waveforms
gtkwave ALU_TESTBENCH.vcd
```

### Open in ModelSim

```bash
# Load the existing project
vsim -do "do ALU_Project.mpf"
```

---

## 📊 Simulation Waveforms

The testbench runs 32 test vectors over 320 ps, covering all 18 operations plus dedicated overflow and auxiliary carry edge cases. Waveform screenshots are included in the repo:

| File | Time Range |
|---|---|
| `WF_0_100.jpg` | 0 – 100 ps |
| `WF_100_200.jpg` | 100 – 200 ps |
| `WF_200_300.jpg` | 200 – 300 ps |
| `WF_300_400.jpg` | 300 – 400 ps |

---

## 📄 Project Documents

- **`Assignment student version.pdf`** — Original assignment specification
- **`Final_Requirement.pdf`** — Final project requirements and deliverables

---

## 🤝 Contributing

1. Fork the repository
2. Create a new branch (`git checkout -b fix/flag-logic`)
3. Commit your changes
4. Open a Pull Request

---

## 👤 Author

- **[mohamedIOP](https://github.com/mohamedIOP)** (SAWY) — Design, implementation & simulation
