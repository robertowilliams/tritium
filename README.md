# Tritium — SKY130 Ternary Cell Library & Smart Camera NPU

An open-source silicon project targeting the SkyWater SKY130 process node. Tritium combines a
ternary logic cell library exploration with a compact neural processing unit (NPU) for edge
smart-camera inference, designed to run through the OpenLane RTL-to-GDSII flow.

---

## Project Overview

Modern edge AI workloads demand inference at microwatt power budgets. Tritium explores two
complementary approaches on SKY130:

1. **Ternary Cell Library** — custom standard cells that operate on three logic states {−1, 0, +1},
   enabling ternary-weight neural networks with reduced multiplier complexity.
2. **Coral NPU (`coral_npu_top`)** — a systolic-inspired INT8 MAC array that processes grayscale
   camera frames and produces a real-time detection trigger, fully synthesizable with OpenLane
   on SKY130.

---

## Architecture

```
coral_npu_top
├── coral_host_if      APB slave — register map, config, status, IRQ
├── coral_cam_if       Camera pixel stream input (vsync / href / pclk)
├── coral_ctrl         Control sequencer FSM
├── coral_mac_array    8×8 INT8 systolic-inspired MAC array
├── coral_wgt_buf      Weight buffer SRAM wrapper (512 words)
├── coral_act_buf      Activation buffer SRAM wrapper (256 words)
├── coral_spad         Output scratchpad SRAM wrapper (256 words)
├── coral_post_proc    ReLU, quantization shift, pooling
└── coral_trigger      Threshold detection — outputs trig_out + score
```

The host configures the NPU over APB, loads weights into `coral_wgt_buf`, then asserts
`ctrl_start`. On the next camera frame (`cam_vsync`), pixels stream through `coral_cam_if`
into `coral_act_buf`. The control FSM drives the MAC array tile by tile, post-processing
results land in the scratchpad, and `coral_trigger` compares the score against a configurable
threshold to produce a hardware event output.

### Key Parameters

| Parameter    | Default | Description                        |
|--------------|---------|------------------------------------|
| `DATA_W`     | 8       | INT8 data width                    |
| `ACCUM_W`    | 32      | Accumulator width                  |
| `ARRAY_R`    | 8       | MAC array rows                     |
| `ARRAY_C`    | 8       | MAC array columns                  |
| `WGT_DEPTH`  | 512     | Weight buffer depth (words)        |
| `ACT_DEPTH`  | 256     | Activation buffer depth (words)    |
| `SPAD_DEPTH` | 256     | Scratchpad depth (words)           |

---

## Repository Structure

```
tritium/
├── README.md
├── config.json                                   # OpenLane flow configuration
├── SKY130 Ternary Cell Library Project Plan.pdf  # Full project plan
└── src/
    ├── coral_npu_top.v      Top-level integration wrapper
    ├── coral_host_if.v      APB slave & register map
    ├── coral_cam_if.v       Camera pixel stream interface
    ├── coral_ctrl.v         Control / sequencer FSM
    ├── coral_mac_array.v    INT8 MAC array
    ├── coral_wgt_buf.v      Weight buffer SRAM wrapper
    ├── coral_act_buf.v      Activation buffer SRAM wrapper
    ├── coral_spad.v         Output scratchpad SRAM wrapper
    ├── coral_post_proc.v    Post-processing (ReLU, shift, pool)
    └── coral_trigger.v      Detection threshold & event output
```

---

## Getting Started

### Prerequisites

- [OpenLane 2](https://openlane2.readthedocs.io/) with SKY130 PDK
- [OpenRAM](https://openram.org/) (for SRAM macro generation — see notes below)

### Running the Flow

```bash
cd tritium
openlane config.json
```

The flow is configured for a 25 MHz target clock on an 800×800 µm die at 35% utilization —
conservative settings chosen to give routing headroom with 64 MACs and three SRAM wrappers.

### OpenLane Configuration Notes

- **Clock**: 40 ns period (25 MHz) — safe for SKY130 HD cells
- **Die area**: 800×800 µm — sized generously for the initial run
- **Core utilization**: 35% — headroom for MAC array and SRAM placement
- **SRAM wrappers** in `src/` are behavioral stubs; replace with
  [sky130 OpenRAM macros](https://github.com/VLSIDA/OpenRAM) before tapeout
- Scale `ARRAY_R`/`ARRAY_C` down to 4×4 if synthesis is too slow or area too large

---

## Roadmap

- [ ] Synthesis-only run to verify elaboration and check area estimate
- [ ] Migrate behavioral SRAM stubs to sky130 OpenRAM macros
- [ ] Harden NPU as standalone macro
- [ ] Ternary cell library: define cell set ({INV3, BUF3, TMIN, TMAX, TSUM})
- [ ] Characterize ternary cells in sky130 corner (TT/SS/FF)
- [ ] Integrate ternary weight path into MAC array

---

## License

Intended for open-source release under Apache 2.0, consistent with the SKY130 / OpenLane ecosystem.

---

## References

- [SkyWater SKY130 PDK](https://github.com/google/skywater-pdk)
- [OpenLane RTL-to-GDSII flow](https://github.com/The-OpenROAD-Project/OpenLane)
- [OpenRAM SRAM compiler](https://github.com/VLSIDA/OpenRAM)
- [Efabless Open MPW Shuttle](https://efabless.com/open_shuttle_program)
