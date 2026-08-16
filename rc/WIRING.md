# Carbanana RC — Wiring Guide

The remote controls **7 cars** over ESP-NOW. It has two analog joysticks for driving
the currently-selected car, plus a control surface (switches, buttons, rotaries) built on
three MCP23017 I²C port expanders.

- **Left stick** — drive: forward/back + strafe
- **Right stick** — rotation
- **Bank A** (8 switches) — turn each car's sound *sequence* on/off
- **Bank B** (8 switches) — car-select *mask* for one-off sounds
- **Bank C** (8 switches) — car-select *mask* for dance moves
- **Buttons 1–4** — one-off sound triggers (fire on the Bank B mask)
- **Buttons 5–8** — dance move triggers (fire on the Bank C mask)
- **Rotary 1** (8-pos, binary-coded) — choose which car the sticks drive
- **Rotary 2** (8-pos, binary-coded) — second selector (mode / function set)

**Count:** 3×8 switches + 2×4 buttons = 32 inputs = two full MCP23017s (0x20, 0x23). The two
binary rotaries = 6 lines, on a **third MCP23017 (0x21)** with 10 pins to spare.

---

## A word on the two "gotchas"

### I²C pull-ups — already onboard, add nothing
The two I²C lines (**SDA** and **SCL**) need a resistor tugging each back up to 3.3V when
released, or the bus won't communicate. These are **pull-ups**.

**Our boards already have them** — measured **9.87 kΩ** (SDA→VCC and SCL→VCC) = onboard
10 kΩ pull-ups. So:

- **Add no external pull-up resistors.**
- With all three boards on the bus, the three onboard 10 kΩ sets parallel to ≈3.3 kΩ — still
  well within spec (I²C wants roughly 1–10 kΩ), so no action needed.
- (If you ever used a board *without* onboard pull-ups, you'd add one set of ~4.7–10 kΩ for
  the whole bus — never one set per board.)

### Joystick VCC = 3V3, not 5V
The ESP32 ADC reads **0–3.3V max**. Feed a joystick pot from 5V and at full deflection it
outputs ~5V into the pin — over the limit, and can damage it. Power both sticks from **3V3**.

---

## ESP32 (RC) — direct pins

### Joystick 1 — left (drive + strafe), ADC1
| Stick pin | ESP32 |
|---|---|
| VRx | GPIO36 (VP) — strafe |
| VRy | GPIO39 (VN) — forward/back |
| VCC | 3V3 |
| GND | GND |

### Joystick 2 — right (rotation), ADC1
| Stick pin | ESP32 |
|---|---|
| VRx | GPIO34 — rotation |
| VRy | GPIO35 — spare (wired, unused) |
| VCC | 3V3 |
| GND | GND |

> Both sticks **must** be on ADC1 pins (32,33,34,35,36,39) — ESP-NOW/WiFi disables ADC2.

### I²C bus (to both expanders)
| Line | ESP32 |
|---|---|
| SDA | GPIO21 |
| SCL | GPIO22 |
| pull-ups | none needed — onboard (measured 9.87 kΩ) |

> The two rotaries live on the **third expander (0x21)** — see that section below. No rotary
> wires go to the ESP32; it only carries the sticks + I²C.

---

## How I²C addressing works — read this first

All three expanders share the **same two wires** (SDA + SCL). So the ESP32 needs a way to
say which chip it's talking to. That's the **address**: each chip has a unique number, and
you set it by choosing HIGH or LOW on three address inputs called **A0, A1, A2**.

Think of A2 A1 A0 as a 3-digit binary number added onto the base address `0x20`:

| A2 | A1 | A0 | Address |
|----|----|----|---------|
| L  | L  | L  | **0x20** |
| L  | L  | H  | **0x21** |
| L  | H  | L  | 0x22 |
| L  | H  | H  | 0x23 |
| …  | …  | …  | … up to 0x27 |

- **L** = tie that pin to **GND**.  **H** = tie it to **3V3**.
- All three chips **must** have different addresses, or they'll answer at once and the bus
  breaks.

### Setting the address on your three boards (as jumpered)
The address pins `A2 A1 A0` are solder-jumper pads. Open = 0 (pulled low), closed = 1.
Address = `0x20 + (A2·4 + A1·2 + A0·1)`.

| Board | A2 | A1 | A0 | Address |
|-------|----|----|----|---------|
| Chip #1 | open (0) | open (0) | open (0) | **0x20** |
| Chip #2 | open (0) | closed (1) | closed (1) | **0x23** |
| Chip #3 | open (0) | open (0) | **closed (1)** | **0x21** |

They don't need to be consecutive — any three different addresses work. Chips #1 and #2 are
already jumpered; set the **new third board to A0 closed** (A1, A2 open) for 0x21. Confirm
all three with the I²C scanner before building.

### INTA / INTB
Interrupt outputs (on the top connector). **Leave both unconnected** — we poll the chips
on a timer instead.

### RESET
This board handles RESET onboard (held high) — nothing to wire.

---

## MCP23017 #1 — address 0x20

> Board note: your board labels the I/O ports **PA0–PA7 / PB0–PB7** — these are the same
> as the datasheet's GPA/GPB. VCC/GND/SDA/SCL/INTA/INTB are on the top connector.

### Chip setup
| Board pin | To |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SCL | GPIO22 |
| SDA | GPIO21 |
| INTA, INTB | leave unconnected |
| A2, A1, A0 jumpers | all **open** → address **0x20** |

### Inputs
Each switch: one leg to the port pin, the other leg to **GND**. Enable the chip's
internal pull-ups in firmware → switches read **active-LOW** (closed = 0).

| Port | Function |
|---|---|
| PA0–PA7 | **Bank A** — 8 sound-sequence on/off switches (one per car) |
| PB0–PB7 | **Bank B** — 8 one-off-sound car-select switches (mask) |

---

## MCP23017 #2 — address 0x23

### Chip setup
Same as #1, except **A0 and A1 jumpers closed** (A2 open) → address **0x23**.

### Inputs
Switches/buttons: leg to the pin, other leg to GND, internal pull-ups on (active-LOW).

| Port | Function |
|---|---|
| PA0–PA7 | **Bank C** — 8 dance-move car-select switches (mask) |
| PB0–PB3 | Buttons 1–4 — one-off sound triggers |
| PB4–PB7 | Buttons 5–8 — dance move triggers |

---

## MCP23017 #3 — address 0x21 (the two rotaries)

### Chip setup
Same as #1, except **A0 jumper closed** (A1, A2 open) → address **0x21**.

### Inputs
Each 8-position **binary-coded rotary** outputs its position (0–7) as a 3-bit number: 3 code
lines + a common. Common → GND, the 3 code lines to the expander, internal pull-ups on → a
closed contact pulls that bit LOW (firmware inverts to read 0–7).

| Port | Function |
|---|---|
| PA0, PA1, PA2 | **Rotary 1** — bit0, bit1, bit2 (drive-car select) |
| PA3, PA4, PA5 | **Rotary 2** — bit0, bit1, bit2 (mode / function set) |
| PA6–PA7, PB0–PB7 | free (10 spare) |

Both rotaries' commons → **GND**.

> Your rotary has **6 pins (3+3)**. For an 8-position binary switch, the active ones are
> **1 common + 3 code bits** — the remaining pins are usually a second common (tie to the
> same GND) or unused. Use continuity to find them: the pin that connects to a changing set
> of others as you rotate is the **common**; the three that toggle are **bit0/1/2**. Map
> those three to PA0/PA1/PA2 (Rotary 1) and PA3/PA4/PA5 (Rotary 2).

---

## Power & ground rails

- **One GND rail** tying together: ESP32 GND, all three MCP23017 GND, all switch/button/
  rotary commons, both joystick GND.
- **3V3 rail** feeding: all three MCP23017 VCC, both joystick VCC.

> Decoupling caps: the MCP23017 breakout boards already have these onboard (the little SMD
> rectangles near the chip) — nothing to add. Only relevant if you ever wire a *bare* chip.

---

## I/O tally

| | Pins used |
|---|---|
| ESP32 | 4 ADC1 (sticks) + 2 I²C = 6 |
| MCP23017 #1 (0x20) | 16 / 16 — Bank A + Bank B |
| MCP23017 #2 (0x23) | 16 / 16 — Bank C + 8 buttons |
| MCP23017 #3 (0x21) | 6 / 16 — Rotary 1 + Rotary 2 (10 spare) |

Three boards on two I²C wires. The 10 spare pins on chip #3 are room for future controls
(more buttons, a status LED, a "send" button, etc.).
