# Carbanana CAR — Wiring Guide (ESP32 + 2× TB6612)

The car receiver is an **ESP32 DevKit V1 (WROOM-32, 30-pin)** driving **four mecanum
wheels** through **two TB6612FNG motor drivers**, plus one solenoid on its own driver
board. This documents the **current pin layout** (as in `car.ino`), followed by an
appendix on the breadboard pin-alignment option we evaluated.

Motor layout (viewed from above):

```
   FL (M1) --- FR (M2)
   BL (M3) --- BR (M4)
```

Mecanum mix: `FL = y+x+r`, `FR = y-x-r`, `BL = y-x+r`, `BR = y+x-r`
(`y` = forward, `x` = strafe, `r` = rotation).

---

## Current layout — the pins actually wired

TB6612 control header (SparkFun-style):
`PWMA · AIN2 · AIN1 · STBY · BIN1 · BIN2 · PWMB · GND`.
**STBY → 3V3** (always enabled; firmware never toggles it). **GND → ground rail.**

### Driver 1 — front (FL = M1 / channel A, FR = M2 / channel B)

| Header pin | ESP32 | Firmware const |
|---|---|---|
| PWMA | GPIO4  | `M1_PWM` |
| AIN2 | GPIO19 | `M1_IN2` |
| AIN1 | GPIO18 | `M1_IN1` |
| STBY | → 3V3  | — |
| BIN1 | GPIO21 | `M2_IN1` |
| BIN2 | GPIO22 | `M2_IN2` |
| PWMB | GPIO5  | `M2_PWM` |
| GND  | → GND  | — |

### Driver 2 — back (BL = M3 / channel A, BR = M4 / channel B)

| Header pin | ESP32 | Firmware const |
|---|---|---|
| PWMA | GPIO16 | `M3_PWM` |
| AIN2 | GPIO26 | `M3_IN2` |
| AIN1 | GPIO25 | `M3_IN1` |
| STBY | → 3V3  | — |
| BIN1 | GPIO27 | `M4_IN1` |
| BIN2 | GPIO33 | `M4_IN2` |
| PWMB | GPIO17 | `M4_PWM` |
| GND  | → GND  | — |

### Other

| Function | ESP32 | Notes |
|---|---|---|
| Solenoid | GPIO23 | digital out, own driver board; active-high by default |
| USB serial | RX0/TX0 (GPIO3/1) | debug (`DEBUG 1`) — keep free |

### Driver → motor outputs

Each TB6612's **power-side header** (SparkFun order): `VM · VCC · GND · AO1 · AO2 · BO2 · BO1 · GND`.
Channel **A** (`AO1/AO2`) and channel **B** (`BO1/BO2`) each drive one motor.

| Driver | Channel | Output pads | Motor | Firmware |
|---|---|---|---|---|
| Driver 1 (front) | A | `AO1` / `AO2` | **FL** (front-left) | M1 |
| Driver 1 (front) | B | `BO1` / `BO2` | **FR** (front-right) | M2 |
| Driver 2 (back)  | A | `AO1` / `AO2` | **BL** (back-left)  | M3 |
| Driver 2 (back)  | B | `BO1` / `BO2` | **BR** (back-right) | M4 |

Which output pad is "+" only sets spin **direction** — if a wheel runs backwards, swap that
motor's two leads (or flip the sign for that wheel in the mecanum mix in `car.ino`).

### Power & ground (2-battery setup)

Motors are **3.7 V-rated**, so they run off their own 1S battery — **never** the 12 V rail.
The 12 V battery exists only to feed the buck that powers the ESP32.

| Rail | Source | Feeds |
|---|---|---|
| Motor supply (`VM`, both drivers) | **3.7 V battery +** | motor windings |
| Logic (`VCC` + `STBY`, both drivers) | **ESP32 `3V3`** | driver logic (tiny current) |
| ESP32 supply | **12 V battery → buck → 5 V** → ESP32 `VIN` | the ESP32 |
| **Common ground** | tie together | 3.7 V −, 12 V −, buck IN−/OUT−, ESP32 GND, both driver GND |

> ⚠️ The two battery **positives stay separate** (3.7 V → `VM` only; 12 V → buck `IN+` only),
> but **all grounds must be tied together** or the driver logic has no shared reference.
> Buck output: verify it's 5 V → `VIN`; if it reads 3.3 V, feed the ESP32 `3V3` pin instead.
> See the power-plan diagram discussion for the full topology.

### Firmware pin block (`car.ino`)

```cpp
// Driver 1 (front)
const int M1_IN1 = 18;  const int M1_IN2 = 19;  const int M1_PWM = 4;   // FL / AIN
const int M2_IN1 = 21;  const int M2_IN2 = 22;  const int M2_PWM = 5;   // FR / BIN
// Driver 2 (back)
const int M3_IN1 = 25;  const int M3_IN2 = 26;  const int M3_PWM = 16;  // BL / AIN
const int M4_IN1 = 27;  const int M4_IN2 = 33;  const int M4_PWM = 17;  // BR / BIN
const int SOLENOID_PIN = 23;
```

---

## Appendix — the "side-by-side breadboard" alignment option (not used)

**Question:** can the drivers be pin-mapped so each TB6612 sits beside the ESP32 with
every jumper running straight across, no crossings?

**Answer:** yes — but it requires reassigning the GPIOs, so we're **not** doing it on this
build (kept the layout above). Recorded here in case a future build wants it.

**Principle.** Only 6 of the header's 8 pins are GPIOs (PWMA, AIN2, AIN1, BIN1, BIN2,
PWMB). STBY (→3V3) and GND (→rail) aren't signals. Line the 8-pin header against 8
consecutive ESP32 pin-rows and the STBY row + GND row become throwaway rows that tap the
rails; the other six are straight jumpers. One driver per long edge of the ESP32.

On the DevKit V1 silk this lands almost perfectly:

| | PWMA | AIN2 | AIN1 | STBY | BIN1 | BIN2 | PWMB | GND |
|---|---|---|---|---|---|---|---|---|
| **Driver 1 → right edge** | 4 | 16 | 17 | (GPIO5)→3V3 | 18 | 19 | 21 | (RX0)→GND |
| **Driver 2 → left edge**  | 32 | 33 | 25 | (GPIO26)→3V3 | 27 | 14 | 12 ⚠ | (GPIO13)→GND |

- Right edge is clean: the throwaway rows fall on **GPIO5** (a strapping pin, good to
  leave alone) and **RX0** (keeps serial free).
- Left edge needs **GPIO12** for PWMB — a boot-strapping pin. Safe *only* because the
  TB6612 input is high-impedance and GPIO12's internal pull-down holds it low at boot; do
  not add an external pull-up. This is the one wrinkle that made the aligned layout less
  attractive than just keeping the current pins.

Reference — DevKit V1 physical pin order (USB at bottom):
- Left edge: `EN,36,39,34,35,32,33,25,26,27,14,12,13,GND,VIN`
- Right edge: `3V3,GND,15,2,4,16,17,5,18,19,21,RX0,TX0,22,23`
