# CapstoneFirmware

Firmware for the car wash remote-control system. Two ESP32 boards communicate
over **ESP-NOW**:

- **Transmitter** (handheld remote): reads an 8-way digital joystick (4
  direction switches) plus a Soap trigger and an Arm button, and sends
  commands. Reflects the receiver's End-of-Travel state on indicator LEDs.
- **Receiver** (wash vehicle): drives 4 solenoids and a motor, reads four
  End-of-Travel (EoT) microswitches, and reports state back to the remote.

Both boards use an **ESP32-WROOM-32UE** (classic ESP32-D0WD-V3, no PSRAM,
external antenna).

## Layout

```
CapstoneFirmware/
├── Transmitter/Transmitter.ino     # remote sketch
├── Receiver/Receiver.ino           # vehicle sketch
└── libraries/CapstoneProtocol/     # shared ESP-NOW wire protocol + pin map
    ├── library.properties
    └── src/
        ├── BoardPins.h             # GPIO assignments for both boards
        ├── CapstoneProtocol.h      # packet definitions + API
        └── CapstoneProtocol.cpp    # explicit (un)packing + CRC
```

The shared library holds only the wire contract and pin map. Hardware behavior
lives in each sketch.

## Build (Arduino IDE)

> Requires the **arduino-esp32 core 3.0.0 or newer**. Both sketches `#error`
> at compile time on older cores because the ESP-NOW receive callback
> signature differs on 2.x.

1. Install the **esp32** board package (Boards Manager → "esp32" by Espressif,
   3.x).
2. Set the IDE **Sketchbook location** to this `CapstoneFirmware` folder so the
   `libraries/CapstoneProtocol` library is discovered.
3. Open `Transmitter/Transmitter.ino` or `Receiver/Receiver.ino`.
4. Select board **ESP32 Dev Module**, PSRAM **Disabled**.
5. Flashing: this hardware has no USB/auto-reset. Close **JP1** (GPIO0→GND),
   power-cycle, upload, then open JP1 and power-cycle for normal boot. Use a
   3.3 V USB-UART adapter on J1 (TX↔RX crossed).

## Build (Arduino CLI)

```
arduino-cli compile --fqbn esp32:esp32:esp32 --libraries libraries Transmitter
arduino-cli compile --fqbn esp32:esp32:esp32 --libraries libraries Receiver
```

## End-of-Travel safety model

Four independent EoT microswitches, two per terminal block (J9: GPIO18/19,
J11: GPIO21/22). Each switch connects its GPIO to the board GND pad and uses
the ESP32 internal pull-up (`INPUT_PULLUP`) — no external resistor or 3.3 V
reference needed. Switches are **normally closed** for fail-safe operation:

| Switch state | GPIO reads | Meaning |
|---|---|---|
| Normal travel (closed) | LOW | OK to move |
| At end of travel (open) | HIGH | STOP |
| Broken wire / failed switch / unplugged (open) | HIGH | STOP (fails safe) |

EoT limits are **directional** (`kEotLimits[]` in `BoardPins.h`): each switch
blocks only the travel direction it limits, so the operator can always drive
*away* from a tripped limit. Default allocation: EoT1/EoT2 limit **forward**,
EoT3/EoT4 limit **reverse** — confirm on the bench by holding each switch open
and checking which direction is inhibited, then correct the table. A tripped
forward limit blocks forward but still allows reverse (and vice versa); a
broken/unplugged switch reads tripped (NC fail-safe) and blocks its direction.

Each switch maps to one transmitter LED (switch 1→LED1 … switch 4→LED4), lit
when that switch is tripped. If the remote loses the link it blinks all four
LEDs together rather than showing a stale EoT state.

EoT reads are debounced asymmetrically: a trip (open) confirms in
`kEotTripConfirmMs` (short, to reject contactor/motor EMI glitches) while a
clear (closed) confirms in `kEotClearMs` (longer, anti-flap). The state starts
tripped at power-up so any uncertainty fails safe.

## Motor safety (assistive-tech)

The winch/drive motor runs through two mutually-exclusive 12 V direction
contactors (J10 C/D). The receiver enforces several guards:

- **Mutual exclusion + reversal dead-time** (`kMotorDeadTimeMs`): both
  contactors are never energized at once; a direction change opens both and
  waits before closing the other.
- **Start dwell** (`kMotorStartDwellMs`, default 250 ms): the stick must call
  for the same direction continuously before the contactor energizes. Brief
  taps, tremor, or flicking through a direction will not start motion. Tune
  this to the individual user.
  - **First-move dwell waiver** (`#define WAIVE_START_DWELL_ON_FRESH_ARM`,
    enabled by default): the dwell is skipped for the first move right after a
    fresh arm, so the operator can drive off a limit immediately on re-arming
    without holding the stick. Later moves still use the full dwell. Comment
    the define out to always require the dwell.
- **Task watchdog** (`kWdtTimeoutMs`, default 1 s): `loop()` must feed the
  watchdog each pass. If it ever hangs, the chip resets; on reset the hardware
  gate/contactor pull-downs open everything, so a hang fails to the safe state.
- **Motion heartbeat** (`kMotorHeartbeatMs`, default 150 ms): the motor only
  runs if a valid command arrived this recently. This is tighter than the
  general link timeout (`kLinkTimeoutMs`, 300 ms, which drops everything to
  safe) and bounds how far a heavy machine can coast if the receiver loses the
  remote mid-travel.
- **Max continuous run** (`kMotorMaxRunMs`, default 20 s, 0 to disable): a
  single direction cannot run longer than this without the command returning
  to stop first. Bounds a stuck input or a failed EoT. After a trip the
  operator must release to stop and re-command to resume.
- After an EoT stop, the operator must release and re-command (and satisfy the
  start dwell again) to resume — no instant restart.

Hardware safety layers (outside firmware):

- **External E-stop** at the motor control box physically breaks the contactor
  supply. It works even if the MCU is wedged, so it is independent of all the
  firmware guards above. Firmware does not need to sense it to stay safe.
- **Motor current limiting** (bounding force / preventing crushing) is handled
  in hardware. Firmware does not model motor current.

> Note: the digital stick has no analog center, so there is no joystick
> calibration step. The old calibration flow was replaced by the arm model
> above.

## Open items / config points

These are intentionally marked `TODO`/`ASSUMPTION` in the code:

- **Peer MACs** — set `g_rxMac` (transmitter) and `g_txMac` (receiver) to the
  real STA MACs. Until then the TX broadcasts and the RX latches the first
  valid sender.
- **Motor direction contactors** — J10 C/D (GPIO16/17) each drive one 12 V
  contactor for the winch/drive motor. They are mutually exclusive; the
  receiver hard-interlocks them and enforces a dead-time (`kMotorDeadTimeMs`,
  default 100 ms — size to your contactor's break time). Which contactor is
  which physical direction is a config point in `applyCommand()`.
- **EoT direction binding** — each limit blocks only its direction
  (`kEotLimits[]`). Default EoT1/EoT2 = forward, EoT3/EoT4 = reverse; verify the
  physical assignment on the bench and correct the table.
- **Solenoid control sources** — only Soap has a remote input (top trigger).
  Mains water, Sprayer, and Aux have no transmitter control yet and stay OFF;
  decide their control source (manual input, or automation like auto-Mains
  while Soap runs) in the receiver's `applyCommand()`.
- **EoT → LED labeling** — switch i → LED i placeholder, easy to reassign once
  enclosure labeling is final.

### Design decisions (handled outside firmware)

- **Emergency stop** — an external hardware E-stop at the motor control box
  breaks the contactor supply directly. Chosen over a firmware E-stop because
  it is independent of MCU state. See the motor safety section.
- **Battery monitoring** — the PCBs expose no firmware-readable battery sense,
  so low battery is covered by an operator check documented in the user manual
  (first thing to check on a remote). An ADC divider could be added in a future
  board rev for a low-battery LED.

### Not yet addressed (future)

- **ESP-NOW encryption** — the link is currently unencrypted. ESP-NOW supports
  PMK/LMK encryption; if added, keep the key out of source (NVS or build-time
  injection), not hardcoded. (No WiFi SSID/password exists in this design —
  ESP-NOW joins no network — so there are no network credentials to protect.)
- **Link-quality indicator** — the remote shows link lost/OK but not degrading
  signal (RSSI / packet loss).

## Input model (digital stick)

The remote uses an 8-way **digital** joystick (4 switches: U/D/L/R; diagonals
are two adjacent switches) plus two top buttons. The board also supports an
analog stick, but the digital stick is the populated option, so GPIO34/35
(former analog wipers) are used as digital LEFT/RIGHT.

| Input | GPIO | Function |
|---|---|---|
| Stick UP | 33 | forward (with NW/NE) |
| Stick DOWN | 32 | reverse (with SW/SE) |
| Stick LEFT | 34 | unused (read only) |
| Stick RIGHT | 35 | unused (read only) |
| Top trigger | 27 | Soap solenoid |
| Top button | 26 | Arm (latching toggle) |

Motion rule (applied on the receiver): **any UP = forward, any DOWN = reverse**,
both or neither = stop. This gives a forgiving forward/back zone — NW/N/NE all
go forward, SW/S/SE all reverse — chosen for the client's accessibility needs.

## Arm model (assistive-tech)

Motion is gated by a latching **arm** toggle so the client never has to hold a
button:

- Quick press of the arm button toggles armed on; press again to disarm. No
  sustained hold.
- While armed, stick directions drive the motor. While disarmed, the remote
  sends zero directions and the receiver holds the motor stopped.
- **Auto-disarms** (requires a fresh press to re-arm) on any safety condition:
  EoT trip, link loss, boot, or an inactivity timeout (`kArmInactivityMs`,
  default 30 s with no stick input; set 0 to disable).
- The receiver independently requires the `kTxArmed` status bit before moving
  the motor, so a misbehaving transmitter cannot drive motion.
- Soap is allowed whether or not armed (arming only gates motion).

## Wire protocol summary

Little-endian, CRC-16/CCITT-FALSE validated. No raw structs on the wire.
Protocol **v2** (digital input model).

- **Command** (TX→RX, 12 B): seq, dirs (U/D/L/R bits), buttons (Soap), status
  (armed).
- **Feedback** (RX→TX, 11 B): seq, eot_state, output_state.
