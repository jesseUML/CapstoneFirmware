// BoardPins.h
//
// GPIO assignments for both Capstone boards, derived from the KiCad designs:
//   Transmitter: CapstonePCBs/Transmitter/Capstone Transmitter/
//   Receiver   : CapstonePCBs/Reciever/CapstoneReciever/
//
// These are the single source of truth for pin numbers. If the hardware
// changes, change it here and both sketches follow.
//
// Both boards use an ESP32-WROOM-32UE (classic ESP32-D0WD-V3, no PSRAM,
// external antenna). Use a generic ESP32 Arduino board target, PSRAM disabled.

#ifndef CAPSTONE_BOARD_PINS_H
#define CAPSTONE_BOARD_PINS_H

#include <stdint.h>

// ===========================================================================
// Transmitter (remote)
// ===========================================================================
namespace tx_pins {

// 8-way DIGITAL joystick: four direction switches + two top buttons. The board
// was laid out to support an analog stick too, but the digital stick is the
// populated option, so GPIO34/35 (formerly the analog wiper inputs) are used
// as digital LEFT/RIGHT inputs here.
//
// All six inputs have external 10k pull-ups on the board -> idle HIGH, pressed
// LOW. Configure as plain INPUT (pull-up already present). GPIO34/35 are
// input-only with no internal pulls, but the external pull-ups cover that.
constexpr uint8_t kStickUp    = 33;  // net JOY_UP,       J4.2
constexpr uint8_t kStickDown  = 32;  // net JOY_DOWN,     J4.1
constexpr uint8_t kStickLeft  = 34;  // net JOY_X/LEFT,   J3.1 (digital now)
constexpr uint8_t kStickRight = 35;  // net JOY_Y/RIGHT,  J3.2 (digital now)
constexpr uint8_t kTopSoap    = 27;  // net JOY_BTN1, J5.2 -> Soap trigger
constexpr uint8_t kTopArm     = 26;  // net JOY_BTN2, J5.1 -> Arm/enable button

// Indicator outputs. Bare GPIO -> connector pin 1, GND on pin 2. Active HIGH
// (~3.3 V). An external LED needs its own series resistor. Init LOW.
constexpr uint8_t kLed1 = 16;  // net EoTLED1, J9.1
constexpr uint8_t kLed2 = 17;  // net EoTLED2, J10.1
constexpr uint8_t kLed3 = 19;  // net EoTLED3, J11.1
constexpr uint8_t kLed4 = 18;  // net EoTLED4, J12.1

}  // namespace tx_pins

// ===========================================================================
// Receiver (vehicle)
// ===========================================================================
namespace rx_pins {

// Solenoid low-side MOSFET gates. Each MCU pin drives a MOSFET gate through a
// series current-limit/anti-ring resistor; a gate pull-down holds the FET OFF
// through reset/boot. Active HIGH = solenoid ON. Init LOW before OUTPUT.
// Labels per the vehicle plumbing. Mains water is the main supply valve;
// soap and sprayer are downstream of it.
constexpr uint8_t kSolMainWater = 32;  // Solenoid 1, net Solenoid1Gate -> Q3 -> J3
constexpr uint8_t kSolSoap      = 33;  // Solenoid 2, net Solenoid2Gate -> Q4 -> J4
constexpr uint8_t kSolSprayer   = 26;  // Solenoid 3, net Solenoid3Gate -> Q5 -> J5
constexpr uint8_t kSolAux       = 27;  // Solenoid 4, net Solenoid4Gate -> Q6 -> J6

// Winch/drive motor direction contactors. J10 pins C and D each drive one
// 12 V contactor. These are independent ON/OFF enables, NOT dir+PWM.
// SAFETY: the two contactors are mutually exclusive — energizing both at once
// shorts/commands opposing directions. Firmware MUST interlock them.
// Active HIGH (3.3 V logic). Init LOW. Which physical direction each selects
// is a config point until confirmed on the vehicle.
constexpr uint8_t kMotorC = 16;  // net DIR/C, J10.1 -> contactor C (direction 1)
constexpr uint8_t kMotorD = 17;  // net PWM/D, J10.2 -> contactor D (direction 2)

// End-of-Travel microswitches. Four independent NORMALLY CLOSED switches,
// two per terminal block. Each switch connects its GPIO to the board GND pad;
// use the ESP32 internal pull-up (INPUT_PULLUP), so:
//   closed (LOW)  = OK to move
//   open   (HIGH) = at end of travel OR wire/switch fault -> STOP
// No external pull-up or 3.3 V reference is required.
constexpr uint8_t kEot1 = 18;  // J9.1  (EoTSwitch  sensor 1)
constexpr uint8_t kEot2 = 19;  // J9.2  (EoTSwitch  sensor 2)
constexpr uint8_t kEot3 = 21;  // J11.1 (EoTSwitch2 sensor 1)
constexpr uint8_t kEot4 = 22;  // J11.2 (EoTSwitch2 sensor 2)

// Which travel direction each EoT switch limits. The motor has two directions:
// FORWARD (contactor C) and REVERSE (contactor D). A tripped switch blocks
// ONLY its tagged direction, so the operator can always drive AWAY from a
// limit. CONFIG POINT — confirm on the bench by holding each switch open and
// checking which direction gets inhibited, then correct this table.
//
// Default split: EoT1/EoT2 limit FORWARD, EoT3/EoT4 limit REVERSE.
enum EotLimitDir : uint8_t {
  kLimitsForward = 0,  // tripping this switch blocks FORWARD (contactor C)
  kLimitsReverse = 1,  // tripping this switch blocks REVERSE (contactor D)
};
constexpr EotLimitDir kEotLimits[4] = {
  kLimitsForward,  // kEot1 (GPIO18)
  kLimitsForward,  // kEot2 (GPIO19)
  kLimitsReverse,  // kEot3 (GPIO21)
  kLimitsReverse,  // kEot4 (GPIO22)
};

}  // namespace rx_pins

#endif  // CAPSTONE_BOARD_PINS_H
